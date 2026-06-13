#include "visual_servo_grab.hpp"

#include <algorithm>
#include <cmath>

#include "rc26_vision/inference/config/model_profile_loader.hpp"
#include "rc26_vision/inference/runtime/engine_factory.hpp"

namespace rc26_decision {

using namespace std::chrono_literals;

namespace {
double elapsedSec(const std::chrono::steady_clock::time_point& since) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - since).count();
}
}  // namespace

VisualServoGrabAction::VisualServoGrabAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

VisualServoGrabAction::~VisualServoGrabAction() { stopWorker(); }

// onStart: 行为树首次进入本动作时调用一次。
// 负责：从黑板读取 node 和 mc_params → 创建 cmd_vel 发布器(横移)和 GRAB_TIP 服务客户端 →
//       启动独立工作线程 workerLoop，由工作线程接管视觉伺服全流程。
// 返回 RUNNING，后续由 onRunning 轮询工作线程的完成状态。
BT::NodeStatus VisualServoGrabAction::onStart() {
    if (!config().blackboard->get("node", node_) || node_ == nullptr) {
        return BT::NodeStatus::FAILURE;
    }
    if (!config().blackboard->get("mc_params", params_)) {
        RCLCPP_ERROR(node_->get_logger(), "武馆区视觉伺服: 黑板缺少 mc_params");
        return BT::NodeStatus::FAILURE;
    }

    // 创建 cmd_vel 发布器：视觉伺服通过 linear.y 控制底盘横向移动对齐端头
    cmd_pub_ = node_->create_publisher<TwistMsg>(params_.align_cmd_vel_topic, rclcpp::QoS(10));
    // 创建夹取服务客户端：对齐完成后向 /mechanism/send_command 下发 GRAB_TIP
    grab_client_ = node_->create_client<SendCommandSrv>(params_.grab_service_name);

    // 重置状态标志
    stop_requested_ = false;
    done_ = false;
    failed_ = false;
    stable_count_ = 0;
    grab_attempted_ = false;
    lost_active_ = false;
    last_pub_tp_ = {};
    last_grab_tp_ = {};

    // 启动独立工作线程（避免阻塞行为树的 tick 循环）
    worker_ = std::thread(&VisualServoGrabAction::workerLoop, this);
    RCLCPP_INFO(node_->get_logger(),
                "武馆区视觉伺服启动: cmd_vel=%s model=%s grab_service=%s",
                params_.align_cmd_vel_topic.c_str(), params_.model_id.c_str(),
                params_.grab_service_name.c_str());
    return BT::NodeStatus::RUNNING;
}

// onRunning: 行为树每次 tick 时调用。
// 负责：轮询工作线程的完成状态。工作线程结束时若 done_=true 返回 SUCCESS，
//       failed_=true 返回 FAILURE，否则继续返回 RUNNING。
BT::NodeStatus VisualServoGrabAction::onRunning() {
    if (done_ || failed_) {
        // 等待工作线程完全退出后再返回最终状态
        if (worker_.joinable()) {
            worker_.join();
        }
        return done_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

// onHalted: 行为树被外部中断时调用。
// 负责：设置停止标志 → 等待工作线程退出 → 释放相机资源，确保安全停机。
void VisualServoGrabAction::onHalted() { stopWorker(); }

// 停止工作线程并释放相机
void VisualServoGrabAction::stopWorker() {
    stop_requested_ = true;
    if (worker_.joinable()) {
        worker_.join();
    }
    if (camera_.isOpened()) {
        camera_.release();
    }
}

// ================================================================
// 工作线程主循环：初始化引擎/相机 → 目标检测 → 横移对齐 → 夹取 → 消失判定
// ================================================================
void VisualServoGrabAction::workerLoop() {
    // 初始化推理引擎和相机
    if (!initEngine() || !initCamera()) {
        failed_ = true;
        return;
    }
    // 将配置的目标标签（如 "JK"）解析为模型输出的类别 ID
    resolveTargetClassIds();
    if (target_class_ids_.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "武馆区视觉伺服: 目标标签未匹配到任何类别");
        failed_ = true;
        return;
    }

    start_tp_ = std::chrono::steady_clock::now();

    // 主循环：每帧读取相机 → 推理 → 对准/夹取逻辑
    while (!stop_requested_ && rclcpp::ok()) {
        // 超时保护
        if (params_.servo_timeout_s > 0.0 && elapsedSec(start_tp_) > params_.servo_timeout_s) {
            RCLCPP_WARN(node_->get_logger(), "武馆区视觉伺服超时 %.1fs", params_.servo_timeout_s);
            publishCmdVy(0.0, true);
            failed_ = true;
            return;
        }

        // 读取相机帧
        cv::Mat frame;
        if (!camera_.read(frame) || frame.empty()) {
            std::this_thread::sleep_for(20ms);
            continue;
        }

        // 推理：获取本帧所有检测结果
        const std::vector<rc26_vision::Detection> detections = engine_->infer(frame);

        // 从检测结果中筛选目标类别的最大面积框，计算其中心相对图像中心的水平偏移
        bool has_target = false;
        int offset_px = 0;
        int best_area = -1;
        for (const auto& det : detections) {
            // 只关注 mc_target_labels 指定的目标类别
            if (!isTargetClass(det.class_id)) {
                continue;
            }
            const int w = static_cast<int>(std::ceil(det.x2)) - static_cast<int>(std::floor(det.x1));
            const int h = static_cast<int>(std::ceil(det.y2)) - static_cast<int>(std::floor(det.y1));
            if (w <= 0 || h <= 0) {
                continue;
            }
            const int area = w * h;
            if (area > best_area) {
                best_area = area;
                const int cx = static_cast<int>(std::floor(det.x1)) + w / 2;
                // offset_px > 0 表示目标在图像右侧，< 0 在左侧
                offset_px = cx - frame.cols / 2;
                has_target = true;
            }
        }

        if (!has_target) {
            // 没有检测到目标：停止横移，重置稳定计数
            publishCmdVy(0.0, false);
            stable_count_ = 0;
            if (grab_attempted_) {
                // 已夹取过：端头消失计时，持续消失超过 grab_done_lost_time_s 判定完成
                if (!lost_active_) {
                    lost_active_ = true;
                    lost_since_tp_ = std::chrono::steady_clock::now();
                } else if (elapsedSec(lost_since_tp_) >= params_.grab_done_lost_time_s) {
                    RCLCPP_INFO(node_->get_logger(), "武馆区夹取完成: 端头消失 %.1fs",
                                params_.grab_done_lost_time_s);
                    publishCmdVy(0.0, true);
                    done_ = true;
                    return;
                }
            }
            continue;
        }

        // 有目标：重置消失标志
        lost_active_ = false;

        // 判断是否已对准：offset 在容差范围内
        const bool aligned = std::abs(offset_px) <= params_.align_tolerance_px;
        // 稳定计数：连续对准帧数达到阈值后才触发夹取（防止抖动误触发）
        stable_count_ = aligned ? (stable_count_ + 1) : 0;

        if (!grab_attempted_ && stable_count_ >= params_.align_stable_frames) {
            // 首次对准稳定：下发 GRAB_TIP 夹取指令
            sendGrabCommand();
            publishCmdVy(0.0, true);
            stable_count_ = 0;
            continue;
        }

        if (!grab_attempted_) {
            // 未夹取、未对准：计算横移速度，发布 cmd_vel.linear.y 驱动机器人横向对准
            double vy = computeAlignmentVy(offset_px);
            publishCmdVy(vy, false);
        }
        // 已夹取后不做横移，等待端头消失判定
    }
}

// 根据水平像素偏移计算横移速度（P 控制器）
double VisualServoGrabAction::computeAlignmentVy(int offset_px) const {
    // 基础速度 = 偏移量 × Kp（比例增益）
    double speed = std::abs(static_cast<double>(offset_px)) * params_.align_kp;
    // 钳位在 [min_speed, max_speed] 范围
    speed = std::max(params_.align_min_speed_mps, std::min(speed, params_.align_max_speed_mps));
    // 方向：目标在图像右侧(offset>0)则向右横移(vy<0)，左侧反之
    //       mc_align_invert_direction 为 true 时方向反转
    double direction = offset_px > 0 ? -1.0 : 1.0;
    if (params_.align_invert_direction) {
        direction = -direction;
    }
    return direction * speed;
}

// 发布 cmd_vel.linear.y 横移速度（受 mc_align_command_rate_hz 限频）
void VisualServoGrabAction::publishCmdVy(double vy, bool force) {
    if (!cmd_pub_) {
        return;
    }
    const auto now = std::chrono::steady_clock::now();
    // 非强制发布时按设定频率限频
    if (!force && last_pub_tp_ != std::chrono::steady_clock::time_point{}) {
        const double min_period = 1.0 / std::max(1e-6, params_.align_command_rate_hz);
        if (std::chrono::duration<double>(now - last_pub_tp_).count() < min_period) {
            return;
        }
    }
    TwistMsg msg;
    msg.linear.y = vy;
    cmd_pub_->publish(msg);
    last_pub_tp_ = now;
}

// 下发 GRAB_TIP 夹取指令到 /mechanism/send_command 服务
void VisualServoGrabAction::sendGrabCommand() {
    if (!grab_client_ || !grab_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "GRAB_TIP 跳过: 服务 %s 未就绪", params_.grab_service_name.c_str());
        return;
    }

    auto request = std::make_shared<SendCommandSrv::Request>();
    request->command_id = static_cast<uint8_t>(params_.grab_command_id);
    request->payload = params_.grab_payload;

    rclcpp::Node* node = node_;
    grab_client_->async_send_request(
        request, [node](rclcpp::Client<SendCommandSrv>::SharedFuture future) {
            const auto response = future.get();
            if (response && response->accepted) {
                RCLCPP_INFO(node->get_logger(), "GRAB_TIP 已接受: seq=%u",
                            static_cast<unsigned int>(response->seq));
            } else {
                RCLCPP_WARN(node->get_logger(), "GRAB_TIP 被拒绝");
            }
        });
    grab_attempted_ = true;
    RCLCPP_INFO(node_->get_logger(), "GRAB_TIP 已下发: cmd=0x%02X",
                static_cast<unsigned int>(params_.grab_command_id));
}

// 加载视觉推理引擎（从 vision_models.yaml 读取 tip_default 模型配置文件）
bool VisualServoGrabAction::initEngine() {
    try {
        const auto config = rc26_vision::ProfileLoader::loadFromYaml(params_.vision_config_file);
        rc26_vision::ProfileLoader::validate(config);
        const auto it = config.profiles.find(params_.model_id);
        if (it == config.profiles.end()) {
            RCLCPP_ERROR(node_->get_logger(), "视觉 model_id '%s' 未在 %s 中找到",
                         params_.model_id.c_str(), params_.vision_config_file.c_str());
            return false;
        }
        // 保存模型类别名列表（如 tip_labels.txt 中的 "JK"）
        class_names_ = it->second.labels;
        engine_ = rc26_vision::createInferenceEngine(it->second);
        return engine_ != nullptr;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "初始化推理引擎失败: %s", e.what());
        return false;
    }
}

// 打开 USB 相机（优先指定设备号/路径，失败时自动扫描 0-9）
bool VisualServoGrabAction::initCamera() {
    bool ok = params_.camera_device.empty() ? openCamera(params_.camera_index, "")
                                             : openCamera(-1, params_.camera_device);
    if (!ok && params_.auto_scan_camera) {
        for (int i = 0; i < 10 && !ok; ++i) {
            if (i == params_.camera_index) {
                continue;
            }
            ok = openCamera(i, "");
        }
    }
    if (!ok) {
        RCLCPP_ERROR(node_->get_logger(), "武馆区视觉伺服: 无法打开相机 (index=%d device='%s')",
                     params_.camera_index, params_.camera_device.c_str());
    }
    return ok;
}

// 打开单个相机并设置 MJPG 格式、分辨率、帧率
bool VisualServoGrabAction::openCamera(int index, const std::string& path) {
    if (camera_.isOpened()) {
        camera_.release();
    }
    bool opened = path.empty() ? (camera_.open(index, cv::CAP_V4L2) || camera_.open(index))
                               : (camera_.open(path, cv::CAP_V4L2) || camera_.open(path));
    if (!opened) {
        return false;
    }
    camera_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    camera_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(params_.width));
    camera_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(params_.height));
    camera_.set(cv::CAP_PROP_FPS, static_cast<double>(params_.fps));

    // 验证相机可用：读取一帧测试
    cv::Mat test_frame;
    if (!camera_.read(test_frame) || test_frame.empty()) {
        camera_.release();
        return false;
    }
    return true;
}

// 将 mc_target_labels（如 "JK"）解析为模型输出的类别 ID 列表
void VisualServoGrabAction::resolveTargetClassIds() {
    target_class_ids_.clear();
    for (const auto& label : params_.target_labels) {
        const auto it = std::find(class_names_.begin(), class_names_.end(), label);
        if (it != class_names_.end()) {
            target_class_ids_.push_back(static_cast<int>(std::distance(class_names_.begin(), it)));
        } else {
            RCLCPP_WARN(node_->get_logger(), "目标标签 '%s' 不在模型标签中", label.c_str());
        }
    }
}

// 判断检测框的类别 ID 是否属于目标类别
bool VisualServoGrabAction::isTargetClass(int class_id) const {
    return std::find(target_class_ids_.begin(), target_class_ids_.end(), class_id) !=
           target_class_ids_.end();
}

}  // namespace rc26_decision
