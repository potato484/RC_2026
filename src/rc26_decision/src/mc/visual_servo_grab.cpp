#include "visual_servo_grab.hpp"

#include <algorithm>
#include <cmath>
#include <exception>

#include "rc26_vision/inference/config/model_profile_loader.hpp"
#include "rc26_vision/inference/runtime/engine_factory.hpp"

namespace rc26_decision {

using namespace std::chrono_literals;

namespace {
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

double elapsedSec(const std::chrono::steady_clock::time_point& since) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - since).count();
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

VisualServoGrabAction::VisualServoGrabAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

VisualServoGrabAction::~VisualServoGrabAction() { stopWorker(); }

// onStart: 行为树首次进入本动作时调用一次。
// 负责：从黑板读取 node 和 mc_params → 创建 cmd_vel 发布器、0x19 feedback 订阅和 GRAB_TIP 服务客户端 →
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

    // 创建 cmd_vel 发布器：对齐阶段发 linear.y，前探阶段发 linear.x。
    cmd_pub_ = node_->create_publisher<TwistMsg>(params_.align_cmd_vel_topic, rclcpp::QoS(10));
    // 创建夹取服务客户端：限位触发后向 /mechanism/send_command 下发 GRAB_TIP。
    grab_client_ = node_->create_client<SendCommandSrv>(params_.grab_service_name);
    setupFeedbackSubscription();
    setupOdomSubscription();

    // 重置状态标志
    stop_requested_ = false;
    done_ = false;
    failed_ = false;
    phase_ = ServoPhase::Aligning;
    stable_count_ = 0;
    grab_attempted_ = false;
    grab_response_seen_ = false;
    grab_accepted_ = false;
    grab_generation_.fetch_add(1, std::memory_order_relaxed);
    waiting_for_limit_switch_ = false;
    limit_switch_triggered_ = false;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        has_odom_yaw_ = false;
        current_yaw_rad_ = 0.0;
        odom_receive_tp_ = {};
    }
    lost_active_ = false;
    approach_start_tp_ = {};
    last_pub_tp_ = {};
    last_grab_tp_ = {};
    target_lock_state_.reset();

    // 启动独立工作线程（避免阻塞行为树的 tick 循环）
    worker_ = std::thread(&VisualServoGrabAction::workerLoop, this);
    RCLCPP_INFO(node_->get_logger(),
                "武馆区视觉伺服启动: cmd_vel=%s model=%s odom=%s heading=%s target_yaw=%.3f grab_service=%s limit_feedback=%s id=0x%02X",
                params_.align_cmd_vel_topic.c_str(), params_.model_id.c_str(),
                params_.odom_topic.c_str(), params_.align_heading_hold_enable ? "开" : "关",
                params_.align_target_yaw_rad,
                params_.grab_service_name.c_str(),
                params_.grab_limit_switch_feedback_topic.c_str(),
                static_cast<unsigned int>(params_.grab_limit_switch_feedback_id & 0xFF));
    return BT::NodeStatus::RUNNING;
}

// onRunning: 行为树每次 tick 时调用。
// 负责：轮询工作线程的完成状态。工作线程结束时若 done_=true 返回 SUCCESS，
//       failed_=true 返回 FAILURE，否则继续返回 RUNNING。
BT::NodeStatus VisualServoGrabAction::onRunning() {
    if (done_ || failed_) {
        const bool succeeded = done_.load();
        stopWorker();
        return succeeded ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

// onHalted: 行为树被外部中断时调用。
// 负责：设置停止标志 → 等待工作线程退出 → 释放相机资源，确保安全停机。
void VisualServoGrabAction::onHalted() { stopWorker(); }

// 停止工作线程并释放相机
void VisualServoGrabAction::stopWorker() {
    stop_requested_ = true;
    waiting_for_limit_switch_ = false;
    grab_generation_.fetch_add(1, std::memory_order_relaxed);
    publishStop(true);
    if (worker_.joinable()) {
        worker_.join();
    }
    feedback_sub_.reset();
    odom_sub_.reset();
    grab_client_.reset();
    cmd_pub_.reset();
    if (camera_.isOpened()) {
        camera_.release();
    }
}

// ================================================================
// 工作线程主循环：初始化引擎/相机 → 横移对齐 → x 负向前探等 0x19 → 夹取 → 消失判定
// ================================================================
void VisualServoGrabAction::workerLoop() {
    // 初始化推理引擎和相机
    if (!initEngine() || !initCamera()) {
        publishStop(true);
        failed_ = true;
        return;
    }
    // 将配置的目标标签（如 "JK"）解析为模型输出的类别 ID
    resolveTargetClassIds();
    if (target_class_ids_.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "武馆区视觉伺服: 目标标签未匹配到任何类别");
        publishStop(true);
        failed_ = true;
        return;
    }

    start_tp_ = std::chrono::steady_clock::now();

    // 主循环：每帧读取相机 → 推理 → 对准/夹取逻辑
    while (!stop_requested_ && rclcpp::ok()) {
        // 超时保护
        if (params_.servo_timeout_s > 0.0 && elapsedSec(start_tp_) > params_.servo_timeout_s) {
            RCLCPP_WARN(node_->get_logger(), "武馆区视觉伺服超时 %.1fs", params_.servo_timeout_s);
            waiting_for_limit_switch_ = false;
            publishStop(true);
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

        // 按 tip test 同一口径选择并锁定目标，避免双端头同屏时反复切换。
        const auto selection = rc26_vision::updateTipAlignmentTarget(
            detections, frame.cols, target_class_ids_, target_lock_state_, makeAlignmentConfig());
        const bool has_target = selection.has_target;
        const int offset_px = selection.offset_px;

        if (phase_ == ServoPhase::ApproachingLimit) {
            if (limit_switch_triggered_.load(std::memory_order_relaxed)) {
                waiting_for_limit_switch_ = false;
                publishStop(true);
                phase_ = ServoPhase::SendingGrab;
                RCLCPP_INFO(node_->get_logger(), "武馆区前方限位已触发，准备下发 GRAB_TIP");
                continue;
            }
            bool heading_stale = false;
            double yaw_age_s = 0.0;
            const auto heading_control = computeHeadingControl(heading_stale, yaw_age_s);
            if (heading_stale) {
                publishStop(true);
                stable_count_ = 0;
                continue;
            }
            if (elapsedSec(approach_start_tp_) >= params_.grab_approach_timeout_s) {
                waiting_for_limit_switch_ = false;
                publishStop(true);
                RCLCPP_WARN(node_->get_logger(), "武馆区前探等待限位超时 %.2fs",
                            params_.grab_approach_timeout_s);
                failed_ = true;
                return;
            }
            const double vx = heading_control.allow_lateral ? computeApproachVx() : 0.0;
            publishCmd(vx, 0.0, heading_control.angular_z_radps, false);
            continue;
        }

        if (phase_ == ServoPhase::SendingGrab) {
            publishStop(false);
            switch (tickGrabCommand()) {
            case GrabStepStatus::Success:
                phase_ = ServoPhase::WaitingDone;
                lost_active_ = false;
                break;
            case GrabStepStatus::Failure:
                publishStop(true);
                failed_ = true;
                return;
            case GrabStepStatus::Running:
                break;
            }
            continue;
        }

        if (phase_ == ServoPhase::WaitingDone) {
            publishStop(false);
            if (!has_target) {
                if (!lost_active_) {
                    lost_active_ = true;
                    lost_since_tp_ = std::chrono::steady_clock::now();
                } else if (elapsedSec(lost_since_tp_) >= params_.grab_done_lost_time_s) {
                    RCLCPP_INFO(node_->get_logger(), "武馆区夹取完成: 端头消失 %.1fs",
                                params_.grab_done_lost_time_s);
                    publishStop(true);
                    done_ = true;
                    return;
                }
            } else {
                lost_active_ = false;
            }
            continue;
        }

        if (!has_target) {
            // 没有检测到目标：停止横移，重置稳定计数
            publishStop(false);
            stable_count_ = 0;
            continue;
        }

        // 有目标：重置消失标志
        lost_active_ = false;

        bool heading_stale = false;
        double yaw_age_s = 0.0;
        const auto heading_control = computeHeadingControl(heading_stale, yaw_age_s);
        if (heading_stale) {
            publishStop(true);
            stable_count_ = 0;
            continue;
        }

        // 判断是否已对准：像素 offset 和车身 yaw 都进入容差。
        const bool pixel_aligned = std::abs(offset_px) <= params_.align_tolerance_px;
        const bool aligned = pixel_aligned && heading_control.aligned;
        // 稳定计数：连续对准帧数达到阈值后才触发夹取（防止抖动误触发）
        stable_count_ = aligned ? (stable_count_ + 1) : 0;

        if (!grab_attempted_ && stable_count_ >= params_.align_stable_frames) {
            publishStop(true);
            stable_count_ = 0;
            beginApproach();
            continue;
        }

        if (!grab_attempted_) {
            // 未夹取、未对准：yaw 偏差过大时先只转向，进入 gate 后再允许横移对准。
            const double vy = heading_control.allow_lateral ? computeAlignmentVy(offset_px) : 0.0;
            publishCmd(0.0, vy, heading_control.angular_z_radps, false);
        }
    }

    publishStop(true);
}

// 根据水平像素偏移计算横移速度（P 控制器）
double VisualServoGrabAction::computeAlignmentVy(int offset_px) const {
    return rc26_vision::computeTipAlignmentVy(offset_px, makeAlignmentConfig());
}

bool VisualServoGrabAction::readAlignmentYaw(double& yaw_rad, double& age_s) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!has_odom_yaw_) {
        yaw_rad = 0.0;
        age_s = 0.0;
        return false;
    }
    yaw_rad = current_yaw_rad_;
    age_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - odom_receive_tp_).count();
    return age_s <= params_.align_odom_timeout_s;
}

rc26_vision::TipHeadingControl VisualServoGrabAction::computeHeadingControl(
    bool& stale, double& yaw_age_s) {
    stale = false;
    yaw_age_s = 0.0;
    auto config = makeAlignmentConfig();
    if (!config.heading_hold_enable) {
        return rc26_vision::computeTipHeadingControl(0.0, config);
    }

    double yaw_rad = 0.0;
    if (!readAlignmentYaw(yaw_rad, yaw_age_s)) {
        stale = true;
        rc26_vision::TipHeadingControl control;
        control.aligned = false;
        control.within_gate = false;
        control.allow_lateral = false;
        return control;
    }
    return rc26_vision::computeTipHeadingControl(yaw_rad, config);
}

double VisualServoGrabAction::computeApproachVx() const {
    return rc26_vision::computeTipApproachVx(params_.grab_approach_speed_mps);
}

// 发布 cmd_vel 速度（受 mc_align_command_rate_hz 限频）
void VisualServoGrabAction::publishCmd(double vx, double vy, double wz, bool force) {
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
    msg.linear.x = vx;
    msg.linear.y = vy;
    msg.angular.z = wz;
    cmd_pub_->publish(msg);
    last_pub_tp_ = now;
}

void VisualServoGrabAction::publishStop(bool force) {
    publishCmd(0.0, 0.0, 0.0, force);
}

void VisualServoGrabAction::setupFeedbackSubscription() {
    if (!node_) {
        return;
    }
    feedback_sub_ = node_->create_subscription<FeedbackMsg>(
        params_.grab_limit_switch_feedback_topic, rclcpp::QoS(32).reliable(),
        [this](const FeedbackMsg::SharedPtr msg) { handleFeedback(msg); });
}

void VisualServoGrabAction::setupOdomSubscription() {
    if (!node_ || !params_.align_heading_hold_enable) {
        return;
    }
    odom_sub_ = node_->create_subscription<OdomMsg>(
        params_.odom_topic, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const OdomMsg::SharedPtr msg) { handleOdom(msg); });
}

void VisualServoGrabAction::handleFeedback(const FeedbackMsg::SharedPtr msg) {
    if (!msg || !waiting_for_limit_switch_.load(std::memory_order_relaxed)) {
        return;
    }
    if (msg->feedback_id != static_cast<uint8_t>(params_.grab_limit_switch_feedback_id & 0xFF)) {
        return;
    }

    limit_switch_triggered_.store(true, std::memory_order_relaxed);
    if (cmd_pub_) {
        cmd_pub_->publish(TwistMsg{});
    }
}

void VisualServoGrabAction::handleOdom(const OdomMsg::SharedPtr msg) {
    if (!msg) {
        return;
    }
    std::lock_guard<std::mutex> lock(odom_mutex_);
    current_yaw_rad_ = yawFromQuaternion(msg->pose.pose.orientation);
    odom_receive_tp_ = std::chrono::steady_clock::now();
    has_odom_yaw_ = true;
}

void VisualServoGrabAction::beginApproach() {
    limit_switch_triggered_ = false;
    waiting_for_limit_switch_ = true;
    approach_start_tp_ = std::chrono::steady_clock::now();
    phase_ = ServoPhase::ApproachingLimit;
    RCLCPP_INFO(node_->get_logger(),
                "武馆区端头已对齐，开始 x 负向前探等待限位: vx=%.3f timeout=%.2fs feedback=0x%02X",
                computeApproachVx(), params_.grab_approach_timeout_s,
                static_cast<unsigned int>(params_.grab_limit_switch_feedback_id & 0xFF));
    publishCmd(computeApproachVx(), 0.0, 0.0, true);
}

VisualServoGrabAction::GrabStepStatus VisualServoGrabAction::tickGrabCommand() {
    if (grab_attempted_) {
        if (!grab_response_seen_.load(std::memory_order_relaxed)) {
            return GrabStepStatus::Running;
        }
        if (grab_accepted_.load(std::memory_order_relaxed)) {
            return GrabStepStatus::Success;
        }
        RCLCPP_WARN(node_->get_logger(), "GRAB_TIP 被拒绝");
        return GrabStepStatus::Failure;
    }

    if (!tryStartGrabCommand()) {
        return GrabStepStatus::Running;
    }
    return GrabStepStatus::Running;
}

// 下发 GRAB_TIP 夹取指令到 /mechanism/send_command 服务
bool VisualServoGrabAction::tryStartGrabCommand() {
    if (!grab_client_ || !grab_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "GRAB_TIP 跳过: 服务 %s 未就绪", params_.grab_service_name.c_str());
        return false;
    }

    auto request = std::make_shared<SendCommandSrv::Request>();
    request->command_id = static_cast<uint8_t>(params_.grab_command_id);
    request->payload = params_.grab_payload;

    rclcpp::Node* node = node_;
    const uint64_t token = grab_generation_.load(std::memory_order_relaxed);
    grab_response_seen_ = false;
    grab_accepted_ = false;
    grab_client_->async_send_request(
        request, [this, node, token](rclcpp::Client<SendCommandSrv>::SharedFuture future) {
            if (token != grab_generation_.load(std::memory_order_relaxed)) {
                return;
            }
            bool accepted = false;
            uint8_t seq = 0;
            try {
                const auto response = future.get();
                accepted = response && response->accepted;
                if (response) {
                    seq = response->seq;
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(node->get_logger(), "GRAB_TIP 响应异常: %s", e.what());
            }
            grab_accepted_.store(accepted, std::memory_order_relaxed);
            grab_response_seen_.store(true, std::memory_order_relaxed);
            if (accepted) {
                RCLCPP_INFO(node->get_logger(), "GRAB_TIP 已接受: seq=%u",
                            static_cast<unsigned int>(seq));
            }
        });
    grab_attempted_ = true;
    RCLCPP_INFO(node_->get_logger(), "GRAB_TIP 已下发: cmd=0x%02X",
                static_cast<unsigned int>(params_.grab_command_id));
    return true;
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

rc26_vision::TipAlignmentConfig VisualServoGrabAction::makeAlignmentConfig() const {
    rc26_vision::TipAlignmentConfig config;
    config.target_lock_enable = params_.align_target_lock_enable;
    config.target_lock_max_jump_px = params_.align_target_lock_max_jump_px;
    config.lost_stop_frames = params_.align_lost_stop_frames;
    config.tolerance_px = params_.align_tolerance_px;
    config.kp = params_.align_kp;
    config.min_speed_mps = params_.align_min_speed_mps;
    config.max_speed_mps = params_.align_max_speed_mps;
    config.invert_direction = params_.align_invert_direction;
    config.heading_hold_enable = params_.align_heading_hold_enable;
    config.target_yaw_rad = params_.align_target_yaw_rad;
    config.heading_kp = params_.align_heading_kp;
    config.heading_max_speed_radps = params_.align_heading_max_speed_radps;
    config.heading_tolerance_rad = params_.align_heading_tolerance_deg * kDeg2Rad;
    config.heading_gate_rad = params_.align_heading_gate_deg * kDeg2Rad;
    return config;
}

}  // namespace rc26_decision
