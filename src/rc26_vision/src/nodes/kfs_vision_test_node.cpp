/// kfs 主链(D455 + kfs.onnx)联调测试节点,带实时 OpenCV overlay 窗口。
/// 推荐用法:
///   ros2 launch rc26_vision test_kfs_vision.launch.py
/// 也可单独运行:
///   ros2 run rc26_vision kfs_vision_test_node --ros-args \
///     -p vision_config_file:=$(ros2 pkg prefix rc26_vision)/share/rc26_vision/config/vision_models.yaml
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rc26_vision/inference/config/model_profile_loader.hpp"
#include "rc26_vision/inference/runtime/vision_inference_manager.hpp"

namespace rc26_vision {

class VisionTestNode : public rclcpp::Node {
public:
    VisionTestNode() : Node("kfs_vision_test_node") {
        this->declare_parameter<int>("print_rate_ms", 500);
        this->declare_parameter<std::string>("vision_config_file", "");
        this->declare_parameter<bool>("show_window", true);
        this->declare_parameter<std::string>("window_name", std::string("KFS Vision - kfs.onnx"));
        this->declare_parameter<int>("display_rate_ms", 33);

        const int print_rate_ms = this->get_parameter("print_rate_ms").as_int();
        const std::string config_file = this->get_parameter("vision_config_file").as_string();
        show_window_ = this->get_parameter("show_window").as_bool();
        window_name_ = this->get_parameter("window_name").as_string();
        int display_rate_ms = this->get_parameter("display_rate_ms").as_int();
        if (display_rate_ms <= 0) {
            display_rate_ms = 33;
        }

        if (config_file.empty()) {
            RCLCPP_FATAL(this->get_logger(), "vision_config_file 参数为空");
            throw std::runtime_error("vision_config_file 参数为空");
        }

        manager_ = std::make_shared<VisionInferenceManager>(*this);

        try {
            auto config = ProfileLoader::loadFromYaml(config_file);
            manager_->loadConfig(config);
            if (!config.default_model.empty()) {
                manager_->selectModel(config.default_model);
            }
            RCLCPP_INFO(this->get_logger(), "视觉配置已加载: %s", config_file.c_str());
        } catch (const std::exception& e) {
            RCLCPP_FATAL(this->get_logger(), "视觉配置加载失败: %s", e.what());
            throw;
        }

        manager_->setResultCallback([this](const TargetResult& result) {
            if (result.has_target) {
                RCLCPP_INFO(this->get_logger(), "[检测] attr=%d dist=%.2fm score=%.2f bbox=(%d,%d)",
                    static_cast<int>(result.attr_kind), result.distance_m, result.score,
                    result.bbox_cx, result.bbox_cy);
            }
        });

        if (!manager_->start()) {
            RCLCPP_FATAL(this->get_logger(), "视觉模块启动失败");
            throw std::runtime_error("视觉模块启动失败");
        }

        RCLCPP_INFO(this->get_logger(), "视觉推理已启动");

        // 仅在需要显示时创建窗口;无显示环境(无 DISPLAY)会抛 cv::Exception,降级为无头。
        if (show_window_) {
            try {
                cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
            } catch (const cv::Exception& e) {
                RCLCPP_WARN(this->get_logger(),
                    "无法创建显示窗口(无显示环境?),降级为无头运行: %s", e.what());
                show_window_ = false;
            }
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(print_rate_ms),
            [this]() { printStatus(); });

        // 显示定时器在主线程(单线程 spin)执行,满足 OpenCV HighGUI 必须主线程的约束。
        if (show_window_) {
            display_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(display_rate_ms),
                [this]() { renderDisplay(); });
        }
    }

    ~VisionTestNode() {
        if (display_timer_) {
            display_timer_->cancel();   // 先停显示回调,避免析构期间再画窗口
        }
        if (manager_) {
            manager_->stop();
        }
        if (show_window_) {
            cv::destroyWindow(window_name_);
        }
    }

private:
    void printStatus() {
        const bool running = manager_->isRunning();
        const bool ready = manager_->isReady();
        const auto result = manager_->getLatestResult();

        RCLCPP_INFO(this->get_logger(),
            "[状态] running=%d ready=%d has_target=%d attr=%d dist=%.2fm",
            running, ready, result.has_target,
            static_cast<int>(result.attr_kind), result.distance_m);
    }

    void renderDisplay() {
        cv::Mat frame;
        std::vector<Detection> dets;
        TargetResult result;
        if (!manager_->getLatestDisplay(frame, dets, result) || frame.empty()) {
            // 首帧到达前显示占位图，避免窗口全黑
            cv::Mat placeholder(480, 640, CV_8UC3, cv::Scalar(30, 30, 30));
            cv::putText(placeholder, "Waiting for camera...",
                cv::Point(140, 250), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(200, 200, 200), 2, cv::LINE_AA);
            cv::imshow(window_name_, placeholder);
            cv::waitKey(1);
            return;
        }

        // 帧率(按显示回调次数 / 经过秒数统计,区别于推理帧率)
        ++fps_frame_count_;
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - fps_last_tp_).count();
        if (elapsed >= 0.5) {
            display_fps_ = static_cast<double>(fps_frame_count_) / elapsed;
            fps_frame_count_ = 0;
            fps_last_tp_ = now;
        }

        // 1) 所有检测框 + 类别名[id] + 置信度
        for (const auto& det : dets) {
            const int x1 = static_cast<int>(std::floor(det.x1));
            const int y1 = static_cast<int>(std::floor(det.y1));
            const int x2 = static_cast<int>(std::ceil(det.x2));
            const int y2 = static_cast<int>(std::ceil(det.y2));
            const cv::Rect box(x1, y1, std::max(0, x2 - x1), std::max(0, y2 - y1));
            if (box.width <= 0 || box.height <= 0) {
                continue;
            }
            cv::rectangle(frame, box, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

            std::ostringstream ss;
            const std::string name = det.class_name.empty()
                ? ("class_" + std::to_string(det.class_id)) : det.class_name;
            ss << name << "[" << det.class_id << "] "
               << std::fixed << std::setprecision(2) << det.score;
            const std::string text = ss.str();

            int baseline = 0;
            const cv::Size tsz =
                cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);
            (void)baseline;
            const int bx = std::max(0, box.x);
            const int by = std::max(0, box.y - tsz.height - 8);
            const cv::Rect bg(bx, by,
                std::min(frame.cols - bx, tsz.width + 8), tsz.height + 8);
            cv::rectangle(frame, bg, cv::Scalar(0, 120, 0), cv::FILLED);
            cv::putText(frame, text, cv::Point(bg.x + 4, bg.y + bg.height - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
        }

        // 2) 角落帧率 + 检测数
        std::ostringstream fps_ss;
        fps_ss << "FPS:" << std::fixed << std::setprecision(1) << display_fps_
               << "  Det:" << dets.size();
        cv::putText(frame, fps_ss.str(), cv::Point(12, 28),
            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(50, 220, 50), 2, cv::LINE_AA);

        // 3) 主目标(best target)类别 + D455 深度距离(kfs 主链特有)
        if (result.has_target) {
            std::ostringstream tgt_ss;
            tgt_ss << "TARGET " << result.class_name
                   << "  dist=" << std::fixed << std::setprecision(2) << result.distance_m << "m"
                   << "  score=" << std::setprecision(2) << result.score;
            cv::putText(frame, tgt_ss.str(), cv::Point(12, 56),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 215, 255), 2, cv::LINE_AA);
            cv::drawMarker(frame, cv::Point(result.bbox_cx, result.bbox_cy),
                cv::Scalar(0, 215, 255), cv::MARKER_CROSS, 18, 2, cv::LINE_AA);
        }

        cv::imshow(window_name_, frame);
        const int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) {
            RCLCPP_INFO(this->get_logger(), "键盘请求退出。");
            rclcpp::shutdown();
        }
    }

    std::shared_ptr<VisionInferenceManager> manager_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr display_timer_;

    bool show_window_ = true;
    std::string window_name_;
    std::chrono::steady_clock::time_point fps_last_tp_ = std::chrono::steady_clock::now();
    int fps_frame_count_ = 0;
    double display_fps_ = 0.0;
};

}  // namespace rc26_vision

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<rc26_vision::VisionTestNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("kfs_vision_test_node"), "启动失败: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
