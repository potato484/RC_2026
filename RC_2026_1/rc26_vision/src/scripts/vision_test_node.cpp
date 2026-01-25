/// ros2 run rc26_vision vision_test_node --ros-args -p vision_model_path:=/path/to/model.onnx
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "rc26_vision/vision_inference_manager.hpp"

namespace rc26_vision {

class VisionTestNode : public rclcpp::Node {
public:
    VisionTestNode() : Node("vision_test_node") {
        this->declare_parameter<std::string>("vision_model_path", "");
        this->declare_parameter<double>("vision_conf_thresh", 0.5);
        this->declare_parameter<int>("print_rate_ms", 500);

        const std::string model_path = this->get_parameter("vision_model_path").as_string();
        const double conf_thresh = this->get_parameter("vision_conf_thresh").as_double();
        const int print_rate_ms = this->get_parameter("print_rate_ms").as_int();

        if (model_path.empty()) {
            RCLCPP_FATAL(this->get_logger(), "vision_model_path 参数为空");
            throw std::runtime_error("vision_model_path 参数为空");
        }

        manager_ = std::make_shared<VisionInferenceManager>(*this);

        std::vector<std::string> class_names = {
            "R_R1", "B_R1",
            "T_03", "T_04", "T_05", "T_06", "T_07", "T_08", "T_09", "T_10",
            "T_11", "T_12", "T_13", "T_14", "T_15", "T_16", "T_17",
            "F_18", "F_19", "F_20", "F_21", "F_22", "F_23", "F_24", "F_25",
            "F_26", "F_27", "F_28", "F_29", "F_30", "F_31", "F_32"
        };

        if (!manager_->configure(model_path, class_names, static_cast<float>(conf_thresh))) {
            RCLCPP_FATAL(this->get_logger(), "视觉模块配置失败: %s", model_path.c_str());
            throw std::runtime_error("视觉模块配置失败");
        }

        RCLCPP_INFO(this->get_logger(), "视觉模块已配置: %s (conf=%.2f)", model_path.c_str(), conf_thresh);

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

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(print_rate_ms),
            [this]() { printStatus(); });
    }

    ~VisionTestNode() {
        if (manager_) {
            manager_->stop();
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

    std::shared_ptr<VisionInferenceManager> manager_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rc26_vision

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<rc26_vision::VisionTestNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("vision_test_node"), "启动失败: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
