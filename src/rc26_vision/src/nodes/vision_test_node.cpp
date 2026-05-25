/// 推荐用法（多 Profile 配置）：
///   ros2 run rc26_vision vision_test_node --ros-args \
///     -p vision_config_file:=$(ros2 pkg prefix rc26_vision)/share/rc26_vision/config/vision_models.yaml
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "rc26_vision/inference/model_profile_loader.hpp"
#include "rc26_vision/inference/vision_inference_manager.hpp"

namespace rc26_vision {

class VisionTestNode : public rclcpp::Node {
public:
    VisionTestNode() : Node("vision_test_node") {
        this->declare_parameter<int>("print_rate_ms", 500);
        this->declare_parameter<std::string>("vision_config_file", "");

        const int print_rate_ms = this->get_parameter("print_rate_ms").as_int();
        const std::string config_file = this->get_parameter("vision_config_file").as_string();

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
