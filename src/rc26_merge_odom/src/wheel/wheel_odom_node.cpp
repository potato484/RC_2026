// RC2026 串口轮式里程计独立节点
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include "rc26_merge_odom/wheel/wheel_odom.hpp"
#include "rc26_serial/serial_driver.hpp"

class WheelOdomNode : public rclcpp::Node {
public:
    WheelOdomNode() : Node("wheel_odom_node") {
        this->declare_parameter("chassis_model", "tracked_diff");
        this->declare_parameter("wheel_feedback_format", "tracked_lr_8b");
        this->declare_parameter("serial_port", "/dev/ttyUSB0");
        this->declare_parameter("baudrate", 1000000);
        this->declare_parameter("wheel_base", 0.62326);
        this->declare_parameter("track_width", 0.7);
        this->declare_parameter("publish_rate_hz", 50);
        this->declare_parameter("odom_topic", "wheel_odom");
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_frame", "base_link");
        this->declare_parameter("data_timeout_ms", 100.0);

        const std::string serial_port = this->get_parameter("serial_port").as_string();
        const int baudrate = this->get_parameter("baudrate").as_int();

        serial_ = std::make_shared<rc26_decision::SerialDriver>();
        if (!serial_->open(serial_port, baudrate)) {
            throw std::runtime_error("串口打开失败: " + serial_port);
        }

        rc26_merge_odom::WheelOdom::Config config;
        config.chassis_model = this->get_parameter("chassis_model").as_string();
        config.wheel_feedback_format = this->get_parameter("wheel_feedback_format").as_string();
        config.wheel_base = this->get_parameter("wheel_base").as_double();
        config.track_width = this->get_parameter("track_width").as_double();
        config.publish_rate_hz = this->get_parameter("publish_rate_hz").as_int();
        config.odom_topic = this->get_parameter("odom_topic").as_string();
        config.odom_frame = this->get_parameter("odom_frame").as_string();
        config.base_frame = this->get_parameter("base_frame").as_string();
        config.data_timeout_ms = this->get_parameter("data_timeout_ms").as_double();

        wheel_odom_ = std::make_unique<rc26_merge_odom::WheelOdom>(*this, serial_, config);
        if (!wheel_odom_->isReady()) {
            throw std::runtime_error("WheelOdom 初始化失败");
        }
    }

private:
    std::shared_ptr<rc26_decision::SerialDriver> serial_;
    std::unique_ptr<rc26_merge_odom::WheelOdom> wheel_odom_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try {
        auto node = std::make_shared<WheelOdomNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("wheel_odom_node"), "Exception: %s", e.what());
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}
