// RC2026 串口轮式里程计独立节点
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "rc26_merge_odom/wheel/wheel_odom.hpp"
#include "rc26_serial/serial_driver.hpp"

class WheelOdomNode : public rclcpp::Node {
public:
    WheelOdomNode() : Node("wheel_odom_node") {
        this->declare_parameter("serial_port", "/dev/ttyUSB1");
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
            RCLCPP_WARN(this->get_logger(), "串口打开失败: %s", serial_port.c_str());
        }

        rc26_merge_odom::WheelOdom::Config config;
        config.wheel_base = this->get_parameter("wheel_base").as_double();
        config.track_width = this->get_parameter("track_width").as_double();
        config.publish_rate_hz = this->get_parameter("publish_rate_hz").as_int();
        config.odom_topic = this->get_parameter("odom_topic").as_string();
        config.odom_frame = this->get_parameter("odom_frame").as_string();
        config.base_frame = this->get_parameter("base_frame").as_string();
        config.data_timeout_ms = this->get_parameter("data_timeout_ms").as_double();

        wheel_odom_ = std::make_unique<rc26_merge_odom::WheelOdom>(*this, serial_, config);
    }

private:
    std::shared_ptr<rc26_decision::SerialDriver> serial_;
    std::unique_ptr<rc26_merge_odom::WheelOdom> wheel_odom_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WheelOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
