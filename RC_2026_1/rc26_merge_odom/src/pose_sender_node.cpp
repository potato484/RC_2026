// RC2026 位姿发送独立节点
#include <rclcpp/rclcpp.hpp>
#include "rc26_merge_odom/pose_sender.hpp"
#include "rc26_serial/serial_driver.hpp"

class PoseSenderNode : public rclcpp::Node
{
public:
    PoseSenderNode() : Node("pose_sender_node")
    {
        this->declare_parameter("serial_port", "/dev/ttyUSB0");
        this->declare_parameter("baudrate", 115200);
        this->declare_parameter("cmd_vel_topic", "cmd_vel");
        this->declare_parameter("odom_topic", "merge_odom");
        this->declare_parameter("send_rate_hz", 50);

        std::string serial_port = this->get_parameter("serial_port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();

        serial_driver_ = std::make_shared<rc26_decision::SerialDriver>();
        if (!serial_driver_->open(serial_port, baudrate))
        {
            RCLCPP_ERROR(this->get_logger(), "串口打开失败: %s", serial_port.c_str());
            return;
        }

        rc26_merge_odom::PoseSender::Config config;
        config.cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
        config.odom_topic = this->get_parameter("odom_topic").as_string();
        config.send_rate_hz = this->get_parameter("send_rate_hz").as_int();

        pose_sender_ = std::make_unique<rc26_merge_odom::PoseSender>(*this, serial_driver_, config);
    }

private:
    std::shared_ptr<rc26_decision::SerialDriver> serial_driver_;
    std::unique_ptr<rc26_merge_odom::PoseSender> pose_sender_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseSenderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
