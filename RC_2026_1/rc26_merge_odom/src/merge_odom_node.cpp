// RC2026 融合里程计主节点
// 整合CAN里程计和位姿下传功能
#include <rclcpp/rclcpp.hpp>
#include "rc26_merge_odom/can_odom.hpp"
#include "rc26_merge_odom/pose_sender.hpp"
#include "rc26_serial/serial_driver.hpp"

class MergeOdomNode : public rclcpp::Node
{
public:
    MergeOdomNode() : Node("merge_odom_node")
    {
        // CAN里程计参数
        this->declare_parameter("can_interface", "can0");
        this->declare_parameter("wheel_radius", 0.07625);
        this->declare_parameter("wheel_base", 0.62326);
        this->declare_parameter("track_width", 0.7);
        this->declare_parameter("gear_ratio", 3591.0 / 187.0);
        this->declare_parameter("can_publish_rate_hz", 50);
        this->declare_parameter("can_odom_topic", "Can_Odom");
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_frame", "base_link");

        // 串口参数
        this->declare_parameter("serial_port", "/dev/ttyUSB0");
        this->declare_parameter("baudrate", 115200);

        // 位姿发送参数
        this->declare_parameter("cmd_vel_topic", "cmd_vel");
        this->declare_parameter("merge_odom_topic", "merge_odom");
        this->declare_parameter("pose_send_rate_hz", 50);

        // 初始化CAN里程计
        rc26_merge_odom::CanOdom::Config can_config;
        can_config.can_interface = this->get_parameter("can_interface").as_string();
        can_config.wheel_radius = this->get_parameter("wheel_radius").as_double();
        can_config.wheel_base = this->get_parameter("wheel_base").as_double();
        can_config.track_width = this->get_parameter("track_width").as_double();
        can_config.gear_ratio = this->get_parameter("gear_ratio").as_double();
        can_config.publish_rate_hz = this->get_parameter("can_publish_rate_hz").as_int();
        can_config.odom_topic = this->get_parameter("can_odom_topic").as_string();
        can_config.odom_frame = this->get_parameter("odom_frame").as_string();
        can_config.base_frame = this->get_parameter("base_frame").as_string();

        can_odom_ = std::make_unique<rc26_merge_odom::CanOdom>(*this, can_config);

        // 初始化串口
        std::string serial_port = this->get_parameter("serial_port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();

        serial_driver_ = std::make_shared<rc26_decision::SerialDriver>();
        if (!serial_driver_->open(serial_port, baudrate))
        {
            RCLCPP_WARN(this->get_logger(), "串口打开失败: %s，位姿下传功能禁用", serial_port.c_str());
        }
        else
        {
            // 初始化位姿发送
            rc26_merge_odom::PoseSender::Config pose_config;
            pose_config.cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
            pose_config.odom_topic = this->get_parameter("merge_odom_topic").as_string();
            pose_config.send_rate_hz = this->get_parameter("pose_send_rate_hz").as_int();

            pose_sender_ = std::make_unique<rc26_merge_odom::PoseSender>(*this, serial_driver_, pose_config);
        }

        RCLCPP_INFO(this->get_logger(), "融合里程计节点启动");
    }

private:
    std::unique_ptr<rc26_merge_odom::CanOdom> can_odom_;
    std::shared_ptr<rc26_decision::SerialDriver> serial_driver_;
    std::unique_ptr<rc26_merge_odom::PoseSender> pose_sender_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MergeOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
