// RC2026 轮式里程计接收模块
// 从串口解析 ODOM_DATA(vx, vy, omega, yaw)，发布到 ROS2 话题 /wheel_odom
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rc26_decision
{

class WheelOdomReceiver
{
public:
    struct Config
    {
        std::string topic = "wheel_odom";
        std::string odom_frame = "odom";
        std::string base_frame = "base_link";
    };

    WheelOdomReceiver(rclcpp::Node& node, Config config);

    void handleFrame(uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload);

private:
    rclcpp::Node& node_;
    Config config_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

}  // namespace rc26_decision
