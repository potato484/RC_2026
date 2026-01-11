// RC2026 位姿发送模块
// 订阅融合里程计话题，定时通过串口发送到MCU
#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rc26_serial/serial_driver.hpp"

namespace rc26_merge_odom {

class PoseSender {
public:
    struct Config {
        std::string cmd_vel_topic = "cmd_vel";
        std::string odom_topic = "merge_odom";
        int send_rate_hz = 50;
    };

    PoseSender(rclcpp::Node& node, std::shared_ptr<rc26_decision::SerialDriver> serial_driver, Config config);

private:
    rclcpp::Node& node_;
    std::shared_ptr<rc26_decision::SerialDriver> serial_driver_;
    Config config_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::TimerBase::SharedPtr send_timer_;

    mutable std::mutex data_mutex_;
    float vx_ = 0.0f;
    float vy_ = 0.0f;
    float wx_ = 0.0f;
    float wy_ = 0.0f;
    float wz_ = 0.0f;
    float roll_ = 0.0f;
    float pitch_ = 0.0f;
    float yaw_ = 0.0f;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void sendTimerCallback();
};

}  // namespace rc26_merge_odom
