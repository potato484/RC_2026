// RC2026 速度发送模块
// 双串口架构：反馈速度 + 目标速度，用于MCU速度闭环控制
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

    PoseSender(rclcpp::Node& node,
               std::shared_ptr<rc26_decision::SerialDriver> feedback_serial,
               std::shared_ptr<rc26_decision::SerialDriver> target_serial,
               Config config);

private:
    struct Velocity {
        float vx = 0.0f;
        float vy = 0.0f;
        float wz = 0.0f;
    };

    rclcpp::Node& node_;
    std::shared_ptr<rc26_decision::SerialDriver> feedback_serial_;
    std::shared_ptr<rc26_decision::SerialDriver> target_serial_;
    Config config_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr send_timer_;

    mutable std::mutex data_mutex_;
    Velocity target_vel_;
    Velocity feedback_vel_;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void sendTimerCallback();
};

}  // namespace rc26_merge_odom
