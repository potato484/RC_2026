// RC2026 速度发送模块
// 双串口架构：反馈速度 + 目标速度，用于MCU速度闭环控制
#pragma once

#include <chrono>
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
        int feedback_send_rate_hz = 50;
        int target_send_rate_hz = 25;
        int cmd_vel_timeout_ms = 200;
    };

    PoseSender(rclcpp::Node& node, std::shared_ptr<rc26_decision::SerialDriver> feedback_serial,
               std::shared_ptr<rc26_decision::SerialDriver> target_serial, Config config);

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
    rclcpp::TimerBase::SharedPtr feedback_timer_;
    rclcpp::TimerBase::SharedPtr target_timer_;

    mutable std::mutex data_mutex_;
    Velocity target_vel_;
    Velocity feedback_vel_;
    std::chrono::steady_clock::time_point last_cmd_vel_time_;
    bool cmd_vel_received_ = false;
    bool timeout_zero_sent_ = false;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void feedbackTimerCallback();
    void targetTimerCallback();
};

}  // namespace rc26_merge_odom
