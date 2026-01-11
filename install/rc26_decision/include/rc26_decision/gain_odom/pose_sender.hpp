/*
 * @Author: potato potato@potato.com
 * @Date: 2025-12-30 17:40:14
 * @LastEditors: potato potato@potato.com
 * @LastEditTime: 2026-01-02 21:39:02
 * @FilePath: /RC_2026/RC_2026_1/rc26_decision/include/rc26_decision/gain_odom/pose_sender.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
// RC2026 位姿发送模块
// 订阅 ROS2 位姿话题，定时通过串口发送到下位机
#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "rc26_serial/serial_driver.hpp"

namespace rc26_decision
{

class PoseSender
{
public:
    struct Config
    {
        std::string cmd_vel_topic = "cmd_vel";
        std::string odom_topic = "odometry";
        int send_rate_hz = 50;
    };

    PoseSender(rclcpp::Node& node, std::shared_ptr<SerialDriver> serial_driver, Config config);

private:
    rclcpp::Node& node_;
    std::shared_ptr<SerialDriver> serial_driver_;
    Config config_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::TimerBase::SharedPtr send_timer_;

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

}  // namespace rc26_decision
