// RC2026 位姿发送模块实现
#include "rc26_merge_odom/pose_sender.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <stdexcept>

namespace rc26_merge_odom
{

PoseSender::PoseSender(
    rclcpp::Node& node, std::shared_ptr<rc26_decision::SerialDriver> serial_driver, Config config)
    : node_(node), serial_driver_(std::move(serial_driver)), config_(std::move(config))
{
    if (config_.send_rate_hz <= 0)
    {
        throw std::runtime_error("pose_send_rate_hz 必须 > 0");
    }

    cmd_vel_sub_ = node_.create_subscription<geometry_msgs::msg::Twist>(
        config_.cmd_vel_topic, 10,
        std::bind(&PoseSender::cmdVelCallback, this, std::placeholders::_1));

    odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
        config_.odom_topic, 10,
        std::bind(&PoseSender::odomCallback, this, std::placeholders::_1));

    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / static_cast<double>(config_.send_rate_hz)));
    send_timer_ = node_.create_wall_timer(
        period, std::bind(&PoseSender::sendTimerCallback, this));

    RCLCPP_INFO(
        node_.get_logger(),
        "PoseSender 启动，cmd_vel: %s, odom: %s, 发送频率: %d Hz",
        config_.cmd_vel_topic.c_str(),
        config_.odom_topic.c_str(),
        config_.send_rate_hz);
}

void PoseSender::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    vx_ = static_cast<float>(msg->linear.x);
    vy_ = static_cast<float>(msg->linear.y);
    wx_ = static_cast<float>(msg->angular.x);
    wy_ = static_cast<float>(msg->angular.y);
    wz_ = static_cast<float>(msg->angular.z);
}

void PoseSender::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    double r, p, y;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(r, p, y);
    roll_ = static_cast<float>(r);
    pitch_ = static_cast<float>(p);
    yaw_ = static_cast<float>(y);
}

void PoseSender::sendTimerCallback()
{
    if (serial_driver_ && serial_driver_->isOpen())
    {
        serial_driver_->sendPose(vx_, vy_, wx_, wy_, wz_, roll_, pitch_, yaw_);
    }
}

}  // namespace rc26_merge_odom
