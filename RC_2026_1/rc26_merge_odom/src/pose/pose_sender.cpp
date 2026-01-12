// RC2026 速度发送模块实现
#include "rc26_merge_odom/pose/pose_sender.hpp"

#include <chrono>
#include <stdexcept>

#include "rc26_serial/protocol.hpp"

namespace rc26_merge_odom {

PoseSender::PoseSender(rclcpp::Node& node,
                       std::shared_ptr<rc26_decision::SerialDriver> feedback_serial,
                       std::shared_ptr<rc26_decision::SerialDriver> target_serial,
                       Config config)
    : node_(node),
      feedback_serial_(std::move(feedback_serial)),
      target_serial_(std::move(target_serial)),
      config_(std::move(config)) {
    if (config_.send_rate_hz <= 0) {
        throw std::runtime_error("send_rate_hz 必须 > 0");
    }

    cmd_vel_sub_ = node_.create_subscription<geometry_msgs::msg::Twist>(
        config_.cmd_vel_topic, 10, std::bind(&PoseSender::cmdVelCallback, this, std::placeholders::_1));

    odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
        config_.odom_topic, 10, std::bind(&PoseSender::odomCallback, this, std::placeholders::_1));

    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / static_cast<double>(config_.send_rate_hz)));
    send_timer_ = node_.create_wall_timer(period, std::bind(&PoseSender::sendTimerCallback, this));

    RCLCPP_INFO(node_.get_logger(), "PoseSender 启动 (双串口模式)，cmd_vel: %s, odom: %s, 频率: %d Hz",
                config_.cmd_vel_topic.c_str(), config_.odom_topic.c_str(), config_.send_rate_hz);
}

void PoseSender::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    target_vel_.vx = static_cast<float>(msg->linear.x);
    target_vel_.vy = static_cast<float>(msg->linear.y);
    target_vel_.wz = static_cast<float>(msg->angular.z);
}

void PoseSender::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    feedback_vel_.vx = static_cast<float>(msg->twist.twist.linear.x);
    feedback_vel_.vy = static_cast<float>(msg->twist.twist.linear.y);
    feedback_vel_.wz = static_cast<float>(msg->twist.twist.angular.z);
}

void PoseSender::sendTimerCallback() {
    Velocity feedback, target;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        feedback = feedback_vel_;
        target = target_vel_;
    }

    if (feedback_serial_ && feedback_serial_->isOpen()) {
        feedback_serial_->sendPose(rc26_decision::CommandID::POSE_FEEDBACK,
                                   feedback.vx, feedback.vy, feedback.wz);
    }

    if (target_serial_ && target_serial_->isOpen()) {
        target_serial_->sendPose(rc26_decision::CommandID::POSE_TARGET,
                                 target.vx, target.vy, target.wz);
    }
}

}  // namespace rc26_merge_odom
