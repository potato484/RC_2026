// RC2026 速度发送模块实现
#include "rc26_merge_odom/pose/pose_sender.hpp"

#include <chrono>
#include <stdexcept>

#include "rc26_serial/protocol.hpp"

namespace rc26_merge_odom {

PoseSender::PoseSender(rclcpp::Node& node, std::shared_ptr<rc26_decision::SerialDriver> feedback_serial,
                       std::shared_ptr<rc26_decision::SerialDriver> target_serial, Config config)
    : node_(node), feedback_serial_(std::move(feedback_serial)), target_serial_(std::move(target_serial)),
      config_(std::move(config)) {
    if (config_.feedback_send_rate_hz <= 0) {
        throw std::runtime_error("feedback_send_rate_hz 必须 > 0");
    }
    if (config_.target_send_rate_hz <= 0) {
        throw std::runtime_error("target_send_rate_hz 必须 > 0");
    }

    // Bridge ROS velocity commands to MCU speed closed-loop:
    // - Subscribe `cmd_vel_topic` (Twist)
    // - Send POSE_TARGET(0x22) over UART as (vx, vy, wz) floats
    cmd_vel_sub_ = node_.create_subscription<geometry_msgs::msg::Twist>(
        config_.cmd_vel_topic, 10, std::bind(&PoseSender::cmdVelCallback, this, std::placeholders::_1));

    odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
        config_.odom_topic, 10, std::bind(&PoseSender::odomCallback, this, std::placeholders::_1));

    auto feedback_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / static_cast<double>(config_.feedback_send_rate_hz)));
    feedback_timer_ = node_.create_wall_timer(feedback_period, std::bind(&PoseSender::feedbackTimerCallback, this));

    auto target_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / static_cast<double>(config_.target_send_rate_hz)));
    target_timer_ = node_.create_wall_timer(target_period, std::bind(&PoseSender::targetTimerCallback, this));

    RCLCPP_INFO(node_.get_logger(),
                "PoseSender 启动 (双串口模式)，cmd_vel: %s, odom: %s, 反馈: %d Hz, 目标: %d Hz",
                config_.cmd_vel_topic.c_str(), config_.odom_topic.c_str(), config_.feedback_send_rate_hz,
                config_.target_send_rate_hz);
}

void PoseSender::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    target_vel_.vx = static_cast<float>(msg->linear.x);
    target_vel_.vy = static_cast<float>(msg->linear.y);
    target_vel_.wz = static_cast<float>(msg->angular.z);
    last_cmd_vel_time_ = std::chrono::steady_clock::now();
    cmd_vel_received_ = true;
    timeout_zero_sent_ = false;
}

void PoseSender::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    feedback_vel_.vx = static_cast<float>(msg->twist.twist.linear.x);
    feedback_vel_.vy = static_cast<float>(msg->twist.twist.linear.y);
    feedback_vel_.wz = static_cast<float>(msg->twist.twist.angular.z);
}

void PoseSender::feedbackTimerCallback() {
    Velocity feedback;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        feedback = feedback_vel_;
    }

    if (feedback_serial_ && feedback_serial_->isOpen()) {
        feedback_serial_->sendPose(rc26_decision::CommandID::POSE_FEEDBACK, feedback.vx, feedback.vy, feedback.wz);
    }
}

void PoseSender::targetTimerCallback() {
    Velocity target;
    bool send_target = false;
    bool send_zero = false;
    auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        target = target_vel_;
        if (cmd_vel_received_) {
            auto timeout_duration = std::chrono::milliseconds(config_.cmd_vel_timeout_ms);
            if (now - last_cmd_vel_time_ <= timeout_duration) {
                send_target = true;
                timeout_zero_sent_ = false;
            } else if (!timeout_zero_sent_) {
                send_zero = true;
                timeout_zero_sent_ = true;
            }
        }
    }

    if (target_serial_ && target_serial_->isOpen()) {
        if (send_target) {
            target_serial_->sendPose(rc26_decision::CommandID::POSE_TARGET, target.vx, target.vy, target.wz);
        } else if (send_zero) {
            target_serial_->sendPose(rc26_decision::CommandID::POSE_TARGET, 0.0f, 0.0f, 0.0f);
        }
    }
}

}  // namespace rc26_merge_odom
