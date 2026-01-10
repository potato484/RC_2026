
#include "rc26_decision/gain_odom/wheel_odom_receiver.hpp"

#include <cstring>

#include <tf2/LinearMath/Quaternion.h>

#include "rc26_serial/protocol.hpp"

namespace rc26_decision
{

WheelOdomReceiver::WheelOdomReceiver(rclcpp::Node& node, Config config)
    : node_(node), config_(std::move(config))
{
    odom_pub_ = node_.create_publisher<nav_msgs::msg::Odometry>(config_.topic, 10);

    RCLCPP_INFO(
        node_.get_logger(),
        "WheelOdomReceiver 启动，发布话题: %s",
        config_.topic.c_str());
}

void WheelOdomReceiver::handleFrame(
    uint8_t /*seq*/, uint8_t cmd, const std::vector<uint8_t>& payload)
{
    if (cmd != static_cast<uint8_t>(FeedbackID::ODOM_DATA))
    {
        return;
    }

    constexpr size_t kExpectedSize = sizeof(float) * 4;
    if (payload.size() != kExpectedSize)
    {
        RCLCPP_WARN(
            node_.get_logger(),
            "ODOM_DATA 载荷长度错误: %zu (期望 %zu)，丢弃",
            payload.size(),
            kExpectedSize);
        return;
    }

    float vx = 0.0f;
    float vy = 0.0f;
    float omega = 0.0f;
    float yaw = 0.0f;

    std::memcpy(&vx, &payload[0], sizeof(float));
    std::memcpy(&vy, &payload[4], sizeof(float));
    std::memcpy(&omega, &payload[8], sizeof(float));
    std::memcpy(&yaw, &payload[12], sizeof(float));

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = node_.now();
    msg.header.frame_id = config_.odom_frame;
    msg.child_frame_id = config_.base_frame;

    msg.pose.pose.position.x = 0.0;
    msg.pose.pose.position.y = 0.0;
    msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, static_cast<double>(yaw));
    q.normalize();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    msg.twist.twist.linear.x = static_cast<double>(vx);
    msg.twist.twist.linear.y = static_cast<double>(vy);
    msg.twist.twist.linear.z = 0.0;
    msg.twist.twist.angular.x = 0.0;
    msg.twist.twist.angular.y = 0.0;
    msg.twist.twist.angular.z = static_cast<double>(omega);

    odom_pub_->publish(msg);
}

}  // namespace rc26_decision
