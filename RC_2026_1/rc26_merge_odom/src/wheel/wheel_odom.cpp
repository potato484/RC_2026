// RC2026 串口轮式里程计实现
#include "rc26_merge_odom/wheel/wheel_odom.hpp"

#include <cmath>
#include <cstring>

#include <tf2/LinearMath/Quaternion.h>

#include "rc26_serial/protocol.hpp"
#include "rc26_serial/serial_driver.hpp"

namespace rc26_merge_odom {

WheelOdom::WheelOdom(rclcpp::Node& node,
                     std::shared_ptr<rc26_decision::SerialDriver> serial,
                     Config config)
    : node_(node), config_(std::move(config)), serial_(std::move(serial)) {
    odom_pub_ = node_.create_publisher<nav_msgs::msg::Odometry>(config_.odom_topic, 10);

    last_publish_time_ = std::chrono::steady_clock::now();
    last_data_time_ = last_publish_time_;

    if (serial_) {
        serial_->setReceiveCallback([this](uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) {
            (void)seq;
            if (cmd != static_cast<uint8_t>(rc26_decision::FeedbackID::ODOM_DATA)) {
                return;
            }
            handleOdomData(payload);
        });
    } else {
        RCLCPP_WARN(node_.get_logger(), "WheelOdom: 串口未初始化，ODOM_DATA 将不可用");
    }

    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / static_cast<double>(config_.publish_rate_hz)));
    publish_timer_ = node_.create_wall_timer(period, std::bind(&WheelOdom::publishOdometry, this));

    RCLCPP_INFO(node_.get_logger(), "WheelOdom 启动: topic=%s, rate=%d Hz",
                config_.odom_topic.c_str(), config_.publish_rate_hz);
}

WheelOdom::~WheelOdom() {
    if (publish_timer_) {
        publish_timer_->cancel();
    }
    if (serial_) {
        serial_->setReceiveCallback({});
    }
}

void WheelOdom::handleOdomData(const std::vector<uint8_t>& payload) {
    if (payload.size() < 16) {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
                             "ODOM_DATA payload 大小不足: %zu < 16", payload.size());
        return;
    }

    float v_fl = 0.0f;
    float v_rl = 0.0f;
    float v_rr = 0.0f;
    float v_fr = 0.0f;

    std::memcpy(&v_fl, &payload[0], sizeof(float));
    std::memcpy(&v_rl, &payload[4], sizeof(float));
    std::memcpy(&v_rr, &payload[8], sizeof(float));
    std::memcpy(&v_fr, &payload[12], sizeof(float));

    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        v_fl_ = static_cast<double>(v_fl);
        v_rl_ = static_cast<double>(v_rl);
        v_rr_ = static_cast<double>(v_rr);
        v_fr_ = static_cast<double>(v_fr);
        last_data_time_ = std::chrono::steady_clock::now();
        data_received_ = true;
    }
}

void WheelOdom::wheelSpeedsToBodyVelocity(double v_fl, double v_rl, double v_rr, double v_fr,
                                          double& vx, double& vy, double& omega) const {
    double l_plus_w = (config_.wheel_base + config_.track_width) / 2.0;

    vx = (v_fl + v_fr + v_rl + v_rr) / 4.0;
    vy = (-v_fl + v_fr + v_rl - v_rr) / 4.0;
    omega = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * l_plus_w);
}

void WheelOdom::publishOdometry() {
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_publish_time_).count();
    last_publish_time_ = now;

    if (dt <= 0.0 || dt > 1.0) {
        return;
    }

    double v_fl = 0.0;
    double v_rl = 0.0;
    double v_rr = 0.0;
    double v_fr = 0.0;
    bool data_valid = false;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        auto timeout_duration = std::chrono::duration<double, std::milli>(config_.data_timeout_ms);
        if (data_received_ && (now - last_data_time_ <= timeout_duration)) {
            v_fl = v_fl_;
            v_rl = v_rl_;
            v_rr = v_rr_;
            v_fr = v_fr_;
            data_valid = true;
        }
    }

    if (!data_valid) {
        return;
    }

    double vx = 0.0;
    double vy = 0.0;
    double omega = 0.0;
    wheelSpeedsToBodyVelocity(v_fl, v_rl, v_rr, v_fr, vx, vy, omega);

    {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        vx_ = vx;
        vy_ = vy;
        omega_ = omega;

        double half_dt = dt / 2.0;
        double mid_yaw = yaw_ + omega * half_dt;

        double cos_yaw = std::cos(mid_yaw);
        double sin_yaw = std::sin(mid_yaw);

        x_ += (vx * cos_yaw - vy * sin_yaw) * dt;
        y_ += (vx * sin_yaw + vy * cos_yaw) * dt;
        yaw_ += omega * dt;

        while (yaw_ > M_PI) {
            yaw_ -= 2.0 * M_PI;
        }
        while (yaw_ < -M_PI) {
            yaw_ += 2.0 * M_PI;
        }
    }

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = node_.now();
    odom_msg.header.frame_id = config_.odom_frame;
    odom_msg.child_frame_id = config_.base_frame;

    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_);
        q.normalize();
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
    }

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = omega;

    odom_msg.pose.covariance[0] = 0.01;
    odom_msg.pose.covariance[7] = 0.01;
    odom_msg.pose.covariance[35] = 0.03;

    odom_msg.twist.covariance[0] = 0.01;
    odom_msg.twist.covariance[7] = 0.01;
    odom_msg.twist.covariance[35] = 0.03;

    odom_pub_->publish(odom_msg);
}

void WheelOdom::getPose(double& x, double& y, double& yaw) const {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    x = x_;
    y = y_;
    yaw = yaw_;
}

void WheelOdom::getVelocity(double& vx, double& vy, double& omega) const {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    vx = vx_;
    vy = vy_;
    omega = omega_;
}

void WheelOdom::reset() {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
}

}  // namespace rc26_merge_odom
