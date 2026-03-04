// RC2026 速度发送模块实现
#include "rc26_merge_odom/pose/pose_sender.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>

#include "rc26_serial/protocol.hpp"

namespace rc26_merge_odom {

namespace {
constexpr double kMaxValidDtSec = 1.0;
}

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

    imu_sub_ = node_.create_subscription<sensor_msgs::msg::Imu>(
        config_.imu_topic, 20, std::bind(&PoseSender::imuCallback, this, std::placeholders::_1));
    if (!config_.terrain_speed_limit_topic.empty()) {
        terrain_speed_limit_sub_ = node_.create_subscription<std_msgs::msg::Float32>(
            config_.terrain_speed_limit_topic, 10,
            std::bind(&PoseSender::terrainSpeedLimitCallback, this, std::placeholders::_1));
    }

    feedback_protected_pub_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>(
        "pose_sender/feedback_protected", 10);
    target_protected_pub_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>("pose_sender/target_protected",
                                                                                       10);
    imu_spike_pub_ = node_.create_publisher<std_msgs::msg::Bool>("pose_sender/imu_spike_active", 10);

    auto feedback_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / static_cast<double>(config_.feedback_send_rate_hz)));
    feedback_timer_ = node_.create_wall_timer(feedback_period, std::bind(&PoseSender::feedbackTimerCallback, this));

    auto target_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / static_cast<double>(config_.target_send_rate_hz)));
    target_timer_ = node_.create_wall_timer(target_period, std::bind(&PoseSender::targetTimerCallback, this));

    RCLCPP_INFO(node_.get_logger(),
                "PoseSender 启动 (双串口模式)，cmd_vel: %s, odom: %s, imu: %s, 反馈: %d Hz, 目标: %d Hz",
                config_.cmd_vel_topic.c_str(), config_.odom_topic.c_str(), config_.imu_topic.c_str(),
                config_.feedback_send_rate_hz, config_.target_send_rate_hz);
    if (!config_.terrain_speed_limit_topic.empty()) {
        RCLCPP_INFO(node_.get_logger(),
                    "PoseSender terrain_speed_limit 已启用: topic=%s timeout=%dms",
                    config_.terrain_speed_limit_topic.c_str(), config_.terrain_speed_limit_timeout_ms);
    }
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

void PoseSender::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const double ax = static_cast<double>(msg->linear_acceleration.x);
    const double ay = static_cast<double>(msg->linear_acceleration.y);
    const double az = static_cast<double>(msg->linear_acceleration.z);
    const double gx = static_cast<double>(msg->angular_velocity.x);
    const double gy = static_cast<double>(msg->angular_velocity.y);
    const double gz = static_cast<double>(msg->angular_velocity.z);

    const double acc_norm = std::sqrt(ax * ax + ay * ay + az * az);
    const double gyro_norm = std::sqrt(gx * gx + gy * gy + gz * gz);

    {
        std::lock_guard<std::mutex> lock(imu_cache_mutex_);
        cached_imu_.ax = static_cast<float>(ax);
        cached_imu_.ay = static_cast<float>(ay);
        cached_imu_.gz = static_cast<float>(gz);
        cached_imu_.stamp = std::chrono::steady_clock::now();
        cached_imu_.valid = true;
    }

    const bool acc_spike = acc_norm > static_cast<double>(config_.spike_accel_threshold);
    const bool gyro_spike = gyro_norm > static_cast<double>(config_.spike_gyro_threshold);

    const auto now = std::chrono::steady_clock::now();
    bool enter = false;
    bool exit = false;
    {
        std::lock_guard<std::mutex> lock(imu_spike_mutex_);
        const bool active = imu_spike_active_.load(std::memory_order_relaxed);
        if (acc_spike || gyro_spike) {
            imu_spike_deadline_ = now + std::chrono::milliseconds(config_.spike_freeze_duration_ms);
            if (!active) {
                imu_spike_active_.store(true, std::memory_order_relaxed);
                enter = true;
            }
        } else if (active && now >= imu_spike_deadline_) {
            imu_spike_active_.store(false, std::memory_order_relaxed);
            exit = true;
        }
    }

    if (enter) {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 500,
                             "[PoseSender] spike enter: acc=%.1f gyro=%.1f", acc_norm, gyro_norm);
    }
    if (exit) {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 500,
                             "[PoseSender] spike exit: acc=%.1f gyro=%.1f", acc_norm, gyro_norm);
    }
}

void PoseSender::terrainSpeedLimitCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    if (!msg) {
        return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    terrain_speed_limit_mps_ = msg->data;
    terrain_speed_limit_time_ = std::chrono::steady_clock::now();
    terrain_speed_limit_received_ = true;
}

PoseSender::Velocity PoseSender::protectVelocity(Velocity raw, Velocity& prev_vel, double dt) {
    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(imu_spike_mutex_);
        if (imu_spike_active_.load(std::memory_order_relaxed) && now >= imu_spike_deadline_) {
            imu_spike_active_.store(false, std::memory_order_relaxed);
        }
    }

    if (imu_spike_active_.load(std::memory_order_relaxed)) {
        const double tau = std::max(1e-6, static_cast<double>(config_.spike_decay_tau_s));
        const double decay = (dt > 0.0) ? std::exp(-dt / tau) : 1.0;

        Velocity output;
        output.vx = static_cast<float>(prev_vel.vx * decay);
        output.vy = static_cast<float>(prev_vel.vy * decay);
        output.wz = static_cast<float>(prev_vel.wz * decay);
        prev_vel = output;
        return output;
    }

    float effective_v_max = std::fabs(config_.v_max_mps);
    if (!config_.terrain_speed_limit_topic.empty()) {
        float terrain_limit = NAN;
        bool terrain_limit_valid = false;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (terrain_speed_limit_received_) {
                const auto timeout_ms = std::max(0, config_.terrain_speed_limit_timeout_ms);
                const auto age = now - terrain_speed_limit_time_;
                if (age <= std::chrono::milliseconds(timeout_ms)) {
                    terrain_limit = terrain_speed_limit_mps_;
                    terrain_limit_valid = true;
                }
            }
        }
        if (terrain_limit_valid && std::isfinite(terrain_limit)) {
            terrain_limit = std::max(0.0f, terrain_limit);
            effective_v_max = std::min(effective_v_max, terrain_limit);
        }
    }

    const float w_limit = std::fabs(config_.w_max_rps);
    const float a_limit = std::fabs(config_.a_max_mps2);
    const float alpha_limit = std::fabs(config_.alpha_max_rps2);

    Velocity output;
    output.vx = std::clamp(raw.vx, -effective_v_max, effective_v_max);
    output.vy = std::clamp(raw.vy, -effective_v_max, effective_v_max);
    output.wz = std::clamp(raw.wz, -w_limit, w_limit);

    if (dt > 0.0 && dt <= kMaxValidDtSec) {
        const float dv_limit = static_cast<float>(a_limit * static_cast<float>(dt));
        const float dw_limit = static_cast<float>(alpha_limit * static_cast<float>(dt));
        output.vx = std::clamp(output.vx, prev_vel.vx - dv_limit, prev_vel.vx + dv_limit);
        output.vy = std::clamp(output.vy, prev_vel.vy - dv_limit, prev_vel.vy + dv_limit);
        output.wz = std::clamp(output.wz, prev_vel.wz - dw_limit, prev_vel.wz + dw_limit);
    }

    prev_vel = output;
    return output;
}

void PoseSender::feedbackTimerCallback() {
    Velocity feedback;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        feedback = feedback_vel_;
    }

    if (config_.latency_comp_enable) {
        ImuCache imu_cache;
        {
            std::lock_guard<std::mutex> lock(imu_cache_mutex_);
            imu_cache = cached_imu_;
        }
        if (imu_cache.valid) {
            feedback.vx += imu_cache.ax * config_.latency_comp_s;
            feedback.vy += imu_cache.ay * config_.latency_comp_s;
        }
    }

    const auto now = std::chrono::steady_clock::now();
    double dt = 0.0;
    Velocity prev_feedback;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        prev_feedback = prev_feedback_vel_;
        if (prev_feedback_output_valid_) {
            dt = std::chrono::duration<double>(now - prev_feedback_output_time_).count();
        }
        prev_feedback_output_time_ = now;
        prev_feedback_output_valid_ = true;
    }

    Velocity protected_vel = protectVelocity(feedback, prev_feedback, dt);
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        prev_feedback_vel_ = prev_feedback;
    }

    if (feedback_serial_ && feedback_serial_->isOpen()) {
        feedback_serial_->sendPose(rc26_decision::CommandID::POSE_FEEDBACK, protected_vel.vx, protected_vel.vy,
                                   protected_vel.wz);
    }

    geometry_msgs::msg::TwistStamped feedback_msg;
    feedback_msg.header.stamp = node_.now();
    feedback_msg.twist.linear.x = protected_vel.vx;
    feedback_msg.twist.linear.y = protected_vel.vy;
    feedback_msg.twist.angular.z = protected_vel.wz;
    feedback_protected_pub_->publish(feedback_msg);

    std_msgs::msg::Bool spike_msg;
    spike_msg.data = imu_spike_active_.load(std::memory_order_relaxed);
    imu_spike_pub_->publish(spike_msg);
}

void PoseSender::targetTimerCallback() {
    Velocity target;
    bool send_target = false;
    bool send_zero = false;
    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        target = target_vel_;
        if (cmd_vel_received_) {
            const auto timeout_duration = std::chrono::milliseconds(config_.cmd_vel_timeout_ms);
            if (now - last_cmd_vel_time_ <= timeout_duration) {
                send_target = true;
                timeout_zero_sent_ = false;
            } else if (!timeout_zero_sent_) {
                send_zero = true;
                timeout_zero_sent_ = true;
                target = Velocity{};
            }
        }
    }

    if (!send_target && !send_zero) {
        return;
    }

    double dt = 0.0;
    Velocity prev_target;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        prev_target = prev_target_vel_;
        if (prev_target_output_valid_) {
            dt = std::chrono::duration<double>(now - prev_target_output_time_).count();
        }
        prev_target_output_time_ = now;
        prev_target_output_valid_ = true;
    }

    Velocity protected_vel = protectVelocity(target, prev_target, dt);
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        prev_target_vel_ = prev_target;
    }

    if (target_serial_ && target_serial_->isOpen()) {
        target_serial_->sendPose(rc26_decision::CommandID::POSE_TARGET, protected_vel.vx, protected_vel.vy,
                                 protected_vel.wz);
    }

    geometry_msgs::msg::TwistStamped target_msg;
    target_msg.header.stamp = node_.now();
    target_msg.twist.linear.x = protected_vel.vx;
    target_msg.twist.linear.y = protected_vel.vy;
    target_msg.twist.angular.z = protected_vel.wz;
    target_protected_pub_->publish(target_msg);

    std_msgs::msg::Bool spike_msg;
    spike_msg.data = imu_spike_active_.load(std::memory_order_relaxed);
    imu_spike_pub_->publish(spike_msg);
}

}  // namespace rc26_merge_odom
