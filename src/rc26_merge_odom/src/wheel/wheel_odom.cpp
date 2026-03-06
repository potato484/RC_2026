// RC2026 串口轮式里程计实现
#include "rc26_merge_odom/wheel/wheel_odom.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>

#include <tf2/LinearMath/Quaternion.h>

#include "rc26_serial/protocol.hpp"
#include "rc26_serial/serial_driver.hpp"

namespace rc26_merge_odom {

namespace {
constexpr double kEpsilon = 1e-6;
constexpr double kMinCovariance = 1e-6;
}

WheelOdom::WheelOdom(rclcpp::Node& node, std::shared_ptr<rc26_decision::SerialDriver> serial, Config config)
    : node_(node), config_(std::move(config)), serial_(std::move(serial)) {
    if (!(std::fabs(config_.wheel_base) > kEpsilon)) {
        RCLCPP_WARN(node_.get_logger(), "wheel wheel_base=%.6f invalid, fallback to %.6f", config_.wheel_base,
                    Config{}.wheel_base);
        config_.wheel_base = Config{}.wheel_base;
    }
    if (!(std::fabs(config_.track_width) > kEpsilon)) {
        RCLCPP_WARN(node_.get_logger(), "wheel track_width=%.6f invalid, fallback to %.6f", config_.track_width,
                    Config{}.track_width);
        config_.track_width = Config{}.track_width;
    }
    if (config_.publish_rate_hz <= 0) {
        RCLCPP_WARN(node_.get_logger(), "wheel publish_rate_hz=%d invalid, fallback to 1", config_.publish_rate_hz);
        config_.publish_rate_hz = 1;
    }
    config_.data_timeout_ms = std::max(0.0, config_.data_timeout_ms);
    config_.slip_threshold = std::max(0.0, config_.slip_threshold);
    config_.slip_k_acc = std::max(0.0, config_.slip_k_acc);
    config_.cov_nominal_v = std::max(config_.cov_nominal_v, kMinCovariance);
    config_.cov_nominal_wz = std::max(config_.cov_nominal_wz, kMinCovariance);
    config_.cov_slip_v = std::max(config_.cov_slip_v, config_.cov_nominal_v);
    config_.cov_slip_wz = std::max(config_.cov_slip_wz, config_.cov_nominal_wz);
    config_.recovery_tau_s = std::max(config_.recovery_tau_s, kEpsilon);

    odom_pub_ = node_.create_publisher<nav_msgs::msg::Odometry>(config_.odom_topic, 10);
    slip_score_pub_ = node_.create_publisher<std_msgs::msg::Float32>("wheel_odom/slip_score", 10);
    cov_state_pub_ = node_.create_publisher<std_msgs::msg::Float32>("wheel_odom/cov_state", 10);

    imu_sub_ = node_.create_subscription<sensor_msgs::msg::Imu>(
        config_.imu_topic, 20, std::bind(&WheelOdom::imuCallback, this, std::placeholders::_1));

    prev_pub_time_ = std::chrono::steady_clock::now();
    last_data_time_ = prev_pub_time_;
    last_slip_time_ = prev_pub_time_;

    if (!serial_ || !serial_->isOpen()) {
        RCLCPP_ERROR(node_.get_logger(), "WheelOdom: 串口未就绪，ODOM_DATA 无法接收");
        return;
    }

    serial_->setReceiveCallback([this](uint8_t seq, uint8_t cmd, const std::vector<uint8_t>& payload) {
        (void)seq;
        if (cmd != static_cast<uint8_t>(rc26_decision::FeedbackID::ODOM_DATA)) {
            return;
        }
        handleOdomData(payload);
    });

    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / static_cast<double>(config_.publish_rate_hz)));
    publish_timer_ = node_.create_wall_timer(period, std::bind(&WheelOdom::publishOdometry, this));
    ready_.store(true, std::memory_order_release);

    RCLCPP_INFO(node_.get_logger(), "WheelOdom 启动: topic=%s, imu=%s, rate=%d Hz, slip=%s", config_.odom_topic.c_str(),
                config_.imu_topic.c_str(), config_.publish_rate_hz, config_.slip_enable ? "on" : "off");
}

WheelOdom::~WheelOdom() {
    ready_.store(false, std::memory_order_release);
    if (publish_timer_) {
        publish_timer_->cancel();
    }
    if (serial_) {
        serial_->setReceiveCallback({});
    }
}

void WheelOdom::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_snapshot_.gz = static_cast<double>(msg->angular_velocity.z);
    imu_snapshot_.ax = static_cast<double>(msg->linear_acceleration.x);
    imu_snapshot_.ay = static_cast<double>(msg->linear_acceleration.y);
    imu_snapshot_.stamp = std::chrono::steady_clock::now();
    imu_snapshot_.valid = true;
}

void WheelOdom::handleOdomData(const std::vector<uint8_t>& payload) {
    if (payload.size() < 16) {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000, "ODOM_DATA payload 大小不足: %zu < 16",
                             payload.size());
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

void WheelOdom::wheelSpeedsToBodyVelocity(double v_fl, double v_rl, double v_rr, double v_fr, double& vx, double& vy,
                                          double& omega) const {
    double l_plus_w = (config_.wheel_base + config_.track_width) / 2.0;

    vx = (v_fl + v_fr + v_rl + v_rr) / 4.0;
    vy = (-v_fl + v_fr + v_rl - v_rr) / 4.0;
    omega = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * l_plus_w);
}

void WheelOdom::publishOdometry() {
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - prev_pub_time_).count();
    prev_pub_time_ = now;

    if (dt <= 0.0 || dt > 1.0) {
        return;
    }

    double v_fl = 0.0;
    double v_rl = 0.0;
    double v_rr = 0.0;
    double v_fr = 0.0;
    bool data_valid = false;
    const auto timeout_duration = std::chrono::duration<double, std::milli>(config_.data_timeout_ms);
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
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

    double wheel_acc_xy = 0.0;
    if (prev_vel_valid_) {
        const double ax_wheel = (vx - prev_vx_) / dt;
        const double ay_wheel = (vy - prev_vy_) / dt;
        wheel_acc_xy = std::hypot(ax_wheel, ay_wheel);
    }
    prev_vx_ = vx;
    prev_vy_ = vy;
    prev_vel_valid_ = true;

    ImuSnapshot imu_snapshot;
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_snapshot = imu_snapshot_;
    }
    const bool imu_fresh = imu_snapshot.valid && (now - imu_snapshot.stamp <= timeout_duration);

    double slip_score = 0.0;
    if (config_.slip_enable && imu_fresh) {
        const double imu_acc_xy = std::hypot(imu_snapshot.ax, imu_snapshot.ay);
        const double omega_diff = std::fabs(imu_snapshot.gz - omega);
        const double acc_mismatch =
            std::fabs(imu_acc_xy - wheel_acc_xy) / (std::fabs(imu_acc_xy) + kEpsilon);
        slip_score = omega_diff + config_.slip_k_acc * acc_mismatch;

        if (slip_score > config_.slip_threshold) {
            slip_detected_ = true;
            last_slip_time_ = now;
        }
    }

    double cov_v = config_.cov_nominal_v;
    double cov_wz = config_.cov_nominal_wz;
    if (config_.slip_enable && slip_detected_) {
        const double elapsed = std::chrono::duration<double>(now - last_slip_time_).count();
        const double tau = std::max(config_.recovery_tau_s, kEpsilon);
        const double scale = std::exp(-elapsed / tau);
        cov_v = config_.cov_nominal_v + (config_.cov_slip_v - config_.cov_nominal_v) * scale;
        cov_wz = config_.cov_nominal_wz + (config_.cov_slip_wz - config_.cov_nominal_wz) * scale;

        if (elapsed > 5.0 * tau && scale < 1e-3) {
            slip_detected_ = false;
        }
    }

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = node_.now();
    odom_msg.header.frame_id = config_.odom_frame;
    odom_msg.child_frame_id = config_.base_frame;

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

    odom_msg.pose.covariance[0] = cov_v;
    odom_msg.pose.covariance[7] = cov_v;
    odom_msg.pose.covariance[35] = cov_wz;

    odom_msg.twist.covariance[0] = cov_v;
    odom_msg.twist.covariance[7] = cov_v;
    odom_msg.twist.covariance[35] = cov_wz;

    odom_pub_->publish(odom_msg);

    std_msgs::msg::Float32 slip_msg;
    slip_msg.data = static_cast<float>(slip_score);
    slip_score_pub_->publish(slip_msg);

    std_msgs::msg::Float32 cov_msg;
    cov_msg.data = static_cast<float>(cov_v);
    cov_state_pub_->publish(cov_msg);
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
