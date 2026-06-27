// RC2026 CAN里程计实现
#include "rc26_merge_odom/can/can_odom.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>

#include "rc26_merge_odom/mecanum_kinematics.hpp"

namespace rc26_merge_odom {

namespace {
constexpr double kEpsilon = 1e-6;
constexpr double kMinCovariance = 1e-6;
}

CanOdom::CanOdom(rclcpp::Node& node, Config config) : node_(node), config_(std::move(config)) {
    if (config_.publish_rate_hz <= 0) {
        RCLCPP_WARN(node_.get_logger(), "can publish_rate_hz=%d invalid, fallback to 1", config_.publish_rate_hz);
        config_.publish_rate_hz = 1;
    }
    if (!(config_.wheel_radius > kEpsilon)) {
        RCLCPP_WARN(node_.get_logger(), "can wheel_radius=%.6f invalid, fallback to %.6f", config_.wheel_radius,
                    Config{}.wheel_radius);
        config_.wheel_radius = Config{}.wheel_radius;
    }
    if (!(config_.gear_ratio > kEpsilon)) {
        RCLCPP_WARN(node_.get_logger(), "can gear_ratio=%.6f invalid, fallback to %.6f", config_.gear_ratio,
                    Config{}.gear_ratio);
        config_.gear_ratio = Config{}.gear_ratio;
    }
    config_.data_timeout_ms = std::max(0.0, config_.data_timeout_ms);
    config_.slip_threshold = std::max(0.0, config_.slip_threshold);
    config_.slip_k_acc = std::max(0.0, config_.slip_k_acc);
    config_.cov_nominal_v = std::max(config_.cov_nominal_v, kMinCovariance);
    config_.cov_nominal_wz = std::max(config_.cov_nominal_wz, kMinCovariance);
    config_.cov_slip_v = std::max(config_.cov_slip_v, config_.cov_nominal_v);
    config_.cov_slip_wz = std::max(config_.cov_slip_wz, config_.cov_nominal_wz);
    config_.recovery_tau_s = std::max(config_.recovery_tau_s, kEpsilon);

    rpm_to_wheel_speed_factor_ = 2.0 * M_PI * config_.wheel_radius / (60.0 * config_.gear_ratio);

    odom_pub_ = node_.create_publisher<nav_msgs::msg::Odometry>(config_.odom_topic, 10);
    slip_score_pub_ = node_.create_publisher<std_msgs::msg::Float32>("can_odom/slip_score", 10);
    cov_state_pub_ = node_.create_publisher<std_msgs::msg::Float32>("can_odom/cov_state", 10);
    if (!config_.imu_topic.empty()) {
        imu_sub_ = node_.create_subscription<sensor_msgs::msg::Imu>(
            config_.imu_topic, 20, std::bind(&CanOdom::imuCallback, this, std::placeholders::_1));
    }

    last_update_time_ = std::chrono::steady_clock::now();
    last_slip_time_ = last_update_time_;

    if (!initCan()) {
        RCLCPP_ERROR(node_.get_logger(), "CAN 接口初始化失败: %s", config_.can_interface.c_str());
        closeCan();
        return;
    }

    try {
        running_ = true;
        can_thread_ = std::thread(&CanOdom::canThreadFunc, this);

        auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / static_cast<double>(config_.publish_rate_hz)));
        publish_timer_ = node_.create_wall_timer(period, std::bind(&CanOdom::publishOdometry, this));
    } catch (...) {
        running_ = false;
        if (can_thread_.joinable()) {
            can_thread_.join();
        }
        closeCan();
        throw;
    }

    ready_.store(true, std::memory_order_release);

    RCLCPP_INFO(node_.get_logger(), "CAN 里程计启动: interface=%s, odom_topic=%s, imu=%s, rate=%d Hz, slip=%s",
                config_.can_interface.c_str(), config_.odom_topic.c_str(),
                config_.imu_topic.empty() ? "disabled" : config_.imu_topic.c_str(), config_.publish_rate_hz,
                config_.slip_enable ? "on" : "off");
}

CanOdom::~CanOdom() {
    ready_.store(false, std::memory_order_release);
    if (publish_timer_) {
        publish_timer_->cancel();
    }
    running_ = false;
    if (can_thread_.joinable()) {
        can_thread_.join();
    }
    closeCan();
}

bool CanOdom::initCan() {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {
        RCLCPP_ERROR(node_.get_logger(), "创建 CAN 套接字失败: %s", strerror(errno));
        return false;
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, config_.can_interface.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(node_.get_logger(), "获取 CAN 接口索引失败: %s, %s", config_.can_interface.c_str(),
                     strerror(errno));
        close(can_socket_);
        can_socket_ = -1;
        return false;
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        RCLCPP_ERROR(node_.get_logger(), "绑定 CAN 套接字失败: %s", strerror(errno));
        close(can_socket_);
        can_socket_ = -1;
        return false;
    }

    std::array<can_filter, WHEEL_COUNT> filters{};
    for (size_t i = 0; i < WHEEL_COUNT; ++i) {
        filters[i].can_id = CAN_BASE_ID + static_cast<uint32_t>(i) + 1;
        filters[i].can_mask = CAN_SFF_MASK;
    }
    if (setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(), filters.size() * sizeof(can_filter)) <
        0) {
        RCLCPP_WARN(node_.get_logger(), "设置 CAN 过滤器失败: %s", strerror(errno));
    }

    // 设置接收超时
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 50000;
    if (setsockopt(can_socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        RCLCPP_WARN(node_.get_logger(), "设置 CAN 接收超时失败: %s", strerror(errno));
    }

    return true;
}

void CanOdom::closeCan() {
    if (can_socket_ >= 0) {
        close(can_socket_);
        can_socket_ = -1;
    }
}

void CanOdom::canThreadFunc() {
    RCLCPP_DEBUG(node_.get_logger(), "CAN 接收线程启动");

    struct can_frame frame;

    while (running_) {
        ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));
        if (nbytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                continue;
            }
            RCLCPP_WARN(node_.get_logger(), "CAN 读取错误: %s", strerror(errno));
            continue;
        }

        if (nbytes == sizeof(frame)) {
            parseCanFrame(frame.can_id, frame.data, frame.can_dlc);
        }
    }

    RCLCPP_DEBUG(node_.get_logger(), "CAN 接收线程退出");
}

void CanOdom::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_snapshot_.gz = static_cast<double>(msg->angular_velocity.z);
    imu_snapshot_.ax = static_cast<double>(msg->linear_acceleration.x);
    imu_snapshot_.ay = static_cast<double>(msg->linear_acceleration.y);
    imu_snapshot_.stamp = std::chrono::steady_clock::now();
    imu_snapshot_.valid = true;
}

void CanOdom::parseCanFrame(uint32_t can_id, const uint8_t* data, uint8_t len) {
    if (len < 8) {
        return;
    }

    uint16_t angle_raw = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    int16_t rpm = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[3]);
    int16_t current = static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | data[5]);
    uint8_t temperature = data[6];

    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        uint32_t motor_id = can_id - CAN_BASE_ID;
        if (motor_id < 1 || motor_id > WHEEL_COUNT) {
            return;
        }
        MotorFeedback* target = &motor_feedback_[static_cast<size_t>(motor_id - 1)];
        target->angle_raw = angle_raw;
        target->rpm = rpm;
        target->current = current;
        target->temperature = temperature;
        target->last_update = std::chrono::steady_clock::now();
    }
}

void CanOdom::publishOdometry() {
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_update_time_).count();
    last_update_time_ = now;

    if (dt <= 0.0 || dt > 1.0) {
        return;
    }

    const auto timeout_duration = std::chrono::duration<double, std::milli>(config_.data_timeout_ms);
    bool data_valid = true;
    double vx = 0.0;
    double vy = 0.0;
    double omega = 0.0;
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        std::array<int16_t, WHEEL_COUNT> rpm_values{};
        for (int i = 0; i < WHEEL_COUNT; ++i) {
            if (now - motor_feedback_[i].last_update > timeout_duration) {
                data_valid = false;
                break;
            }
            rpm_values[static_cast<size_t>(i)] = motor_feedback_[static_cast<size_t>(i)].rpm;
        }
        if (data_valid) {
            const double v_fl = static_cast<double>(rpm_values[FRONT_LEFT]) * rpm_to_wheel_speed_factor_;
            const double v_rl = static_cast<double>(rpm_values[REAR_LEFT]) * rpm_to_wheel_speed_factor_;
            const double v_rr = static_cast<double>(rpm_values[REAR_RIGHT]) * rpm_to_wheel_speed_factor_;
            const double v_fr = static_cast<double>(rpm_values[FRONT_RIGHT]) * rpm_to_wheel_speed_factor_;
            const BodyVelocity velocity =
                bodyVelocityFromWheelSpeeds(config_.wheel_base, config_.track_width, v_fl, v_rl, v_rr, v_fr);
            vx = velocity.vx;
            vy = velocity.vy;
            omega = velocity.omega;
        }
    }

    if (!data_valid) {
        return;
    }

    double wheel_acc_measure = 0.0;
    if (prev_vel_valid_) {
        const double ax_wheel = (vx - prev_vx_) / dt;
        const double ay_wheel = (vy - prev_vy_) / dt;
        wheel_acc_measure = std::hypot(ax_wheel, ay_wheel);
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
        const double omega_diff = std::fabs(imu_snapshot.gz - omega);
        const double imu_acc_xy = std::hypot(imu_snapshot.ax, imu_snapshot.ay);
        const double acc_mismatch = std::fabs(imu_acc_xy - wheel_acc_measure) / (std::fabs(imu_acc_xy) + kEpsilon);
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

    {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        vx_ = vx;
        vy_ = vy;
        omega_ = omega;
        integrateHolonomicBodyVelocity(x_, y_, yaw_, vx, vy, omega, dt);
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

void CanOdom::getPose(double& x, double& y, double& yaw) const {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    x = x_;
    y = y_;
    yaw = yaw_;
}

void CanOdom::getVelocity(double& vx, double& vy, double& omega) const {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    vx = vx_;
    vy = vy_;
    omega = omega_;
}

void CanOdom::reset() {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
    RCLCPP_INFO(node_.get_logger(), "里程计已重置");
}

}  // namespace rc26_merge_odom
