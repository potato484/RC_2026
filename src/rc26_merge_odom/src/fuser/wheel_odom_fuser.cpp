// RC2026 轮式里程计融合器实现
#include "rc26_merge_odom/fuser/wheel_odom_fuser.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

namespace rc26_merge_odom {

namespace {
constexpr size_t kCovVxIndex = 0;
constexpr size_t kCovVyIndex = 7;
constexpr size_t kCovWzIndex = 35;
constexpr double kMinVar = 1e-4;
constexpr double kEpsilon = 1e-9;
constexpr double kTrackedLateralVariance = 1e-4;

std::string formatDouble(double value) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << value;
    return oss.str();
}

void fuseDimension(double mean_can, double mean_wheel, double w_can, double w_wheel, double fallback_mean,
                   double fallback_var, double& fused_mean, double& fused_var) {
    const double w_sum = w_can + w_wheel;
    if (w_sum > kEpsilon) {
        fused_mean = (w_can * mean_can + w_wheel * mean_wheel) / w_sum;
        fused_var = 1.0 / w_sum;
        return;
    }
    fused_mean = fallback_mean;
    fused_var = std::max(fallback_var, kMinVar);
}
}  // namespace

WheelOdomFuser::WheelOdomFuser(rclcpp::Node& node, Config config) : node_(node), config_(std::move(config)) {
    const std::string raw_chassis_model = config_.chassis_model;
    config_.chassis_model = normalizeChassisModel(config_.chassis_model);
    if (raw_chassis_model != config_.chassis_model) {
        RCLCPP_WARN(node_.get_logger(), "wheel_odom_fuser chassis_model=%s invalid, fallback to %s",
                    raw_chassis_model.c_str(), config_.chassis_model.c_str());
    }
    chassis_model_ = parseChassisModel(config_.chassis_model);
    if (config_.publish_rate_hz <= 0) {
        RCLCPP_WARN(node_.get_logger(), "wheel_odom_fuser publish_rate_hz=%d invalid, fallback to 1",
                    config_.publish_rate_hz);
        config_.publish_rate_hz = 1;
    }
    config_.data_timeout_ms = std::max(0.0, config_.data_timeout_ms);
    config_.omega_sigma_rps = std::max(config_.omega_sigma_rps, kEpsilon);
    config_.chi2_threshold_dof3 = std::max(config_.chi2_threshold_dof3, kEpsilon);
    config_.outlier_penalty = std::clamp(config_.outlier_penalty, 0.0, 1.0);
    config_.recovery_tau_s = std::max(config_.recovery_tau_s, kEpsilon);
    if (config_.can_odom_topic.empty()) {
        config_.can_odom_topic = Config{}.can_odom_topic;
    }
    if (config_.wheel_odom_topic.empty()) {
        config_.wheel_odom_topic = Config{}.wheel_odom_topic;
    }
    if (config_.imu_topic.empty()) {
        config_.imu_topic = Config{}.imu_topic;
    }
    if (config_.fused_odom_topic.empty()) {
        config_.fused_odom_topic = Config{}.fused_odom_topic;
    }
    if (config_.health_topic.empty()) {
        config_.health_topic = Config{}.health_topic;
    }

    fused_pub_ = node_.create_publisher<nav_msgs::msg::Odometry>(config_.fused_odom_topic, 10);
    health_pub_ = node_.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(config_.health_topic, 10);

    can_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
        config_.can_odom_topic, 20, std::bind(&WheelOdomFuser::canOdomCallback, this, std::placeholders::_1));
    wheel_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
        config_.wheel_odom_topic, 20, std::bind(&WheelOdomFuser::wheelOdomCallback, this, std::placeholders::_1));
    imu_sub_ = node_.create_subscription<sensor_msgs::msg::Imu>(
        config_.imu_topic, 50, std::bind(&WheelOdomFuser::imuCallback, this, std::placeholders::_1));

    const int publish_rate_hz = config_.publish_rate_hz;
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / static_cast<double>(publish_rate_hz)));
    timer_ = node_.create_wall_timer(period, std::bind(&WheelOdomFuser::onPublishTimer, this));

    RCLCPP_INFO(node_.get_logger(),
                "WheelOdomFuser 启动: can=%s, wheel=%s, imu=%s, fused=%s, health=%s, rate=%dHz, timeout=%.1fms, "
                "chassis_model=%s",
                config_.can_odom_topic.c_str(), config_.wheel_odom_topic.c_str(), config_.imu_topic.c_str(),
                config_.fused_odom_topic.c_str(), config_.health_topic.c_str(), publish_rate_hz,
                config_.data_timeout_ms, chassisModelName(chassis_model_));
}

WheelOdomFuser::~WheelOdomFuser() {
    if (timer_) {
        timer_->cancel();
    }
}

void WheelOdomFuser::canOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    can_snapshot_.odom = *msg;
    can_snapshot_.stamp = std::chrono::steady_clock::now();
    can_snapshot_.received = true;
}

void WheelOdomFuser::wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    wheel_snapshot_.odom = *msg;
    wheel_snapshot_.stamp = std::chrono::steady_clock::now();
    wheel_snapshot_.received = true;
}

void WheelOdomFuser::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    imu_snapshot_.omega_z = static_cast<double>(msg->angular_velocity.z);
    imu_snapshot_.stamp = std::chrono::steady_clock::now();
    imu_snapshot_.received = true;
}

double WheelOdomFuser::extractVariance(const std::array<double, 36>& covariance, size_t index) const {
    if (index >= covariance.size()) {
        return kMinVar;
    }
    const double value = covariance[index];
    if (!std::isfinite(value) || value < kMinVar) {
        return kMinVar;
    }
    return value;
}

double WheelOdomFuser::clamp01(double value) {
    return std::clamp(value, 0.0, 1.0);
}

WheelOdomFuser::SourceState WheelOdomFuser::buildSourceState(const OdomSnapshot& snapshot, bool imu_valid, double imu_wz,
                                                             const std::chrono::steady_clock::time_point& now_steady) const {
    SourceState state;
    state.var_vx = kMinVar;
    state.var_vy = kMinVar;
    state.var_wz = kMinVar;
    if (!snapshot.received) {
        return state;
    }

    state.odom = snapshot.odom;
    state.age_ms = std::chrono::duration<double, std::milli>(now_steady - snapshot.stamp).count();
    state.valid = (state.age_ms <= config_.data_timeout_ms);
    if (!state.valid) {
        return state;
    }

    state.vx = static_cast<double>(state.odom.twist.twist.linear.x);
    state.vy = static_cast<double>(state.odom.twist.twist.linear.y);
    state.wz = static_cast<double>(state.odom.twist.twist.angular.z);

    state.var_vx = extractVariance(state.odom.twist.covariance, kCovVxIndex);
    state.var_vy = extractVariance(state.odom.twist.covariance, kCovVyIndex);
    state.var_wz = extractVariance(state.odom.twist.covariance, kCovWzIndex);

    state.omega_diff = imu_valid ? (state.wz - imu_wz) : 0.0;
    const double sigma = std::max(config_.omega_sigma_rps, kEpsilon);
    const double normalized = state.omega_diff / sigma;
    state.h_inst = std::exp(-0.5 * normalized * normalized);
    return state;
}

void WheelOdomFuser::onPublishTimer() {
    const auto now_steady = std::chrono::steady_clock::now();

    OdomSnapshot can_snapshot;
    OdomSnapshot wheel_snapshot;
    ImuSnapshot imu_snapshot;
    double h_can = 0.0;
    double h_wheel = 0.0;
    double dt = 0.0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        can_snapshot = can_snapshot_;
        wheel_snapshot = wheel_snapshot_;
        imu_snapshot = imu_snapshot_;
        h_can = h_can_;
        h_wheel = h_wheel_;

        if (!has_last_update_) {
            dt = 1.0 / static_cast<double>(std::max(config_.publish_rate_hz, 1));
            has_last_update_ = true;
        } else {
            dt = std::chrono::duration<double>(now_steady - last_update_time_).count();
        }
        last_update_time_ = now_steady;
    }

    if (!std::isfinite(dt) || dt <= 0.0) {
        dt = 1.0 / static_cast<double>(std::max(config_.publish_rate_hz, 1));
    }
    dt = std::min(dt, 1.0);

    const double imu_age_ms =
        imu_snapshot.received ? std::chrono::duration<double, std::milli>(now_steady - imu_snapshot.stamp).count() : -1.0;
    const bool imu_valid = imu_snapshot.received && (imu_age_ms <= config_.data_timeout_ms);

    SourceState can_state = buildSourceState(can_snapshot, imu_valid, imu_snapshot.omega_z, now_steady);
    SourceState wheel_state = buildSourceState(wheel_snapshot, imu_valid, imu_snapshot.omega_z, now_steady);

    double d2 = 0.0;
    bool chi2_outlier = false;
    if (can_state.valid && wheel_state.valid) {
        const double dvx = can_state.vx - wheel_state.vx;
        const double dvy = can_state.vy - wheel_state.vy;
        const double dwz = can_state.wz - wheel_state.wz;

        const double den_vx = std::max(can_state.var_vx + wheel_state.var_vx, kMinVar);
        const double den_vy = std::max(can_state.var_vy + wheel_state.var_vy, kMinVar);
        const double den_wz = std::max(can_state.var_wz + wheel_state.var_wz, kMinVar);
        d2 = (dvx * dvx) / den_vx + (dvy * dvy) / den_vy + (dwz * dwz) / den_wz;

        if (d2 > config_.chi2_threshold_dof3) {
            chi2_outlier = true;
            const double penalty = std::max(config_.outlier_penalty, 0.0);
            if (can_state.h_inst <= wheel_state.h_inst) {
                can_state.h_inst *= penalty;
            } else {
                wheel_state.h_inst *= penalty;
            }
        }
    }

    const double tau = std::max(config_.recovery_tau_s, kEpsilon);
    const double alpha = clamp01(1.0 - std::exp(-dt / tau));
    can_state.h_smooth = clamp01(h_can + alpha * (can_state.h_inst - h_can));
    wheel_state.h_smooth = clamp01(h_wheel + alpha * (wheel_state.h_inst - h_wheel));

    {
        std::lock_guard<std::mutex> lock(mutex_);
        h_can_ = can_state.h_smooth;
        h_wheel_ = wheel_state.h_smooth;
    }

    const double can_h_for_weight = can_state.valid ? can_state.h_smooth : 0.0;
    const double wheel_h_for_weight = wheel_state.valid ? wheel_state.h_smooth : 0.0;

    can_state.w_vx = can_h_for_weight / can_state.var_vx;
    can_state.w_vy = can_h_for_weight / can_state.var_vy;
    can_state.w_wz = can_h_for_weight / can_state.var_wz;
    can_state.total_weight = can_state.w_vx + can_state.w_vy + can_state.w_wz;

    wheel_state.w_vx = wheel_h_for_weight / wheel_state.var_vx;
    wheel_state.w_vy = wheel_h_for_weight / wheel_state.var_vy;
    wheel_state.w_wz = wheel_h_for_weight / wheel_state.var_wz;
    wheel_state.total_weight = wheel_state.w_vx + wheel_state.w_vy + wheel_state.w_wz;

    std::string state_text = "NO_DATA";
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    if (can_state.valid && wheel_state.valid) {
        if (chi2_outlier) {
            state_text = "CHI2_OUTLIER";
            level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        } else {
            state_text = "OK";
            level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        }
    } else if (can_state.valid) {
        state_text = "CAN_ONLY";
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else if (wheel_state.valid) {
        state_text = "WHEEL_ONLY";
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }

    if (can_state.valid || wheel_state.valid) {
        const SourceState* pose_source = nullptr;
        if (can_state.valid && !wheel_state.valid) {
            pose_source = &can_state;
        } else if (!can_state.valid && wheel_state.valid) {
            pose_source = &wheel_state;
        } else if (can_state.valid && wheel_state.valid) {
            pose_source = (can_state.total_weight >= wheel_state.total_weight) ? &can_state : &wheel_state;
        }

        if (pose_source != nullptr) {
            nav_msgs::msg::Odometry fused_msg;
            fused_msg.header.stamp = node_.now();
            fused_msg.header.frame_id = pose_source->odom.header.frame_id;
            fused_msg.child_frame_id = pose_source->odom.child_frame_id;
            fused_msg.pose = pose_source->odom.pose;

            double fused_vx = 0.0;
            double fused_vy = 0.0;
            double fused_wz = 0.0;
            double fused_var_vx = kMinVar;
            double fused_var_vy = kMinVar;
            double fused_var_wz = kMinVar;
            fuseDimension(can_state.vx, wheel_state.vx, can_state.w_vx, wheel_state.w_vx, pose_source->vx,
                          pose_source->var_vx, fused_vx, fused_var_vx);
            fuseDimension(can_state.vy, wheel_state.vy, can_state.w_vy, wheel_state.w_vy, pose_source->vy,
                          pose_source->var_vy, fused_vy, fused_var_vy);
            fuseDimension(can_state.wz, wheel_state.wz, can_state.w_wz, wheel_state.w_wz, pose_source->wz,
                          pose_source->var_wz, fused_wz, fused_var_wz);

            if (isTrackedDiffModel(chassis_model_)) {
                fused_vy = 0.0;
                fused_var_vy = kTrackedLateralVariance;
            }

            fused_msg.twist.twist.linear.x = fused_vx;
            fused_msg.twist.twist.linear.y = fused_vy;
            fused_msg.twist.twist.linear.z = pose_source->odom.twist.twist.linear.z;
            fused_msg.twist.twist.angular.x = pose_source->odom.twist.twist.angular.x;
            fused_msg.twist.twist.angular.y = pose_source->odom.twist.twist.angular.y;
            fused_msg.twist.twist.angular.z = fused_wz;

            std::fill(fused_msg.twist.covariance.begin(), fused_msg.twist.covariance.end(), 0.0);
            fused_msg.twist.covariance[kCovVxIndex] = fused_var_vx;
            fused_msg.twist.covariance[kCovVyIndex] = fused_var_vy;
            fused_msg.twist.covariance[kCovWzIndex] = fused_var_wz;

            fused_pub_->publish(fused_msg);
        }
    }

    publishHealth(state_text, level, can_state, wheel_state, d2);
}

void WheelOdomFuser::publishHealth(const std::string& state_text, uint8_t level, const SourceState& can_state,
                                   const SourceState& wheel_state, double d2) {
    diagnostic_msgs::msg::DiagnosticArray health_msg;
    health_msg.header.stamp = node_.now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level;
    status.name = "wheel_odom_fuser";
    status.hardware_id = "rc26_merge_odom";
    status.message = state_text;

    auto addKeyValue = [&status](const std::string& key, const std::string& value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        status.values.push_back(std::move(kv));
    };

    addKeyValue("state", state_text);
    addKeyValue("chassis_model", chassisModelName(chassis_model_));
    addKeyValue("h_can", formatDouble(can_state.h_smooth));
    addKeyValue("h_wheel", formatDouble(wheel_state.h_smooth));
    addKeyValue("omega_diff_can", formatDouble(can_state.omega_diff));
    addKeyValue("omega_diff_wheel", formatDouble(wheel_state.omega_diff));
    addKeyValue("d2", formatDouble(d2));
    addKeyValue("w_can_vx", formatDouble(can_state.w_vx));
    addKeyValue("w_can_vy", formatDouble(can_state.w_vy));
    addKeyValue("w_can_wz", formatDouble(can_state.w_wz));
    addKeyValue("w_wheel_vx", formatDouble(wheel_state.w_vx));
    addKeyValue("w_wheel_vy", formatDouble(wheel_state.w_vy));
    addKeyValue("w_wheel_wz", formatDouble(wheel_state.w_wz));
    addKeyValue("age_can_ms", formatDouble(can_state.age_ms));
    addKeyValue("age_wheel_ms", formatDouble(wheel_state.age_ms));

    health_msg.status.push_back(std::move(status));
    health_pub_->publish(health_msg);
}

}  // namespace rc26_merge_odom
