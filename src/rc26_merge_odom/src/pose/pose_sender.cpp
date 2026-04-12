// RC2026 速度发送模块实现
#include "rc26_merge_odom/pose/pose_sender.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <sstream>
#include <stdexcept>

#include "rc26_serial/protocol.hpp"

namespace rc26_merge_odom {

namespace {
constexpr double kMaxValidDtSec = 1.0;
constexpr float kEpsilon = 1e-6f;
constexpr float kMinImuVar = 1e-4f;
constexpr double kTwoPi = 6.28318530717958647692;

bool isValidDt(double dt) {
    return dt > 0.0 && dt <= kMaxValidDtSec;
}

void projectToCircle(float& vx, float& vy, float radius) {
    const float mag = std::hypot(vx, vy);
    if (radius <= kEpsilon || mag <= radius || mag <= kEpsilon) {
        if (radius <= kEpsilon) {
            vx = 0.0f;
            vy = 0.0f;
        }
        return;
    }
    const float scale = radius / mag;
    vx *= scale;
    vy *= scale;
}

float pickIntersectionScale(float s1, float s2) {
    float selected = 1.0f;
    bool found = false;
    if (s1 >= 0.0f && s1 <= 1.0f) {
        selected = s1;
        found = true;
    }
    if (s2 >= 0.0f && s2 <= 1.0f) {
        if (!found || s2 > selected) {
            selected = s2;
            found = true;
        }
    }
    return found ? selected : std::clamp(s2, 0.0f, 1.0f);
}
}  // namespace

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

    const auto logNormalized = [this](const char* name, auto before, auto after) {
        if (before != after) {
            std::ostringstream oss;
            oss << before;
            std::ostringstream normalized;
            normalized << after;
            RCLCPP_WARN(node_.get_logger(), "PoseSender 参数 %s 非法/越界，已从 %s 归一化为 %s",
                        name, oss.str().c_str(), normalized.str().c_str());
        }
    };

    const int raw_cmd_vel_timeout_ms = config_.cmd_vel_timeout_ms;
    const int raw_spike_freeze_duration_ms = config_.spike_freeze_duration_ms;
    const std::string raw_chassis_model = config_.chassis_model;
    const float raw_v_max_mps = config_.v_max_mps;
    const float raw_w_max_rps = config_.w_max_rps;
    const float raw_a_max_mps2 = config_.a_max_mps2;
    const float raw_alpha_max_rps2 = config_.alpha_max_rps2;
    const float raw_track_width_m = config_.track_width_m;
    const float raw_track_speed_max_mps = config_.track_speed_max_mps;
    const float raw_track_accel_max_mps2 = config_.track_accel_max_mps2;
    const float raw_imu_gate_ema_alpha = config_.imu_gate_ema_alpha;
    const float raw_imu_gate_chi2_threshold = config_.imu_gate_chi2_threshold;
    const float raw_accel_agree_threshold_mps2 = config_.accel_agree_threshold_mps2;
    const float raw_spike_decay_tau_s = config_.spike_decay_tau_s;
    const float raw_governor_lambda = config_.governor_lambda;
    const float raw_dob_lpf_hz = config_.dob_lpf_hz;
    const float raw_latency_comp_s = config_.latency_comp_s;
    const std::string raw_cmd_vel_topic = config_.cmd_vel_topic;
    const std::string raw_odom_topic = config_.odom_topic;
    const std::string raw_imu_topic = config_.imu_topic;

    config_.chassis_model = normalizeChassisModel(config_.chassis_model);
    config_.cmd_vel_timeout_ms = std::max(0, config_.cmd_vel_timeout_ms);
    config_.spike_freeze_duration_ms = std::max(0, config_.spike_freeze_duration_ms);
    config_.v_max_mps = std::fabs(config_.v_max_mps);
    config_.w_max_rps = std::fabs(config_.w_max_rps);
    config_.a_max_mps2 = std::fabs(config_.a_max_mps2);
    config_.alpha_max_rps2 = std::fabs(config_.alpha_max_rps2);
    config_.track_width_m = std::max(std::fabs(config_.track_width_m), kEpsilon);
    config_.track_speed_max_mps = std::fabs(config_.track_speed_max_mps);
    config_.track_accel_max_mps2 = std::fabs(config_.track_accel_max_mps2);
    config_.imu_gate_ema_alpha = std::clamp(config_.imu_gate_ema_alpha, 0.0f, 0.9999f);
    config_.imu_gate_chi2_threshold = std::max(0.1f, config_.imu_gate_chi2_threshold);
    config_.accel_agree_threshold_mps2 = std::max(0.0f, config_.accel_agree_threshold_mps2);
    config_.spike_decay_tau_s = std::max(1e-3f, config_.spike_decay_tau_s);
    config_.governor_lambda = std::max(0.0f, config_.governor_lambda);
    config_.dob_lpf_hz = std::max(0.0f, config_.dob_lpf_hz);
    config_.latency_comp_s = std::max(0.0f, config_.latency_comp_s);
    if (config_.cmd_vel_topic.empty()) {
        config_.cmd_vel_topic = Config{}.cmd_vel_topic;
    }
    if (config_.odom_topic.empty()) {
        config_.odom_topic = Config{}.odom_topic;
    }
    chassis_model_ = parseChassisModel(config_.chassis_model);

    logNormalized("chassis_model", raw_chassis_model, config_.chassis_model);
    logNormalized("cmd_vel_timeout_ms", raw_cmd_vel_timeout_ms, config_.cmd_vel_timeout_ms);
    logNormalized("spike_freeze_duration_ms", raw_spike_freeze_duration_ms, config_.spike_freeze_duration_ms);
    logNormalized("v_max_mps", raw_v_max_mps, config_.v_max_mps);
    logNormalized("w_max_rps", raw_w_max_rps, config_.w_max_rps);
    logNormalized("a_max_mps2", raw_a_max_mps2, config_.a_max_mps2);
    logNormalized("alpha_max_rps2", raw_alpha_max_rps2, config_.alpha_max_rps2);
    logNormalized("track_width_m", raw_track_width_m, config_.track_width_m);
    logNormalized("track_speed_max_mps", raw_track_speed_max_mps, config_.track_speed_max_mps);
    logNormalized("track_accel_max_mps2", raw_track_accel_max_mps2, config_.track_accel_max_mps2);
    logNormalized("imu_gate_ema_alpha", raw_imu_gate_ema_alpha, config_.imu_gate_ema_alpha);
    logNormalized("imu_gate_chi2_threshold", raw_imu_gate_chi2_threshold, config_.imu_gate_chi2_threshold);
    logNormalized("accel_agree_threshold_mps2", raw_accel_agree_threshold_mps2,
                  config_.accel_agree_threshold_mps2);
    logNormalized("spike_decay_tau_s", raw_spike_decay_tau_s, config_.spike_decay_tau_s);
    logNormalized("governor_lambda", raw_governor_lambda, config_.governor_lambda);
    logNormalized("dob_lpf_hz", raw_dob_lpf_hz, config_.dob_lpf_hz);
    logNormalized("latency_comp_s", raw_latency_comp_s, config_.latency_comp_s);
    logNormalized("cmd_vel_topic", raw_cmd_vel_topic, config_.cmd_vel_topic);
    logNormalized("odom_topic", raw_odom_topic, config_.odom_topic);
    logNormalized("imu_topic", raw_imu_topic, config_.imu_topic);

    const auto now = std::chrono::steady_clock::now();
    feedback_stats_.window_start = now;
    target_stats_.window_start = now;
    imu_spike_deadline_ = now;
    imu_scale_last_update_ = now;

    cmd_vel_sub_ = node_.create_subscription<geometry_msgs::msg::Twist>(
        config_.cmd_vel_topic, 10, std::bind(&PoseSender::cmdVelCallback, this, std::placeholders::_1));

    odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
        config_.odom_topic, 10, std::bind(&PoseSender::odomCallback, this, std::placeholders::_1));

    if (!config_.imu_topic.empty()) {
        imu_sub_ = node_.create_subscription<sensor_msgs::msg::Imu>(
            config_.imu_topic, 20, std::bind(&PoseSender::imuCallback, this, std::placeholders::_1));
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

    const bool feedback_serial_ready = feedback_serial_ && feedback_serial_->isOpen();
    const bool target_serial_ready = target_serial_ && target_serial_->isOpen();
    const char* serial_mode = "双串口模式";
    if (feedback_serial_ready && target_serial_ready) {
        serial_mode = "双串口模式";
    } else if (target_serial_ready) {
        serial_mode = "目标串口单链路降级模式";
    } else {
        serial_mode = "仅反馈链路模式";
    }

    RCLCPP_INFO(node_.get_logger(),
                "PoseSender 启动 (%s)，cmd_vel: %s, odom: %s, imu: %s, 反馈: %d Hz, 目标: %d Hz, "
                "chassis_model=%s",
                serial_mode,
                config_.cmd_vel_topic.c_str(), config_.odom_topic.c_str(),
                config_.imu_topic.empty() ? "disabled" : config_.imu_topic.c_str(),
                config_.feedback_send_rate_hz, config_.target_send_rate_hz, chassisModelName(chassis_model_));
    RCLCPP_INFO(node_.get_logger(),
                "PoseSender 保护参数: imu_gate=%s governor=%s dob=%s cmd_vel_timeout=%dms",
                config_.imu_gate_enable ? "on" : "off", config_.governor_enable ? "on" : "off",
                config_.dob_enable ? "on" : "off", config_.cmd_vel_timeout_ms);
}

PoseSender::~PoseSender() {
    if (feedback_timer_) {
        feedback_timer_->cancel();
    }
    if (target_timer_) {
        target_timer_->cancel();
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
    const auto now = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(data_mutex_);
    feedback_vel_.vx = static_cast<float>(msg->twist.twist.linear.x);
    feedback_vel_.vy = static_cast<float>(msg->twist.twist.linear.y);
    feedback_vel_.wz = static_cast<float>(msg->twist.twist.angular.z);

    if (prev_odom_valid_) {
        const double dt = std::chrono::duration<double>(now - prev_odom_time_).count();
        if (isValidDt(dt)) {
            const float dvx = feedback_vel_.vx - prev_odom_vel_.vx;
            if (isTrackedDiffModel(chassis_model_)) {
                wheel_accel_mps2_ = std::fabs(dvx) / static_cast<float>(dt);
            } else {
                const float dvy = feedback_vel_.vy - prev_odom_vel_.vy;
                wheel_accel_mps2_ = std::hypot(dvx, dvy) / static_cast<float>(dt);
            }
            wheel_accel_valid_ = true;
        } else {
            wheel_accel_valid_ = false;
        }
    } else {
        wheel_accel_valid_ = false;
    }

    prev_odom_vel_ = feedback_vel_;
    prev_odom_time_ = now;
    prev_odom_valid_ = true;
}

void PoseSender::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const auto now = std::chrono::steady_clock::now();
    const float ax = static_cast<float>(msg->linear_acceleration.x);
    const float ay = static_cast<float>(msg->linear_acceleration.y);
    const float gz = static_cast<float>(msg->angular_velocity.z);

    {
        std::lock_guard<std::mutex> lock(imu_cache_mutex_);
        cached_imu_.ax = ax;
        cached_imu_.ay = ay;
        cached_imu_.gz = gz;
        cached_imu_.stamp = now;
        cached_imu_.valid = true;
    }

    if (!config_.imu_gate_enable) {
        return;
    }

    const float alpha_old = std::clamp(config_.imu_gate_ema_alpha, 0.0f, 0.9999f);
    const float alpha_new = 1.0f - alpha_old;

    float d2 = 0.0f;
    {
        std::lock_guard<std::mutex> lock(imu_gate_mutex_);
        if (!imu_gate_initialized_) {
            imu_gate_initialized_ = true;
            imu_gate_mean_ax_ = ax;
            imu_gate_var_ax_ = kMinImuVar;
            imu_last_d2_ = 0.0f;
            return;
        }

        const float prev_mean = imu_gate_mean_ax_;
        imu_gate_mean_ax_ = alpha_old * imu_gate_mean_ax_ + alpha_new * ax;
        const float innovation = ax - prev_mean;
        imu_gate_var_ax_ = std::max(kMinImuVar, alpha_old * imu_gate_var_ax_ + alpha_new * innovation * innovation);

        const float residual = ax - imu_gate_mean_ax_;
        d2 = (residual * residual) / std::max(imu_gate_var_ax_, kMinImuVar);
        imu_last_d2_ = d2;
    }

    const float chi2_threshold = std::max(config_.imu_gate_chi2_threshold, 0.1f);
    if (d2 <= chi2_threshold) {
        return;
    }

    float wheel_accel = 0.0f;
    bool wheel_accel_valid = false;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        wheel_accel_valid = wheel_accel_valid_ && hasFreshOdomLocked(now);
        if (wheel_accel_valid) {
            wheel_accel = wheel_accel_mps2_;
        }
    }

    if (wheel_accel_valid && wheel_accel >= config_.accel_agree_threshold_mps2) {
        RCLCPP_DEBUG_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
                              "[PoseSender] IMU χ² outlier ignored by wheel acceleration: d2=%.3f aw=%.3f", d2,
                              wheel_accel);
        return;
    }

    const float gate_scale = std::exp(-d2 / (2.0f * chi2_threshold));
    {
        std::lock_guard<std::mutex> lock(imu_spike_mutex_);
        imu_spike_scale_ = std::min(imu_spike_scale_, std::clamp(gate_scale, 0.0f, 1.0f));
        imu_spike_deadline_ = now + std::chrono::milliseconds(std::max(0, config_.spike_freeze_duration_ms));
        if (!imu_scale_initialized_) {
            imu_scale_initialized_ = true;
            imu_scale_last_update_ = now;
        }
    }

    if (config_.imu_gate_log_enable) {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 500,
                             "[PoseSender] IMU gate trigger: d2=%.3f scale=%.3f wheel_acc=%.3f", d2,
                             gate_scale, wheel_accel);
    }
}

std::chrono::milliseconds PoseSender::sensorFreshnessTimeout() const {
    const int fastest_rate_hz = std::max(std::max(config_.feedback_send_rate_hz, config_.target_send_rate_hz), 1);
    const int period_ms = std::max(1, static_cast<int>(std::ceil(1000.0 / static_cast<double>(fastest_rate_hz))));
    return std::chrono::milliseconds(std::max(100, period_ms * 3));
}

bool PoseSender::hasFreshOdomLocked(const std::chrono::steady_clock::time_point& now) const {
    return prev_odom_valid_ && (now - prev_odom_time_ <= sensorFreshnessTimeout());
}

bool PoseSender::getFreshImuCache(const std::chrono::steady_clock::time_point& now, ImuCache& imu_cache) const {
    std::lock_guard<std::mutex> lock(imu_cache_mutex_);
    if (!cached_imu_.valid || now - cached_imu_.stamp > sensorFreshnessTimeout()) {
        return false;
    }
    imu_cache = cached_imu_;
    return true;
}

float PoseSender::getEffectiveVMax(const std::chrono::steady_clock::time_point&) const {
    float effective_v_max = std::fabs(config_.v_max_mps);
    if (isTrackedDiffModel(chassis_model_)) {
        effective_v_max = std::min(effective_v_max, std::fabs(config_.track_speed_max_mps));
    }
    return effective_v_max;
}

void PoseSender::normalizeVelocityForChassis(Velocity& velocity) const {
    if (isTrackedDiffModel(chassis_model_)) {
        velocity.vy = 0.0f;
    }
}

void PoseSender::applyImuSpikeScale(Velocity& velocity, const std::chrono::steady_clock::time_point& now) {
    if (!config_.imu_gate_enable) {
        return;
    }

    float scale = 1.0f;
    {
        std::lock_guard<std::mutex> lock(imu_spike_mutex_);
        if (!imu_scale_initialized_) {
            imu_scale_initialized_ = true;
            imu_scale_last_update_ = now;
        }

        const double elapsed = std::chrono::duration<double>(now - imu_scale_last_update_).count();
        imu_scale_last_update_ = now;

        if (now >= imu_spike_deadline_) {
            const double tau = std::max(1e-6, static_cast<double>(config_.spike_decay_tau_s));
            const double alpha = 1.0 - std::exp(-std::max(0.0, elapsed) / tau);
            imu_spike_scale_ = static_cast<float>(imu_spike_scale_ + (1.0f - imu_spike_scale_) * alpha);
        }

        imu_spike_scale_ = std::clamp(imu_spike_scale_, 0.0f, 1.0f);
        scale = imu_spike_scale_;
    }

    velocity.vx *= scale;
    velocity.vy *= scale;
    velocity.wz *= scale;
    normalizeVelocityForChassis(velocity);
}

bool PoseSender::imuSpikeActive() const {
    if (!config_.imu_gate_enable) {
        return false;
    }

    std::lock_guard<std::mutex> lock(imu_spike_mutex_);
    return imu_spike_scale_ < 0.999f;
}

PoseSender::Velocity PoseSender::applyFallbackProtect(Velocity raw, const Velocity& prev_vel, double dt,
                                                      float effective_v_max) const {
    normalizeVelocityForChassis(raw);

    if (isTrackedDiffModel(chassis_model_)) {
        const float w_limit = std::fabs(config_.w_max_rps);
        const float track_speed_limit = std::max(std::fabs(config_.track_speed_max_mps), kEpsilon);
        const float track_accel_limit = std::fabs(config_.track_accel_max_mps2);
        const float half_track_width = std::max(std::fabs(config_.track_width_m) * 0.5f, kEpsilon);

        Velocity prev_projected = prev_vel;
        normalizeVelocityForChassis(prev_projected);

        float raw_vx = std::clamp(raw.vx, -effective_v_max, effective_v_max);
        float raw_wz = std::clamp(raw.wz, -w_limit, w_limit);

        float left = raw_vx - raw_wz * half_track_width;
        float right = raw_vx + raw_wz * half_track_width;

        const float max_track_mag = std::max(std::fabs(left), std::fabs(right));
        if (max_track_mag > track_speed_limit) {
            const float scale = track_speed_limit / max_track_mag;
            left *= scale;
            right *= scale;
        }

        if (isValidDt(dt)) {
            const float dv_limit = track_accel_limit * static_cast<float>(dt);
            const float prev_left = prev_projected.vx - prev_projected.wz * half_track_width;
            const float prev_right = prev_projected.vx + prev_projected.wz * half_track_width;
            left = std::clamp(left, prev_left - dv_limit, prev_left + dv_limit);
            right = std::clamp(right, prev_right - dv_limit, prev_right + dv_limit);
        }

        const float vx = (left + right) * 0.5f;
        if (std::fabs(vx) > effective_v_max) {
            const float shift = vx > 0.0f ? (vx - effective_v_max) : (vx + effective_v_max);
            left -= shift;
            right -= shift;
        }

        Velocity output;
        output.vx = (left + right) * 0.5f;
        output.vy = 0.0f;
        output.wz = std::clamp((right - left) / (2.0f * half_track_width), -w_limit, w_limit);
        return output;
    }

    const float w_limit = std::fabs(config_.w_max_rps);
    const float a_limit = std::fabs(config_.a_max_mps2);
    const float alpha_limit = std::fabs(config_.alpha_max_rps2);

    Velocity output;
    output.wz = std::clamp(raw.wz, -w_limit, w_limit);
    output.vx = raw.vx;
    output.vy = raw.vy;

    projectToCircle(output.vx, output.vy, effective_v_max);

    if (isValidDt(dt)) {
        const float dv_limit = a_limit * static_cast<float>(dt);
        const float dw_limit = alpha_limit * static_cast<float>(dt);

        const float dvx = output.vx - prev_vel.vx;
        const float dvy = output.vy - prev_vel.vy;
        const float dv_mag = std::hypot(dvx, dvy);
        if (dv_mag > dv_limit && dv_mag > kEpsilon) {
            const float scale = dv_limit / dv_mag;
            output.vx = prev_vel.vx + dvx * scale;
            output.vy = prev_vel.vy + dvy * scale;
        }
        output.wz = std::clamp(output.wz, prev_vel.wz - dw_limit, prev_vel.wz + dw_limit);
    }

    projectToCircle(output.vx, output.vy, effective_v_max);
    return output;
}

PoseSender::Velocity PoseSender::applyGovernorProtect(Velocity raw, const Velocity& prev_vel, double dt,
                                                      float effective_v_max) const {
    normalizeVelocityForChassis(raw);
    const bool dt_valid = isValidDt(dt);
    const float w_limit = std::fabs(config_.w_max_rps);
    const float a_limit = std::fabs(config_.a_max_mps2);
    const float alpha_limit = std::fabs(config_.alpha_max_rps2);

    if (isTrackedDiffModel(chassis_model_)) {
        const float track_speed_limit = std::max(std::fabs(config_.track_speed_max_mps), kEpsilon);
        const float track_accel_limit = std::fabs(config_.track_accel_max_mps2);
        const float half_track_width = std::max(std::fabs(config_.track_width_m) * 0.5f, kEpsilon);

        Velocity prev_projected = prev_vel;
        normalizeVelocityForChassis(prev_projected);

        const float lambda = std::max(0.0f, config_.governor_lambda);
        const float inv_denom = 1.0f / (1.0f + lambda);

        float raw_vx = std::clamp(raw.vx, -effective_v_max, effective_v_max);
        float raw_wz = std::clamp(raw.wz, -w_limit, w_limit);
        const float raw_left = raw_vx - raw_wz * half_track_width;
        const float raw_right = raw_vx + raw_wz * half_track_width;
        const float prev_left = prev_projected.vx - prev_projected.wz * half_track_width;
        const float prev_right = prev_projected.vx + prev_projected.wz * half_track_width;

        float left = (raw_left + lambda * prev_left) * inv_denom;
        float right = (raw_right + lambda * prev_right) * inv_denom;

        if (dt_valid) {
            const float dv_limit = track_accel_limit * static_cast<float>(dt);
            left = std::clamp(left, prev_left - dv_limit, prev_left + dv_limit);
            right = std::clamp(right, prev_right - dv_limit, prev_right + dv_limit);
        }

        const float max_track_mag = std::max(std::fabs(left), std::fabs(right));
        if (max_track_mag > track_speed_limit) {
            const float scale = track_speed_limit / max_track_mag;
            left *= scale;
            right *= scale;
        }

        const float vx = (left + right) * 0.5f;
        if (std::fabs(vx) > effective_v_max) {
            const float shift = vx > 0.0f ? (vx - effective_v_max) : (vx + effective_v_max);
            left -= shift;
            right -= shift;
        }

        Velocity output;
        output.vx = (left + right) * 0.5f;
        output.vy = 0.0f;
        output.wz = std::clamp((right - left) / (2.0f * half_track_width), -w_limit, w_limit);
        if (dt_valid) {
            const float dw_limit = alpha_limit * static_cast<float>(dt);
            output.wz = std::clamp(output.wz, prev_projected.wz - dw_limit, prev_projected.wz + dw_limit);
        }
        return output;
    }

    Velocity prev_projected = prev_vel;
    projectToCircle(prev_projected.vx, prev_projected.vy, effective_v_max);

    const float lambda = std::max(0.0f, config_.governor_lambda);
    const float inv_denom = 1.0f / (1.0f + lambda);

    Velocity v_star;
    v_star.vx = (raw.vx + lambda * prev_projected.vx) * inv_denom;
    v_star.vy = (raw.vy + lambda * prev_projected.vy) * inv_denom;
    v_star.wz = (raw.wz + lambda * prev_projected.wz) * inv_denom;

    Velocity output;
    output.wz = std::clamp(v_star.wz, -w_limit, w_limit);

    if (dt_valid) {
        const float dw_limit = alpha_limit * static_cast<float>(dt);
        output.wz = std::clamp(output.wz, prev_projected.wz - dw_limit, prev_projected.wz + dw_limit);
    }

    Velocity v_box;
    if (dt_valid) {
        const float dv_limit = a_limit * static_cast<float>(dt);
        v_box.vx = std::clamp(v_star.vx, prev_projected.vx - dv_limit, prev_projected.vx + dv_limit);
        v_box.vy = std::clamp(v_star.vy, prev_projected.vy - dv_limit, prev_projected.vy + dv_limit);
    } else {
        v_box.vx = v_star.vx;
        v_box.vy = v_star.vy;
    }

    if (effective_v_max <= kEpsilon) {
        output.vx = 0.0f;
        output.vy = 0.0f;
        return output;
    }

    const float v_box_mag = std::hypot(v_box.vx, v_box.vy);
    if (v_box_mag <= effective_v_max + kEpsilon) {
        output.vx = v_box.vx;
        output.vy = v_box.vy;
        return output;
    }

    const float dx = v_box.vx - prev_projected.vx;
    const float dy = v_box.vy - prev_projected.vy;
    const float a = dx * dx + dy * dy;
    if (a <= kEpsilon) {
        output.vx = prev_projected.vx;
        output.vy = prev_projected.vy;
        return output;
    }

    const float b = 2.0f * (prev_projected.vx * dx + prev_projected.vy * dy);
    const float c = prev_projected.vx * prev_projected.vx + prev_projected.vy * prev_projected.vy -
                    effective_v_max * effective_v_max;
    const float discriminant = std::max(0.0f, b * b - 4.0f * a * c);
    const float sqrt_discriminant = std::sqrt(discriminant);
    const float s1 = (-b - sqrt_discriminant) / (2.0f * a);
    const float s2 = (-b + sqrt_discriminant) / (2.0f * a);
    const float s = pickIntersectionScale(s1, s2);

    output.vx = prev_projected.vx + s * dx;
    output.vy = prev_projected.vy + s * dy;
    projectToCircle(output.vx, output.vy, effective_v_max);
    return output;
}

PoseSender::Velocity PoseSender::protectVelocity(Velocity raw, Velocity& prev_vel, double dt, bool enable_dob) {
    const auto now = std::chrono::steady_clock::now();

    Velocity command = raw;
    normalizeVelocityForChassis(command);
    if (enable_dob && config_.dob_enable && isValidDt(dt)) {
        Velocity actual;
        bool actual_valid = false;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            if (hasFreshOdomLocked(now)) {
                actual = feedback_vel_;
                normalizeVelocityForChassis(actual);
                actual_valid = true;
            }
        }

        std::lock_guard<std::mutex> lock(dob_mutex_);
        if (actual_valid && prev_governor_output_valid_) {
            const float dt_f = static_cast<float>(dt);
            const float d_raw_vx = (actual.vx - prev_governor_output_.vx) / dt_f;
            const float d_raw_vy = (actual.vy - prev_governor_output_.vy) / dt_f;
            const float d_raw_wz = (actual.wz - prev_governor_output_.wz) / dt_f;

            const float dob_lpf_hz = std::max(0.0f, config_.dob_lpf_hz);
            float alpha = 1.0f;
            if (dob_lpf_hz > kEpsilon) {
                const float tau = 1.0f / static_cast<float>(kTwoPi * static_cast<double>(dob_lpf_hz));
                alpha = dt_f / (tau + dt_f);
            }
            alpha = std::clamp(alpha, 0.0f, 1.0f);

            dob_hat_.vx += alpha * (d_raw_vx - dob_hat_.vx);
            dob_hat_.vy += alpha * (d_raw_vy - dob_hat_.vy);
            dob_hat_.wz += alpha * (d_raw_wz - dob_hat_.wz);

            command.vx += config_.dob_kd * dob_hat_.vx;
            command.vy += config_.dob_kd * dob_hat_.vy;
            command.wz += config_.dob_kd * dob_hat_.wz;
        }
    }
    normalizeVelocityForChassis(command);

    const float effective_v_max = getEffectiveVMax(now);

    Velocity output;
    if (config_.governor_enable) {
        output = applyGovernorProtect(command, prev_vel, dt, effective_v_max);
    } else {
        output = applyFallbackProtect(command, prev_vel, dt, effective_v_max);
    }

    applyImuSpikeScale(output, now);
    prev_vel = output;

    if (enable_dob && config_.dob_enable) {
        std::lock_guard<std::mutex> lock(dob_mutex_);
        prev_governor_output_ = output;
        prev_governor_output_valid_ = true;
    }

    return output;
}

void PoseSender::feedbackTimerCallback() {
    Velocity feedback;
    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        feedback = feedback_vel_;
    }

    if (config_.latency_comp_enable) {
        ImuCache imu_cache;
        if (getFreshImuCache(now, imu_cache)) {
            feedback.vx += imu_cache.ax * config_.latency_comp_s;
            if (!isTrackedDiffModel(chassis_model_)) {
                feedback.vy += imu_cache.ay * config_.latency_comp_s;
            }
        }
    }
    normalizeVelocityForChassis(feedback);

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

    Velocity protected_vel = protectVelocity(feedback, prev_feedback, dt, false);
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        prev_feedback_vel_ = prev_feedback;
    }

    uint8_t seq = 0;
    bool attempted = false;
    bool sent_ok = false;
    if (feedback_serial_ && feedback_serial_->isOpen()) {
        attempted = true;
        sent_ok = feedback_serial_->sendPose(rc26_decision::CommandID::POSE_FEEDBACK, protected_vel.vx, protected_vel.vy,
                                             protected_vel.wz, seq);
    }

    if (attempted) {
        auto& stats = feedback_stats_;
        if (sent_ok) {
            if (stats.last_valid) {
                const uint8_t delta = static_cast<uint8_t>(seq - stats.last_ok_seq);
                if (delta > 1U) {
                    stats.missing_total += static_cast<uint64_t>(delta - 1U);
                    stats.missing_1s += static_cast<uint64_t>(delta - 1U);
                }
            }
            stats.last_ok_seq = seq;
            stats.last_valid = true;
            ++stats.ok_total;
            ++stats.ok_1s;
        } else {
            ++stats.fail_total;
            ++stats.fail_1s;
        }
        const auto feedback_now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(feedback_now - stats.window_start).count() >= 1.0) {
            if (config_.stats_log_enable) {
                RCLCPP_INFO_THROTTLE(
                    node_.get_logger(), *node_.get_clock(), 1000,
                    "[PoseSender/feedback] ok=%llu fail=%llu miss=%llu (1s: ok=%llu fail=%llu miss=%llu)",
                    static_cast<unsigned long long>(stats.ok_total),
                    static_cast<unsigned long long>(stats.fail_total),
                    static_cast<unsigned long long>(stats.missing_total),
                    static_cast<unsigned long long>(stats.ok_1s),
                    static_cast<unsigned long long>(stats.fail_1s),
                    static_cast<unsigned long long>(stats.missing_1s));
            }
            if (stats.fail_1s > 1U || stats.missing_1s > 1U) {
                RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
                                     "[PoseSender/feedback] 链路异常: fail=%llu miss=%llu",
                                     static_cast<unsigned long long>(stats.fail_1s),
                                     static_cast<unsigned long long>(stats.missing_1s));
            }
            stats.ok_1s = 0;
            stats.fail_1s = 0;
            stats.missing_1s = 0;
            stats.window_start = feedback_now;
        }
    }

    geometry_msgs::msg::TwistStamped feedback_msg;
    feedback_msg.header.stamp = node_.now();
    feedback_msg.twist.linear.x = protected_vel.vx;
    feedback_msg.twist.linear.y = protected_vel.vy;
    feedback_msg.twist.angular.z = protected_vel.wz;
    feedback_protected_pub_->publish(feedback_msg);

    std_msgs::msg::Bool spike_msg;
    spike_msg.data = imuSpikeActive();
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
    normalizeVelocityForChassis(target);

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

    Velocity protected_vel = protectVelocity(target, prev_target, dt, true);
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        prev_target_vel_ = prev_target;
    }

    uint8_t seq = 0;
    bool attempted = false;
    bool sent_ok = false;
    if (target_serial_ && target_serial_->isOpen()) {
        attempted = true;
        sent_ok = target_serial_->sendPose(rc26_decision::CommandID::POSE_TARGET, protected_vel.vx, protected_vel.vy,
                                           protected_vel.wz, seq);
    }

    if (attempted) {
        auto& stats = target_stats_;
        if (sent_ok) {
            if (stats.last_valid) {
                const uint8_t delta = static_cast<uint8_t>(seq - stats.last_ok_seq);
                if (delta > 1U) {
                    stats.missing_total += static_cast<uint64_t>(delta - 1U);
                    stats.missing_1s += static_cast<uint64_t>(delta - 1U);
                }
            }
            stats.last_ok_seq = seq;
            stats.last_valid = true;
            ++stats.ok_total;
            ++stats.ok_1s;
        } else {
            ++stats.fail_total;
            ++stats.fail_1s;
        }
        const auto target_now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(target_now - stats.window_start).count() >= 1.0) {
            if (config_.stats_log_enable) {
                RCLCPP_INFO_THROTTLE(
                    node_.get_logger(), *node_.get_clock(), 1000,
                    "[PoseSender/target] ok=%llu fail=%llu miss=%llu (1s: ok=%llu fail=%llu miss=%llu)",
                    static_cast<unsigned long long>(stats.ok_total),
                    static_cast<unsigned long long>(stats.fail_total),
                    static_cast<unsigned long long>(stats.missing_total),
                    static_cast<unsigned long long>(stats.ok_1s),
                    static_cast<unsigned long long>(stats.fail_1s),
                    static_cast<unsigned long long>(stats.missing_1s));
            }
            if (stats.fail_1s > 1U || stats.missing_1s > 1U) {
                RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
                                     "[PoseSender/target] 链路异常: fail=%llu miss=%llu",
                                     static_cast<unsigned long long>(stats.fail_1s),
                                     static_cast<unsigned long long>(stats.missing_1s));
            }
            stats.ok_1s = 0;
            stats.fail_1s = 0;
            stats.missing_1s = 0;
            stats.window_start = target_now;
        }
    }

    geometry_msgs::msg::TwistStamped target_msg;
    target_msg.header.stamp = node_.now();
    target_msg.twist.linear.x = protected_vel.vx;
    target_msg.twist.linear.y = protected_vel.vy;
    target_msg.twist.angular.z = protected_vel.wz;
    target_protected_pub_->publish(target_msg);

    std_msgs::msg::Bool spike_msg;
    spike_msg.data = imuSpikeActive();
    imu_spike_pub_->publish(spike_msg);
}

}  // namespace rc26_merge_odom
