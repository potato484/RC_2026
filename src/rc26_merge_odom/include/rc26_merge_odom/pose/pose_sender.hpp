// RC2026 速度发送模块
// 双串口架构：反馈速度 + 目标速度，用于MCU速度闭环控制
#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "rc26_serial/serial_driver.hpp"

namespace rc26_merge_odom {

class PoseSender {
public:
    struct Config {
        std::string cmd_vel_topic = "cmd_vel";
        std::string odom_topic = "merge_odom";
        std::string imu_topic = "DM_IMU";
        int feedback_send_rate_hz = 50;
        int target_send_rate_hz = 25;
        int cmd_vel_timeout_ms = 200;
        float v_max_mps = 2.0f;
        float w_max_rps = 4.0f;
        float a_max_mps2 = 15.0f;
        float alpha_max_rps2 = 40.0f;
        bool imu_gate_enable = true;
        float imu_gate_ema_alpha = 0.98f;
        float imu_gate_chi2_threshold = 6.635f;
        float accel_agree_threshold_mps2 = 3.0f;
        int spike_freeze_duration_ms = 100;
        float spike_decay_tau_s = 0.3f;
        bool governor_enable = true;
        float governor_lambda = 0.2f;
        bool dob_enable = false;
        float dob_lpf_hz = 5.0f;
        float dob_kd = 0.3f;
        bool latency_comp_enable = true;
        float latency_comp_s = 0.03f;
        std::string terrain_speed_limit_topic = "";
        int terrain_speed_limit_timeout_ms = 500;
    };

    PoseSender(rclcpp::Node& node, std::shared_ptr<rc26_decision::SerialDriver> feedback_serial,
               std::shared_ptr<rc26_decision::SerialDriver> target_serial, Config config);
    ~PoseSender();

private:
    struct Velocity {
        float vx = 0.0f;
        float vy = 0.0f;
        float wz = 0.0f;
    };

    struct ImuCache {
        float ax = 0.0f;
        float ay = 0.0f;
        float gz = 0.0f;
        std::chrono::steady_clock::time_point stamp;
        bool valid = false;
    };

    struct SeqStats {
        bool last_valid = false;
        uint8_t last_ok_seq = 0;
        uint64_t ok_total = 0;
        uint64_t fail_total = 0;
        uint64_t missing_total = 0;
        uint64_t ok_1s = 0;
        uint64_t fail_1s = 0;
        uint64_t missing_1s = 0;
        std::chrono::steady_clock::time_point window_start{};
    };

    rclcpp::Node& node_;
    std::shared_ptr<rc26_decision::SerialDriver> feedback_serial_;
    std::shared_ptr<rc26_decision::SerialDriver> target_serial_;
    Config config_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr terrain_speed_limit_sub_;
    rclcpp::TimerBase::SharedPtr feedback_timer_;
    rclcpp::TimerBase::SharedPtr target_timer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr feedback_protected_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr target_protected_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr imu_spike_pub_;

    mutable std::mutex data_mutex_;
    Velocity target_vel_;
    Velocity feedback_vel_;
    Velocity prev_feedback_vel_;
    Velocity prev_target_vel_;
    Velocity prev_odom_vel_;
    std::chrono::steady_clock::time_point last_cmd_vel_time_;
    std::chrono::steady_clock::time_point prev_feedback_output_time_;
    std::chrono::steady_clock::time_point prev_target_output_time_;
    std::chrono::steady_clock::time_point prev_odom_time_;
    std::chrono::steady_clock::time_point terrain_speed_limit_time_;
    bool cmd_vel_received_ = false;
    bool timeout_zero_sent_ = false;
    bool prev_feedback_output_valid_ = false;
    bool prev_target_output_valid_ = false;
    bool prev_odom_valid_ = false;
    bool terrain_speed_limit_received_ = false;
    bool wheel_accel_valid_ = false;
    float terrain_speed_limit_mps_ = NAN;
    float wheel_accel_mps2_ = 0.0f;

    mutable std::mutex imu_cache_mutex_;
    ImuCache cached_imu_;

    mutable std::mutex imu_gate_mutex_;
    bool imu_gate_initialized_ = false;
    float imu_gate_mean_ax_ = 0.0f;
    float imu_gate_var_ax_ = 1.0f;
    float imu_last_d2_ = 0.0f;

    mutable std::mutex imu_spike_mutex_;
    float imu_spike_scale_ = 1.0f;
    bool imu_scale_initialized_ = false;
    std::chrono::steady_clock::time_point imu_spike_deadline_;
    std::chrono::steady_clock::time_point imu_scale_last_update_;

    mutable std::mutex dob_mutex_;
    Velocity dob_hat_;
    Velocity prev_governor_output_;
    bool prev_governor_output_valid_ = false;

    SeqStats feedback_stats_;
    SeqStats target_stats_;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void terrainSpeedLimitCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void feedbackTimerCallback();
    void targetTimerCallback();

    std::chrono::milliseconds sensorFreshnessTimeout() const;
    bool hasFreshOdomLocked(const std::chrono::steady_clock::time_point& now) const;
    bool getFreshImuCache(const std::chrono::steady_clock::time_point& now, ImuCache& imu_cache) const;
    float getEffectiveVMax(const std::chrono::steady_clock::time_point& now) const;
    void applyImuSpikeScale(Velocity& velocity, const std::chrono::steady_clock::time_point& now);
    bool imuSpikeActive() const;
    Velocity applyFallbackProtect(Velocity raw, const Velocity& prev_vel, double dt, float effective_v_max) const;
    Velocity applyGovernorProtect(Velocity raw, const Velocity& prev_vel, double dt, float effective_v_max) const;
    Velocity protectVelocity(Velocity raw, Velocity& prev_vel, double dt, bool enable_dob);
};

}  // namespace rc26_merge_odom
