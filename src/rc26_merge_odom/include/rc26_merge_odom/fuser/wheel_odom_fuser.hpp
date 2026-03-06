// RC2026 轮式里程计融合器
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace rc26_merge_odom {

class WheelOdomFuser {
public:
    struct Config {
        int publish_rate_hz = 50;
        double data_timeout_ms = 100.0;
        double omega_sigma_rps = 0.5;
        double chi2_threshold_dof3 = 11.34;
        double outlier_penalty = 0.1;
        double recovery_tau_s = 0.5;
        std::string can_odom_topic = "Can_Odom";
        std::string wheel_odom_topic = "wheel_odom";
        std::string imu_topic = "DM_IMU";
        std::string fused_odom_topic = "wheel_odom_fused";
        std::string health_topic = "wheel_odom_fuser/health";
    };

    WheelOdomFuser(rclcpp::Node& node, Config config);
    ~WheelOdomFuser();

    WheelOdomFuser(const WheelOdomFuser&) = delete;
    WheelOdomFuser& operator=(const WheelOdomFuser&) = delete;

private:
    struct OdomSnapshot {
        nav_msgs::msg::Odometry odom;
        std::chrono::steady_clock::time_point stamp;
        bool received = false;
    };

    struct ImuSnapshot {
        double omega_z = 0.0;
        std::chrono::steady_clock::time_point stamp;
        bool received = false;
    };

    struct SourceState {
        bool valid = false;
        double age_ms = -1.0;
        double vx = 0.0;
        double vy = 0.0;
        double wz = 0.0;
        double var_vx = 0.0;
        double var_vy = 0.0;
        double var_wz = 0.0;
        double omega_diff = 0.0;
        double h_inst = 0.0;
        double h_smooth = 0.0;
        double w_vx = 0.0;
        double w_vy = 0.0;
        double w_wz = 0.0;
        double total_weight = 0.0;
        nav_msgs::msg::Odometry odom;
    };

    void canOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void onPublishTimer();

    SourceState buildSourceState(const OdomSnapshot& snapshot, bool imu_valid, double imu_wz,
                                 const std::chrono::steady_clock::time_point& now_steady) const;
    void publishHealth(const std::string& state_text, uint8_t level, const SourceState& can_state,
                       const SourceState& wheel_state, double d2);

    double extractVariance(const std::array<double, 36>& covariance, size_t index) const;
    static double clamp01(double value);

    rclcpp::Node& node_;
    Config config_;

    mutable std::mutex mutex_;
    OdomSnapshot can_snapshot_;
    OdomSnapshot wheel_snapshot_;
    ImuSnapshot imu_snapshot_;
    double h_can_ = 0.0;
    double h_wheel_ = 0.0;
    std::chrono::steady_clock::time_point last_update_time_;
    bool has_last_update_ = false;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr can_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rc26_merge_odom
