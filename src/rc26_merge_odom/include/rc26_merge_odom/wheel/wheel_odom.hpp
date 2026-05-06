// RC2026 串口轮式里程计模块
// 从串口读取 MCU 反馈速度并发布轮式里程计
#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>

namespace rc26_decision {
class SerialDriver;
}

namespace rc26_merge_odom {

class WheelOdom {
public:
    struct Config {
        double wheel_base = 0.62326;
        double track_width = 0.7;
        int publish_rate_hz = 50;
        std::string odom_topic = "wheel_odom";
        std::string odom_frame = "odom";
        std::string base_frame = "base_link";
        std::string imu_topic = "DM_IMU";
        double data_timeout_ms = 100.0;
        bool slip_enable = true;
        double slip_threshold = 0.5;
        double slip_k_acc = 0.5;
        double cov_nominal_v = 0.01;
        double cov_nominal_wz = 0.03;
        double cov_slip_v = 0.5;
        double cov_slip_wz = 0.5;
        double recovery_tau_s = 0.5;
    };

    WheelOdom(rclcpp::Node& node, std::shared_ptr<rc26_decision::SerialDriver> serial, Config config);
    ~WheelOdom();

    WheelOdom(const WheelOdom&) = delete;
    WheelOdom& operator=(const WheelOdom&) = delete;

    bool isReady() const noexcept { return ready_.load(std::memory_order_acquire); }

    void getPose(double& x, double& y, double& yaw) const;
    void getVelocity(double& vx, double& vy, double& omega) const;
    void reset();

private:
    struct ImuSnapshot {
        double gz = 0.0;
        double ax = 0.0;
        double ay = 0.0;
        std::chrono::steady_clock::time_point stamp;
        bool valid = false;
    };

    void publishOdometry();
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void handleOdomData(const std::vector<uint8_t>& payload);

    rclcpp::Node& node_;
    Config config_;
    std::shared_ptr<rc26_decision::SerialDriver> serial_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_score_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cov_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    std::atomic<bool> ready_{false};

    mutable std::mutex data_mutex_;
    double v_fl_ = 0.0;
    double v_rl_ = 0.0;
    double v_rr_ = 0.0;
    double v_fr_ = 0.0;
    bool data_received_ = false;
    std::chrono::steady_clock::time_point last_data_time_;

    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;
    double vx_ = 0.0;
    double vy_ = 0.0;
    double omega_ = 0.0;
    mutable std::mutex pose_mutex_;

    std::mutex imu_mutex_;
    ImuSnapshot imu_snapshot_;
    std::chrono::steady_clock::time_point prev_pub_time_;
    std::chrono::steady_clock::time_point last_slip_time_;
    bool slip_detected_ = false;
    double prev_vx_ = 0.0;
    double prev_vy_ = 0.0;
    bool prev_vel_valid_ = false;
};

}  // namespace rc26_merge_odom
