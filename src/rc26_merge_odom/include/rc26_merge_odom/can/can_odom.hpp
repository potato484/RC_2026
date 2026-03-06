// RC2026 CAN里程计模块
// 从CAN总线读取四驱麦轮电机转速，计算并发布轮式里程计
#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>

namespace rc26_merge_odom {

// CAN电调反馈报文格式 (标识符 = 0x200 + 电调ID)
// DATA[0]: 转子机械角度高8位
// DATA[1]: 转子机械角度低8位 (范围0~8191对应0~360°)
// DATA[2]: 转子转速高8位
// DATA[3]: 转子转速低8位 (单位: RPM)
// DATA[4]: 实际转矩电流高8位
// DATA[5]: 实际转矩电流低8位
// DATA[6]: 电机温度
// DATA[7]: Null

constexpr uint32_t CAN_BASE_ID = 0x200;

enum WheelID : uint8_t { FRONT_LEFT = 0, REAR_LEFT = 1, REAR_RIGHT = 2, FRONT_RIGHT = 3, WHEEL_COUNT = 4 };

struct MotorFeedback {
    uint16_t angle_raw = 0;
    int16_t rpm = 0;
    int16_t current = 0;
    uint8_t temperature = 0;
    std::chrono::steady_clock::time_point last_update;
};

class CanOdom {
public:
    struct Config {
        std::string can_interface = "can0";
        double wheel_radius = 0.07625;
        double wheel_base = 0.62326;
        double track_width = 0.7;
        double gear_ratio = 3591.0 / 187.0;
        int publish_rate_hz = 50;
        std::string odom_topic = "Can_Odom";
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

    CanOdom(rclcpp::Node& node, Config config);
    ~CanOdom();

    CanOdom(const CanOdom&) = delete;
    CanOdom& operator=(const CanOdom&) = delete;

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

    rclcpp::Node& node_;
    Config config_;

    double rpm_to_wheel_speed_factor_;

    int can_socket_ = -1;

    std::array<MotorFeedback, WHEEL_COUNT> motor_feedback_;
    mutable std::mutex feedback_mutex_;

    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;
    double vx_ = 0.0;
    double vy_ = 0.0;
    double omega_ = 0.0;
    mutable std::mutex pose_mutex_;

    std::chrono::steady_clock::time_point last_update_time_;

    std::atomic<bool> running_{false};
    std::atomic<bool> ready_{false};
    std::thread can_thread_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr slip_score_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cov_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    std::mutex imu_mutex_;
    ImuSnapshot imu_snapshot_;
    std::chrono::steady_clock::time_point last_slip_time_;
    bool slip_detected_ = false;
    double prev_vx_ = 0.0;
    double prev_vy_ = 0.0;
    bool prev_vel_valid_ = false;

    bool initCan();
    void closeCan();
    void canThreadFunc();
    void parseCanFrame(uint32_t can_id, const uint8_t* data, uint8_t len);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void publishOdometry();
    void wheelSpeedsToBodyVelocity(double v_fl, double v_rl, double v_rr, double v_fr, double& vx, double& vy,
                                   double& omega) const;
};

}  // namespace rc26_merge_odom
