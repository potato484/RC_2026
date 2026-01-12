// RC2026 串口轮式里程计模块
// 从串口读取MCU四轮实际速度，计算并发布轮式里程计
#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

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
        std::string odom_topic = "Can_Odom";
        std::string odom_frame = "odom";
        std::string base_frame = "base_link";
        double data_timeout_ms = 100.0;
    };

    WheelOdom(rclcpp::Node& node,
              std::shared_ptr<rc26_decision::SerialDriver> serial,
              Config config);
    ~WheelOdom();

    WheelOdom(const WheelOdom&) = delete;
    WheelOdom& operator=(const WheelOdom&) = delete;

    void getPose(double& x, double& y, double& yaw) const;
    void getVelocity(double& vx, double& vy, double& omega) const;
    void reset();

private:
    void publishOdometry();
    void wheelSpeedsToBodyVelocity(double v_fl, double v_rl, double v_rr, double v_fr,
                                   double& vx, double& vy, double& omega) const;
    void handleOdomData(const std::vector<uint8_t>& payload);

    rclcpp::Node& node_;
    Config config_;
    std::shared_ptr<rc26_decision::SerialDriver> serial_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

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

    std::chrono::steady_clock::time_point last_publish_time_;
};

}  // namespace rc26_merge_odom
