#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

namespace rc26_lio_state_predictor {

class LioStatePredictorNode : public rclcpp::Node {
public:
    explicit LioStatePredictorNode(const rclcpp::NodeOptions& options);

private:
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
    void degenerateScoreCallback(const std_msgs::msg::Float64::ConstSharedPtr msg);
    void publishPredictedState();
    void inflatePoseCovariance(std::array<double, 36>& pose_covariance, double dt) const;

    std::mutex data_mutex_;
    nav_msgs::msg::Odometry::ConstSharedPtr latest_odometry_;
    sensor_msgs::msg::Imu::ConstSharedPtr latest_imu_;
    rclcpp::Time latest_imu_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time latest_degenerate_score_stamp_{0, 0, RCL_ROS_TIME};
    bool has_degenerate_score_{false};
    double latest_degenerate_score_{1.0};

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr degenerate_score_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr control_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_degenerate_score_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr control_degraded_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::string odometry_topic_;
    std::string imu_topic_;
    std::string degenerate_score_topic_;
    std::string control_state_topic_;
    std::string control_degenerate_score_topic_;
    std::string control_degraded_topic_;

    double publish_rate_hz_{200.0};
    double fixed_predict_ahead_sec_{0.0};
    double pos_cov_increase_m2_per_s_{0.02};
    double yaw_cov_increase_rad2_per_s_{0.01};
    double degeneracy_ratio_threshold_{0.02};
    double imu_timeout_sec_{0.05};
    double degenerate_score_timeout_sec_{0.2};
    double max_prediction_horizon_sec_{0.2};
};

}  // namespace rc26_lio_state_predictor
