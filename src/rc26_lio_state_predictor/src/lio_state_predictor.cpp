#include "rc26_lio_state_predictor/lio_state_predictor.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <functional>

#include "rclcpp/create_timer.hpp"

namespace rc26_lio_state_predictor {

LioStatePredictorNode::LioStatePredictorNode(const rclcpp::NodeOptions& options)
    : Node("lio_state_predictor", options) {
    this->declare_parameter<std::string>("odometry_topic", "odometry");
    this->declare_parameter<std::string>("imu_topic", "livox/imu");
    this->declare_parameter<std::string>("degenerate_score_topic", "degenerate_score");
    this->declare_parameter<std::string>("control_state_topic", "control_state");
    this->declare_parameter<std::string>("control_degenerate_score_topic", "control_degenerate_score");
    this->declare_parameter<std::string>("control_degraded_topic", "control_degraded");
    this->declare_parameter<double>("publish_rate_hz", 200.0);
    this->declare_parameter<double>("fixed_predict_ahead_sec", 0.0);
    this->declare_parameter<double>("pos_cov_increase_m2_per_s", 0.02);
    this->declare_parameter<double>("yaw_cov_increase_rad2_per_s", 0.01);
    this->declare_parameter<double>("degeneracy_ratio_threshold", 0.02);

    this->get_parameter("odometry_topic", odometry_topic_);
    this->get_parameter("imu_topic", imu_topic_);
    this->get_parameter("degenerate_score_topic", degenerate_score_topic_);
    this->get_parameter("control_state_topic", control_state_topic_);
    this->get_parameter("control_degenerate_score_topic", control_degenerate_score_topic_);
    this->get_parameter("control_degraded_topic", control_degraded_topic_);
    this->get_parameter("publish_rate_hz", publish_rate_hz_);
    this->get_parameter("fixed_predict_ahead_sec", fixed_predict_ahead_sec_);
    this->get_parameter("pos_cov_increase_m2_per_s", pos_cov_increase_m2_per_s_);
    this->get_parameter("yaw_cov_increase_rad2_per_s", yaw_cov_increase_rad2_per_s_);
    this->get_parameter("degeneracy_ratio_threshold", degeneracy_ratio_threshold_);

    if (publish_rate_hz_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "publish_rate_hz <= 0, fallback to 200Hz");
        publish_rate_hz_ = 200.0;
    }
    if (fixed_predict_ahead_sec_ < 0.0) {
        fixed_predict_ahead_sec_ = 0.0;
    }
    if (pos_cov_increase_m2_per_s_ < 0.0) {
        pos_cov_increase_m2_per_s_ = 0.0;
    }
    if (yaw_cov_increase_rad2_per_s_ < 0.0) {
        yaw_cov_increase_rad2_per_s_ = 0.0;
    }

    control_state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(control_state_topic_, 20);
    control_degenerate_score_pub_ = this->create_publisher<std_msgs::msg::Float64>(control_degenerate_score_topic_, 20);
    control_degraded_pub_ = this->create_publisher<std_msgs::msg::Bool>(control_degraded_topic_, 20);

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic_, 20, std::bind(&LioStatePredictorNode::odometryCallback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(), std::bind(&LioStatePredictorNode::imuCallback, this, std::placeholders::_1));
    degenerate_score_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        degenerate_score_topic_, 20,
        std::bind(&LioStatePredictorNode::degenerateScoreCallback, this, std::placeholders::_1));

    const auto timer_period = rclcpp::Duration::from_seconds(1.0 / publish_rate_hz_);
    publish_timer_ = rclcpp::create_timer(this, this->get_clock(), timer_period,
                                          std::bind(&LioStatePredictorNode::publishPredictedState, this));
}

void LioStatePredictorNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_odometry_ = msg;
}

void LioStatePredictorNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_imu_ = msg;
}

void LioStatePredictorNode::degenerateScoreCallback(const std_msgs::msg::Float64::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_degenerate_score_ = msg->data;
    has_degenerate_score_ = true;
}

void LioStatePredictorNode::inflatePoseCovariance(std::array<double, 36>& pose_covariance, double dt) const {
    const double dt_non_negative = std::max(dt, 0.0);
    const double pos_increase = pos_cov_increase_m2_per_s_ * dt_non_negative;
    pose_covariance[0] += pos_increase;
    pose_covariance[7] += pos_increase;
    pose_covariance[14] += pos_increase;
    pose_covariance[35] += yaw_cov_increase_rad2_per_s_ * dt_non_negative;
}

void LioStatePredictorNode::publishPredictedState() {
    nav_msgs::msg::Odometry::ConstSharedPtr odom_msg;
    sensor_msgs::msg::Imu::ConstSharedPtr imu_msg;
    bool has_degenerate_score = false;
    double degenerate_score = 1.0;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!latest_odometry_) {
            return;
        }
        odom_msg = latest_odometry_;
        imu_msg = latest_imu_;
        has_degenerate_score = has_degenerate_score_;
        degenerate_score = latest_degenerate_score_;
    }

    nav_msgs::msg::Odometry out = *odom_msg;
    const rclcpp::Time last_odom_stamp(odom_msg->header.stamp);
    rclcpp::Time target_stamp = this->get_clock()->now();
    if (fixed_predict_ahead_sec_ > 0.0) {
        target_stamp = last_odom_stamp + rclcpp::Duration::from_seconds(fixed_predict_ahead_sec_);
    }

    double dt = (target_stamp - last_odom_stamp).seconds();
    if (dt < 0.0) {
        dt = 0.0;
    }

    Eigen::Quaterniond q_world_from_body(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                                         odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    q_world_from_body.normalize();

    Eigen::Vector3d omega_body(odom_msg->twist.twist.angular.x, odom_msg->twist.twist.angular.y,
                               odom_msg->twist.twist.angular.z);
    if (imu_msg) {
        omega_body << imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z;
    }

    const double omega_norm = omega_body.norm();
    if (omega_norm > 1e-9 && dt > 0.0) {
        const Eigen::Quaterniond dq(Eigen::AngleAxisd(omega_norm * dt, omega_body / omega_norm));
        q_world_from_body = (q_world_from_body * dq).normalized();
    }

    Eigen::Vector3d p_world(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
    const Eigen::Vector3d v_body(odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y,
                                 odom_msg->twist.twist.linear.z);
    const Eigen::Vector3d v_world = q_world_from_body * v_body;
    p_world += v_world * dt;

    out.header.stamp = target_stamp;
    out.pose.pose.position.x = p_world.x();
    out.pose.pose.position.y = p_world.y();
    out.pose.pose.position.z = p_world.z();
    out.pose.pose.orientation.x = q_world_from_body.x();
    out.pose.pose.orientation.y = q_world_from_body.y();
    out.pose.pose.orientation.z = q_world_from_body.z();
    out.pose.pose.orientation.w = q_world_from_body.w();

    inflatePoseCovariance(out.pose.covariance, dt);
    control_state_pub_->publish(out);

    if (has_degenerate_score) {
        std_msgs::msg::Float64 score_msg;
        score_msg.data = degenerate_score;
        control_degenerate_score_pub_->publish(score_msg);
    }

    std_msgs::msg::Bool degraded_msg;
    degraded_msg.data = has_degenerate_score && degenerate_score < degeneracy_ratio_threshold_;
    control_degraded_pub_->publish(degraded_msg);
}

}  // namespace rc26_lio_state_predictor

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rc26_lio_state_predictor::LioStatePredictorNode)
