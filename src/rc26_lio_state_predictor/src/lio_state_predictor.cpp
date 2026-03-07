#include "rc26_lio_state_predictor/lio_state_predictor.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <functional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/create_timer.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rc26_lio_state_predictor {

namespace {

constexpr size_t kMaxPathPoses = 4000;
constexpr double kControlPathMinDtSec = 0.05;

visualization_msgs::msg::MarkerArray makePoseMarkerArray(const std::string& frame_id,
                                                         const builtin_interfaces::msg::Time& stamp,
                                                         const std::string& ns_prefix,
                                                         const geometry_msgs::msg::Pose& pose,
                                                         float red,
                                                         float green,
                                                         float blue,
                                                         const std::string& label,
                                                         double text_z_offset) {
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker sphere_marker;
    sphere_marker.header.frame_id = frame_id;
    sphere_marker.header.stamp = stamp;
    sphere_marker.ns = ns_prefix;
    sphere_marker.id = 0;
    sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
    sphere_marker.action = visualization_msgs::msg::Marker::ADD;
    sphere_marker.pose = pose;
    sphere_marker.scale.x = 0.08;
    sphere_marker.scale.y = 0.08;
    sphere_marker.scale.z = 0.08;
    sphere_marker.color.r = red;
    sphere_marker.color.g = green;
    sphere_marker.color.b = blue;
    sphere_marker.color.a = 0.95F;
    marker_array.markers.emplace_back(std::move(sphere_marker));

    visualization_msgs::msg::Marker arrow_marker;
    arrow_marker.header.frame_id = frame_id;
    arrow_marker.header.stamp = stamp;
    arrow_marker.ns = ns_prefix;
    arrow_marker.id = 1;
    arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
    arrow_marker.action = visualization_msgs::msg::Marker::ADD;
    arrow_marker.pose = pose;
    arrow_marker.scale.x = 0.32;
    arrow_marker.scale.y = 0.05;
    arrow_marker.scale.z = 0.05;
    arrow_marker.color.r = red;
    arrow_marker.color.g = green;
    arrow_marker.color.b = blue;
    arrow_marker.color.a = 0.95F;
    marker_array.markers.emplace_back(std::move(arrow_marker));

    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = frame_id;
    text_marker.header.stamp = stamp;
    text_marker.ns = ns_prefix;
    text_marker.id = 2;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position = pose.position;
    text_marker.pose.position.z += text_z_offset;
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.16;
    text_marker.color.r = red;
    text_marker.color.g = green;
    text_marker.color.b = blue;
    text_marker.color.a = 1.0F;
    text_marker.text = label;
    marker_array.markers.emplace_back(std::move(text_marker));

    return marker_array;
}

bool isFiniteVector3(const Eigen::Vector3d& value) {
    return value.array().isFinite().all();
}

bool isFiniteQuaternion(const Eigen::Quaterniond& value) {
    return std::isfinite(value.w()) && std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

}  // namespace

LioStatePredictorNode::LioStatePredictorNode(const rclcpp::NodeOptions& options)
    : Node("lio_state_predictor", options) {
    this->declare_parameter<std::string>("odometry_topic", "odometry");
    this->declare_parameter<std::string>("imu_topic", "livox/imu");
    this->declare_parameter<std::string>("degenerate_score_topic", "degenerate_score");
    this->declare_parameter<std::string>("control_state_topic", "control_state");
    this->declare_parameter<std::string>("control_degenerate_score_topic", "control_degenerate_score");
    this->declare_parameter<std::string>("control_degraded_topic", "control_degraded");
    this->declare_parameter<bool>("publish_control_tf", false);
    this->declare_parameter<std::string>("control_tf_child_frame", "base_link_control");
    this->declare_parameter<double>("publish_rate_hz", 200.0);
    this->declare_parameter<double>("fixed_predict_ahead_sec", 0.0);
    this->declare_parameter<double>("pos_cov_increase_m2_per_s", 0.02);
    this->declare_parameter<double>("yaw_cov_increase_rad2_per_s", 0.01);
    this->declare_parameter<double>("degeneracy_ratio_threshold", 0.02);
    this->declare_parameter<double>("imu_timeout_sec", 0.05);
    this->declare_parameter<double>("degenerate_score_timeout_sec", 0.2);
    this->declare_parameter<double>("max_prediction_horizon_sec", 0.2);
    this->declare_parameter<double>("startup_no_prediction_sec", 1.0);
    this->declare_parameter<bool>("debug_pose_log", false);
    this->declare_parameter<double>("debug_pose_log_interval_sec", 1.0);

    this->get_parameter("odometry_topic", odometry_topic_);
    this->get_parameter("imu_topic", imu_topic_);
    this->get_parameter("degenerate_score_topic", degenerate_score_topic_);
    this->get_parameter("control_state_topic", control_state_topic_);
    this->get_parameter("control_degenerate_score_topic", control_degenerate_score_topic_);
    this->get_parameter("control_degraded_topic", control_degraded_topic_);
    this->get_parameter("publish_control_tf", publish_control_tf_);
    this->get_parameter("control_tf_child_frame", control_tf_child_frame_);
    this->get_parameter("publish_rate_hz", publish_rate_hz_);
    this->get_parameter("fixed_predict_ahead_sec", fixed_predict_ahead_sec_);
    this->get_parameter("pos_cov_increase_m2_per_s", pos_cov_increase_m2_per_s_);
    this->get_parameter("yaw_cov_increase_rad2_per_s", yaw_cov_increase_rad2_per_s_);
    this->get_parameter("degeneracy_ratio_threshold", degeneracy_ratio_threshold_);
    this->get_parameter("imu_timeout_sec", imu_timeout_sec_);
    this->get_parameter("degenerate_score_timeout_sec", degenerate_score_timeout_sec_);
    this->get_parameter("max_prediction_horizon_sec", max_prediction_horizon_sec_);
    this->get_parameter("startup_no_prediction_sec", startup_no_prediction_sec_);
    this->get_parameter("debug_pose_log", debug_pose_log_);
    this->get_parameter("debug_pose_log_interval_sec", debug_pose_log_interval_sec_);

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
    if (imu_timeout_sec_ < 0.0) {
        imu_timeout_sec_ = 0.0;
    }
    if (degenerate_score_timeout_sec_ < 0.0) {
        degenerate_score_timeout_sec_ = 0.0;
    }
    if (max_prediction_horizon_sec_ < 0.0) {
        max_prediction_horizon_sec_ = 0.0;
    }
    if (startup_no_prediction_sec_ < 0.0) {
        startup_no_prediction_sec_ = 0.0;
    }
    if (debug_pose_log_interval_sec_ <= 0.0) {
        debug_pose_log_interval_sec_ = 1.0;
    }
    node_start_time_ = this->get_clock()->now();

    control_state_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(control_state_topic_, 20);
    control_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("control_path", 5);
    control_pose_markers_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("control_pose_markers", 5);
    control_path_msg_.header.frame_id = "odom";
    control_degenerate_score_pub_ = this->create_publisher<std_msgs::msg::Float64>(control_degenerate_score_topic_, 20);
    control_degraded_pub_ = this->create_publisher<std_msgs::msg::Bool>(control_degraded_topic_, 20);
    if (publish_control_tf_) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

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
    latest_imu_stamp_ = rclcpp::Time(msg->header.stamp);
}

void LioStatePredictorNode::degenerateScoreCallback(const std_msgs::msg::Float64::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_degenerate_score_ = msg->data;
    has_degenerate_score_ = true;
    latest_degenerate_score_stamp_ = this->get_clock()->now();
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
    rclcpp::Time imu_stamp{0, 0, this->get_clock()->get_clock_type()};
    rclcpp::Time degenerate_score_stamp{0, 0, this->get_clock()->get_clock_type()};
    bool has_degenerate_score = false;
    double degenerate_score = 1.0;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!latest_odometry_) {
            return;
        }
        odom_msg = latest_odometry_;
        imu_msg = latest_imu_;
        imu_stamp = latest_imu_stamp_;
        has_degenerate_score = has_degenerate_score_;
        degenerate_score = latest_degenerate_score_;
        degenerate_score_stamp = latest_degenerate_score_stamp_;
    }

    nav_msgs::msg::Odometry out = *odom_msg;
    const rclcpp::Time last_odom_stamp(odom_msg->header.stamp);
    rclcpp::Time target_stamp = this->get_clock()->now();
    if (fixed_predict_ahead_sec_ > 0.0) {
        target_stamp = last_odom_stamp + rclcpp::Duration::from_seconds(fixed_predict_ahead_sec_);
    }

    const bool startup_hold = startup_no_prediction_sec_ > 0.0 &&
                              (this->get_clock()->now() - node_start_time_).seconds() < startup_no_prediction_sec_;
    if (startup_hold) {
        target_stamp = last_odom_stamp;
    }

    const double raw_dt = std::max((target_stamp - last_odom_stamp).seconds(), 0.0);
    const bool clamp_prediction_horizon = !startup_hold && fixed_predict_ahead_sec_ <= 0.0 && max_prediction_horizon_sec_ > 0.0;
    const bool odom_stale = clamp_prediction_horizon && raw_dt > max_prediction_horizon_sec_;
    const double dt = odom_stale ? max_prediction_horizon_sec_ : raw_dt;
    if (odom_stale) {
        target_stamp = last_odom_stamp + rclcpp::Duration::from_seconds(dt);
    }
    if (odom_stale) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Odometry is stale by %.3f s, clamp predictor horizon to %.3f s", raw_dt,
                             max_prediction_horizon_sec_);
    }

    Eigen::Quaterniond q_world_from_body(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                                         odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);
    if (!isFiniteQuaternion(q_world_from_body)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Skip predictor publish due to non-finite odometry quaternion");
        return;
    }
    const double quaternion_norm = q_world_from_body.norm();
    if (quaternion_norm < 1e-9) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Invalid zero-norm odometry quaternion, fallback to identity");
        q_world_from_body = Eigen::Quaterniond::Identity();
    } else {
        q_world_from_body.normalize();
    }

    Eigen::Vector3d omega_body(odom_msg->twist.twist.angular.x, odom_msg->twist.twist.angular.y,
                               odom_msg->twist.twist.angular.z);
    if (imu_msg) {
        const double imu_age_sec = std::abs((target_stamp - imu_stamp).seconds());
        const double imu_before_odom_sec = std::max((last_odom_stamp - imu_stamp).seconds(), 0.0);
        const bool imu_recent = imu_timeout_sec_ <= 0.0 || imu_age_sec <= imu_timeout_sec_;
        const bool imu_not_older_than_odom = imu_timeout_sec_ <= 0.0 || imu_before_odom_sec <= imu_timeout_sec_;
        Eigen::Vector3d imu_omega(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
        if (imu_recent && imu_not_older_than_odom && isFiniteVector3(imu_omega)) {
            omega_body = imu_omega;
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "IMU gyro unavailable/stale (age=%.3f s, before_odom=%.3f s), fallback to odom twist",
                                 imu_age_sec, imu_before_odom_sec);
        }
    }
    if (!isFiniteVector3(omega_body)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Non-finite angular velocity detected, fallback to zero angular velocity");
        omega_body.setZero();
    }

    const double omega_norm = omega_body.norm();
    if (omega_norm > 1e-9 && dt > 0.0) {
        const Eigen::Quaterniond dq(Eigen::AngleAxisd(omega_norm * dt, omega_body / omega_norm));
        q_world_from_body = (q_world_from_body * dq).normalized();
    }

    Eigen::Vector3d p_world(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
    const Eigen::Vector3d v_body(odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y,
                                 odom_msg->twist.twist.linear.z);
    if (!isFiniteVector3(p_world) || !isFiniteVector3(v_body)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Skip predictor publish due to non-finite odometry position or velocity");
        return;
    }
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

    const bool append_control_path =
        control_path_msg_.poses.empty() ||
        (rclcpp::Time(out.header.stamp) - rclcpp::Time(control_path_msg_.poses.back().header.stamp)).seconds() >=
            kControlPathMinDtSec;
    if (append_control_path) {
        geometry_msgs::msg::PoseStamped control_pose;
        control_pose.header = out.header;
        control_pose.pose = out.pose.pose;
        control_path_msg_.header.stamp = out.header.stamp;
        control_path_msg_.header.frame_id = out.header.frame_id;
        control_path_msg_.poses.emplace_back(std::move(control_pose));
        if (control_path_msg_.poses.size() > kMaxPathPoses) {
            control_path_msg_.poses.erase(control_path_msg_.poses.begin());
        }
        control_path_pub_->publish(control_path_msg_);
    }
    control_pose_markers_pub_->publish(makePoseMarkerArray(out.header.frame_id, out.header.stamp, "control_pose",
                                                           out.pose.pose, 0.24F, 0.86F, 0.24F, "CONTROL", 0.38));

    if (debug_pose_log_) {
        const auto throttle_ms = static_cast<int64_t>(debug_pose_log_interval_sec_ * 1000.0);
        const double dx = out.pose.pose.position.x - odom_msg->pose.pose.position.x;
        const double dy = out.pose.pose.position.y - odom_msg->pose.pose.position.y;
        const double dz = out.pose.pose.position.z - odom_msg->pose.pose.position.z;
        const double delta_norm = std::sqrt(dx * dx + dy * dy + dz * dz);
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), throttle_ms,
            "CONTROL vs ODOM: hold=%s raw_dt=%.3f dt=%.3f odom=(%.3f, %.3f, %.3f) control=(%.3f, %.3f, %.3f) delta=(%.3f, %.3f, %.3f | %.3f m) path_size=%zu",
            startup_hold ? "true" : "false", raw_dt, dt, odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
            odom_msg->pose.pose.position.z, out.pose.pose.position.x, out.pose.pose.position.y,
            out.pose.pose.position.z, dx, dy, dz, delta_norm, control_path_msg_.poses.size());
    }

    if (publish_control_tf_ && tf_broadcaster_ && !out.header.frame_id.empty() && !control_tf_child_frame_.empty()) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = out.header.stamp;
        tf_msg.header.frame_id = out.header.frame_id;
        tf_msg.child_frame_id = control_tf_child_frame_;
        tf_msg.transform.translation.x = out.pose.pose.position.x;
        tf_msg.transform.translation.y = out.pose.pose.position.y;
        tf_msg.transform.translation.z = out.pose.pose.position.z;
        tf_msg.transform.rotation = out.pose.pose.orientation;
        tf_broadcaster_->sendTransform(tf_msg);
    }

    const bool score_is_fresh = has_degenerate_score &&
                                (degenerate_score_timeout_sec_ <= 0.0 ||
                                 (this->get_clock()->now() - degenerate_score_stamp).seconds() <=
                                     degenerate_score_timeout_sec_);

    if (score_is_fresh && std::isfinite(degenerate_score)) {
        std_msgs::msg::Float64 score_msg;
        score_msg.data = degenerate_score;
        control_degenerate_score_pub_->publish(score_msg);
    } else if (has_degenerate_score) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Degenerate score is stale/invalid, skip control_degenerate_score publish");
    }

    std_msgs::msg::Bool degraded_msg;
    degraded_msg.data = odom_stale || (has_degenerate_score && !score_is_fresh) ||
                        (score_is_fresh && degenerate_score < degeneracy_ratio_threshold_);
    control_degraded_pub_->publish(degraded_msg);
}

}  // namespace rc26_lio_state_predictor

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rc26_lio_state_predictor::LioStatePredictorNode)
