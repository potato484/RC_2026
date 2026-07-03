// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// Maintained by DongXuan Chen <2220362462@qq.com>

#include "rc26_odom_interface/odom_interface.hpp"

#include <algorithm>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rc26_odom_interface {

namespace {

constexpr size_t kMaxPathPoses = 2000;
constexpr double kCloudOdomStampToleranceSec = 0.005;  // 吸收 ROS 回调调度造成的毫秒级时间戳抖动

bool sameStamp(const builtin_interfaces::msg::Time& lhs, const builtin_interfaces::msg::Time& rhs) {
    return lhs.sec == rhs.sec && lhs.nanosec == rhs.nanosec;
}

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

template <typename ArrayT>
bool allFinite(const ArrayT& values) {
    return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

bool isFinite(const geometry_msgs::msg::Point& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool isFinite(const geometry_msgs::msg::Quaternion& quaternion) {
    return std::isfinite(quaternion.x) && std::isfinite(quaternion.y) && std::isfinite(quaternion.z) &&
           std::isfinite(quaternion.w);
}

bool isFinite(const geometry_msgs::msg::Vector3& vector) {
    return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

bool hasUsableQuaternion(const geometry_msgs::msg::Quaternion& quaternion) {
    if (!isFinite(quaternion)) {
        return false;
    }

    const double norm_sq = quaternion.x * quaternion.x + quaternion.y * quaternion.y + quaternion.z * quaternion.z +
                           quaternion.w * quaternion.w;
    return norm_sq > 1e-12;
}

bool isFiniteVector3(const std::vector<double>& values) {
    return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

bool isUsableOdometry(const nav_msgs::msg::Odometry& msg) {
    return isFinite(msg.pose.pose.position) && hasUsableQuaternion(msg.pose.pose.orientation) &&
           isFinite(msg.twist.twist.linear) && isFinite(msg.twist.twist.angular) && allFinite(msg.pose.covariance) &&
           allFinite(msg.twist.covariance);
}

Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& vector) {
    Eigen::Matrix3d skew = Eigen::Matrix3d::Zero();
    skew << 0.0, -vector.z(), vector.y(), vector.z(), 0.0, -vector.x(), -vector.y(), vector.x(), 0.0;
    return skew;
}

struct BaseFrameSplitResult {
    tf2::Transform odom_to_base_frame;
    tf2::Transform base_frame_to_base_link;
    bool split_enabled{false};
};

BaseFrameSplitResult splitBaseLinkTransform(const tf2::Transform& odom_to_base_link,
                                            double base_link_height_above_base_footprint_m,
                                            bool split_enabled) {
    BaseFrameSplitResult result;
    result.odom_to_base_frame = odom_to_base_link;
    result.base_frame_to_base_link.setIdentity();
    result.split_enabled = split_enabled;

    if (!split_enabled) {
        return result;
    }

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(odom_to_base_link.getRotation()).getRPY(roll, pitch, yaw);

    result.odom_to_base_frame.setOrigin(tf2::Vector3(odom_to_base_link.getOrigin().x(), odom_to_base_link.getOrigin().y(),
                                                     odom_to_base_link.getOrigin().z() -
                                                         base_link_height_above_base_footprint_m));
    tf2::Quaternion yaw_only_rotation;
    yaw_only_rotation.setRPY(0.0, 0.0, yaw);
    yaw_only_rotation.normalize();
    result.odom_to_base_frame.setRotation(yaw_only_rotation);

    result.base_frame_to_base_link.setOrigin(tf2::Vector3(0.0, 0.0, base_link_height_above_base_footprint_m));
    tf2::Quaternion roll_pitch_rotation;
    roll_pitch_rotation.setRPY(roll, pitch, 0.0);
    roll_pitch_rotation.normalize();
    result.base_frame_to_base_link.setRotation(roll_pitch_rotation);

    return result;
}

}  // namespace

OdomInterfaceNode::OdomInterfaceNode(const rclcpp::NodeOptions& options) : Node("odom_interface", options) {
    // Layer A 生产桥：本节点是自动链中 odom -> base_footprint 与 base_footprint -> base_link 的唯一权威发布者。
    this->declare_parameter<std::string>("state_estimation_topic", "");
    this->declare_parameter<std::string>("registered_scan_topic", "");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "");
    this->declare_parameter<std::string>("base_link_frame", "base_link");
    this->declare_parameter<std::string>("input_body_frame", "");
    this->declare_parameter<double>("base_link_height_above_base_footprint_m", 0.2);
    this->declare_parameter<std::vector<double>>("base_to_input_body_xyz_m", std::vector<double>{});
    this->declare_parameter<std::vector<double>>("base_to_input_body_rpy_rad", std::vector<double>{});
    this->declare_parameter<double>("max_time_diff_sec", 0.2);
    this->declare_parameter<bool>("clamp_cloud_stamp_to_latest_odom", true);
    this->declare_parameter<bool>("defer_cloud_until_matching_odom", true);
    this->declare_parameter<bool>("publish_debug_path", true);
    this->declare_parameter<bool>("publish_pose_markers", true);
    this->declare_parameter<bool>("publish_bootstrap_pose", true);
    this->declare_parameter<double>("bootstrap_pose_rate_hz", 20.0);
    this->declare_parameter<int>("odom_stamp_history_size", 128);
    this->declare_parameter<int>("pending_cloud_queue_size", 8);
    this->declare_parameter<int>("cloud_queue_size", 5);
    this->declare_parameter<bool>("use_input_twist", true);
    this->declare_parameter<bool>("zero_origin_to_first_frame", true);
    this->declare_parameter<int>("zero_origin_warmup_frames", 10);
    this->declare_parameter<double>("zero_origin_max_linear_speed_mps", 0.05);
    this->declare_parameter<double>("zero_origin_max_angular_speed_radps", 0.10);
    this->declare_parameter<bool>("debug_pose_log", false);
    this->declare_parameter<double>("debug_pose_log_interval_sec", 1.0);

    this->get_parameter("state_estimation_topic", state_estimation_topic_);
    this->get_parameter("registered_scan_topic", registered_scan_topic_);
    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("base_link_frame", base_link_frame_);
    this->get_parameter("input_body_frame", input_body_frame_);
    this->get_parameter("base_link_height_above_base_footprint_m", base_link_height_above_base_footprint_m_);
    std::vector<double> base_to_input_body_xyz_m;
    std::vector<double> base_to_input_body_rpy_rad;
    this->get_parameter("base_to_input_body_xyz_m", base_to_input_body_xyz_m);
    this->get_parameter("base_to_input_body_rpy_rad", base_to_input_body_rpy_rad);
    this->get_parameter("max_time_diff_sec", max_time_diff_sec_);
    this->get_parameter("clamp_cloud_stamp_to_latest_odom", clamp_cloud_stamp_to_latest_odom_);
    this->get_parameter("defer_cloud_until_matching_odom", defer_cloud_until_matching_odom_);
    this->get_parameter("publish_debug_path", publish_debug_path_);
    this->get_parameter("publish_pose_markers", publish_pose_markers_);
    this->get_parameter("publish_bootstrap_pose", publish_bootstrap_pose_);
    this->get_parameter("bootstrap_pose_rate_hz", bootstrap_pose_rate_hz_);
    int odom_stamp_history_size_param = 128;
    int pending_cloud_queue_size_param = 8;
    this->get_parameter("odom_stamp_history_size", odom_stamp_history_size_param);
    this->get_parameter("pending_cloud_queue_size", pending_cloud_queue_size_param);
    this->get_parameter("cloud_queue_size", cloud_queue_size_);
    this->get_parameter("use_input_twist", use_input_twist_);
    this->get_parameter("zero_origin_to_first_frame", zero_origin_to_first_frame_);
    this->get_parameter("zero_origin_warmup_frames", zero_origin_warmup_frames_);
    this->get_parameter("zero_origin_max_linear_speed_mps", zero_origin_max_linear_speed_mps_);
    this->get_parameter("zero_origin_max_angular_speed_radps", zero_origin_max_angular_speed_radps_);
    this->get_parameter("debug_pose_log", debug_pose_log_);
    this->get_parameter("debug_pose_log_interval_sec", debug_pose_log_interval_sec_);

    auto require_non_empty = [&](const char* name, const std::string& value) {
        if (value.empty()) {
            throw std::runtime_error(std::string("未配置参数: ") + name + "，请检查 odom_interface.yaml");
        }
    };
    require_non_empty("state_estimation_topic", state_estimation_topic_);
    require_non_empty("registered_scan_topic", registered_scan_topic_);
    require_non_empty("base_frame", base_frame_);
    require_non_empty("base_link_frame", base_link_frame_);
    require_non_empty("input_body_frame", input_body_frame_);

    if (max_time_diff_sec_ < 0.0) {
        RCLCPP_WARN(this->get_logger(), "max_time_diff_sec=%.3f 非法，已禁用点云时间差守卫", max_time_diff_sec_);
        max_time_diff_sec_ = 0.0;
    }
    if (zero_origin_warmup_frames_ < 1) {
        RCLCPP_WARN(this->get_logger(), "zero_origin_warmup_frames=%d 非法，已钳制为 1", zero_origin_warmup_frames_);
        zero_origin_warmup_frames_ = 1;
    }
    if (zero_origin_max_linear_speed_mps_ < 0.0) {
        RCLCPP_WARN(this->get_logger(), "zero_origin_max_linear_speed_mps=%.3f 非法，已钳制为 0.0",
                    zero_origin_max_linear_speed_mps_);
        zero_origin_max_linear_speed_mps_ = 0.0;
    }
    if (zero_origin_max_angular_speed_radps_ < 0.0) {
        RCLCPP_WARN(this->get_logger(), "zero_origin_max_angular_speed_radps=%.3f 非法，已钳制为 0.0",
                    zero_origin_max_angular_speed_radps_);
        zero_origin_max_angular_speed_radps_ = 0.0;
    }
    if (debug_pose_log_interval_sec_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "debug_pose_log_interval_sec=%.3f 非法，已回退为 1.0",
                    debug_pose_log_interval_sec_);
        debug_pose_log_interval_sec_ = 1.0;
    }
    if (!std::isfinite(bootstrap_pose_rate_hz_) || bootstrap_pose_rate_hz_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "bootstrap_pose_rate_hz=%.3f 非法，已回退为 20.0",
                    bootstrap_pose_rate_hz_);
        bootstrap_pose_rate_hz_ = 20.0;
    }
    if (odom_stamp_history_size_param < 16) {
        RCLCPP_WARN(this->get_logger(), "odom_stamp_history_size=%d 过小，已钳制为 16",
                    odom_stamp_history_size_param);
        odom_stamp_history_size_param = 16;
    }
    if (pending_cloud_queue_size_param < 1) {
        RCLCPP_WARN(this->get_logger(), "pending_cloud_queue_size=%d 非法，已钳制为 1",
                    pending_cloud_queue_size_param);
        pending_cloud_queue_size_param = 1;
    }
    if (cloud_queue_size_ < 1) {
        RCLCPP_WARN(this->get_logger(), "cloud_queue_size=%d 非法，已钳制为 1", cloud_queue_size_);
        cloud_queue_size_ = 1;
    }
    odom_stamp_history_size_ = static_cast<size_t>(odom_stamp_history_size_param);
    pending_cloud_queue_size_ = static_cast<size_t>(pending_cloud_queue_size_param);
    if (!std::isfinite(base_link_height_above_base_footprint_m_)) {
        throw std::runtime_error("base_link_height_above_base_footprint_m 必须是有限数值");
    }
    if (base_link_height_above_base_footprint_m_ < 0.0) {
        throw std::runtime_error("base_link_height_above_base_footprint_m 不能为负数");
    }

    tf_input_odom_to_output_odom_.setIdentity();
    zero_origin_translation_sum_.setZero();
    zero_origin_yaw_sin_sum_ = 0.0;
    zero_origin_yaw_cos_sum_ = 0.0;

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if (!isFiniteVector3(base_to_input_body_xyz_m)) {
        throw std::runtime_error("base_to_input_body_xyz_m 必须全部是有限数值");
    }
    if (!isFiniteVector3(base_to_input_body_rpy_rad)) {
        throw std::runtime_error("base_to_input_body_rpy_rad 必须全部是有限数值");
    }
    if (base_to_input_body_xyz_m.size() != 3U) {
        throw std::runtime_error("base_to_input_body_xyz_m 必须是长度为 3 的数组");
    }
    if (base_to_input_body_rpy_rad.size() != 3U) {
        throw std::runtime_error("base_to_input_body_rpy_rad 必须是长度为 3 的数组");
    }

    tf_base_to_input_body_.setOrigin(
        tf2::Vector3(base_to_input_body_xyz_m[0], base_to_input_body_xyz_m[1], base_to_input_body_xyz_m[2]));
    tf2::Quaternion rotation;
    rotation.setRPY(base_to_input_body_rpy_rad[0], base_to_input_body_rpy_rad[1], base_to_input_body_rpy_rad[2]);
    rotation.normalize();
    tf_base_to_input_body_.setRotation(rotation);

    pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", cloud_queue_size_);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", cloud_queue_size_);
    if (publish_debug_path_) {
        odom_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("odom_path", 5);
    }
    if (publish_pose_markers_) {
        odom_pose_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("odom_pose_markers", 5);
    }
    odom_path_msg_.header.frame_id = odom_frame_;

    pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        registered_scan_topic_, cloud_queue_size_,
        std::bind(&OdomInterfaceNode::pointCloudCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        state_estimation_topic_, cloud_queue_size_,
        std::bind(&OdomInterfaceNode::odometryCallback, this, std::placeholders::_1));

    if (publish_bootstrap_pose_) {
        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / bootstrap_pose_rate_hz_));
        bootstrap_pose_timer_ = this->create_wall_timer(period, [this]() { publishBootstrapPose(); });
    }

    RCLCPP_INFO(this->get_logger(), "debug publishers: odom_path=%s, odom_pose_markers=%s",
                publish_debug_path_ ? "enabled" : "disabled",
                publish_pose_markers_ ? "enabled" : "disabled");
    RCLCPP_INFO(this->get_logger(),
                "启动位姿与缓存: bootstrap=%s@%.1fHz, cloud_queue=%d, pending_cloud_queue=%zu, odom_history=%zu",
                publish_bootstrap_pose_ ? "enabled" : "disabled", bootstrap_pose_rate_hz_, cloud_queue_size_,
                pending_cloud_queue_size_, odom_stamp_history_size_);
    if (base_frame_ == base_link_frame_) {
        RCLCPP_WARN(this->get_logger(),
                    "base_frame 与 base_link_frame 同名 (%s)，将回退为单边 odom -> %s 兼容模式",
                    base_frame_.c_str(), base_frame_.c_str());
    }
    RCLCPP_INFO(this->get_logger(),
                "已注入内部 body 外参: base_frame=%s, base_link_frame=%s, input_body_frame=%s, base_link_height=%.3f m, "
                "xyz=(%.3f, %.3f, %.3f), rpy=(%.3f, %.3f, %.3f)",
                base_frame_.c_str(), base_link_frame_.c_str(), input_body_frame_.c_str(),
                base_link_height_above_base_footprint_m_, base_to_input_body_xyz_m[0], base_to_input_body_xyz_m[1],
                base_to_input_body_xyz_m[2], base_to_input_body_rpy_rad[0], base_to_input_body_rpy_rad[1],
                base_to_input_body_rpy_rad[2]);
}

void OdomInterfaceNode::storeOdometryStampLocked(const rclcpp::Time& odom_stamp) {
    const auto stamp_ns = odom_stamp.nanoseconds();
    auto it = std::lower_bound(odometry_stamp_history_.begin(), odometry_stamp_history_.end(), stamp_ns,
                               [](const rclcpp::Time& stamp, int64_t target_ns) {
                                   return stamp.nanoseconds() < target_ns;
                               });
    if (it != odometry_stamp_history_.end() && it->nanoseconds() == stamp_ns) {
        *it = odom_stamp;
    } else {
        odometry_stamp_history_.insert(it, odom_stamp);
    }

    while (odometry_stamp_history_.size() > odom_stamp_history_size_) {
        odometry_stamp_history_.pop_front();
    }
}

OdomInterfaceNode::OdomHistoryLookupResult OdomInterfaceNode::lookupOdometryStampLocked(
    const rclcpp::Time& stamp, double max_abs_diff_sec) const {
    OdomHistoryLookupResult result;
    if (odometry_stamp_history_.empty()) {
        return result;
    }

    result.has_history = true;
    result.cloud_ahead_of_latest = (stamp - odometry_stamp_history_.back()).seconds() > kCloudOdomStampToleranceSec;

    const auto stamp_ns = stamp.nanoseconds();
    auto it = std::lower_bound(odometry_stamp_history_.begin(), odometry_stamp_history_.end(), stamp_ns,
                               [](const rclcpp::Time& history_stamp, int64_t target_ns) {
                                   return history_stamp.nanoseconds() < target_ns;
                               });

    bool has_candidate = false;
    auto update_best = [&](const rclcpp::Time& candidate_stamp) {
        const double signed_diff_sec = (stamp - candidate_stamp).seconds();
        const double abs_diff_sec = std::abs(signed_diff_sec);
        if (!has_candidate || abs_diff_sec < std::abs(result.closest_signed_diff_sec) ||
            (abs_diff_sec == std::abs(result.closest_signed_diff_sec) && candidate_stamp > result.closest_stamp)) {
            has_candidate = true;
            result.closest_stamp = candidate_stamp;
            result.closest_signed_diff_sec = signed_diff_sec;
        }
    };

    if (it != odometry_stamp_history_.end()) {
        update_best(*it);
    }
    if (it != odometry_stamp_history_.begin()) {
        update_best(*std::prev(it));
    }

    if (!has_candidate) {
        return result;
    }

    result.has_match = std::abs(result.closest_signed_diff_sec) <= max_abs_diff_sec;
    return result;
}

void OdomInterfaceNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    // Point-LIO 输出的点云已经是 odom frame 下的，直接转发
    // 当前配置: Point-LIO odom_frame == odom_interface odom_frame_ == "odom"
    if (msg->header.frame_id != odom_frame_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "输入点云 frame_id (%s) != odom_frame (%s)，丢弃点云", msg->header.frame_id.c_str(),
                             odom_frame_.c_str());
        return;
    }

    const rclcpp::Time cloud_stamp(msg->header.stamp);
    const double guarded_max_diff_sec =
        max_time_diff_sec_ > 0.0 ? (max_time_diff_sec_ + kCloudOdomStampToleranceSec) : std::numeric_limits<double>::infinity();

    rclcpp::Time latest_stamp;
    bool odom_ready = false;
    bool odom_origin_ready = false;
    tf2::Transform tf_input_odom_to_output_odom;
    OdomHistoryLookupResult odom_lookup;
    {
        std::lock_guard<std::mutex> lock(transform_mutex_);
        latest_stamp = latest_odometry_stamp_;
        odom_ready = odom_pose_ready_;
        odom_origin_ready = !zero_origin_to_first_frame_ || odom_origin_initialized_;
        tf_input_odom_to_output_odom = tf_input_odom_to_output_odom_;
        odom_lookup = lookupOdometryStampLocked(cloud_stamp, guarded_max_diff_sec);
    }

    if (!odom_ready) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "尚未收到里程计，丢弃点云");
        return;
    }
    if (!odom_origin_ready) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "首帧原点尚未建立，丢弃点云");
        return;
    }

    const bool has_latest_stamp = latest_stamp.nanoseconds() > 0;
    const double signed_diff_sec = has_latest_stamp ? (cloud_stamp - latest_stamp).seconds() : 0.0;
    const double abs_diff_sec = std::abs(signed_diff_sec);

    // 点云超前于最新 odom 时，优先进入等待队列，避免 future-stamp 点云提前发布。
    if (has_latest_stamp && signed_diff_sec > kCloudOdomStampToleranceSec) {
        if (defer_cloud_until_matching_odom_) {
            std::lock_guard<std::mutex> lock(transform_mutex_);
            if (!pending_clouds_.empty() && sameStamp(pending_clouds_.back()->header.stamp, msg->header.stamp)) {
                pending_clouds_.back() = msg;
                return;
            }
            pending_clouds_.push_back(msg);
            if (pending_clouds_.size() > pending_cloud_queue_size_) {
                const auto dropped_stamp = rclcpp::Time(pending_clouds_.front()->header.stamp);
                pending_clouds_.pop_front();
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 2000,
                    "pending registered_scan 队列已满，丢弃最旧点云 (stamp=%.3f, depth=%zu)",
                    dropped_stamp.seconds(), pending_clouds_.size());
            }
            return;
        }

        if (max_time_diff_sec_ > 0.0 && abs_diff_sec > guarded_max_diff_sec) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "点云超前最新 odom %.3f s，超过 %.3f s 限制且未启用等待匹配，丢弃点云",
                                 signed_diff_sec, max_time_diff_sec_);
            return;
        }
    }

    // 点云未超前于最新 odom 时，不再只和 latest_stamp 比较，而是到最近一段 odom 历史里找最近帧，
    // 这样即使 odom 回调更早到、latest 已经推进到下一帧，也不会误把当前点云当成“严重落后”。
    if (max_time_diff_sec_ > 0.0 && !odom_lookup.has_match) {
        const double history_abs_diff_sec =
            odom_lookup.has_history ? std::abs(odom_lookup.closest_signed_diff_sec) : abs_diff_sec;
        if (signed_diff_sec < -kCloudOdomStampToleranceSec && history_abs_diff_sec > guarded_max_diff_sec) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "点云与最近 odom 时间差 %.3f s，超过 %.3f s 限制，丢弃点云 (cloud=%.3f, nearest_odom=%.3f, latest_odom=%.3f)",
                history_abs_diff_sec, max_time_diff_sec_, cloud_stamp.seconds(),
                odom_lookup.has_history ? odom_lookup.closest_stamp.seconds() : 0.0, latest_stamp.seconds());
            return;
        }
    }

    rclcpp::Time output_stamp = cloud_stamp;
    if (clamp_cloud_stamp_to_latest_odom_ && latest_stamp.nanoseconds() > 0 && output_stamp > latest_stamp) {
        const double lead_sec = (output_stamp - latest_stamp).seconds();
        output_stamp = latest_stamp;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "registered_scan 时间戳超前 odom %.3f s，已钳制到最新 odom 时间", lead_sec);
    }
    publishRegisteredCloud(msg, output_stamp, tf_input_odom_to_output_odom);
}

void OdomInterfaceNode::publishRegisteredCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
                                               const rclcpp::Time& output_stamp,
                                               const tf2::Transform& tf_input_odom_to_output_odom) {
    if (zero_origin_to_first_frame_) {
        sensor_msgs::msg::PointCloud2 out;
        try {
            pcl_ros::transformPointCloud(odom_frame_, tf_input_odom_to_output_odom, *msg, out);
        } catch (const std::exception& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "首帧归零后的点云变换失败: %s", ex.what());
            return;
        }
        out.header.stamp = output_stamp;
        out.header.frame_id = odom_frame_;
        pcd_pub_->publish(out);
        return;
    }

    sensor_msgs::msg::PointCloud2 out = *msg;
    out.header.stamp = output_stamp;
    pcd_pub_->publish(out);
}

void OdomInterfaceNode::publishBootstrapPose() {
    bool should_publish = false;
    double yaw = 0.0;
    {
        std::lock_guard<std::mutex> lock(transform_mutex_);
        should_publish = !odom_pose_ready_;
        yaw = bootstrap_yaw_rad_;
    }
    if (!should_publish) {
        if (bootstrap_pose_timer_) {
            bootstrap_pose_timer_->cancel();
        }
        return;
    }

    tf2::Quaternion yaw_rotation;
    yaw_rotation.setRPY(0.0, 0.0, yaw);
    yaw_rotation.normalize();

    tf2::Transform tf_odom_to_base;
    tf_odom_to_base.setIdentity();
    tf_odom_to_base.setRotation(yaw_rotation);

    nav_msgs::msg::Odometry out;
    out.header.stamp = this->now();
    out.header.frame_id = odom_frame_;
    out.child_frame_id = base_frame_;
    out.pose.pose.orientation = tf2::toMsg(yaw_rotation);
    out.pose.covariance.fill(0.0);
    out.twist.covariance.fill(0.0);
    odom_pub_->publish(out);

    std::vector<geometry_msgs::msg::TransformStamped> tf_msgs;
    geometry_msgs::msg::TransformStamped tf_base_frame_msg;
    tf_base_frame_msg.header = out.header;
    tf_base_frame_msg.child_frame_id = base_frame_;
    tf_base_frame_msg.transform = tf2::toMsg(tf_odom_to_base);
    tf_msgs.emplace_back(std::move(tf_base_frame_msg));

    if (base_frame_ != base_link_frame_) {
        tf2::Transform tf_base_to_base_link;
        tf_base_to_base_link.setIdentity();
        tf_base_to_base_link.setOrigin(tf2::Vector3(0.0, 0.0, base_link_height_above_base_footprint_m_));

        geometry_msgs::msg::TransformStamped tf_base_link_msg;
        tf_base_link_msg.header.stamp = out.header.stamp;
        tf_base_link_msg.header.frame_id = base_frame_;
        tf_base_link_msg.child_frame_id = base_link_frame_;
        tf_base_link_msg.transform = tf2::toMsg(tf_base_to_base_link);
        tf_msgs.emplace_back(std::move(tf_base_link_msg));
    }

    tf_broadcaster_->sendTransform(tf_msgs);
}

void OdomInterfaceNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    if (msg->header.frame_id != odom_frame_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "输入里程计 header.frame_id (%s) != odom_frame (%s)，丢弃该帧",
                             msg->header.frame_id.c_str(), odom_frame_.c_str());
        return;
    }
    if (msg->child_frame_id != input_body_frame_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "输入里程计 child_frame_id (%s) != input_body_frame (%s)，丢弃该帧",
                             msg->child_frame_id.c_str(), input_body_frame_.c_str());
        return;
    }
    if (!isUsableOdometry(*msg)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "输入里程计存在非有限值或非法四元数，丢弃该帧");
        return;
    }

    // NOTE: Input odometry message is based on the Point-LIO internal body frame.
    // The input body frame origin is at the sensor/body initial position.
    // We need to transform it to align with our odom frame and split the automatic chain into:
    // odom -> base_footprint -> base_link -> input_body.
    const rclcpp::Time odom_stamp(msg->header.stamp);
    const bool split_base_frame = base_frame_ != base_link_frame_;
    const tf2::Transform tf_base_to_input_body = tf_base_to_input_body_;
    const tf2::Transform tf_input_body_to_base_link = tf_base_to_input_body.inverse();

    // Input: input_body_odom -> input_body (from Point-LIO)
    // We first recover odom -> base_link, then split it into:
    // odom -> base_footprint -> base_link for the automatic navigation chain.
    // Transform chain:
    //   T_odom_base = T_odom_input_body * inverse(T_base_input_body)
    tf2::Transform tf_input_body_odom_to_input_body;
    tf2::fromMsg(msg->pose.pose, tf_input_body_odom_to_input_body);

    // Compute the raw Point-LIO odom -> base_link transform before splitting.
    tf2::Transform tf_input_odom_to_base_link = tf_input_body_odom_to_input_body * tf_input_body_to_base_link;

    {
        std::lock_guard<std::mutex> lock(transform_mutex_);
        if (!bootstrap_yaw_locked_) {
            double roll = 0.0;
            double pitch = 0.0;
            double yaw = 0.0;
            tf2::Matrix3x3(tf_input_odom_to_base_link.getRotation()).getRPY(roll, pitch, yaw);
            bootstrap_yaw_rad_ = zero_origin_to_first_frame_ ? 0.0 : yaw;
            bootstrap_yaw_locked_ = true;
        }
    }

    if (zero_origin_to_first_frame_) {
        std::lock_guard<std::mutex> lock(transform_mutex_);
        if (!odom_origin_initialized_) {
            const tf2::Vector3 linear_velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                                               msg->twist.twist.linear.z);
            const tf2::Vector3 angular_velocity(msg->twist.twist.angular.x, msg->twist.twist.angular.y,
                                                msg->twist.twist.angular.z);
            const bool stationary = linear_velocity.length() <= zero_origin_max_linear_speed_mps_ &&
                                    angular_velocity.length() <= zero_origin_max_angular_speed_radps_;

            if (!stationary) {
                if (zero_origin_accumulated_frames_ > 0) {
                    zero_origin_accumulated_frames_ = 0;
                    zero_origin_translation_sum_.setZero();
                    zero_origin_yaw_sin_sum_ = 0.0;
                    zero_origin_yaw_cos_sum_ = 0.0;
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "启动归零阶段检测到运动，重置静止均值累计");
                }
                return;
            }

            const auto zero_origin_reference =
                splitBaseLinkTransform(tf_input_odom_to_base_link, base_link_height_above_base_footprint_m_,
                                       split_base_frame)
                    .odom_to_base_frame;
            double zero_roll = 0.0;
            double zero_pitch = 0.0;
            double zero_yaw = 0.0;
            tf2::Matrix3x3(zero_origin_reference.getRotation()).getRPY(zero_roll, zero_pitch, zero_yaw);
            zero_origin_translation_sum_ += zero_origin_reference.getOrigin();
            zero_origin_yaw_sin_sum_ += std::sin(zero_yaw);
            zero_origin_yaw_cos_sum_ += std::cos(zero_yaw);
            ++zero_origin_accumulated_frames_;

            if (zero_origin_accumulated_frames_ < zero_origin_warmup_frames_) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "静止归零累计中: %d/%d 帧", zero_origin_accumulated_frames_,
                                     zero_origin_warmup_frames_);
                return;
            }

            const tf2::Vector3 averaged_origin =
                zero_origin_translation_sum_ / static_cast<double>(zero_origin_accumulated_frames_);
            const double averaged_yaw = std::atan2(zero_origin_yaw_sin_sum_, zero_origin_yaw_cos_sum_);
            tf2::Quaternion zero_yaw_rotation;
            zero_yaw_rotation.setRPY(0.0, 0.0, -averaged_yaw);
            zero_yaw_rotation.normalize();
            tf2::Transform zero_origin_to_output_odom;
            zero_origin_to_output_odom.setIdentity();
            zero_origin_to_output_odom.setRotation(zero_yaw_rotation);
            zero_origin_to_output_odom.setOrigin(tf2::quatRotate(zero_yaw_rotation, -averaged_origin));
            tf_input_odom_to_output_odom_.setIdentity();
            tf_input_odom_to_output_odom_ = zero_origin_to_output_odom;
            odom_origin_initialized_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "已建立 odom 静止均值归零: frames=%d, input_origin=(%.3f, %.3f, %.3f), input_yaw=%.3f rad",
                        zero_origin_accumulated_frames_, averaged_origin.x(), averaged_origin.y(), averaged_origin.z(),
                        averaged_yaw);
        }
        tf_input_odom_to_base_link = tf_input_odom_to_output_odom_ * tf_input_odom_to_base_link;
    }

    const auto base_frame_split =
        splitBaseLinkTransform(tf_input_odom_to_base_link, base_link_height_above_base_footprint_m_, split_base_frame);
    const tf2::Transform& tf_odom_to_output_base = base_frame_split.odom_to_base_frame;

    if (bootstrap_pose_timer_) {
        bootstrap_pose_timer_->cancel();
    }

    nav_msgs::msg::Odometry out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = odom_frame_;
    out.child_frame_id = base_frame_;

    const auto& origin = tf_odom_to_output_base.getOrigin();
    out.pose.pose.position.x = origin.x();
    out.pose.pose.position.y = origin.y();
    out.pose.pose.position.z = origin.z();
    out.pose.pose.orientation = tf2::toMsg(tf_odom_to_output_base.getRotation());
    out.pose.covariance = msg->pose.covariance;
    std::fill(out.twist.covariance.begin(), out.twist.covariance.end(), 0.0);

    // Publish TFs so downstream visualization / localization / Nav2 can resolve:
    // odom -> base_footprint and, when split is enabled, base_footprint -> base_link.
    std::vector<geometry_msgs::msg::TransformStamped> tf_msgs;
    geometry_msgs::msg::TransformStamped tf_base_frame_msg;
    tf_base_frame_msg.header.stamp = msg->header.stamp;
    tf_base_frame_msg.header.frame_id = odom_frame_;
    tf_base_frame_msg.child_frame_id = base_frame_;
    tf_base_frame_msg.transform = tf2::toMsg(tf_odom_to_output_base);
    tf_msgs.emplace_back(std::move(tf_base_frame_msg));

    if (base_frame_split.split_enabled) {
        geometry_msgs::msg::TransformStamped tf_base_link_msg;
        tf_base_link_msg.header.stamp = msg->header.stamp;
        tf_base_link_msg.header.frame_id = base_frame_;
        tf_base_link_msg.child_frame_id = base_link_frame_;
        tf_base_link_msg.transform = tf2::toMsg(base_frame_split.base_frame_to_base_link);
        tf_msgs.emplace_back(std::move(tf_base_link_msg));
    }
    tf_broadcaster_->sendTransform(tf_msgs);

    // [P3] 速度来源可切换：优先使用 Point-LIO 输入 twist，必要时回退到差分估计
    bool update_state = true;
    std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pending_clouds_to_publish;
    tf2::Transform tf_input_odom_to_output_odom_snapshot;
    const double guarded_max_diff_sec =
        max_time_diff_sec_ > 0.0 ? (max_time_diff_sec_ + kCloudOdomStampToleranceSec) : std::numeric_limits<double>::infinity();
    {
        std::lock_guard<std::mutex> lock(transform_mutex_);
        if (!odom_pose_ready_ || odom_stamp > latest_odometry_stamp_) {
            latest_odometry_stamp_ = odom_stamp;
        }
        odom_pose_ready_ = true;
        tf_input_odom_to_output_odom_snapshot = tf_input_odom_to_output_odom_;
        storeOdometryStampLocked(odom_stamp);

        if (use_input_twist_) {
            const tf2::Vector3 r_input_body_in_base = tf_base_to_input_body.getOrigin();
            const tf2::Quaternion rotation_input_body_to_base = tf_base_to_input_body.getRotation();
            const tf2::Vector3 v_input_body(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                                            msg->twist.twist.linear.z);
            const tf2::Vector3 w_input_body(msg->twist.twist.angular.x, msg->twist.twist.angular.y,
                                            msg->twist.twist.angular.z);

            const tf2::Vector3 w_base = tf2::quatRotate(rotation_input_body_to_base, w_input_body);
            const tf2::Vector3 v_base =
                tf2::quatRotate(rotation_input_body_to_base, v_input_body) - w_base.cross(r_input_body_in_base);

            const Eigen::Quaterniond rotation_input_body_to_base_eigen(rotation_input_body_to_base.w(),
                                                                       rotation_input_body_to_base.x(),
                                                                       rotation_input_body_to_base.y(),
                                                                       rotation_input_body_to_base.z());
            const Eigen::Matrix3d R = rotation_input_body_to_base_eigen.toRotationMatrix();
            const Eigen::Vector3d r(r_input_body_in_base.x(), r_input_body_in_base.y(), r_input_body_in_base.z());
            Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
            J.topLeftCorner<3, 3>() = R;
            J.topRightCorner<3, 3>() = -skewSymmetric(r) * R;
            J.bottomRightCorner<3, 3>() = R;
            Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> sigma_in(msg->twist.covariance.data());
            Eigen::Matrix<double, 6, 6> sigma_output = Eigen::Matrix<double, 6, 6>::Zero();
            if (sigma_in.allFinite()) {
                const Eigen::Matrix<double, 6, 6> sigma_base = J * sigma_in * J.transpose();
                if (base_frame_split.split_enabled) {
                    const tf2::Vector3 r_base_frame_to_base_link = base_frame_split.base_frame_to_base_link.getOrigin();
                    const tf2::Quaternion rotation_base_link_to_base_frame =
                        base_frame_split.base_frame_to_base_link.getRotation();
                    const Eigen::Quaterniond rotation_base_link_to_base_frame_eigen(
                        rotation_base_link_to_base_frame.w(), rotation_base_link_to_base_frame.x(),
                        rotation_base_link_to_base_frame.y(), rotation_base_link_to_base_frame.z());
                    const Eigen::Matrix3d R_base_link_to_base_frame =
                        rotation_base_link_to_base_frame_eigen.toRotationMatrix();
                    const Eigen::Vector3d r_base_frame(r_base_frame_to_base_link.x(), r_base_frame_to_base_link.y(),
                                                       r_base_frame_to_base_link.z());
                    Eigen::Matrix<double, 6, 6> J_base_to_base_frame = Eigen::Matrix<double, 6, 6>::Zero();
                    J_base_to_base_frame.topLeftCorner<3, 3>() = R_base_link_to_base_frame;
                    J_base_to_base_frame.topRightCorner<3, 3>() =
                        -skewSymmetric(r_base_frame) * R_base_link_to_base_frame;
                    J_base_to_base_frame.bottomRightCorner<3, 3>() = R_base_link_to_base_frame;
                    sigma_output = J_base_to_base_frame * sigma_base * J_base_to_base_frame.transpose();
                    sigma_output.row(3).setZero();
                    sigma_output.col(3).setZero();
                    sigma_output.row(4).setZero();
                    sigma_output.col(4).setZero();
                } else {
                    sigma_output = sigma_base;
                }
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Input twist covariance contains non-finite values, publish zero covariance instead");
            }
            Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> sigma_out(out.twist.covariance.data());
            sigma_out = sigma_output;

            if (base_frame_split.split_enabled) {
                const tf2::Vector3 linear_base_odom =
                    tf2::quatRotate(tf_input_odom_to_base_link.getRotation(), v_base);
                const tf2::Vector3 angular_base_odom =
                    tf2::quatRotate(tf_input_odom_to_base_link.getRotation(), w_base);
                const tf2::Vector3 r_base_frame_to_base_link_odom = tf2::quatRotate(
                    tf_odom_to_output_base.getRotation(), base_frame_split.base_frame_to_base_link.getOrigin());
                const tf2::Vector3 linear_base_frame_odom =
                    linear_base_odom - angular_base_odom.cross(r_base_frame_to_base_link_odom);
                const tf2::Quaternion rotation_odom_to_base_frame = tf_odom_to_output_base.getRotation().inverse();
                const tf2::Vector3 linear_base_frame =
                    tf2::quatRotate(rotation_odom_to_base_frame, linear_base_frame_odom);
                const tf2::Vector3 angular_base_frame =
                    tf2::quatRotate(rotation_odom_to_base_frame, angular_base_odom);

                out.twist.twist.linear.x = linear_base_frame.x();
                out.twist.twist.linear.y = linear_base_frame.y();
                out.twist.twist.linear.z = linear_base_frame.z();
                out.twist.twist.angular.x = 0.0;
                out.twist.twist.angular.y = 0.0;
                out.twist.twist.angular.z = angular_base_frame.z();
            } else {
                out.twist.twist.linear.x = v_base.x();
                out.twist.twist.linear.y = v_base.y();
                out.twist.twist.linear.z = v_base.z();
                out.twist.twist.angular.x = w_base.x();
                out.twist.twist.angular.y = w_base.y();
                out.twist.twist.angular.z = w_base.z();
            }
        } else if (odom_state_.initialized) {
            const double dt = (odom_stamp - odom_state_.previous_stamp).seconds();

            if (dt <= 0.0) {
                update_state = false;
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "里程计时间戳非递增 (dt=%.6f)，跳过速度/状态更新", dt);
            } else if (dt > 1e-6 && dt < 1.0) {
                // 线速度 = 位移差 / 时间差（odom frame 下）
                const tf2::Vector3 linear_velocity_odom =
                    (tf_odom_to_output_base.getOrigin() - odom_state_.previous_transform.getOrigin()) / dt;

                // 将线速度从 odom frame 变换到输出基座 frame
                const tf2::Vector3 linear_velocity_base =
                    tf2::quatRotate(tf_odom_to_output_base.getRotation().inverse(), linear_velocity_odom);

                // 角速度：使用输出基座 frame 下的旋转差 q_prev^{-1} * q_current
                tf2::Quaternion q_diff =
                    odom_state_.previous_transform.getRotation().inverse() * tf_odom_to_output_base.getRotation();
                q_diff.normalize();

                // 确保取最短路径
                if (q_diff.w() < 0) {
                    q_diff = tf2::Quaternion(-q_diff.x(), -q_diff.y(), -q_diff.z(), -q_diff.w());
                }

                const double angle = q_diff.getAngle();

                out.twist.twist.linear.x = linear_velocity_base.x();
                out.twist.twist.linear.y = linear_velocity_base.y();
                out.twist.twist.linear.z = linear_velocity_base.z();

                // 角度接近零时，getAxis() 不稳定，直接输出零角速度
                if (angle > 1e-6) {
                    const tf2::Vector3 axis = q_diff.getAxis();
                    out.twist.twist.angular.x = axis.x() * angle / dt;
                    out.twist.twist.angular.y = axis.y() * angle / dt;
                    out.twist.twist.angular.z = axis.z() * angle / dt;
                } else {
                    out.twist.twist.angular.x = 0.0;
                    out.twist.twist.angular.y = 0.0;
                    out.twist.twist.angular.z = 0.0;
                }
                if (base_frame_split.split_enabled) {
                    out.twist.twist.angular.x = 0.0;
                    out.twist.twist.angular.y = 0.0;
                }
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "跳过速度估计: dt=%.3f s 超出范围 (1e-6, 1.0)，请检查里程计帧率", dt);
            }
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "差分回退链路等待首帧初始化，当前输出零速度");
        }

        // 更新状态
        if (update_state) {
            odom_state_.previous_transform = tf_odom_to_output_base;
            odom_state_.previous_stamp = odom_stamp;
            odom_state_.initialized = true;
        }

        while (!pending_clouds_.empty()) {
            const auto& pending_cloud = pending_clouds_.front();
            const rclcpp::Time pending_stamp(pending_cloud->header.stamp);
            const auto pending_lookup = lookupOdometryStampLocked(pending_stamp, guarded_max_diff_sec);

            if (pending_lookup.has_match && !pending_lookup.cloud_ahead_of_latest) {
                pending_clouds_to_publish.emplace_back(pending_cloud);
                pending_clouds_.pop_front();
                continue;
            }

            if (!pending_lookup.has_history || pending_lookup.cloud_ahead_of_latest) {
                break;
            }

            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "pending registered_scan 在 odom 历史窗口内未找到匹配帧，已丢弃 (cloud=%.3f, nearest_odom=%.3f, diff=%.3f)",
                pending_stamp.seconds(), pending_lookup.closest_stamp.seconds(), pending_lookup.closest_signed_diff_sec);
            pending_clouds_.pop_front();
        }
    }

    for (const auto& pending_cloud_to_publish : pending_clouds_to_publish) {
        publishRegisteredCloud(pending_cloud_to_publish, rclcpp::Time(pending_cloud_to_publish->header.stamp),
                               tf_input_odom_to_output_odom_snapshot);
    }

    odom_pub_->publish(out);

    if (publish_debug_path_ && odom_path_pub_) {
        geometry_msgs::msg::PoseStamped odom_pose;
        odom_pose.header = out.header;
        odom_pose.pose = out.pose.pose;
        odom_path_msg_.header.stamp = out.header.stamp;
        odom_path_msg_.header.frame_id = out.header.frame_id;
        odom_path_msg_.poses.emplace_back(std::move(odom_pose));
        if (odom_path_msg_.poses.size() > kMaxPathPoses) {
            odom_path_msg_.poses.erase(odom_path_msg_.poses.begin());
        }
        odom_path_pub_->publish(odom_path_msg_);
    }
    if (publish_pose_markers_ && odom_pose_markers_pub_) {
        odom_pose_markers_pub_->publish(
            makePoseMarkerArray(out.header.frame_id, out.header.stamp, "odom_pose", out.pose.pose, 0.86F, 0.24F,
                                0.24F, "ODOM", 0.22));
    }

    if (debug_pose_log_) {
        const auto throttle_ms = static_cast<int64_t>(debug_pose_log_interval_sec_ * 1000.0);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), throttle_ms,
                             "ODOM pose: p=(%.3f, %.3f, %.3f) v=(%.3f, %.3f, %.3f) w=(%.3f, %.3f, %.3f) path_size=%zu",
                             out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z,
                             out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z,
                             out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z,
                             odom_path_msg_.poses.size());
    }
}

}  // namespace rc26_odom_interface

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rc26_odom_interface::OdomInterfaceNode)
