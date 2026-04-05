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
constexpr size_t kMaxOdomStampHistorySize = 128;
constexpr size_t kMaxPendingClouds = 8;
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

bool isUsableOdometry(const nav_msgs::msg::Odometry& msg) {
    return isFinite(msg.pose.pose.position) && hasUsableQuaternion(msg.pose.pose.orientation) &&
           isFinite(msg.twist.twist.linear) && isFinite(msg.twist.twist.angular) && allFinite(msg.pose.covariance) &&
           allFinite(msg.twist.covariance);
}

}  // namespace

OdomInterfaceNode::OdomInterfaceNode(const rclcpp::NodeOptions& options) : Node("odom_interface", options) {
    // Layer A 生产桥：本节点是自动链中 odom -> base_link 的唯一权威发布者。
    this->declare_parameter<std::string>("state_estimation_topic", "");
    this->declare_parameter<std::string>("registered_scan_topic", "");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "");
    this->declare_parameter<std::string>("lidar_frame", "");
    this->declare_parameter<double>("tf_lookup_timeout_sec", 0.5);
    this->declare_parameter<double>("max_time_diff_sec", 0.2);
    this->declare_parameter<double>("tf_refresh_interval_sec", 1.0);
    this->declare_parameter<bool>("clamp_cloud_stamp_to_latest_odom", true);
    this->declare_parameter<bool>("defer_cloud_until_matching_odom", true);
    this->declare_parameter<bool>("publish_debug_path", true);
    this->declare_parameter<bool>("publish_pose_markers", true);
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
    this->get_parameter("lidar_frame", lidar_frame_);
    this->get_parameter("tf_lookup_timeout_sec", tf_timeout_sec_);
    this->get_parameter("max_time_diff_sec", max_time_diff_sec_);
    this->get_parameter("tf_refresh_interval_sec", tf_refresh_interval_sec_);
    this->get_parameter("clamp_cloud_stamp_to_latest_odom", clamp_cloud_stamp_to_latest_odom_);
    this->get_parameter("defer_cloud_until_matching_odom", defer_cloud_until_matching_odom_);
    this->get_parameter("publish_debug_path", publish_debug_path_);
    this->get_parameter("publish_pose_markers", publish_pose_markers_);
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
    require_non_empty("lidar_frame", lidar_frame_);

    if (tf_timeout_sec_ < 0.0) {
        RCLCPP_WARN(this->get_logger(), "tf_lookup_timeout_sec=%.3f 非法，已钳制为 0.0", tf_timeout_sec_);
        tf_timeout_sec_ = 0.0;
    }
    if (max_time_diff_sec_ < 0.0) {
        RCLCPP_WARN(this->get_logger(), "max_time_diff_sec=%.3f 非法，已禁用点云时间差守卫", max_time_diff_sec_);
        max_time_diff_sec_ = 0.0;
    }
    if (tf_refresh_interval_sec_ < 0.0) {
        RCLCPP_WARN(this->get_logger(), "tf_refresh_interval_sec=%.3f 非法，已禁用 TF 周期刷新", tf_refresh_interval_sec_);
        tf_refresh_interval_sec_ = 0.0;
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

    base_frame_to_lidar_initialized_ = false;
    tf_input_odom_to_output_odom_.setIdentity();
    zero_origin_translation_sum_.setZero();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 5);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 5);
    if (publish_debug_path_) {
        odom_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("odom_path", 5);
    }
    if (publish_pose_markers_) {
        odom_pose_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("odom_pose_markers", 5);
    }
    odom_path_msg_.header.frame_id = odom_frame_;

    pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        registered_scan_topic_, 5, std::bind(&OdomInterfaceNode::pointCloudCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        state_estimation_topic_, 5, std::bind(&OdomInterfaceNode::odometryCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "debug publishers: odom_path=%s, odom_pose_markers=%s",
                publish_debug_path_ ? "enabled" : "disabled",
                publish_pose_markers_ ? "enabled" : "disabled");
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

    while (odometry_stamp_history_.size() > kMaxOdomStampHistorySize) {
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
            if (pending_clouds_.size() > kMaxPendingClouds) {
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

void OdomInterfaceNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    if (msg->header.frame_id != odom_frame_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "输入里程计 header.frame_id (%s) != odom_frame (%s)，丢弃该帧",
                             msg->header.frame_id.c_str(), odom_frame_.c_str());
        return;
    }
    if (msg->child_frame_id != lidar_frame_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "输入里程计 child_frame_id (%s) != lidar_frame (%s)，丢弃该帧",
                             msg->child_frame_id.c_str(), lidar_frame_.c_str());
        return;
    }
    if (!isUsableOdometry(*msg)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "输入里程计存在非有限值或非法四元数，丢弃该帧");
        return;
    }

    // NOTE: Input odometry message is based on the `lidar_odom` (Point-LIO output)
    // The lidar_odom frame origin is at the lidar's initial position
    // We need to transform it to align with our odom frame (base_link initial position)
    // Transform chain: odom -> base_link -> lidar_link (static), then apply lidar_odom pose
    const rclcpp::Time odom_stamp(msg->header.stamp);
    tf2::Transform tf_base_to_lidar;
    bool base_to_lidar_ready = false;
    bool need_tf_refresh = false;
    {
        std::lock_guard<std::mutex> lock(transform_mutex_);
        base_to_lidar_ready = base_frame_to_lidar_initialized_;
        if (base_to_lidar_ready) {
            tf_base_to_lidar = tf_base_to_lidar_;
        }
        need_tf_refresh = !base_to_lidar_ready ||
                          (tf_refresh_interval_sec_ > 0.0 &&
                           (last_tf_lookup_.nanoseconds() == 0 ||
                            (odom_stamp - last_tf_lookup_).seconds() > tf_refresh_interval_sec_));
    }

    if (need_tf_refresh) {
        // ===== 基座与雷达的静态 TF 可能在运行时断开，这里按配置周期重新拉取 =====
        try {
            auto tf_stamped = tf_buffer_->lookupTransform(base_frame_, lidar_frame_, odom_stamp,
                                                          rclcpp::Duration::from_seconds(tf_timeout_sec_));
            tf2::Transform tf_lidar_to_base;
            tf2::fromMsg(tf_stamped.transform, tf_lidar_to_base);
            tf_base_to_lidar = tf_lidar_to_base.inverse();
            {
                std::lock_guard<std::mutex> lock(transform_mutex_);
                tf_base_to_lidar_ = tf_base_to_lidar;
                base_frame_to_lidar_initialized_ = true;
                last_tf_lookup_ = odom_stamp;
            }
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "TF %s -> %s 刷新成功",
                                 base_frame_.c_str(), lidar_frame_.c_str());
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "TF %s -> %s 查询失败: %s，使用上次有效TF继续发布", base_frame_.c_str(),
                                 lidar_frame_.c_str(), ex.what());
            // [修复] TF查询失败时不直接return，使用上次有效的TF继续发布
            // 只有在从未成功初始化过时才return
            if (!base_to_lidar_ready) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF从未成功初始化，无法发布odom");
                return;
            }
        }
    }

    // Input: lidar_odom -> lidar (from Point-LIO)
    // We want: odom -> base_link
    // Transform chain:
    //   T_odom_base = T_lidar_odom_lidar * T_lidar_base
    //   where T_lidar_base = tf_base_to_lidar_ (computed once at init)
    tf2::Transform tf_lidar_odom_to_lidar;
    tf2::fromMsg(msg->pose.pose, tf_lidar_odom_to_lidar);

    // Compute raw Point-LIO odom -> base_link transform
    tf2::Transform tf_input_odom_to_base = tf_lidar_odom_to_lidar * tf_base_to_lidar;
    tf2::Transform tf_odom_to_base = tf_input_odom_to_base;

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
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "启动归零阶段检测到运动，重置静止均值累计");
                }
                return;
            }

            zero_origin_translation_sum_ += tf_input_odom_to_base.getOrigin();
            ++zero_origin_accumulated_frames_;

            if (zero_origin_accumulated_frames_ < zero_origin_warmup_frames_) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "静止归零累计中: %d/%d 帧", zero_origin_accumulated_frames_,
                                     zero_origin_warmup_frames_);
                return;
            }

            const tf2::Vector3 averaged_origin =
                zero_origin_translation_sum_ / static_cast<double>(zero_origin_accumulated_frames_);
            tf_input_odom_to_output_odom_.setIdentity();
            tf_input_odom_to_output_odom_.setOrigin(-averaged_origin);
            odom_origin_initialized_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "已建立 odom 静止均值归零: frames=%d, avg_origin=(%.3f, %.3f, %.3f)",
                        zero_origin_accumulated_frames_, averaged_origin.x(), averaged_origin.y(), averaged_origin.z());
        }
        tf_odom_to_base = tf_input_odom_to_output_odom_ * tf_input_odom_to_base;
    }

    nav_msgs::msg::Odometry out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = odom_frame_;
    out.child_frame_id = base_frame_;

    const auto& origin = tf_odom_to_base.getOrigin();
    out.pose.pose.position.x = origin.x();
    out.pose.pose.position.y = origin.y();
    out.pose.pose.position.z = origin.z();
    out.pose.pose.orientation = tf2::toMsg(tf_odom_to_base.getRotation());
    out.pose.covariance = msg->pose.covariance;
    std::fill(out.twist.covariance.begin(), out.twist.covariance.end(), 0.0);

    // Publish TF: odom -> base_link so downstream visualization/control can resolve the TF tree even if sync drops frames.
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform = tf2::toMsg(tf_odom_to_base);
    tf_broadcaster_->sendTransform(tf_msg);

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
            const tf2::Vector3 r_base_in_lidar = tf_base_to_lidar.getOrigin();
            const tf2::Quaternion rotation_lidar_to_base = tf_base_to_lidar.getRotation().inverse();
            const tf2::Vector3 v_lidar(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
            const tf2::Vector3 w_lidar(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

            const tf2::Vector3 v_base =
                tf2::quatRotate(rotation_lidar_to_base, v_lidar + w_lidar.cross(r_base_in_lidar));
            const tf2::Vector3 w_base = tf2::quatRotate(rotation_lidar_to_base, w_lidar);

            out.twist.twist.linear.x = v_base.x();
            out.twist.twist.linear.y = v_base.y();
            out.twist.twist.linear.z = v_base.z();
            out.twist.twist.angular.x = w_base.x();
            out.twist.twist.angular.y = w_base.y();
            out.twist.twist.angular.z = w_base.z();

            const Eigen::Quaterniond rotation_lidar_to_base_eigen(rotation_lidar_to_base.w(), rotation_lidar_to_base.x(),
                                                                  rotation_lidar_to_base.y(), rotation_lidar_to_base.z());
            const Eigen::Matrix3d R = rotation_lidar_to_base_eigen.toRotationMatrix();
            const Eigen::Vector3d r(r_base_in_lidar.x(), r_base_in_lidar.y(), r_base_in_lidar.z());
            Eigen::Matrix3d skew_r;
            skew_r << 0.0, -r(2), r(1), r(2), 0.0, -r(0), -r(1), r(0), 0.0;
            Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
            J.topLeftCorner<3, 3>() = R;
            J.topRightCorner<3, 3>() = -R * skew_r;
            J.bottomRightCorner<3, 3>() = R;
            Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> sigma_in(msg->twist.covariance.data());
            Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> sigma_out(out.twist.covariance.data());
            if (sigma_in.allFinite()) {
                sigma_out = J * sigma_in * J.transpose();
            } else {
                sigma_out.setZero();
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Input twist covariance contains non-finite values, publish zero covariance instead");
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
                    (tf_odom_to_base.getOrigin() - odom_state_.previous_transform.getOrigin()) / dt;

                // 将线速度从 odom frame 变换到 base_link frame
                const tf2::Vector3 linear_velocity_base =
                    tf2::quatRotate(tf_odom_to_base.getRotation().inverse(), linear_velocity_odom);

                // 角速度：使用 body frame 下的旋转差 q_prev^{-1} * q_current
                tf2::Quaternion q_diff =
                    odom_state_.previous_transform.getRotation().inverse() * tf_odom_to_base.getRotation();
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
            odom_state_.previous_transform = tf_odom_to_base;
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
