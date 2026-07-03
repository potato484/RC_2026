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

#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"

namespace rc26_odom_interface {

// OdomInterfaceNode 节点
// 作用：将 Point-LIO 输出的里程计 / 点云，转换成导航栈统一使用的坐标系与话题
// Layer A 职责：生产链中权威发布自动链的 odom -> base_footprint 与 base_footprint -> base_link，
// 向 Layer B 持续供给 odom 与 registered_scan。
//  - 输入:
//      * state_estimation_topic_ : lidar_odom 坐标系下的里程计 (Point-LIO 输出)
//      * registered_scan_topic_  : lidar_odom 坐标系下的点云
//  - 输出:
//      * 话题 odom               : odom -> base_frame 的里程计；默认 child_frame_id=base_footprint
//      * 话题 registered_scan    : odom 坐标系下的点云
//  - 坐标链路约定:
//      * 自动导航链约定为 map -> odom -> base_footprint -> base_link -> laser_link
//  - 时间同步约束:
//      * 使用 max_time_diff_sec_ 限制点云与里程计的严重失配
//      * 对毫秒级回调抖动做小容差吸收，避免边界值误丢云
class OdomInterfaceNode : public rclcpp::Node {
public:
    explicit OdomInterfaceNode(const rclcpp::NodeOptions& options);

private:
    struct OdomHistoryLookupResult {
        bool has_history{false};
        bool has_match{false};
        rclcpp::Time closest_stamp;
        double closest_signed_diff_sec{0.0};
        bool cloud_ahead_of_latest{false};
    };

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

    void publishRegisteredCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg,
                                const rclcpp::Time& output_stamp,
                                const tf2::Transform& tf_input_odom_to_output_odom);

    void publishBootstrapPose();

    void storeOdometryStampLocked(const rclcpp::Time& odom_stamp);

    OdomHistoryLookupResult lookupOdometryStampLocked(const rclcpp::Time& stamp, double max_abs_diff_sec) const;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr odom_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr odom_pose_markers_pub_;
    rclcpp::TimerBase::SharedPtr bootstrap_pose_timer_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string state_estimation_topic_;
    std::string registered_scan_topic_;
    std::string odom_frame_;
    std::string input_body_frame_;
    std::string base_frame_;
    std::string base_link_frame_;
    double base_link_height_above_base_footprint_m_{0.2};

    // TF-style pose of input_body_frame in base_link coordinates.
    tf2::Transform tf_base_to_input_body_;
    std::mutex transform_mutex_;
    rclcpp::Time latest_odometry_stamp_;
    bool odom_pose_ready_{false};
    bool use_input_twist_{true};
    bool zero_origin_to_first_frame_{true};
    bool odom_origin_initialized_{false};
    int zero_origin_warmup_frames_{10};
    int zero_origin_accumulated_frames_{0};
    double zero_origin_max_linear_speed_mps_{0.05};
    double zero_origin_max_angular_speed_radps_{0.10};
    bool debug_pose_log_{false};
    double debug_pose_log_interval_sec_{1.0};
    double max_time_diff_sec_{0.2};        // 点云与里程计之间允许的最大时间差 (秒)
    bool clamp_cloud_stamp_to_latest_odom_{true};  // 防止输出点云时间戳超前于已发布 odom
    bool defer_cloud_until_matching_odom_{true};  // 点云超前时先缓存，等待对应 odom 到达后再发布
    bool publish_debug_path_{true};
    bool publish_pose_markers_{true};
    bool publish_bootstrap_pose_{true};
    bool bootstrap_yaw_locked_{false};
    double bootstrap_pose_rate_hz_{20.0};
    double bootstrap_yaw_rad_{0.0};
    size_t odom_stamp_history_size_{128};
    size_t pending_cloud_queue_size_{8};
    int cloud_queue_size_{5};
    tf2::Transform tf_input_odom_to_output_odom_;  // 首帧位姿归零: 将 Point-LIO odom 对齐到输出基座首帧原点和 yaw
    tf2::Vector3 zero_origin_translation_sum_{0.0, 0.0, 0.0};
    double zero_origin_yaw_sin_sum_{0.0};
    double zero_origin_yaw_cos_sum_{0.0};
    nav_msgs::msg::Path odom_path_msg_;
    std::deque<rclcpp::Time> odometry_stamp_history_;
    std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pending_clouds_;

    // [C2 修复] 速度估计所需的状态变量
    struct OdomState {
        tf2::Transform previous_transform;
        rclcpp::Time previous_stamp;
        bool initialized{false};
    } odom_state_;
};

}  // namespace rc26_odom_interface
