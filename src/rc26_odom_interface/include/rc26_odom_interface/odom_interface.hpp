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

#include <memory>
#include <mutex>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace rc26_odom_interface {

// OdomInterfaceNode 节点
// 作用：将 Point-LIO 输出的里程计 / 点云，转换成导航栈统一使用的坐标系与话题
//  - 输入:
//      * state_estimation_topic_ : lidar_odom 坐标系下的里程计 (Point-LIO 输出)
//      * registered_scan_topic_  : lidar_odom 坐标系下的点云
//  - 输出:
//      * 话题 odom               : odom -> base_link 的里程计
//      * 话题 registered_scan    : odom 坐标系下的点云
//  - 坐标链路约定:
//      * 全局约定为 map -> odom -> base_link -> laser_link
//  - 时间同步约束:
//      * 使用 max_time_diff_sec_ 限制点云与里程计的时间差，防止严重对不齐
class OdomInterfaceNode : public rclcpp::Node {
public:
    explicit OdomInterfaceNode(const rclcpp::NodeOptions& options);

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string state_estimation_topic_;
    std::string registered_scan_topic_;
    std::string odom_frame_;
    std::string lidar_frame_;
    std::string base_frame_;

    bool base_frame_to_lidar_initialized_;
    tf2::Transform tf_base_to_lidar_;  // T_{lidar<-base}: 将 base_link 点变换到 lidar 坐标系
    std::mutex transform_mutex_;
    rclcpp::Time latest_odometry_stamp_;
    rclcpp::Time last_tf_lookup_;
    bool odom_pose_ready_{false};
    bool use_input_twist_{true};
    double tf_timeout_sec_{0.5};           // TF 查询超时时间 (秒)
    double max_time_diff_sec_{0.2};        // 点云与里程计之间允许的最大时间差 (秒)
    double tf_refresh_interval_sec_{1.0};  // TF 断连时的重新拉取周期 (秒)

    // [C2 修复] 速度估计所需的状态变量
    struct OdomState {
        tf2::Transform previous_transform;
        rclcpp::Time previous_stamp;
        bool initialized{false};
    } odom_state_;
};

}  // namespace rc26_odom_interface
