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

#include <array>
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include <memory>
#include <optional>
#include <string>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace rc26_sensor_scan {

class SensorScanNode : public rclcpp::Node {
public:
    explicit SensorScanNode(const rclcpp::NodeOptions& options);

private:
    void laserCloudAndOdometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr& odometry,
                                      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& laserCloud2);

    std::optional<tf2::Transform> lookupTransform(const std::string& target_frame, const std::string& source_frame,
                                                  const rclcpp::Time& stamp);

    void publishOdometry(const tf2::Transform& transform, const geometry_msgs::msg::TwistWithCovariance& twist,
                         const std::array<double, 36>& pose_covariance, const std::string& parent_frame,
                         const std::string& child_frame, const rclcpp::Time& stamp);

    std::string lidar_frame_;
    std::string base_frame_;
    std::string robot_base_frame_;
    std::string lidar_odometry_topic_;
    std::string registered_scan_topic_;
    std::string scan_topic_;
    std::string odometry_topic_;
    double max_time_diff_sec_{0.1};

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_chassis_odometry_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    double tf_timeout_sec_{0.5};  // TF 查询超时时间 (秒)

    message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> laser_cloud_sub_;

    using SyncPolicy =
        message_filters::sync_policies::ExactTime<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;
    std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

}  // namespace rc26_sensor_scan
