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

#include "rc26_odom_interface/odom_interface.hpp"

#include <cmath>
#include <stdexcept>

#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace rc26_odom_interface
{

OdomInterfaceNode::OdomInterfaceNode(const rclcpp::NodeOptions & options)
: Node("odom_interface", options)
{
  this->declare_parameter<std::string>("state_estimation_topic", "");
  this->declare_parameter<std::string>("registered_scan_topic", "");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<std::string>("lidar_frame", "");
  this->declare_parameter<double>("tf_lookup_timeout_sec", 0.5);
  this->declare_parameter<double>("max_time_diff_sec", 0.2);
  this->declare_parameter<double>("tf_refresh_interval_sec", 1.0);

  this->get_parameter("state_estimation_topic", state_estimation_topic_);
  this->get_parameter("registered_scan_topic", registered_scan_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("tf_lookup_timeout_sec", tf_timeout_sec_);
  this->get_parameter("max_time_diff_sec", max_time_diff_sec_);
  this->get_parameter("tf_refresh_interval_sec", tf_refresh_interval_sec_);

  auto require_non_empty = [&](const char * name, const std::string & value) {
    if (value.empty()) {
      throw std::runtime_error(std::string("未配置参数: ") + name + "，请检查 odom_interface.yaml");
    }
  };
  require_non_empty("state_estimation_topic", state_estimation_topic_);
  require_non_empty("registered_scan_topic", registered_scan_topic_);
  require_non_empty("base_frame", base_frame_);
  require_non_empty("lidar_frame", lidar_frame_);

  base_frame_to_lidar_initialized_ = false;
  tf_odom_to_lidar_.setIdentity();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 5);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 5);

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    registered_scan_topic_, 5,
    std::bind(&OdomInterfaceNode::pointCloudCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    state_estimation_topic_, 5,
    std::bind(&OdomInterfaceNode::odometryCallback, this, std::placeholders::_1));
}

void OdomInterfaceNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // Point-LIO 输出的点云已经是 odom frame 下的，直接转发
  // 当前配置: Point-LIO odom_frame == odom_interface odom_frame_ == "odom"
  pcd_pub_->publish(*msg);
}

void OdomInterfaceNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  // NOTE: Input odometry message is based on the `lidar_odom` (Point-LIO output)
  // The lidar_odom frame origin is at the lidar's initial position
  // We need to transform it to align with our odom frame (base_link initial position)
  // Transform chain: odom -> base_link -> lidar_link (static), then apply lidar_odom pose
  const rclcpp::Time odom_stamp(msg->header.stamp);
  const bool need_tf_refresh =
    !base_frame_to_lidar_initialized_ ||
    (tf_refresh_interval_sec_ > 0.0 &&
    (last_tf_lookup_.nanoseconds() == 0 ||
    (odom_stamp - last_tf_lookup_).seconds() > tf_refresh_interval_sec_));

  if (need_tf_refresh) {
    // ===== 基座与雷达的静态 TF 可能在运行时断开，这里按配置周期重新拉取 =====
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, odom_stamp,
        rclcpp::Duration::from_seconds(tf_timeout_sec_));
      tf2::Transform tf_base_frame_to_lidar;
      tf2::fromMsg(tf_stamped.transform, tf_base_frame_to_lidar);
      tf_lidar_to_base_ = tf_base_frame_to_lidar.inverse();
      base_frame_to_lidar_initialized_ = true;
      last_tf_lookup_ = msg->header.stamp;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "TF %s -> %s 刷新成功", base_frame_.c_str(), lidar_frame_.c_str());
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "TF %s -> %s 查询失败: %s，使用上次有效TF继续发布", 
        base_frame_.c_str(), lidar_frame_.c_str(), ex.what());
      // [修复] TF查询失败时不直接return，使用上次有效的TF继续发布
      // 只有在从未成功初始化过时才return
      if (!base_frame_to_lidar_initialized_) {
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "TF从未成功初始化，无法发布odom");
        return;
      }
    }
  }

  // Input: lidar_odom -> lidar (from Point-LIO)
  // We want: odom -> base_link
  // Transform chain:
  //   T_odom_base = T_lidar_odom_lidar * T_lidar_base
  //   where T_lidar_base = tf_lidar_to_base_ (computed once at init)
  tf2::Transform tf_lidar_odom_to_lidar;
  tf2::fromMsg(msg->pose.pose, tf_lidar_odom_to_lidar);

  // Compute odom -> base_link transform
  tf2::Transform tf_odom_to_base = tf_lidar_odom_to_lidar * tf_lidar_to_base_;

  const tf2::Transform tf_base_to_lidar = tf_lidar_to_base_.inverse();
  const tf2::Transform tf_odom_to_lidar = tf_odom_to_base * tf_base_to_lidar;

  {
    std::lock_guard<std::mutex> lock(transform_mutex_);
    latest_odometry_stamp_ = msg->header.stamp;
    odom_pose_ready_ = true;
    tf_odom_to_lidar_ = tf_odom_to_lidar;
  }

  nav_msgs::msg::Odometry out;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = odom_frame_;
  out.child_frame_id = base_frame_;

  const auto & origin = tf_odom_to_base.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(tf_odom_to_base.getRotation());

  // [C3 修复] 计算速度信息（含奇异性保护，速度输出到 base_link 坐标系）
  if (odom_state_.initialized) {
    const double dt = (odom_stamp - odom_state_.previous_stamp).seconds();

    if (dt > 1e-6 && dt < 1.0) {
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
    }
  }
  
  // 更新状态
  odom_state_.previous_transform = tf_odom_to_base;
  odom_state_.previous_stamp = odom_stamp;
  odom_state_.initialized = true;

  odom_pub_->publish(out);
}

}  // namespace rc26_odom_interface

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rc26_odom_interface::OdomInterfaceNode)
