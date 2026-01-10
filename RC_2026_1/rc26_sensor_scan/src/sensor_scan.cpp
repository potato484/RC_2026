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

#include "rc26_sensor_scan/sensor_scan.hpp"

#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>
#include <stdexcept>

namespace rc26_sensor_scan
{

SensorScanNode::SensorScanNode(const rclcpp::NodeOptions & options)
: Node("sensor_scan", options)
{
  // ===== 参数声明：所有可调配置全部放在 YAML 中 =====
  this->declare_parameter<std::string>("lidar_frame", "");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<std::string>("robot_base_frame", "");
  this->declare_parameter<std::string>("lidar_odometry_topic", "lidar_odometry");
  this->declare_parameter<std::string>("registered_scan_topic", "registered_scan");
  this->declare_parameter<std::string>("scan_topic", "sensor_scan");
  this->declare_parameter<std::string>("odometry_topic", "odometry");
  this->declare_parameter<double>("tf_timeout_sec", 0.5);
  this->declare_parameter<double>("max_time_diff_sec", 0.1);

  // ===== 参数读取与校验，任何空参数立即报错 =====
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_odometry_topic", lidar_odometry_topic_);
  this->get_parameter("registered_scan_topic", registered_scan_topic_);
  this->get_parameter("scan_topic", scan_topic_);
  this->get_parameter("odometry_topic", odometry_topic_);
  this->get_parameter("tf_timeout_sec", tf_timeout_sec_);
  this->get_parameter("max_time_diff_sec", max_time_diff_sec_);

  auto require_non_empty = [&](const char * param_name, const std::string & value) {
    if (value.empty()) {
      throw std::runtime_error(
        std::string("未配置参数: ") + param_name +
        "，请在 sensor_scan.yaml 中补齐");
    }
  };

  require_non_empty("lidar_frame", lidar_frame_);
  require_non_empty("base_frame", base_frame_);
  require_non_empty("robot_base_frame", robot_base_frame_);
  require_non_empty("lidar_odometry_topic", lidar_odometry_topic_);
  require_non_empty("registered_scan_topic", registered_scan_topic_);
  require_non_empty("scan_topic", scan_topic_);
  require_non_empty("odometry_topic", odometry_topic_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pub_laser_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(scan_topic_, 2);
  pub_chassis_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, 2);

  rmw_qos_profile_t qos_profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

  odometry_sub_.subscribe(this, lidar_odometry_topic_, qos_profile);
  laser_cloud_sub_.subscribe(this, registered_scan_topic_, qos_profile);

  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odometry_sub_, laser_cloud_sub_);
  sync_->registerCallback(
    std::bind(
      &SensorScanNode::laserCloudAndOdometryHandler, this, std::placeholders::_1,
      std::placeholders::_2));
}

void SensorScanNode::laserCloudAndOdometryHandler(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcd_msg)
{
  // ===== 时间同步守卫：不同步的数据直接丢弃，避免 TF 抖动 =====
  const rclcpp::Time cloud_stamp(pcd_msg->header.stamp);
  const rclcpp::Time odom_stamp(odometry_msg->header.stamp);
  const double time_diff = std::fabs((cloud_stamp - odom_stamp).seconds());
  if (time_diff > max_time_diff_sec_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "点云与里程计时间差 %.3f s 超过阈值 %.3f s，丢弃该帧",
      time_diff, max_time_diff_sec_);
    return;
  }

  // NOTE: odometry_msg 来自 rc26_odom_interface，其 pose 表示 odom → base_link
  // child_frame_id = base_link，所以这里直接使用，不需要额外的坐标变换
  tf2::Transform tf_odom_to_base;
  tf2::fromMsg(odometry_msg->pose.pose, tf_odom_to_base);

  // 查询 base_link → laser_link 的静态变换（用于点云转换）
  auto tf_base_to_lidar = getTransform(base_frame_, lidar_frame_, cloud_stamp);
  if (!tf_base_to_lidar) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "TF %s -> %s unavailable, skip scan", base_frame_.c_str(), lidar_frame_.c_str());
    return;
  }

  // 发布 TF: odom → base_frame
  publishTransform(tf_odom_to_base, odometry_msg->header.frame_id, base_frame_, cloud_stamp);
  
  // 仅当 robot_base_frame 与 base_frame 不同时才额外发布
  if (robot_base_frame_ != base_frame_) {
    publishTransform(tf_odom_to_base, odometry_msg->header.frame_id, robot_base_frame_, cloud_stamp);
  }
  
  publishOdometry(tf_odom_to_base, odometry_msg->header.frame_id, robot_base_frame_, cloud_stamp);

  // 将 odom 坐标系的点云转换到 laser_link 坐标系
  // T_laser_odom = T_laser_base * T_base_odom = tf_base_to_lidar^(-1) * tf_odom_to_base^(-1)
  tf2::Transform tf_odom_to_lidar = tf_odom_to_base * (*tf_base_to_lidar);
  sensor_msgs::msg::PointCloud2 out;
  pcl_ros::transformPointCloud(lidar_frame_, tf_odom_to_lidar.inverse(), *pcd_msg, out);
  pub_laser_cloud_->publish(out);
}

std::optional<tf2::Transform> SensorScanNode::getTransform(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time)
{
  try {
    auto transform_stamped = tf_buffer_->lookupTransform(
      target_frame, source_frame, time, rclcpp::Duration::from_seconds(tf_timeout_sec_));
    tf2::Transform transform;
    tf2::fromMsg(transform_stamped.transform, transform);
    return transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "TF lookup failed (%s -> %s): %s", target_frame.c_str(), source_frame.c_str(), ex.what());
    return std::nullopt;
  }
}

void SensorScanNode::publishTransform(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;
  transform_msg.transform = tf2::toMsg(transform);
  br_->sendTransform(transform_msg);
}

void SensorScanNode::publishOdometry(
  const tf2::Transform & transform, std::string parent_frame, const std::string & child_frame,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry out;
  out.header.stamp = stamp;
  out.header.frame_id = parent_frame;
  out.child_frame_id = child_frame;

  const auto & origin = transform.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(transform.getRotation());

  if (odom_state_.initialized) {
    const double dt = (stamp - odom_state_.previous_stamp).seconds();

    if (dt > 1e-6 && dt < 1.0) {
      const auto linear_velocity_world =
        (transform.getOrigin() - odom_state_.previous_transform.getOrigin()) / dt;
      
      // [BUG修复] 将世界坐标系速度转换到机器人坐标系
      // ROS REP-103: twist 应在 child_frame_id（机器人坐标系）中表示
      const auto linear_velocity_body = transform.getRotation().inverse() * linear_velocity_world;

      // 角速度计算（含奇异性保护）
      tf2::Quaternion q_diff =
        transform.getRotation() * odom_state_.previous_transform.getRotation().inverse();
      q_diff.normalize();

      // 确保取最短路径
      if (q_diff.w() < 0) {
        q_diff = tf2::Quaternion(-q_diff.x(), -q_diff.y(), -q_diff.z(), -q_diff.w());
      }

      const double angle = q_diff.getAngle();

      out.twist.twist.linear.x = linear_velocity_body.x();
      out.twist.twist.linear.y = linear_velocity_body.y();
      out.twist.twist.linear.z = linear_velocity_body.z();

      // 角度接近零时，getAxis() 不稳定，直接输出零角速度
      if (angle > 1e-6) {
        const auto axis = q_diff.getAxis();
        out.twist.twist.angular.x = axis.x() * angle / dt;
        out.twist.twist.angular.y = axis.y() * angle / dt;
        out.twist.twist.angular.z = axis.z() * angle / dt;
      } else {
        out.twist.twist.angular.x = 0.0;
        out.twist.twist.angular.y = 0.0;
        out.twist.twist.angular.z = 0.0;
      }
    } else {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Skip velocity estimation (dt=%.3f out of bounds)", dt);
    }
  }

  odom_state_.previous_transform = transform;
  odom_state_.previous_stamp = stamp;
  odom_state_.initialized = true;

  pub_chassis_odometry_->publish(out);
}

}  // namespace rc26_sensor_scan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rc26_sensor_scan::SensorScanNode)
