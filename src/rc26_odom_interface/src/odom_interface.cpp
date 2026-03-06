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
#include <stdexcept>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace rc26_odom_interface {

namespace {

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
    this->declare_parameter<std::string>("state_estimation_topic", "");
    this->declare_parameter<std::string>("registered_scan_topic", "");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "");
    this->declare_parameter<std::string>("lidar_frame", "");
    this->declare_parameter<double>("tf_lookup_timeout_sec", 0.5);
    this->declare_parameter<double>("max_time_diff_sec", 0.2);
    this->declare_parameter<double>("tf_refresh_interval_sec", 1.0);
    this->declare_parameter<bool>("use_input_twist", true);

    this->get_parameter("state_estimation_topic", state_estimation_topic_);
    this->get_parameter("registered_scan_topic", registered_scan_topic_);
    this->get_parameter("odom_frame", odom_frame_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("lidar_frame", lidar_frame_);
    this->get_parameter("tf_lookup_timeout_sec", tf_timeout_sec_);
    this->get_parameter("max_time_diff_sec", max_time_diff_sec_);
    this->get_parameter("tf_refresh_interval_sec", tf_refresh_interval_sec_);
    this->get_parameter("use_input_twist", use_input_twist_);

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

    base_frame_to_lidar_initialized_ = false;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 5);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 5);

    pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        registered_scan_topic_, 5, std::bind(&OdomInterfaceNode::pointCloudCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        state_estimation_topic_, 5, std::bind(&OdomInterfaceNode::odometryCallback, this, std::placeholders::_1));
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

    if (max_time_diff_sec_ > 0.0) {
        rclcpp::Time latest_stamp;
        bool odom_ready = false;
        {
            std::lock_guard<std::mutex> lock(transform_mutex_);
            latest_stamp = latest_odometry_stamp_;
            odom_ready = odom_pose_ready_;
        }

        if (!odom_ready) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "尚未收到里程计，丢弃点云");
            return;
        }

        const double diff_sec = std::abs((rclcpp::Time(msg->header.stamp) - latest_stamp).seconds());
        if (diff_sec > max_time_diff_sec_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "点云与里程计时间差 %.3f > %.3f，丢弃点云", diff_sec, max_time_diff_sec_);
            return;
        }
    }
    pcd_pub_->publish(*msg);
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

    // Compute odom -> base_link transform
    tf2::Transform tf_odom_to_base = tf_lidar_odom_to_lidar * tf_base_to_lidar;

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

    // Publish TF: odom -> base_link (so RViz/Nav2 can resolve the TF tree even if downstream sync drops frames).
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform = tf2::toMsg(tf_odom_to_base);
    tf_broadcaster_->sendTransform(tf_msg);

    // [P3] 速度来源可切换：优先使用 Point-LIO 输入 twist，必要时回退到差分估计
    bool update_state = true;
    {
        std::lock_guard<std::mutex> lock(transform_mutex_);
        if (!odom_pose_ready_ || odom_stamp > latest_odometry_stamp_) {
            latest_odometry_stamp_ = odom_stamp;
        }
        odom_pose_ready_ = true;

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
    }

    odom_pub_->publish(out);
}

}  // namespace rc26_odom_interface

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rc26_odom_interface::OdomInterfaceNode)
