#include "rc26_localization/localization.hpp"

#include <cmath>

#include "localization_internal.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace rc26_localization {

void LocalizationNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const rclcpp::Time stamp =
        (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ? now() : rclcpp::Time(msg->header.stamp);
    const double acc_norm = std::sqrt(
        msg->linear_acceleration.x * msg->linear_acceleration.x +
        msg->linear_acceleration.y * msg->linear_acceleration.y +
        msg->linear_acceleration.z * msg->linear_acceleration.z);
    const double gyro_norm = std::sqrt(
        msg->angular_velocity.x * msg->angular_velocity.x +
        msg->angular_velocity.y * msg->angular_velocity.y +
        msg->angular_velocity.z * msg->angular_velocity.z);

    // S1: Spike 检测
    if (s1_enable_ && (acc_norm > s1_accel_threshold_ || gyro_norm > s1_gyro_threshold_)) {
        std::lock_guard<std::mutex> lk(imu_spike_mutex_);
        imu_spike_deadline_ = stamp + rclcpp::Duration(0, static_cast<int32_t>(s1_freeze_duration_ms_) * 1'000'000);
        imu_spike_last_stamp_ = stamp;
        imu_spike_active_.store(true);
        imu_spike_recent_.store(true);
    }

    // T8: 重力加速度估计 roll/pitch（准静态假设）
    if (slope_roll_pitch_from_imu_) {
        const double ax = msg->linear_acceleration.x;
        const double ay = msg->linear_acceleration.y;
        const double az = msg->linear_acceleration.z;
        const double roll  = std::atan2(ay, az);
        const double pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
        std::lock_guard<std::mutex> lk(imu_attitude_mutex_);
        imu_roll_  = roll;
        imu_pitch_ = pitch;
        imu_attitude_valid_ = true;
    }

    if (imu_spike_active_.load()) {
        std::lock_guard<std::mutex> lk(imu_buffer_mutex_);
        imu_buffer_.push_back(
            {Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
             Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z), stamp});
        while (imu_buffer_.size() > 4096) {
            imu_buffer_.pop_front();
        }
    }

    if (esikf_enable_) {
        double dt = 0.0;
        {
            std::lock_guard<std::mutex> lk(imu_spike_mutex_);
            if (last_imu_stamp_valid_) {
                dt = (stamp - last_imu_stamp_).seconds();
            }
            last_imu_stamp_ = stamp;
            last_imu_stamp_valid_ = true;
        }

        if (dt > 0.0 && dt < 0.2) {
            Eigen::Isometry3d predicted = Eigen::Isometry3d::Identity();
            {
                std::lock_guard<std::mutex> lk(esikf_mutex_);
                esikf_.predict(
                    Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
                    Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z), dt);
                predicted = esikf_.getMapToOdom();
            }
            std::lock_guard<std::mutex> lock(result_mutex_);
            result_t_ = predicted;
        }
    }
}

void LocalizationNode::uwbCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(uwb_mutex_);
    uwb_position_.x() = msg->point.x;
    uwb_position_.y() = msg->point.y;
    uwb_last_stamp_ = rclcpp::Time(msg->header.stamp);
    uwb_available_ = true;
}

void LocalizationNode::registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!map_loaded_) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        last_scan_time_ = msg->header.stamp;
        current_scan_frame_id_ = msg->header.frame_id;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *scan);

    std::lock_guard<std::mutex> lock(cloud_mutex_);
    const size_t current_size = accumulated_cloud_->size();
    if (current_size >= max_accumulated_points_) {
        return;
    }
    const size_t remaining = max_accumulated_points_ - current_size;
    const size_t scan_size = scan->size();
    if (scan_size > remaining) {
        accumulated_cloud_->points.insert(accumulated_cloud_->points.end(), scan->points.begin(),
                                          scan->points.begin() + static_cast<std::ptrdiff_t>(remaining));
        accumulated_cloud_->width = static_cast<decltype(accumulated_cloud_->width)>(accumulated_cloud_->points.size());
        accumulated_cloud_->height = 1;
    } else {
        *accumulated_cloud_ += *scan;
    }
}

void LocalizationNode::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    consecutive_high_error_count_.store(0);

    RCLCPP_INFO(this->get_logger(), "收到初始位姿: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
                msg->pose.pose.position.y, msg->pose.pose.position.z);

    Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
    map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    map_to_robot_base.linear() = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                                    msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                     .toRotationMatrix();

    try {
        auto transform = tf_buffer_->lookupTransform(odom_frame_, robot_base_frame_, rclcpp::Time(msg->header.stamp),
                                                     rclcpp::Duration::from_seconds(tf_timeout_sec_));
        Eigen::Isometry3d odom_to_robot_base = tf2::transformToEigen(transform.transform);
        Eigen::Isometry3d robot_to_odom = odom_to_robot_base.inverse();
        Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_to_odom;

        markRelocalizationSuccess(map_to_odom);
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "无法查询 %s -> %s: %s", odom_frame_.c_str(), robot_base_frame_.c_str(),
                    ex.what());
        setLocalizationState(LocalizationState::SUSPECT, "initial_pose_tf_lookup_failed");
    }
}

}  // namespace rc26_localization

