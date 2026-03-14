#include "rc26_localization/map_to_odom_smoother.hpp"

#include <algorithm>
#include <cmath>

namespace rc26_localization {

namespace {
double normalizeAngle(double angle_rad) {
    return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}
}  // namespace

MapToOdomSmoother::MapToOdomSmoother() : MapToOdomSmoother(Config{}) {}

MapToOdomSmoother::MapToOdomSmoother(const Config& cfg) : config_(cfg) {}

void MapToOdomSmoother::setConfig(const Config& cfg) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = cfg;
}

MapToOdomSmoother::Config MapToOdomSmoother::getConfig() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
}

void MapToOdomSmoother::reset(const Eigen::Isometry3d& initial_map_to_odom, const rclcpp::Time& stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    initialized_ = true;
    current_map_to_odom_ = initial_map_to_odom;
    target_map_to_odom_ = initial_map_to_odom;
    last_update_stamp_ = stamp;
}

void MapToOdomSmoother::setTarget(const Eigen::Isometry3d& target_map_to_odom, const rclcpp::Time& stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
        initialized_ = true;
        current_map_to_odom_ = target_map_to_odom;
    }
    target_map_to_odom_ = target_map_to_odom;
    if (last_update_stamp_.nanoseconds() <= 0 || last_update_stamp_.get_clock_type() != stamp.get_clock_type()) {
        last_update_stamp_ = stamp;
    }
}

bool MapToOdomSmoother::step(const rclcpp::Time& now, Eigen::Isometry3d& out_map_to_odom) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
        return false;
    }
    if (last_update_stamp_.nanoseconds() <= 0 || last_update_stamp_.get_clock_type() != now.get_clock_type()) {
        last_update_stamp_ = now;
        out_map_to_odom = current_map_to_odom_;
        return true;
    }

    const double dt = std::max(0.0, (now - last_update_stamp_).seconds());
    if (dt <= 1e-9) {
        out_map_to_odom = current_map_to_odom_;
        return true;
    }

    Eigen::Vector3d current_translation = current_map_to_odom_.translation();
    const Eigen::Vector3d target_translation = target_map_to_odom_.translation();
    const Eigen::Vector2d delta_xy = target_translation.head<2>() - current_translation.head<2>();
    const double delta_xy_norm = delta_xy.norm();
    const double max_xy_step = std::max(0.0, config_.max_translation_speed_mps) * dt;
    if (delta_xy_norm > max_xy_step && delta_xy_norm > 1e-9) {
        current_translation.x() += delta_xy.x() * (max_xy_step / delta_xy_norm);
        current_translation.y() += delta_xy.y() * (max_xy_step / delta_xy_norm);
    } else {
        current_translation.x() = target_translation.x();
        current_translation.y() = target_translation.y();
    }
    current_translation.z() = target_translation.z();

    const double yaw_current = yawOf(current_map_to_odom_);
    const double yaw_target = yawOf(target_map_to_odom_);
    const double yaw_delta = normalizeAngle(yaw_target - yaw_current);
    const double max_yaw_step = std::max(0.0, config_.max_yaw_speed_radps) * dt;
    const double yaw_step = std::clamp(yaw_delta, -max_yaw_step, max_yaw_step);
    const double yaw_next = normalizeAngle(yaw_current + yaw_step);

    const Eigen::Vector3d target_euler = target_map_to_odom_.rotation().eulerAngles(2, 1, 0);
    const double target_pitch = target_euler[1];
    const double target_roll = target_euler[2];

    Eigen::Isometry3d next_pose = Eigen::Isometry3d::Identity();
    next_pose.translation() = current_translation;
    next_pose.linear() = (Eigen::AngleAxisd(yaw_next, Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(target_pitch, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(target_roll, Eigen::Vector3d::UnitX()))
                             .toRotationMatrix();

    current_map_to_odom_ = next_pose;
    last_update_stamp_ = now;
    out_map_to_odom = current_map_to_odom_;
    return true;
}

bool MapToOdomSmoother::isInitialized() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return initialized_;
}

Eigen::Isometry3d MapToOdomSmoother::current() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_map_to_odom_;
}

Eigen::Isometry3d MapToOdomSmoother::target() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return target_map_to_odom_;
}

double MapToOdomSmoother::yawOf(const Eigen::Isometry3d& pose) {
    return std::atan2(pose.rotation()(1, 0), pose.rotation()(0, 0));
}

}  // namespace rc26_localization
