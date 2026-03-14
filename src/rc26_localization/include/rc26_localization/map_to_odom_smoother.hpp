// Copyright 2025 RC2026

#pragma once

#include <cmath>
#include <mutex>

#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"

namespace rc26_localization {

class MapToOdomSmoother {
public:
    struct Config {
        double max_translation_speed_mps{0.25};
        double max_yaw_speed_radps{10.0 * M_PI / 180.0};
    };

    MapToOdomSmoother();
    explicit MapToOdomSmoother(const Config& cfg);

    void setConfig(const Config& cfg);
    Config getConfig() const;

    void reset(const Eigen::Isometry3d& initial_map_to_odom, const rclcpp::Time& stamp);
    void setTarget(const Eigen::Isometry3d& target_map_to_odom, const rclcpp::Time& stamp);
    bool step(const rclcpp::Time& now, Eigen::Isometry3d& out_map_to_odom);

    bool isInitialized() const;
    Eigen::Isometry3d current() const;
    Eigen::Isometry3d target() const;

private:
    static double yawOf(const Eigen::Isometry3d& pose);

    mutable std::mutex mutex_;
    Config config_;
    bool initialized_{false};
    Eigen::Isometry3d current_map_to_odom_{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d target_map_to_odom_{Eigen::Isometry3d::Identity()};
    rclcpp::Time last_update_stamp_;
};

}  // namespace rc26_localization
