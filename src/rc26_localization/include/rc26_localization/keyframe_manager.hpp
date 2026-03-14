// Copyright 2025 RC2026

#pragma once

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"

namespace rc26_localization {

struct KeyframeData {
    uint32_t id{0U};
    rclcpp::Time stamp;
    Eigen::Isometry3d pose_odom{Eigen::Isometry3d::Identity()};
    Eigen::Isometry3d pose_map_guess{Eigen::Isometry3d::Identity()};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{new pcl::PointCloud<pcl::PointXYZ>()};
    Eigen::MatrixXf descriptor;
    Eigen::VectorXf ring_key;
    Eigen::VectorXf sector_key;
    double sigma_xy{0.0};
    double sigma_yaw_deg{0.0};
    double h_min_eig{0.0};
    double h_cond{1e12};
    bool control_degraded{false};
    std::string trigger_reason{"unknown"};
};

class KeyframeManager {
public:
    struct Config {
        double translation_thresh_m{0.4};
        double yaw_thresh_deg{6.0};
        double time_thresh_sec{1.0};
        bool trigger_on_control_degraded_rising{true};
        bool trigger_on_hessian_drop{true};
        bool trigger_on_sigma_cross{true};
        double hessian_alert_eig_min{50.0};
        double sigma_xy_alert_min{0.12};
    };

    KeyframeManager();
    explicit KeyframeManager(const Config& cfg);

    void setConfig(const Config& cfg);
    Config getConfig() const;
    void clear();

    bool shouldCreate(const KeyframeData& candidate, std::string& reason);
    KeyframeData push(KeyframeData candidate);

    std::optional<KeyframeData> latest() const;
    std::optional<KeyframeData> previous() const;
    std::optional<KeyframeData> getById(uint32_t id) const;
    size_t size() const;

private:
    static double yawFromPose(const Eigen::Isometry3d& pose);

    mutable std::mutex mutex_;
    Config config_;
    std::vector<KeyframeData> keyframes_;
    uint32_t next_id_{1U};
    bool last_observation_valid_{false};
    bool last_control_degraded_{false};
    double last_h_min_eig_{0.0};
    double last_sigma_xy_{0.0};
};

}  // namespace rc26_localization
