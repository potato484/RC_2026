// Copyright 2025 RC2026

#pragma once

#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>

#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"

namespace gtsam {
class ISAM2;
class NonlinearFactorGraph;
class Values;
}  // namespace gtsam

namespace rc26_localization {

enum class GraphConstraintKind : uint8_t {
    ODOM = 0U,
    LOOP = 1U,
    ANCHOR = 2U,
    UWB_SOFT = 3U,
    GYRO_SOFT = 4U,
};

struct GraphConstraint {
    GraphConstraintKind kind{GraphConstraintKind::ODOM};
    uint32_t from_id{0U};
    uint32_t to_id{0U};
    Eigen::Isometry3d relative_pose{Eigen::Isometry3d::Identity()};
    double sigma_translation_m{0.05};
    double sigma_yaw_rad{2.0 * M_PI / 180.0};
    bool robust{false};
};

struct PoseGraphStatus {
    bool optimizer_ready{false};
    std::string optimizer_state{"idle"};
    uint32_t active_keyframe_id{0U};
    double graph_health{0.0};
    uint32_t loop_candidate_count{0U};
    uint32_t accepted_loop_count{0U};
    uint32_t accepted_anchor_count{0U};
    uint32_t candidate_conflict_count{0U};
    bool map_to_odom_jump_suppressed{false};
    double last_loop_age_sec{-1.0};
    double last_anchor_age_sec{-1.0};
};

class PoseGraphBackend {
public:
    struct Config {
        double prior_sigma_xy_m{0.05};
        double prior_sigma_yaw_rad{3.0 * M_PI / 180.0};
        double graph_health_conflict_penalty{0.05};
    };

    PoseGraphBackend();
    explicit PoseGraphBackend(const Config& cfg);
    ~PoseGraphBackend();

    void reset();
    bool addKeyframeNode(uint32_t id, const Eigen::Isometry3d& pose_map_guess);
    bool addConstraint(const GraphConstraint& constraint);
    bool addAnchorPrior(uint32_t id, const Eigen::Isometry3d& pose_map, double sigma_translation_m,
                        double sigma_yaw_rad, bool robust);
    bool update(const rclcpp::Time& stamp);
    bool queryPoseMap(uint32_t id, Eigen::Isometry3d& out_pose_map) const;

    void markLoopCandidate(bool accepted, bool conflict, const rclcpp::Time& stamp);
    void markAnchorCandidate(bool accepted, bool conflict, const rclcpp::Time& stamp);
    void setMapToOdomJumpSuppressed(bool suppressed);
    PoseGraphStatus statusSnapshot(const rclcpp::Time& now) const;

private:
    static double computeAgeSec(const rclcpp::Time& now, const rclcpp::Time& stamp);
    static Eigen::Isometry3d pose2ToEigen(double x, double y, double yaw);
    static void eigenToPose2(const Eigen::Isometry3d& pose, double& x, double& y, double& yaw);

    Config config_;
    mutable std::mutex mutex_;
    std::unique_ptr<gtsam::ISAM2> isam_;
    std::unique_ptr<gtsam::NonlinearFactorGraph> pending_graph_;
    std::unique_ptr<gtsam::Values> pending_values_;
    std::unique_ptr<gtsam::Values> current_values_;
    std::unordered_set<uint32_t> inserted_ids_;

    bool has_prior_{false};
    PoseGraphStatus status_;
    rclcpp::Time last_loop_stamp_;
    rclcpp::Time last_anchor_stamp_;
    rclcpp::Time last_update_stamp_;
};

}  // namespace rc26_localization
