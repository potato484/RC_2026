#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <Eigen/Eigen>

#include "common_lib.hpp"

namespace rc26_point_lio {

struct ExperimentalLoopClosureConfig {
    bool enable = false;
    double frequency_hz = 0.5;
    double keyframe_dist_threshold_m = 1.0;
    double keyframe_angle_threshold_rad = 0.2;
    double search_radius_m = 15.0;
    double time_diff_threshold_sec = 30.0;
    int exclude_recent_keyframes = 30;
    double sc_dist_threshold = 0.20;
    double icp_fitness_threshold = 0.5;
    double icp_max_correspondence_dist_m = 100.0;
    int icp_max_iterations = 100;
    int submap_size = 25;
    double keyframe_cloud_voxel_size_m = 0.15;
    int max_keyframes_with_cloud = 500;
};

struct ExperimentalLoopCorrection {
    bool valid = false;
    size_t keyframe_id = 0;
    double timestamp = 0.0;
    M3D rotation = M3D::Identity();
    V3D position = V3D::Zero();
    M3D original_rotation = M3D::Identity();
    V3D original_position = V3D::Zero();
};

struct ExperimentalOptimizedPose {
    size_t id = 0;
    double timestamp = 0.0;
    M3D rotation = M3D::Identity();
    V3D position = V3D::Zero();
};

class ExperimentalLoopClosureBackend {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit ExperimentalLoopClosureBackend(const ExperimentalLoopClosureConfig& config);
    ~ExperimentalLoopClosureBackend();

    ExperimentalLoopClosureBackend(const ExperimentalLoopClosureBackend&) = delete;
    ExperimentalLoopClosureBackend& operator=(const ExperimentalLoopClosureBackend&) = delete;

    bool enabled() const { return config_.enable; }
    void start();
    void stop();
    void reset();

    bool maybeAddKeyFrame(double timestamp,
                          const M3D& rotation,
                          const V3D& position,
                          const PointCloudXYZI::ConstPtr& body_cloud);

    bool consumePendingCorrection(ExperimentalLoopCorrection& correction);
    std::vector<ExperimentalOptimizedPose> optimizedPoses() const;
    PointCloudXYZI::Ptr buildOptimizedMap() const;
    size_t keyframeCount() const;

private:
    struct KeyFrame {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        size_t id = 0;
        double timestamp = 0.0;
        M3D rotation = M3D::Identity();
        V3D position = V3D::Zero();
        M3D original_rotation = M3D::Identity();
        V3D original_position = V3D::Zero();
        PointCloudXYZI::Ptr cloud;
        bool cloud_valid = false;
        Eigen::MatrixXd scan_context;
    };

    struct LoopCandidate {
        int current_idx = -1;
        int loop_idx = -1;
        double sc_score = 1.0;
        double yaw_offset_rad = 0.0;
        PointCloudXYZI::Ptr source_submap;
        PointCloudXYZI::Ptr target_submap;
    };

    bool shouldAddKeyFrameLocked(const M3D& rotation, const V3D& position) const;
    void addPriorLocked(const KeyFrame& keyframe);
    void addOdomFactorLocked(const KeyFrame& from, const KeyFrame& to);
    void addLoopFactorLocked(size_t from_id,
                             size_t to_id,
                             const Eigen::Affine3f& relative_transform,
                             float fitness_score);
    void optimizeLocked(bool loop_closed);
    void updateKeyFramePosesFromEstimateLocked();
    void cleanupOldCloudsLocked();

    Eigen::MatrixXd makeScanContext(const PointCloudXYZI& cloud) const;
    double scanContextDistance(const Eigen::MatrixXd& current,
                               const Eigen::MatrixXd& candidate,
                               double& yaw_offset_rad) const;
    bool findLoopCandidate(LoopCandidate& candidate) const;
    PointCloudXYZI::Ptr buildSubmapInBodyFrameLocked(int center_idx, int half_window) const;
    bool verifyWithIcp(const LoopCandidate& candidate,
                       Eigen::Affine3f& current_to_loop,
                       float& fitness_score) const;
    void workerLoop();

    ExperimentalLoopClosureConfig config_;

    mutable std::mutex mutex_;
    std::vector<KeyFrame, Eigen::aligned_allocator<KeyFrame>> keyframes_;
    ExperimentalLoopCorrection pending_correction_;

    std::atomic<bool> running_{false};
    std::thread worker_;

    struct GtsamState;
    std::unique_ptr<GtsamState> gtsam_;
};

}  // namespace rc26_point_lio
