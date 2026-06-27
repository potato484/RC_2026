#include "experimental_loop_closure.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <utility>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

namespace rc26_point_lio {

namespace {
constexpr int kScanContextRings = 20;
constexpr int kScanContextSectors = 60;
constexpr double kScanContextMaxRadius = 30.0;
constexpr double kMinFiniteFrequencyHz = 0.05;

void finalizeCloud(PointCloudXYZI& cloud) {
    cloud.width = static_cast<uint32_t>(cloud.points.size());
    cloud.height = 1;
    cloud.is_dense = false;
}

gtsam::Pose3 eigenToPose3(const M3D& rotation, const V3D& position) {
    return gtsam::Pose3(gtsam::Rot3(rotation), gtsam::Point3(position.x(), position.y(), position.z()));
}

void pose3ToEigen(const gtsam::Pose3& pose, M3D& rotation, V3D& position) {
    rotation = pose.rotation().matrix();
    position = V3D(pose.translation().x(), pose.translation().y(), pose.translation().z());
}

Eigen::Matrix4f poseMatrix(const M3D& rotation, const V3D& position) {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = rotation.cast<float>();
    transform.block<3, 1>(0, 3) = position.cast<float>();
    return transform;
}

double rotationDistance(const M3D& from, const M3D& to) {
    Eigen::AngleAxisd angle_axis(from.transpose() * to);
    return std::abs(angle_axis.angle());
}

}  // namespace

struct ExperimentalLoopClosureBackend::GtsamState {
    GtsamState() {
        gtsam::ISAM2Params params;
        params.relinearizeThreshold = 0.01;
        params.relinearizeSkip = 1;
        isam = std::make_unique<gtsam::ISAM2>(params);
    }

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    std::unique_ptr<gtsam::ISAM2> isam;
    gtsam::Values estimate;
    std::set<std::pair<int, int>> loop_pairs;
};

ExperimentalLoopClosureBackend::ExperimentalLoopClosureBackend(const ExperimentalLoopClosureConfig& config)
    : config_(config), gtsam_(std::make_unique<GtsamState>()) {
    config_.frequency_hz = std::max(kMinFiniteFrequencyHz, config_.frequency_hz);
    config_.keyframe_dist_threshold_m = std::max(0.05, config_.keyframe_dist_threshold_m);
    config_.keyframe_angle_threshold_rad = std::max(0.01, config_.keyframe_angle_threshold_rad);
    config_.search_radius_m = std::max(0.1, config_.search_radius_m);
    config_.time_diff_threshold_sec = std::max(0.0, config_.time_diff_threshold_sec);
    config_.exclude_recent_keyframes = std::max(1, config_.exclude_recent_keyframes);
    config_.sc_dist_threshold = std::max(0.01, config_.sc_dist_threshold);
    config_.icp_fitness_threshold = std::max(0.001, config_.icp_fitness_threshold);
    config_.icp_max_correspondence_dist_m = std::max(0.1, config_.icp_max_correspondence_dist_m);
    config_.icp_max_iterations = std::max(1, config_.icp_max_iterations);
    config_.submap_size = std::max(0, config_.submap_size);
    config_.keyframe_cloud_voxel_size_m = std::max(0.0, config_.keyframe_cloud_voxel_size_m);
    config_.max_keyframes_with_cloud = std::max(3, config_.max_keyframes_with_cloud);
}

ExperimentalLoopClosureBackend::~ExperimentalLoopClosureBackend() {
    stop();
}

void ExperimentalLoopClosureBackend::start() {
    if (!config_.enable || running_.exchange(true)) {
        return;
    }
    worker_ = std::thread(&ExperimentalLoopClosureBackend::workerLoop, this);
}

void ExperimentalLoopClosureBackend::stop() {
    running_ = false;
    if (worker_.joinable()) {
        worker_.join();
    }
}

void ExperimentalLoopClosureBackend::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    keyframes_.clear();
    pending_correction_ = ExperimentalLoopCorrection();
    gtsam_ = std::make_unique<GtsamState>();
}

bool ExperimentalLoopClosureBackend::shouldAddKeyFrameLocked(const M3D& rotation, const V3D& position) const {
    if (keyframes_.empty()) {
        return true;
    }

    const KeyFrame& last = keyframes_.back();
    const double dist = (position - last.original_position).norm();
    const double angle = rotationDistance(last.original_rotation, rotation);
    return dist >= config_.keyframe_dist_threshold_m || angle >= config_.keyframe_angle_threshold_rad;
}

bool ExperimentalLoopClosureBackend::maybeAddKeyFrame(double timestamp,
                                                      const M3D& rotation,
                                                      const V3D& position,
                                                      const PointCloudXYZI::ConstPtr& body_cloud) {
    if (!config_.enable || !body_cloud || body_cloud->empty()) {
        return false;
    }

    KeyFrame keyframe;
    keyframe.timestamp = timestamp;
    keyframe.rotation = rotation;
    keyframe.position = position;
    keyframe.original_rotation = rotation;
    keyframe.original_position = position;
    keyframe.cloud.reset(new PointCloudXYZI(*body_cloud));
    finalizeCloud(*keyframe.cloud);

    if (config_.keyframe_cloud_voxel_size_m > 0.0 && keyframe.cloud && !keyframe.cloud->empty()) {
        pcl::VoxelGrid<PointType> voxel;
        const float leaf = static_cast<float>(config_.keyframe_cloud_voxel_size_m);
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.setInputCloud(keyframe.cloud);
        PointCloudXYZI::Ptr filtered(new PointCloudXYZI());
        voxel.filter(*filtered);
        finalizeCloud(*filtered);
        keyframe.cloud = filtered;
    }

    if (!keyframe.cloud || keyframe.cloud->empty()) {
        return false;
    }
    keyframe.cloud_valid = true;
    keyframe.scan_context = makeScanContext(*keyframe.cloud);

    std::lock_guard<std::mutex> lock(mutex_);
    if (!shouldAddKeyFrameLocked(rotation, position)) {
        return false;
    }

    keyframe.id = keyframes_.size();
    if (keyframes_.empty()) {
        addPriorLocked(keyframe);
    } else {
        addOdomFactorLocked(keyframes_.back(), keyframe);
    }
    keyframes_.push_back(keyframe);
    optimizeLocked(false);
    cleanupOldCloudsLocked();

    std::cout << "[ExperimentalLoopClosure] 关键帧已加入: id=" << keyframe.id
              << " total=" << keyframes_.size() << '\n';
    return true;
}

void ExperimentalLoopClosureBackend::addPriorLocked(const KeyFrame& keyframe) {
    gtsam::Vector6 noise;
    noise << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    gtsam_->graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        keyframe.id, eigenToPose3(keyframe.rotation, keyframe.position),
        gtsam::noiseModel::Diagonal::Variances(noise)));
    gtsam_->initial.insert(keyframe.id, eigenToPose3(keyframe.rotation, keyframe.position));
}

void ExperimentalLoopClosureBackend::addOdomFactorLocked(const KeyFrame& from, const KeyFrame& to) {
    gtsam::Vector6 noise;
    noise << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    const gtsam::Pose3 pose_from = eigenToPose3(from.original_rotation, from.original_position);
    const gtsam::Pose3 pose_to = eigenToPose3(to.original_rotation, to.original_position);
    gtsam_->graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        from.id, to.id, pose_from.between(pose_to),
        gtsam::noiseModel::Diagonal::Variances(noise)));
    if (!gtsam_->initial.exists(to.id)) {
        gtsam_->initial.insert(to.id, pose_to);
    }
}

void ExperimentalLoopClosureBackend::addLoopFactorLocked(size_t from_id,
                                                         size_t to_id,
                                                         const Eigen::Affine3f& relative_transform,
                                                         float fitness_score) {
    const double scale = 1.0 + std::min(2.0, static_cast<double>(fitness_score));
    const double rot_noise = std::max(1e-4, 0.01 * scale);
    const double trans_noise = std::max(1e-4, 0.05 * scale);
    gtsam::Vector6 noise;
    noise << rot_noise, rot_noise, rot_noise, trans_noise, trans_noise, trans_noise;

    const Eigen::Matrix3d rotation = relative_transform.rotation().cast<double>();
    const Eigen::Vector3d translation = relative_transform.translation().cast<double>();
    const gtsam::Pose3 relative_pose(gtsam::Rot3(rotation),
                                     gtsam::Point3(translation.x(), translation.y(), translation.z()));

    gtsam_->graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        from_id, to_id, relative_pose,
        gtsam::noiseModel::Diagonal::Variances(noise)));
}

void ExperimentalLoopClosureBackend::optimizeLocked(bool loop_closed) {
    if (gtsam_->graph.empty() && gtsam_->initial.empty()) {
        return;
    }

    try {
        gtsam_->isam->update(gtsam_->graph, gtsam_->initial);
        gtsam_->isam->update();
        if (loop_closed) {
            for (int i = 0; i < 4; ++i) {
                gtsam_->isam->update();
            }
        }
        gtsam_->estimate = gtsam_->isam->calculateEstimate();
        gtsam_->graph.resize(0);
        gtsam_->initial.clear();
    } catch (const std::exception& ex) {
        std::cerr << "[ExperimentalLoopClosure] GTSAM 优化失败: " << ex.what() << '\n';
        gtsam_->graph.resize(0);
        gtsam_->initial.clear();
    }
}

void ExperimentalLoopClosureBackend::updateKeyFramePosesFromEstimateLocked() {
    for (KeyFrame& keyframe : keyframes_) {
        if (!gtsam_->estimate.exists(keyframe.id)) {
            continue;
        }
        M3D rotation;
        V3D position;
        pose3ToEigen(gtsam_->estimate.at<gtsam::Pose3>(keyframe.id), rotation, position);
        keyframe.rotation = rotation;
        keyframe.position = position;
    }
}

void ExperimentalLoopClosureBackend::cleanupOldCloudsLocked() {
    int valid_count = 0;
    for (const KeyFrame& keyframe : keyframes_) {
        if (keyframe.cloud_valid) {
            ++valid_count;
        }
    }
    for (KeyFrame& keyframe : keyframes_) {
        if (valid_count <= config_.max_keyframes_with_cloud) {
            return;
        }
        if (keyframe.cloud_valid) {
            keyframe.cloud.reset();
            keyframe.cloud_valid = false;
            --valid_count;
        }
    }
}

Eigen::MatrixXd ExperimentalLoopClosureBackend::makeScanContext(const PointCloudXYZI& cloud) const {
    Eigen::MatrixXd descriptor = Eigen::MatrixXd::Zero(kScanContextRings, kScanContextSectors);
    Eigen::MatrixXd seen = Eigen::MatrixXd::Zero(kScanContextRings, kScanContextSectors);

    for (const auto& point : cloud.points) {
        const double x = point.x;
        const double y = point.y;
        const double z = point.z;
        const double radius = std::hypot(x, y);
        if (!std::isfinite(radius) || radius < 1e-3 || radius > kScanContextMaxRadius) {
            continue;
        }

        int ring = static_cast<int>(std::floor(radius / kScanContextMaxRadius * kScanContextRings));
        ring = std::clamp(ring, 0, kScanContextRings - 1);

        double angle = std::atan2(y, x);
        if (angle < 0.0) {
            angle += 2.0 * M_PI;
        }
        int sector = static_cast<int>(std::floor(angle / (2.0 * M_PI) * kScanContextSectors));
        sector = std::clamp(sector, 0, kScanContextSectors - 1);

        if (seen(ring, sector) == 0.0 || z > descriptor(ring, sector)) {
            descriptor(ring, sector) = z;
            seen(ring, sector) = 1.0;
        }
    }

    return descriptor;
}

double ExperimentalLoopClosureBackend::scanContextDistance(const Eigen::MatrixXd& current,
                                                           const Eigen::MatrixXd& candidate,
                                                           double& yaw_offset_rad) const {
    if (current.rows() != kScanContextRings || candidate.rows() != kScanContextRings ||
        current.cols() != kScanContextSectors || candidate.cols() != kScanContextSectors) {
        yaw_offset_rad = 0.0;
        return 1.0;
    }

    double best_dist = std::numeric_limits<double>::infinity();
    int best_shift = 0;
    for (int shift = 0; shift < kScanContextSectors; ++shift) {
        double dot = 0.0;
        double norm_a = 0.0;
        double norm_b = 0.0;
        for (int r = 0; r < kScanContextRings; ++r) {
            for (int s = 0; s < kScanContextSectors; ++s) {
                const double a = current(r, s);
                const double b = candidate(r, (s + shift) % kScanContextSectors);
                dot += a * b;
                norm_a += a * a;
                norm_b += b * b;
            }
        }

        if (norm_a < 1e-9 || norm_b < 1e-9) {
            continue;
        }
        const double sim = dot / (std::sqrt(norm_a) * std::sqrt(norm_b));
        const double dist = 1.0 - std::clamp(sim, -1.0, 1.0);
        if (dist < best_dist) {
            best_dist = dist;
            best_shift = shift;
        }
    }

    if (!std::isfinite(best_dist)) {
        yaw_offset_rad = 0.0;
        return 1.0;
    }

    yaw_offset_rad = static_cast<double>(best_shift) * 2.0 * M_PI / static_cast<double>(kScanContextSectors);
    return best_dist;
}

bool ExperimentalLoopClosureBackend::findLoopCandidate(LoopCandidate& candidate) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (keyframes_.size() <= static_cast<size_t>(config_.exclude_recent_keyframes + 1)) {
        return false;
    }

    const int current_idx = static_cast<int>(keyframes_.size()) - 1;
    const KeyFrame& current = keyframes_[current_idx];
    std::vector<std::pair<double, int>> candidates;
    const int max_idx = current_idx - config_.exclude_recent_keyframes;
    for (int i = 0; i < max_idx; ++i) {
        const KeyFrame& historical = keyframes_[i];
        if (std::abs(current.timestamp - historical.timestamp) < config_.time_diff_threshold_sec) {
            continue;
        }
        const double dist = (current.position - historical.position).norm();
        if (dist <= config_.search_radius_m) {
            candidates.emplace_back(dist, i);
        }
    }

    std::sort(candidates.begin(), candidates.end());
    for (const auto& item : candidates) {
        const int loop_idx = item.second;
        const auto normalized_pair = std::make_pair(std::min(current_idx, loop_idx), std::max(current_idx, loop_idx));
        if (gtsam_->loop_pairs.count(normalized_pair) > 0) {
            continue;
        }

        double yaw_offset = 0.0;
        const double sc_score =
            scanContextDistance(current.scan_context, keyframes_[loop_idx].scan_context, yaw_offset);
        if (sc_score > config_.sc_dist_threshold) {
            continue;
        }

        PointCloudXYZI::Ptr source = buildSubmapInBodyFrameLocked(current_idx, config_.submap_size);
        PointCloudXYZI::Ptr target = buildSubmapInBodyFrameLocked(loop_idx, config_.submap_size);
        if (!source || !target || source->empty() || target->empty()) {
            continue;
        }

        candidate.current_idx = current_idx;
        candidate.loop_idx = loop_idx;
        candidate.sc_score = sc_score;
        candidate.yaw_offset_rad = yaw_offset;
        candidate.source_submap = source;
        candidate.target_submap = target;
        return true;
    }

    return false;
}

PointCloudXYZI::Ptr ExperimentalLoopClosureBackend::buildSubmapInBodyFrameLocked(int center_idx,
                                                                                 int half_window) const {
    PointCloudXYZI::Ptr submap(new PointCloudXYZI());
    if (center_idx < 0 || center_idx >= static_cast<int>(keyframes_.size())) {
        return submap;
    }

    const KeyFrame& center = keyframes_[center_idx];
    if (!center.cloud_valid || !center.cloud || center.cloud->empty()) {
        return submap;
    }

    const Eigen::Matrix4f center_inv = poseMatrix(center.rotation, center.position).inverse();
    const int start = std::max(0, center_idx - half_window);
    const int end = std::min(static_cast<int>(keyframes_.size()) - 1, center_idx + half_window);
    int valid_clouds = 0;
    for (int i = start; i <= end; ++i) {
        const KeyFrame& keyframe = keyframes_[i];
        if (!keyframe.cloud_valid || !keyframe.cloud || keyframe.cloud->empty()) {
            continue;
        }

        if (i == center_idx) {
            *submap += *keyframe.cloud;
        } else {
            const Eigen::Matrix4f relative = center_inv * poseMatrix(keyframe.rotation, keyframe.position);
            PointCloudXYZI transformed;
            pcl::transformPointCloud(*keyframe.cloud, transformed, relative);
            *submap += transformed;
        }
        ++valid_clouds;
    }

    if (valid_clouds < 1) {
        submap->clear();
    }
    finalizeCloud(*submap);
    return submap;
}

bool ExperimentalLoopClosureBackend::verifyWithIcp(const LoopCandidate& candidate,
                                                   Eigen::Affine3f& current_to_loop,
                                                   float& fitness_score) const {
    if (!candidate.source_submap || !candidate.target_submap || candidate.source_submap->empty() ||
        candidate.target_submap->empty()) {
        return false;
    }

    Eigen::Affine3f initial_guess = Eigen::Affine3f::Identity();
    initial_guess.rotate(Eigen::AngleAxisf(static_cast<float>(candidate.yaw_offset_rad), Eigen::Vector3f::UnitZ()));

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(config_.icp_max_correspondence_dist_m);
    icp.setMaximumIterations(config_.icp_max_iterations);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    icp.setInputSource(candidate.source_submap);
    icp.setInputTarget(candidate.target_submap);

    PointCloudXYZI aligned;
    icp.align(aligned, initial_guess.matrix());
    fitness_score = static_cast<float>(icp.getFitnessScore());
    if (!icp.hasConverged() || fitness_score > config_.icp_fitness_threshold) {
        return false;
    }

    current_to_loop = Eigen::Affine3f(icp.getFinalTransformation());
    return true;
}

void ExperimentalLoopClosureBackend::workerLoop() {
    const auto period = std::chrono::milliseconds(
        std::max(1, static_cast<int>(1000.0 / std::max(kMinFiniteFrequencyHz, config_.frequency_hz))));

    while (running_) {
        std::this_thread::sleep_for(period);
        if (!running_) {
            break;
        }

        LoopCandidate candidate;
        if (!findLoopCandidate(candidate)) {
            continue;
        }

        Eigen::Affine3f current_to_loop = Eigen::Affine3f::Identity();
        float fitness_score = 0.0f;
        if (!verifyWithIcp(candidate, current_to_loop, fitness_score)) {
            continue;
        }

        const Eigen::Affine3f relative_constraint = current_to_loop.inverse();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto normalized_pair =
                std::make_pair(std::min(candidate.current_idx, candidate.loop_idx),
                               std::max(candidate.current_idx, candidate.loop_idx));
            if (gtsam_->loop_pairs.count(normalized_pair) > 0) {
                continue;
            }
            gtsam_->loop_pairs.insert(normalized_pair);
            addLoopFactorLocked(candidate.current_idx, candidate.loop_idx, relative_constraint, fitness_score);
            optimizeLocked(true);
            updateKeyFramePosesFromEstimateLocked();

            const KeyFrame& latest = keyframes_.back();
            pending_correction_.valid = true;
            pending_correction_.keyframe_id = latest.id;
            pending_correction_.timestamp = latest.timestamp;
            pending_correction_.rotation = latest.rotation;
            pending_correction_.position = latest.position;
            pending_correction_.original_rotation = latest.original_rotation;
            pending_correction_.original_position = latest.original_position;
        }

        std::cout << "[ExperimentalLoopClosure] 检测到实验闭环: current=" << candidate.current_idx
                  << " loop=" << candidate.loop_idx << " sc=" << candidate.sc_score
                  << " icp=" << fitness_score << '\n';
    }
}

bool ExperimentalLoopClosureBackend::consumePendingCorrection(ExperimentalLoopCorrection& correction) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_correction_.valid) {
        return false;
    }
    correction = pending_correction_;
    pending_correction_ = ExperimentalLoopCorrection();
    return true;
}

std::vector<ExperimentalOptimizedPose> ExperimentalLoopClosureBackend::optimizedPoses() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<ExperimentalOptimizedPose> poses;
    poses.reserve(keyframes_.size());
    for (const KeyFrame& keyframe : keyframes_) {
        ExperimentalOptimizedPose pose;
        pose.id = keyframe.id;
        pose.timestamp = keyframe.timestamp;
        pose.rotation = keyframe.rotation;
        pose.position = keyframe.position;
        poses.push_back(pose);
    }
    return poses;
}

PointCloudXYZI::Ptr ExperimentalLoopClosureBackend::buildOptimizedMap() const {
    std::lock_guard<std::mutex> lock(mutex_);
    PointCloudXYZI::Ptr map(new PointCloudXYZI());
    for (const KeyFrame& keyframe : keyframes_) {
        if (!keyframe.cloud_valid || !keyframe.cloud || keyframe.cloud->empty()) {
            continue;
        }
        PointCloudXYZI transformed;
        pcl::transformPointCloud(*keyframe.cloud, transformed, poseMatrix(keyframe.rotation, keyframe.position));
        *map += transformed;
    }
    finalizeCloud(*map);
    return map;
}

size_t ExperimentalLoopClosureBackend::keyframeCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return keyframes_.size();
}

}  // namespace rc26_point_lio
