#include "rc26_localization/pose_graph_backend.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "gtsam/inference/Symbol.h"
#include "gtsam/geometry/Pose2.h"
#include "gtsam/linear/NoiseModel.h"
#include "gtsam/nonlinear/ISAM2.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/Values.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/slam/PriorFactor.h"

namespace rc26_localization {

namespace {
gtsam::Symbol makeSymbol(uint32_t id) {
    return gtsam::Symbol('x', id);
}

gtsam::noiseModel::Base::shared_ptr buildNoise(double sigma_translation_m, double sigma_yaw_rad, bool robust) {
    const auto base_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(3) << std::max(1e-6, sigma_translation_m), std::max(1e-6, sigma_translation_m),
         std::max(1e-6, sigma_yaw_rad))
            .finished());
    if (!robust) {
        return base_noise;
    }
    return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.0), base_noise);
}
}  // namespace

PoseGraphBackend::PoseGraphBackend() : PoseGraphBackend(Config{}) {}

PoseGraphBackend::PoseGraphBackend(const Config& cfg) : config_(cfg) {
    gtsam::ISAM2Params params;
    params.relinearizeSkip = 1;
    params.relinearizeThreshold = 0.01;
    isam_ = std::make_unique<gtsam::ISAM2>(params);
    pending_graph_ = std::make_unique<gtsam::NonlinearFactorGraph>();
    pending_values_ = std::make_unique<gtsam::Values>();
    current_values_ = std::make_unique<gtsam::Values>();
}

PoseGraphBackend::~PoseGraphBackend() = default;

void PoseGraphBackend::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    gtsam::ISAM2Params params;
    params.relinearizeSkip = 1;
    params.relinearizeThreshold = 0.01;
    isam_ = std::make_unique<gtsam::ISAM2>(params);
    pending_graph_->resize(0);
    pending_values_->clear();
    current_values_->clear();
    inserted_ids_.clear();
    has_prior_ = false;
    status_ = PoseGraphStatus{};
    last_loop_stamp_ = rclcpp::Time{};
    last_anchor_stamp_ = rclcpp::Time{};
    last_update_stamp_ = rclcpp::Time{};
}

bool PoseGraphBackend::addKeyframeNode(uint32_t id, const Eigen::Isometry3d& pose_map_guess) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (id == 0U || inserted_ids_.count(id) > 0U) {
        return false;
    }

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    eigenToPose2(pose_map_guess, x, y, yaw);
    const auto sym = makeSymbol(id);
    pending_values_->insert(sym, gtsam::Pose2(x, y, yaw));
    inserted_ids_.insert(id);
    status_.active_keyframe_id = id;

    if (!has_prior_) {
        const auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(3) << config_.prior_sigma_xy_m, config_.prior_sigma_xy_m, config_.prior_sigma_yaw_rad).finished());
        pending_graph_->add(gtsam::PriorFactor<gtsam::Pose2>(sym, gtsam::Pose2(x, y, yaw), prior_noise));
        has_prior_ = true;
    }

    status_.optimizer_state = "node_buffered";
    return true;
}

bool PoseGraphBackend::addConstraint(const GraphConstraint& constraint) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (constraint.from_id == 0U || constraint.to_id == 0U) {
        return false;
    }
    if (inserted_ids_.count(constraint.from_id) == 0U || inserted_ids_.count(constraint.to_id) == 0U) {
        return false;
    }

    double dx = 0.0;
    double dy = 0.0;
    double dyaw = 0.0;
    eigenToPose2(constraint.relative_pose, dx, dy, dyaw);

    const auto noise = buildNoise(constraint.sigma_translation_m, constraint.sigma_yaw_rad, constraint.robust);

    pending_graph_->add(gtsam::BetweenFactor<gtsam::Pose2>(makeSymbol(constraint.from_id), makeSymbol(constraint.to_id),
                                                            gtsam::Pose2(dx, dy, dyaw), noise));
    status_.optimizer_state = "constraint_buffered";
    return true;
}

bool PoseGraphBackend::addAnchorPrior(uint32_t id, const Eigen::Isometry3d& pose_map, double sigma_translation_m,
                                      double sigma_yaw_rad, bool robust) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (id == 0U || inserted_ids_.count(id) == 0U) {
        return false;
    }

    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    eigenToPose2(pose_map, x, y, yaw);
    const auto noise = buildNoise(sigma_translation_m, sigma_yaw_rad, robust);
    pending_graph_->add(gtsam::PriorFactor<gtsam::Pose2>(makeSymbol(id), gtsam::Pose2(x, y, yaw), noise));
    status_.optimizer_state = "anchor_buffered";
    return true;
}

bool PoseGraphBackend::update(const rclcpp::Time& stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_graph_->empty() && pending_values_->empty()) {
        return false;
    }

    try {
        isam_->update(*pending_graph_, *pending_values_);
        *current_values_ = isam_->calculateEstimate();
        pending_graph_->resize(0);
        pending_values_->clear();
        status_.optimizer_ready = true;
        status_.optimizer_state = "ok";
        last_update_stamp_ = stamp;
    } catch (const std::exception& ex) {
        status_.optimizer_ready = false;
        status_.optimizer_state = std::string("update_failed:") + ex.what();
        status_.graph_health = 0.0;
        return false;
    }

    const double penalty = std::clamp(static_cast<double>(status_.candidate_conflict_count) * config_.graph_health_conflict_penalty,
                                      0.0, 1.0);
    status_.graph_health = std::clamp(1.0 - penalty, 0.0, 1.0);
    return true;
}

bool PoseGraphBackend::queryPoseMap(uint32_t id, Eigen::Isometry3d& out_pose_map) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto sym = makeSymbol(id);
    if (current_values_->exists(sym)) {
        const auto pose = current_values_->at<gtsam::Pose2>(sym);
        out_pose_map = pose2ToEigen(pose.x(), pose.y(), pose.theta());
        return true;
    }
    if (pending_values_->exists(sym)) {
        const auto pose = pending_values_->at<gtsam::Pose2>(sym);
        out_pose_map = pose2ToEigen(pose.x(), pose.y(), pose.theta());
        return true;
    }
    return false;
}

void PoseGraphBackend::markLoopCandidate(bool accepted, bool conflict, const rclcpp::Time& stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++status_.loop_candidate_count;
    if (accepted) {
        ++status_.accepted_loop_count;
        last_loop_stamp_ = stamp;
    }
    if (conflict) {
        ++status_.candidate_conflict_count;
    }
}

void PoseGraphBackend::markAnchorCandidate(bool accepted, bool conflict, const rclcpp::Time& stamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (accepted) {
        ++status_.accepted_anchor_count;
        last_anchor_stamp_ = stamp;
    }
    if (conflict) {
        ++status_.candidate_conflict_count;
    }
}

void PoseGraphBackend::setMapToOdomJumpSuppressed(bool suppressed) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.map_to_odom_jump_suppressed = suppressed;
}

PoseGraphStatus PoseGraphBackend::statusSnapshot(const rclcpp::Time& now) const {
    std::lock_guard<std::mutex> lock(mutex_);
    PoseGraphStatus snapshot = status_;
    snapshot.last_loop_age_sec = computeAgeSec(now, last_loop_stamp_);
    snapshot.last_anchor_age_sec = computeAgeSec(now, last_anchor_stamp_);
    return snapshot;
}

double PoseGraphBackend::computeAgeSec(const rclcpp::Time& now, const rclcpp::Time& stamp) {
    if (stamp.nanoseconds() <= 0 || stamp.get_clock_type() != now.get_clock_type()) {
        return -1.0;
    }
    return std::max(0.0, (now - stamp).seconds());
}

Eigen::Isometry3d PoseGraphBackend::pose2ToEigen(double x, double y, double yaw) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation().x() = x;
    pose.translation().y() = y;
    pose.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())).toRotationMatrix();
    return pose;
}

void PoseGraphBackend::eigenToPose2(const Eigen::Isometry3d& pose, double& x, double& y, double& yaw) {
    x = pose.translation().x();
    y = pose.translation().y();
    yaw = std::atan2(pose.rotation()(1, 0), pose.rotation()(0, 0));
}

}  // namespace rc26_localization
