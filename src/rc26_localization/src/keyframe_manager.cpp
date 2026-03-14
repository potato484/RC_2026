#include "rc26_localization/keyframe_manager.hpp"

#include <algorithm>
#include <cmath>

namespace rc26_localization {

KeyframeManager::KeyframeManager() : KeyframeManager(Config{}) {}

KeyframeManager::KeyframeManager(const Config& cfg) : config_(cfg) {}

void KeyframeManager::setConfig(const Config& cfg) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = cfg;
}

KeyframeManager::Config KeyframeManager::getConfig() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
}

void KeyframeManager::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    keyframes_.clear();
    next_id_ = 1U;
    last_observation_valid_ = false;
    last_control_degraded_ = false;
    last_h_min_eig_ = 0.0;
    last_sigma_xy_ = 0.0;
}

bool KeyframeManager::shouldCreate(const KeyframeData& candidate, std::string& reason) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto update_observation = [this, &candidate]() {
        last_observation_valid_ = true;
        last_control_degraded_ = candidate.control_degraded;
        last_h_min_eig_ = candidate.h_min_eig;
        last_sigma_xy_ = candidate.sigma_xy;
    };

    if (keyframes_.empty()) {
        reason = "bootstrap";
        update_observation();
        return true;
    }

    const auto& last = keyframes_.back();
    const double dt = std::max(0.0, (candidate.stamp - last.stamp).seconds());
    const Eigen::Vector3d dtrans = candidate.pose_odom.translation() - last.pose_odom.translation();
    const double dxy = std::hypot(dtrans.x(), dtrans.y());
    const double yaw_last = yawFromPose(last.pose_odom);
    const double yaw_now = yawFromPose(candidate.pose_odom);
    const double dyaw_deg = std::abs(std::atan2(std::sin(yaw_now - yaw_last), std::cos(yaw_now - yaw_last))) * 180.0 / M_PI;

    if (dxy >= config_.translation_thresh_m) {
        reason = "translation_threshold";
        update_observation();
        return true;
    }
    if (dyaw_deg >= config_.yaw_thresh_deg) {
        reason = "yaw_threshold";
        update_observation();
        return true;
    }
    if (dt >= config_.time_thresh_sec) {
        reason = "time_threshold";
        update_observation();
        return true;
    }
    if (config_.trigger_on_control_degraded_rising && candidate.control_degraded &&
        (!last_observation_valid_ || !last_control_degraded_)) {
        reason = "control_degraded_rising";
        update_observation();
        return true;
    }
    if (config_.trigger_on_hessian_drop && candidate.h_min_eig <= config_.hessian_alert_eig_min &&
        (!last_observation_valid_ || last_h_min_eig_ > config_.hessian_alert_eig_min)) {
        reason = "hessian_drop";
        update_observation();
        return true;
    }
    if (config_.trigger_on_sigma_cross && candidate.sigma_xy >= config_.sigma_xy_alert_min &&
        (!last_observation_valid_ || last_sigma_xy_ < config_.sigma_xy_alert_min)) {
        reason = "sigma_cross";
        update_observation();
        return true;
    }
    reason = "none";
    update_observation();
    return false;
}

KeyframeData KeyframeManager::push(KeyframeData candidate) {
    std::lock_guard<std::mutex> lock(mutex_);
    candidate.id = next_id_++;
    keyframes_.push_back(candidate);
    last_control_degraded_ = candidate.control_degraded;
    last_h_min_eig_ = candidate.h_min_eig;
    last_sigma_xy_ = candidate.sigma_xy;
    return candidate;
}

std::optional<KeyframeData> KeyframeManager::latest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (keyframes_.empty()) {
        return std::nullopt;
    }
    return keyframes_.back();
}

std::optional<KeyframeData> KeyframeManager::previous() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (keyframes_.size() < 2) {
        return std::nullopt;
    }
    return keyframes_[keyframes_.size() - 2];
}

std::optional<KeyframeData> KeyframeManager::getById(uint32_t id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& keyframe : keyframes_) {
        if (keyframe.id == id) {
            return keyframe;
        }
    }
    return std::nullopt;
}

size_t KeyframeManager::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return keyframes_.size();
}

double KeyframeManager::yawFromPose(const Eigen::Isometry3d& pose) {
    return std::atan2(pose.rotation()(1, 0), pose.rotation()(0, 0));
}

}  // namespace rc26_localization
