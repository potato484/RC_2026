#include "rc26_decision/navigation/bt_localization_anchor_recover.hpp"

#include <algorithm>
#include <string>

#include "rc26_decision/navigation/smart_waypoint_navigator.hpp"
#include "rc26_decision/navigation/waypoint_manager.hpp"
#include "rc26_interfaces/msg/localization_health.hpp"

namespace rc26_decision {

LocalizationAnchorRecover::LocalizationAnchorRecover(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList LocalizationAnchorRecover::providedPorts() {
    return {};
}

BT::NodeStatus LocalizationAnchorRecover::onStart() {
    recovery_targets_.clear();
    current_target_idx_ = 0U;
    active_goal_ = false;

    if (!config().blackboard->get("waypoint_manager", waypoint_manager_) || !waypoint_manager_) {
        return BT::NodeStatus::FAILURE;
    }
    if (!config().blackboard->get("smart_waypoint_navigator", navigator_) || !navigator_) {
        return BT::NodeStatus::FAILURE;
    }

    if (checkRecovered()) {
        return BT::NodeStatus::SUCCESS;
    }
    if (!loadRecoveryTargets()) {
        return BT::NodeStatus::FAILURE;
    }
    if (!startCurrentTarget()) {
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LocalizationAnchorRecover::onRunning() {
    if (!navigator_ || !waypoint_manager_) {
        return BT::NodeStatus::FAILURE;
    }

    if (checkRecovered()) {
        navigator_->cancelAndStop();
        return BT::NodeStatus::SUCCESS;
    }

    if (!active_goal_) {
        if (!startCurrentTarget()) {
            return BT::NodeStatus::FAILURE;
        }
    }

    const auto status = navigator_->tick();
    if (status == SmartWaypointNavigator::Status::Running) {
        return BT::NodeStatus::RUNNING;
    }
    if (status == SmartWaypointNavigator::Status::Succeeded) {
        active_goal_ = false;
        if (checkRecovered()) {
            return BT::NodeStatus::SUCCESS;
        }
        ++current_target_idx_;
        if (current_target_idx_ >= recovery_targets_.size()) {
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    active_goal_ = false;
    ++current_target_idx_;
    if (current_target_idx_ >= recovery_targets_.size()) {
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

void LocalizationAnchorRecover::onHalted() {
    if (navigator_) {
        navigator_->cancelAndStop();
    }
    recovery_targets_.clear();
    current_target_idx_ = 0U;
    active_goal_ = false;
}

bool LocalizationAnchorRecover::loadRecoveryTargets() {
    std::vector<std::string> retry_waypoints;
    std::vector<std::string> anchor_waypoints;
    (void)config().blackboard->get("loc_retry_waypoints", retry_waypoints);
    (void)config().blackboard->get("loc_anchor_waypoints", anchor_waypoints);

    if (retry_waypoints.empty()) {
        retry_waypoints = {"loc_retry_zone_1"};
    }
    if (anchor_waypoints.empty()) {
        anchor_waypoints = {"loc_anchor_1", "loc_anchor_2"};
    }

    auto append_unique = [this](const std::vector<std::string>& names) {
        for (const auto& name : names) {
            if (name.empty()) {
                continue;
            }
            if (std::find(recovery_targets_.begin(), recovery_targets_.end(), name) == recovery_targets_.end()) {
                recovery_targets_.push_back(name);
            }
        }
    };
    append_unique(retry_waypoints);
    append_unique(anchor_waypoints);
    return !recovery_targets_.empty();
}

bool LocalizationAnchorRecover::startCurrentTarget() {
    if (!waypoint_manager_ || !navigator_) {
        return false;
    }
    while (current_target_idx_ < recovery_targets_.size()) {
        const std::string& target_name = recovery_targets_[current_target_idx_];
        const SmartWaypointSpec* waypoint = waypoint_manager_->find(target_name);
        if (!waypoint) {
            ++current_target_idx_;
            continue;
        }
        if (!navigator_->start(*waypoint)) {
            ++current_target_idx_;
            continue;
        }
        active_goal_ = true;
        return true;
    }
    return false;
}

bool LocalizationAnchorRecover::checkRecovered() const {
    int loc_level = static_cast<int>(rc26_interfaces::msg::LocalizationHealth::RED);
    (void)config().blackboard->get("loc_level", loc_level);
    return loc_level <= static_cast<int>(rc26_interfaces::msg::LocalizationHealth::YELLOW);
}

}  // namespace rc26_decision
