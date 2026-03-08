#include "rc26_decision/navigation/bt_nav_to_smart_point.hpp"

#include <memory>

#include "rc26_decision/navigation/bt_check_localization_health.hpp"
#include "rc26_decision/navigation/bt_localization_anchor_recover.hpp"
#include "rc26_decision/navigation/bt_localization_observe_spin.hpp"
#include "rc26_decision/navigation/smart_waypoint_navigator.hpp"
#include "rc26_decision/navigation/waypoint_manager.hpp"

namespace rc26_decision {

NavToSmartPointAction::NavToSmartPointAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList NavToSmartPointAction::providedPorts() {
    return {
        BT::InputPort<std::string>("target_name", "Semantic waypoint name (YAML key)"),
    };
}

BT::NodeStatus NavToSmartPointAction::onStart() {
    std::string target_name;
    if (!getInput("target_name", target_name) || target_name.empty()) {
        return BT::NodeStatus::FAILURE;
    }

    std::shared_ptr<WaypointManager> waypoint_manager;
    if (!config().blackboard->get("waypoint_manager", waypoint_manager) || !waypoint_manager) {
        return BT::NodeStatus::FAILURE;
    }

    std::shared_ptr<SmartWaypointNavigator> navigator;
    if (!config().blackboard->get("smart_waypoint_navigator", navigator) || !navigator) {
        return BT::NodeStatus::FAILURE;
    }

    const SmartWaypointSpec* wp = waypoint_manager->find(target_name);
    if (!wp) {
        return BT::NodeStatus::FAILURE;
    }

    return navigator->start(*wp) ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
}

BT::NodeStatus NavToSmartPointAction::onRunning() {
    std::shared_ptr<SmartWaypointNavigator> navigator;
    if (!config().blackboard->get("smart_waypoint_navigator", navigator) || !navigator) {
        return BT::NodeStatus::FAILURE;
    }

    const auto st = navigator->tick();
    switch (st) {
    case SmartWaypointNavigator::Status::Running:
        return BT::NodeStatus::RUNNING;
    case SmartWaypointNavigator::Status::Succeeded:
        return BT::NodeStatus::SUCCESS;
    default:
        return BT::NodeStatus::FAILURE;
    }
}

void NavToSmartPointAction::onHalted() {
    std::shared_ptr<SmartWaypointNavigator> navigator;
    if (config().blackboard->get("smart_waypoint_navigator", navigator) && navigator) {
        navigator->cancelAndStop();
    }
}

void registerNavigationNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<NavToSmartPointAction>("NavToSmartPoint");
    factory.registerNodeType<CheckLocalizationHealth>("CheckLocalizationHealth");
    factory.registerNodeType<LocalizationObserveSpin>("LocalizationObserveSpin");
    factory.registerNodeType<LocalizationAnchorRecover>("LocalizationAnchorRecover");
}

}  // namespace rc26_decision
