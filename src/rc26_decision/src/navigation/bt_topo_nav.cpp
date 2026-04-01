#include "rc26_decision/navigation/bt_topo_nav.hpp"

namespace rc26_decision {

// --- NavToTopoNodeAction ---
NavToTopoNodeAction::NavToTopoNodeAction(const std::string& name, const BT::NodeConfig& config)
    : BtActionNode<NavigateTopoTarget>(
          name, config, "navigate_topo_target", std::chrono::seconds(60)) {}

BT::PortsList NavToTopoNodeAction::providedPorts() {
    auto ports = BtActionNode<NavigateTopoTarget>::basePorts(60.0);
    ports.insert(BT::InputPort<std::string>("node_id", "Target graph node ID"));
    return ports;
}

bool NavToTopoNodeAction::buildGoal(Goal& goal) {
    std::string node_id;
    if (!getInput("node_id", node_id) || node_id.empty()) return false;
    goal.target_type = NavigateTopoTarget::Goal::TARGET_NODE;
    goal.target_id = node_id;
    goal.allow_replan = true;
    // team from blackboard
    std::string team = "blue";
    (void)config().blackboard->get("team", team);
    goal.team = team;
    return true;
}

BT::NodeStatus NavToTopoNodeAction::handleResult(
    const WrappedResult& result, uint16_t& error_code) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result->success) {
        error_code = 1;
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

// --- NavToTaskPoseAction ---
NavToTaskPoseAction::NavToTaskPoseAction(const std::string& name, const BT::NodeConfig& config)
    : BtActionNode<NavigateTopoTarget>(
          name, config, "navigate_topo_target", std::chrono::seconds(60)) {}

BT::PortsList NavToTaskPoseAction::providedPorts() {
    auto ports = BtActionNode<NavigateTopoTarget>::basePorts(60.0);
    ports.insert(BT::InputPort<std::string>("task_tag", "Task tag from field graph"));
    ports.insert(BT::InputPort<int>("grid_id", "Optional Merlin grid selected by rc26_decision"));
    return ports;
}

std::string NavToTaskPoseAction::merlinGridNodeId(int grid_id) {
    if (grid_id < 1 || grid_id > 12) {
        return {};
    }
    return "mf_b" + std::to_string(grid_id);
}

bool NavToTaskPoseAction::buildGoal(Goal& goal) {
    std::string task_tag;
    if (!getInput("task_tag", task_tag) || task_tag.empty()) return false;

    int grid_id = 0;
    if (getInput("grid_id", grid_id)) {
        const auto node_id = merlinGridNodeId(grid_id);
        if (node_id.empty()) {
            return false;
        }
        goal.target_type = NavigateTopoTarget::Goal::TARGET_NODE;
        goal.target_id = node_id;
    } else {
        goal.target_type = NavigateTopoTarget::Goal::TARGET_TASK;
        goal.target_id = task_tag;
    }
    goal.allow_replan = true;
    std::string team = "blue";
    (void)config().blackboard->get("team", team);
    goal.team = team;
    return true;
}

BT::NodeStatus NavToTaskPoseAction::handleResult(
    const WrappedResult& result, uint16_t& error_code) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result->success) {
        error_code = 1;
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

// --- ExecuteTopoRouteAction ---
ExecuteTopoRouteAction::ExecuteTopoRouteAction(
    const std::string& name, const BT::NodeConfig& config)
    : BtActionNode<NavigateTopoTarget>(
          name, config, "navigate_topo_target", std::chrono::seconds(120)) {}

BT::PortsList ExecuteTopoRouteAction::providedPorts() {
    auto ports = BtActionNode<NavigateTopoTarget>::basePorts(120.0);
    ports.insert(BT::InputPort<std::string>("route_tag", "Predefined route tag"));
    return ports;
}

bool ExecuteTopoRouteAction::buildGoal(Goal& goal) {
    std::string route_tag;
    if (!getInput("route_tag", route_tag) || route_tag.empty()) return false;
    goal.target_type = NavigateTopoTarget::Goal::TARGET_ROUTE;
    goal.target_id = route_tag;
    goal.allow_replan = true;
    std::string team = "blue";
    (void)config().blackboard->get("team", team);
    goal.team = team;
    return true;
}

BT::NodeStatus ExecuteTopoRouteAction::handleResult(
    const WrappedResult& result, uint16_t& error_code) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result->success) {
        error_code = 1;
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

// --- Registration ---
void registerTopoNavNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<NavToTopoNodeAction>("NavToTopoNode");
    factory.registerNodeType<NavToTaskPoseAction>("NavToTaskPose");
    factory.registerNodeType<ExecuteTopoRouteAction>("ExecuteTopoRoute");
}

}  // namespace rc26_decision
