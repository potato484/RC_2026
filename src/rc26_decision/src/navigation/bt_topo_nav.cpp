#include "rc26_decision/navigation/bt_topo_nav.hpp"

#include <unordered_map>

namespace rc26_decision {

namespace {

constexpr uint16_t kErrorActionFailed = 1;

const std::unordered_map<std::string, uint16_t>& failureCodeMap() {
    static const std::unordered_map<std::string, uint16_t> codes{
        {"TEAM_MISMATCH", 100},
        {"NO_TF", 101},
        {"NO_PATH", 102},
        {"INVALID_TARGET_TYPE", 103},
        {"LOC_RED_HOLD", 104},
        {"EDGE_EXEC_FAILED", 105},
        {"LOCAL_WAIT_TIMEOUT", 106},
        {"LOCAL_RECOVERY_TIMEOUT", 107},
        {"LOCAL_COLLISION_BLOCKED", 108},
        {"LOCAL_HOLD_TIMEOUT", 109},
        {"LOCAL_ACCEPT_TIMEOUT", 110},
        {"LOCAL_TRACKING_TIMEOUT", 111},
        {"LOCAL_EXEC_TIMEOUT", 112},
        {"LOCAL_REPLAN_REQUESTED", 113},
        {"MODE_REQUEST_FAILED", 114},
        {"EMPTY_CORRIDOR", 115},
        {"MAX_REPLAN_EXCEEDED", 116},
        {"CANCELLED", 117},
        {"THREAD_START_FAILED", 118},
        {"INTERNAL_ERROR", 119},
        {"ACTION_ABORTED", 120},
        {"ACTION_CANCELED", 121},
        {"ACTION_UNKNOWN", 122},
    };
    return codes;
}

BT::Blackboard::Ptr blackboardOf(const BT::TreeNode& node) {
    return node.config().blackboard;
}

void resetNavRuntimeBlackboard(const BT::Blackboard::Ptr& blackboard) {
    if (!blackboard) {
        return;
    }
    blackboard->set("nav_last_exec_state", std::string("PENDING"));
    blackboard->set("nav_last_failure_code", std::string(""));
    blackboard->set("nav_last_failure_reason", std::string(""));
    blackboard->set("nav_last_active_node_id", std::string(""));
    blackboard->set("nav_last_active_edge_id", std::string(""));
    blackboard->set("nav_last_replan_count", static_cast<int>(0));
}

void writeFeedbackToBlackboard(
    const BT::Blackboard::Ptr& blackboard,
    const NavigateTopoTarget::Feedback& feedback) {
    if (!blackboard) {
        return;
    }
    blackboard->set("nav_last_exec_state", feedback.exec_state);
    blackboard->set("nav_last_active_node_id", feedback.active_node_id);
    blackboard->set("nav_last_active_edge_id", feedback.active_edge_id);
    blackboard->set("nav_last_replan_count", static_cast<int>(feedback.replan_count));
}

uint16_t mapFailureCodeToError(const std::string& failure_code) {
    if (failure_code.empty()) {
        return kErrorActionFailed;
    }
    const auto& codes = failureCodeMap();
    const auto it = codes.find(failure_code);
    return it == codes.end() ? kErrorActionFailed : it->second;
}

std::string failureCodeFromWrappedResult(
    const NavigateTopoTarget::Result::SharedPtr& result,
    const rclcpp_action::ResultCode code) {
    if (result && !result->failure_code.empty()) {
        return result->failure_code;
    }
    switch (code) {
        case rclcpp_action::ResultCode::ABORTED:
            return "ACTION_ABORTED";
        case rclcpp_action::ResultCode::CANCELED:
            return "ACTION_CANCELED";
        default:
            return "ACTION_UNKNOWN";
    }
}

std::string failureReasonFromWrappedResult(
    const NavigateTopoTarget::Result::SharedPtr& result,
    const rclcpp_action::ResultCode code) {
    if (result && !result->failure_reason.empty()) {
        return result->failure_reason;
    }
    switch (code) {
        case rclcpp_action::ResultCode::ABORTED:
            return "navigate_topo_target aborted";
        case rclcpp_action::ResultCode::CANCELED:
            return "navigate_topo_target canceled";
        default:
            return "navigate_topo_target failed";
    }
}

BT::NodeStatus handleTopoResultCommon(
    BT::TreeNode& node,
    const rclcpp_action::ClientGoalHandle<NavigateTopoTarget>::WrappedResult& result,
    uint16_t& error_code) {
    const auto blackboard = blackboardOf(node);

    if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
        result.result && result.result->success) {
        if (blackboard) {
            blackboard->set("nav_last_exec_state", std::string("PASS"));
            blackboard->set("nav_last_failure_code", std::string(""));
            blackboard->set("nav_last_failure_reason", std::string(""));
        }
        error_code = 0;
        return BT::NodeStatus::SUCCESS;
    }

    const std::string failure_code = failureCodeFromWrappedResult(result.result, result.code);
    const std::string failure_reason = failureReasonFromWrappedResult(result.result, result.code);
    if (blackboard) {
        blackboard->set("nav_last_exec_state", std::string("FAILED"));
        blackboard->set("nav_last_failure_code", failure_code);
        blackboard->set("nav_last_failure_reason", failure_reason);
    }
    error_code = mapFailureCodeToError(failure_code);
    return BT::NodeStatus::FAILURE;
}

void applyGoalDefaults(
    BT::TreeNode& node,
    NavigateTopoTarget::Goal& goal) {
    std::string team = "blue";
    if (auto blackboard = blackboardOf(node)) {
        (void)blackboard->get("team", team);
        resetNavRuntimeBlackboard(blackboard);
    }
    goal.allow_replan = true;
    goal.team = team;
}

void handleTopoFeedbackCommon(
    BT::TreeNode& node,
    const std::shared_ptr<const NavigateTopoTarget::Feedback>& feedback) {
    if (!feedback) {
        return;
    }
    writeFeedbackToBlackboard(blackboardOf(node), *feedback);
}

}  // namespace

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
    if (!getInput("node_id", node_id) || node_id.empty()) {
        return false;
    }
    goal.target_type = NavigateTopoTarget::Goal::TARGET_NODE;
    goal.target_id = node_id;
    applyGoalDefaults(*this, goal);
    return true;
}

void NavToTopoNodeAction::onFeedback(const std::shared_ptr<const Feedback>& feedback) {
    handleTopoFeedbackCommon(*this, feedback);
}

BT::NodeStatus NavToTopoNodeAction::handleResult(
    const WrappedResult& result, uint16_t& error_code) {
    return handleTopoResultCommon(*this, result, error_code);
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
    if (!getInput("task_tag", task_tag) || task_tag.empty()) {
        return false;
    }

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
    applyGoalDefaults(*this, goal);
    return true;
}

void NavToTaskPoseAction::onFeedback(const std::shared_ptr<const Feedback>& feedback) {
    handleTopoFeedbackCommon(*this, feedback);
}

BT::NodeStatus NavToTaskPoseAction::handleResult(
    const WrappedResult& result, uint16_t& error_code) {
    return handleTopoResultCommon(*this, result, error_code);
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
    if (!getInput("route_tag", route_tag) || route_tag.empty()) {
        return false;
    }
    goal.target_type = NavigateTopoTarget::Goal::TARGET_ROUTE;
    goal.target_id = route_tag;
    applyGoalDefaults(*this, goal);
    return true;
}

void ExecuteTopoRouteAction::onFeedback(const std::shared_ptr<const Feedback>& feedback) {
    handleTopoFeedbackCommon(*this, feedback);
}

BT::NodeStatus ExecuteTopoRouteAction::handleResult(
    const WrappedResult& result, uint16_t& error_code) {
    return handleTopoResultCommon(*this, result, error_code);
}

// --- Registration ---
void registerTopoNavNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<NavToTopoNodeAction>("NavToTopoNode");
    factory.registerNodeType<NavToTaskPoseAction>("NavToTaskPose");
    factory.registerNodeType<ExecuteTopoRouteAction>("ExecuteTopoRoute");
}

}  // namespace rc26_decision
