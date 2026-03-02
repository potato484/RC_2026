// 对抗区 (Combat Area) 行为树节点实现
#include "rc26_decision/combat/combat_area.hpp"

#include <chrono>

#include "rc26_serial/protocol.hpp"

namespace rc26_decision {

namespace {

BT::NodeStatus handleExecuteMechanismResult(
    const BtActionNode<rc26_interfaces::action::ExecuteMechanism>::WrappedResult& result,
    uint16_t& error_code) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result || !result.result->success) {
        error_code = (result.result ? result.result->error_code : 0);
        return BT::NodeStatus::FAILURE;
    }
    error_code = 0;
    return BT::NodeStatus::SUCCESS;
}

}  // namespace

// ============================================================================
// MechUpDuelAction - 对抗区机构抬升
// ============================================================================
MechUpDuelAction::MechUpDuelAction(const std::string& name, const BT::NodeConfig& config)
    : BtActionNode<rc26_interfaces::action::ExecuteMechanism>(
          name, config, "/mechanism/execute", std::chrono::seconds(8)) {}

BT::PortsList MechUpDuelAction::providedPorts() {
    return BtActionNode<rc26_interfaces::action::ExecuteMechanism>::basePorts(8.0);
}

bool MechUpDuelAction::buildGoal(Goal& goal) {
    double timeout_sec = 8.0;
    (void)getInput("timeout_sec", timeout_sec);
    goal.command_id = static_cast<uint8_t>(CommandID::MECH_UP_DUEL);
    goal.payload.clear();
    goal.timeout_sec = static_cast<float>(timeout_sec);
    return true;
}

BT::NodeStatus MechUpDuelAction::handleResult(const WrappedResult& result, uint16_t& error_code) {
    return handleExecuteMechanismResult(result, error_code);
}

// ============================================================================
// PlaceKFSGridAction - 放置 KFS 到九宫格
// ============================================================================
PlaceKFSGridAction::PlaceKFSGridAction(const std::string& name, const BT::NodeConfig& config)
    : BtActionNode<rc26_interfaces::action::PlaceKFSGrid>(
          name, config, "/mechanism/place_kfs_grid", std::chrono::seconds(8)) {}

BT::PortsList PlaceKFSGridAction::providedPorts() {
    auto ports = BtActionNode<rc26_interfaces::action::PlaceKFSGrid>::basePorts(8.0);
    ports.insert(BT::InputPort<int>("grid_position", "九宫格位置 (1-9)"));
    return ports;
}

bool PlaceKFSGridAction::buildGoal(Goal& goal) {
    int grid_position = 0;
    if (!getInput("grid_position", grid_position) || grid_position < 1 || grid_position > 9) {
        return false;
    }
    goal.grid_position = static_cast<uint8_t>(grid_position);
    goal.layer = 1;
    return true;
}

BT::NodeStatus PlaceKFSGridAction::handleResult(const WrappedResult& result, uint16_t& error_code) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result || !result.result->success) {
        error_code = (result.result ? result.result->error_code : 0);
        return BT::NodeStatus::FAILURE;
    }
    error_code = 0;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// PlaceKFSGroundAction - 放置 KFS 到地面
// ============================================================================
PlaceKFSGroundAction::PlaceKFSGroundAction(const std::string& name, const BT::NodeConfig& config)
    : BtActionNode<rc26_interfaces::action::ExecuteMechanism>(
          name, config, "/mechanism/execute", std::chrono::seconds(8)) {}

BT::PortsList PlaceKFSGroundAction::providedPorts() {
    return BtActionNode<rc26_interfaces::action::ExecuteMechanism>::basePorts(8.0);
}

bool PlaceKFSGroundAction::buildGoal(Goal& goal) {
    double timeout_sec = 8.0;
    (void)getInput("timeout_sec", timeout_sec);
    goal.command_id = static_cast<uint8_t>(CommandID::PLACE_KFS_GROUND);
    goal.payload.clear();
    goal.timeout_sec = static_cast<float>(timeout_sec);
    return true;
}

BT::NodeStatus PlaceKFSGroundAction::handleResult(const WrappedResult& result, uint16_t& error_code) {
    return handleExecuteMechanismResult(result, error_code);
}

// ============================================================================
// GimbalMoveAction - 云台控制
// ============================================================================
GimbalMoveAction::GimbalMoveAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList GimbalMoveAction::providedPorts() {
    return {
        BT::InputPort<float>("pitch", "俯仰角"),
        BT::InputPort<float>("yaw", "偏航角"),
    };
}

BT::NodeStatus GimbalMoveAction::onStart() {
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GimbalMoveAction::onRunning() {
    return BT::NodeStatus::RUNNING;
}

void GimbalMoveAction::onHalted() {}

// ============================================================================
// FollowManualRobotAction - 跟随手动机器人
// ============================================================================
FollowManualRobotAction::FollowManualRobotAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList FollowManualRobotAction::providedPorts() {
    return {
        BT::InputPort<double>("follow_distance", 1.5, "跟随距离(米)"),
        BT::InputPort<double>("lost_timeout", 5.0, "丢失超时(秒)"),
    };
}

BT::NodeStatus FollowManualRobotAction::onStart() {
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowManualRobotAction::onRunning() {
    return BT::NodeStatus::RUNNING;
}

void FollowManualRobotAction::onHalted() {}

// ============================================================================
// 注册函数
// ============================================================================
void registerCombatAreaNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<MechUpDuelAction>("MechUpDuel");
    factory.registerNodeType<PlaceKFSGridAction>("PlaceKFSGrid");
    factory.registerNodeType<PlaceKFSGroundAction>("PlaceKFSGround");
    factory.registerNodeType<GimbalMoveAction>("GimbalMove");
    factory.registerNodeType<FollowManualRobotAction>("FollowManualRobot");
}

}  // namespace rc26_decision
