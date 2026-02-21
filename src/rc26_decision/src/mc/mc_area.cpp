// 武馆区 (MC Area) 行为树节点实现
#include "rc26_decision/mc/mc_area.hpp"

namespace rc26_decision {

// ============================================================================
// GrabTipAction - 取端头（ROS2 Action 客户端）
// ============================================================================
GrabTipAction::GrabTipAction(const std::string& name, const BT::NodeConfig& config)
    : BtActionNode<rc26_interfaces::action::GrabTip>(
          name, config, "/mechanism/grab_tip", std::chrono::seconds(8)) {}

BT::PortsList GrabTipAction::providedPorts() {
    return BtActionNode<rc26_interfaces::action::GrabTip>::basePorts(8.0);
}

bool GrabTipAction::buildGoal(Goal& goal) {
    (void)goal;
    return true;
}

BT::NodeStatus GrabTipAction::handleResult(const WrappedResult& result, uint16_t& error_code) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result || !result.result->success) {
        error_code = (result.result ? result.result->error_code : 0);
        return BT::NodeStatus::FAILURE;
    }
    error_code = 0;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// AssembleWeaponAction - 组装兵器（ROS2 Action 客户端）
// ============================================================================
AssembleWeaponAction::AssembleWeaponAction(const std::string& name, const BT::NodeConfig& config)
    : BtActionNode<rc26_interfaces::action::AssembleWeapon>(
          name, config, "/mechanism/assemble_weapon", std::chrono::seconds(30)) {}

BT::PortsList AssembleWeaponAction::providedPorts() {
    return BtActionNode<rc26_interfaces::action::AssembleWeapon>::basePorts(30.0);
}

bool AssembleWeaponAction::buildGoal(Goal& goal) {
    (void)goal;
    return true;
}

BT::NodeStatus AssembleWeaponAction::handleResult(const WrappedResult& result, uint16_t& error_code) {
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result || !result.result->success) {
        error_code = (result.result ? result.result->error_code : 0);
        return BT::NodeStatus::FAILURE;
    }
    error_code = 0;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// CheckManualRobotCondition - 检测手动机器人
// ============================================================================
CheckManualRobotCondition::CheckManualRobotCondition(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList CheckManualRobotCondition::providedPorts() {
    return {
        BT::InputPort<double>("distance_threshold", 0.5, "距离阈值(米)"),
        BT::InputPort<double>("static_time", 2.0, "静止时间阈值(秒)"),
    };
}

BT::NodeStatus CheckManualRobotCondition::tick() {
    // TODO: 检查手动机器人距离和静止状态
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// 注册函数
// ============================================================================
void registerMCAreaNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<GrabTipAction>("GrabTip");
    factory.registerNodeType<AssembleWeaponAction>("AssembleWeapon");
    factory.registerNodeType<CheckManualRobotCondition>("CheckManualRobot");
}

}  // namespace rc26_decision
