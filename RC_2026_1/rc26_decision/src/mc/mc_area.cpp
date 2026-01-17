
// 武馆区 (MC Area) 行为树节点实现
#include "rc26_decision/mc/mc_area.hpp"

namespace rc26_decision {

// ============================================================================
// GrabTipAction - 取矛头
// ============================================================================
GrabTipAction::GrabTipAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList GrabTipAction::providedPorts() {
    return {};
}

BT::NodeStatus GrabTipAction::onStart() {




    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GrabTipAction::onRunning() {
    // TODO: 等待 GRAB_DONE 反馈

    return BT::NodeStatus::RUNNING;
}

void GrabTipAction::onHalted() {
    // TODO: 发送 STOP 指令
}

// ============================================================================
// AssembleWeaponAction - 组装兵器
// ============================================================================
AssembleWeaponAction::AssembleWeaponAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList AssembleWeaponAction::providedPorts() {

    
    return {};
}

BT::NodeStatus AssembleWeaponAction::onStart() {
    // TODO: 发送 ASSEMBLE_WEAPON 指令
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AssembleWeaponAction::onRunning() {
    // TODO: 等待 ASSEMBLE_DONE 反馈
    return BT::NodeStatus::SUCCESS;
}

void AssembleWeaponAction::onHalted() {
    // TODO: 发送 STOP 指令
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
