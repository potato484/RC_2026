// 梅林区 (MF Area) 行为树节点实现
#include "rc26_decision/mf/mf_area.hpp"

namespace rc26_decision {

// ============================================================================
// StairClimbAction - 上阶梯
// ============================================================================
StairClimbAction::StairClimbAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList StairClimbAction::providedPorts() {
    return {};
}

BT::NodeStatus StairClimbAction::onStart() {
    // TODO: 发送 NAV_STAIR_UP 指令
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StairClimbAction::onRunning() {
    // TODO: 等待 STAIR_CLIMB_DONE 反馈
    return BT::NodeStatus::SUCCESS;
}

void StairClimbAction::onHalted() {
    // TODO: 发送 STOP 指令
}

// ============================================================================
// StairDescendAction - 下阶梯
// ============================================================================
StairDescendAction::StairDescendAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList StairDescendAction::providedPorts() {
    return {};
}

BT::NodeStatus StairDescendAction::onStart() {
    // TODO: 发送 NAV_STAIR_DOWN 指令
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StairDescendAction::onRunning() {
    // TODO: 等待 STAIR_DESCEND_DONE 反馈
    return BT::NodeStatus::SUCCESS;
}

void StairDescendAction::onHalted() {
    // TODO: 发送 STOP 指令
}

// ============================================================================
// GrabKFSAction - 夹取 KFS
// ============================================================================
GrabKFSAction::GrabKFSAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList GrabKFSAction::providedPorts() {
    return {
        BT::InputPort<int>("grid_id", "目标格子 ID (1-12)"),
    };
}

BT::NodeStatus GrabKFSAction::onStart() {
    // TODO: 发送 GRAB_KFS 指令
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GrabKFSAction::onRunning() {
    // TODO: 等待 GRAB_KFS_DONE 反馈
    return BT::NodeStatus::SUCCESS;
}

void GrabKFSAction::onHalted() {
    // TODO: 发送 STOP 指令
}

// ============================================================================
// MechUpMerlinAction - 梅林区机构抬升
// ============================================================================
MechUpMerlinAction::MechUpMerlinAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList MechUpMerlinAction::providedPorts() {
    return {};
}

BT::NodeStatus MechUpMerlinAction::onStart() {
    // TODO: 发送 MECH_UP_MERLIN 指令
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MechUpMerlinAction::onRunning() {
    // TODO: 等待 MECH_UP_MERLIN_DONE 反馈
    return BT::NodeStatus::SUCCESS;
}

void MechUpMerlinAction::onHalted() {
    // TODO: 发送 STOP 指令
}

// ============================================================================
// MechDownMerlinAction - 梅林区机构下降
// ============================================================================
MechDownMerlinAction::MechDownMerlinAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList MechDownMerlinAction::providedPorts() {
    return {};
}

BT::NodeStatus MechDownMerlinAction::onStart() {
    // TODO: 发送 MECH_DOWN_MERLIN 指令
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MechDownMerlinAction::onRunning() {
    // TODO: 等待 MECH_DOWN_MERLIN_DONE 反馈
    return BT::NodeStatus::SUCCESS;
}

void MechDownMerlinAction::onHalted() {
    // TODO: 发送 STOP 指令
}

// ============================================================================
// RotateAction - 旋转
// ============================================================================
RotateAction::RotateAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList RotateAction::providedPorts() {
    return {
        BT::InputPort<int>("angle", "旋转角度 (90, -90, 180, -180)"),
    };
}

BT::NodeStatus RotateAction::onStart() {
    // TODO: 根据 angle 发送对应的 ROTATE 指令
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RotateAction::onRunning() {
    // TODO: 等待对应的 ROTATE_DONE 反馈
    return BT::NodeStatus::SUCCESS;
}

void RotateAction::onHalted() {
    // TODO: 发送 STOP 指令
}

// ============================================================================
// CheckKFSCondition - 检查 KFS 存在
// ============================================================================
CheckKFSCondition::CheckKFSCondition(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList CheckKFSCondition::providedPorts() {
    return {
        BT::InputPort<int>("grid_id", "目标格子 ID (1-12)"),
        BT::InputPort<std::string>("expected_state", "AUTO_KFS", "期望状态"),
    };
}

BT::NodeStatus CheckKFSCondition::tick() {
    // TODO: 检查指定格子的 KFS 状态
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// CheckLoadCondition - 检查装载数量
// ============================================================================
CheckLoadCondition::CheckLoadCondition(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList CheckLoadCondition::providedPorts() {
    return {
        BT::InputPort<int>("min_load", 0, "最小装载数"),
        BT::InputPort<int>("max_load", 3, "最大装载数"),
    };
}

BT::NodeStatus CheckLoadCondition::tick() {
    // TODO: 检查当前装载数量是否在范围内
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// 注册函数
// ============================================================================
void registerMFAreaNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<StairClimbAction>("StairClimb");
    factory.registerNodeType<StairDescendAction>("StairDescend");
    factory.registerNodeType<GrabKFSAction>("GrabKFS");
    factory.registerNodeType<MechUpMerlinAction>("MechUpMerlin");
    factory.registerNodeType<MechDownMerlinAction>("MechDownMerlin");
    factory.registerNodeType<RotateAction>("Rotate");
    factory.registerNodeType<CheckKFSCondition>("CheckKFS");
    factory.registerNodeType<CheckLoadCondition>("CheckLoad");
}

}  // namespace rc26_decision
