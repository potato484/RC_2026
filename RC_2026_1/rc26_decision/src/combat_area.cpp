// 对抗区 (Combat Area) 行为树节点实现
#include "rc26_decision/combat_area.hpp"

namespace rc26_decision
{

// ============================================================================
// MechUpDuelAction - 对抗区机构抬升
// ============================================================================
MechUpDuelAction::MechUpDuelAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
{
}

BT::PortsList MechUpDuelAction::providedPorts()
{
    return {};
}

BT::NodeStatus MechUpDuelAction::onStart()
{
    // TODO: 发送 MECH_UP_DUEL 指令
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MechUpDuelAction::onRunning()
{
    // TODO: 等待 MECH_UP_DUEL_DONE 反馈
    return BT::NodeStatus::SUCCESS;
}

void MechUpDuelAction::onHalted()
{
    // TODO: 发送 STOP 指令
}

// ============================================================================
// PlaceKFSGridAction - 放置 KFS 到九宫格
// ============================================================================
PlaceKFSGridAction::PlaceKFSGridAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
{
}

BT::PortsList PlaceKFSGridAction::providedPorts()
{
    return {
        BT::InputPort<int>("grid_position", "九宫格位置 (1-9)"),
    };
}

BT::NodeStatus PlaceKFSGridAction::onStart()
{
    // TODO: 发送 PLACE_KFS_GRID 指令
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PlaceKFSGridAction::onRunning()
{
    // TODO: 等待 PLACE_KFS_GRID_DONE 反馈
    return BT::NodeStatus::SUCCESS;
}

void PlaceKFSGridAction::onHalted()
{
    // TODO: 发送 STOP 指令
}

// ============================================================================
// PlaceKFSGroundAction - 放置 KFS 到地面
// ============================================================================
PlaceKFSGroundAction::PlaceKFSGroundAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
{
}

BT::PortsList PlaceKFSGroundAction::providedPorts()
{
    return {};
}

BT::NodeStatus PlaceKFSGroundAction::onStart()
{
    // TODO: 发送 PLACE_KFS_GROUND 指令
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PlaceKFSGroundAction::onRunning()
{
    // TODO: 等待 PLACE_KFS_GROUND_DONE 反馈
    return BT::NodeStatus::SUCCESS;
}

void PlaceKFSGroundAction::onHalted()
{
    // TODO: 发送 STOP 指令
}

// ============================================================================
// GimbalMoveAction - 云台控制
// ============================================================================
GimbalMoveAction::GimbalMoveAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
{
}

BT::PortsList GimbalMoveAction::providedPorts()
{
    return {
        BT::InputPort<float>("pitch", "俯仰角"),
        BT::InputPort<float>("yaw", "偏航角"),
    };
}

BT::NodeStatus GimbalMoveAction::onStart()
{
    // TODO: 发送 GIMBAL_MOVE 指令
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GimbalMoveAction::onRunning()
{
    // TODO: 等待 GIMBAL_DONE 反馈
    return BT::NodeStatus::SUCCESS;
}

void GimbalMoveAction::onHalted()
{
    // TODO: 发送 GIMBAL_STOP 指令
}

// ============================================================================
// FollowManualRobotAction - 跟随手动机器人
// ============================================================================
FollowManualRobotAction::FollowManualRobotAction(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
{
}

BT::PortsList FollowManualRobotAction::providedPorts()
{
    return {
        BT::InputPort<double>("follow_distance", 1.5, "跟随距离(米)"),
        BT::InputPort<double>("lost_timeout", 5.0, "丢失超时(秒)"),
    };
}

BT::NodeStatus FollowManualRobotAction::onStart()
{
    // TODO: 开始跟随模式
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowManualRobotAction::onRunning()
{
    // TODO: 持续跟随或检测丢失
    return BT::NodeStatus::SUCCESS;
}

void FollowManualRobotAction::onHalted()
{
    // TODO: 停止跟随
}

// ============================================================================
// 注册函数
// ============================================================================
void registerCombatAreaNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<MechUpDuelAction>("MechUpDuel");
    factory.registerNodeType<PlaceKFSGridAction>("PlaceKFSGrid");
    factory.registerNodeType<PlaceKFSGroundAction>("PlaceKFSGround");
    factory.registerNodeType<GimbalMoveAction>("GimbalMove");
    factory.registerNodeType<FollowManualRobotAction>("FollowManualRobot");
}

}  // namespace rc26_decision
