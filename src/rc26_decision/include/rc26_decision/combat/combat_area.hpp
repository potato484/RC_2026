// 对抗区 (Combat Area) 行为树节点
#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include "rc26_decision/common/bt_action_node.hpp"
#include "rc26_interfaces/action/execute_mechanism.hpp"
#include "rc26_interfaces/action/place_kfs_grid.hpp"

namespace rc26_decision {

// 机构抬升节点 (对抗区)
class MechUpDuelAction : public BtActionNode<rc26_interfaces::action::ExecuteMechanism> {
public:
    MechUpDuelAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

protected:
    bool buildGoal(Goal& goal) override;
    BT::NodeStatus handleResult(const WrappedResult& result, uint16_t& error_code) override;
};

// 放置 KFS 到九宫格节点
class PlaceKFSGridAction : public BtActionNode<rc26_interfaces::action::PlaceKFSGrid> {
public:
    PlaceKFSGridAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

protected:
    bool buildGoal(Goal& goal) override;
    BT::NodeStatus handleResult(const WrappedResult& result, uint16_t& error_code) override;
};

// 放置 KFS 到地面节点
class PlaceKFSGroundAction : public BtActionNode<rc26_interfaces::action::ExecuteMechanism> {
public:
    PlaceKFSGroundAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

protected:
    bool buildGoal(Goal& goal) override;
    BT::NodeStatus handleResult(const WrappedResult& result, uint16_t& error_code) override;
};

// 云台控制节点
class GimbalMoveAction : public BT::StatefulActionNode {
public:
    GimbalMoveAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

// 跟随手动机器人节点
class FollowManualRobotAction : public BT::StatefulActionNode {
public:
    FollowManualRobotAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

// 注册 Combat 区域所有节点
void registerCombatAreaNodes(BT::BehaviorTreeFactory& factory);

}  // namespace rc26_decision
