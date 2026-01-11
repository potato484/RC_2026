// 梅林区 (MF Area) 行为树节点
#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include "rc26_serial/protocol.hpp"

namespace rc26_decision
{

// 上阶梯节点
class StairClimbAction : public BT::StatefulActionNode
{
public:
    StairClimbAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

// 下阶梯节点
class StairDescendAction : public BT::StatefulActionNode
{
public:
    StairDescendAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

// 夹取 KFS 节点
class GrabKFSAction : public BT::StatefulActionNode
{
public:
    GrabKFSAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

// 机构抬升节点 (梅林区)
class MechUpMerlinAction : public BT::StatefulActionNode
{
public:
    MechUpMerlinAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

// 机构下降节点 (梅林区)
class MechDownMerlinAction : public BT::StatefulActionNode
{
public:
    MechDownMerlinAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

// 旋转节点
class RotateAction : public BT::StatefulActionNode
{
public:
    RotateAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

// 检查 KFS 存在条件节点
class CheckKFSCondition : public BT::ConditionNode
{
public:
    CheckKFSCondition(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

// 检查装载数量条件节点
class CheckLoadCondition : public BT::ConditionNode
{
public:
    CheckLoadCondition(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

// 注册 MF 区域所有节点
void registerMFAreaNodes(BT::BehaviorTreeFactory& factory);

}  // namespace rc26_decision
