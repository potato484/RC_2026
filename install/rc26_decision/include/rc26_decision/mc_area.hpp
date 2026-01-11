/*
 * @Author: potato potato@potato.com
 * @Date: 2025-12-29 20:30:49
 * @LastEditors: potato potato@potato.com
 * @LastEditTime: 2026-01-02 21:33:02
 * @FilePath: /RC_2026/RC_2026_1/rc26_decision/include/rc26_decision/mc_area.hpp
 * @Description: 武馆区 (MC Area) 行为树节点
 */
#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include "rc26_serial/protocol.hpp"

namespace rc26_decision
{

// 取矛头节点
class GrabTipAction : public BT::StatefulActionNode
{
public:
    GrabTipAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

// 组装兵器节点
class AssembleWeaponAction : public BT::StatefulActionNode
{
public:
    AssembleWeaponAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

// 检测手动机器人条件节点
class CheckManualRobotCondition : public BT::ConditionNode
{
public:
    CheckManualRobotCondition(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

// 注册 MC 区域所有节点
void registerMCAreaNodes(BT::BehaviorTreeFactory& factory);

}  // namespace rc26_decision
