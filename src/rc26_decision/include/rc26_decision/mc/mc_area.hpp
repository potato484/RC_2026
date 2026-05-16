/*
 * @Author: potato potato@potato.com
 * @Date: 2025-12-29 20:30:49
 * @LastEditors: potato potato@potato.com
 * @LastEditTime: 2026-01-02 21:33:02
 * @FilePath: /RC_2026/src/rc26_decision/include/rc26_decision/mc_area.hpp
 * @Description: 武馆区 (MC Area) 行为树节点
 */
#pragma once

#include <behaviortree_cpp/bt_factory.h>

#include "rc26_decision/common/bt_action_node.hpp"
#include "rc26_interfaces/action/assemble_weapon.hpp"
#include "rc26_interfaces/action/grab_tip.hpp"

namespace rc26_decision {

// 取端头节点
class GrabTipAction : public BtActionNode<rc26_interfaces::action::GrabTip> {
public:
    GrabTipAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

protected:
    bool buildGoal(Goal& goal) override;
    BT::NodeStatus handleResult(const WrappedResult& result, uint16_t& error_code) override;
};

// 组装兵器节点
class AssembleWeaponAction : public BtActionNode<rc26_interfaces::action::AssembleWeapon> {
public:
    AssembleWeaponAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

protected:
    bool buildGoal(Goal& goal) override;
    BT::NodeStatus handleResult(const WrappedResult& result, uint16_t& error_code) override;
};

// 检测手动机器人条件节点
class CheckManualRobotCondition : public BT::ConditionNode {
public:
    CheckManualRobotCondition(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

// 注册 MC 区域所有节点
void registerMCAreaNodes(BT::BehaviorTreeFactory& factory);

}  // namespace rc26_decision
