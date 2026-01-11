/*
 * @Author: potato potato@potato.com
 * @Date: 2025-12-29 20:31:03
 * @LastEditors: potato potato@potato.com
 * @LastEditTime: 2026-01-02 21:33:34
 * @FilePath: /RC_2026/RC_2026_1/rc26_decision/include/rc26_decision/combat_area.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
// 对抗区 (Combat Area) 行为树节点
#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include "rc26_serial/protocol.hpp"

namespace rc26_decision
{

// 机构抬升节点 (对抗区)
class MechUpDuelAction : public BT::StatefulActionNode
{
public:
    MechUpDuelAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

// 放置 KFS 到九宫格节点
class PlaceKFSGridAction : public BT::StatefulActionNode
{
public:
    PlaceKFSGridAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

// 放置 KFS 到地面节点
class PlaceKFSGroundAction : public BT::StatefulActionNode
{
public:
    PlaceKFSGroundAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

// 云台控制节点
class GimbalMoveAction : public BT::StatefulActionNode
{
public:
    GimbalMoveAction(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

// 跟随手动机器人节点
class FollowManualRobotAction : public BT::StatefulActionNode
{
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
