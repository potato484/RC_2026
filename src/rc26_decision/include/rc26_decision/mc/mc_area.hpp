// 武馆区 (MC Area) 行为树节点：导航 → 视觉伺服夹取 → 原地旋转180° → 无限期等待
#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

namespace rc26_decision {

// 声明并读取全部 mc_* 运行参数，写入黑板（McParams 结构 + 导航目标键）
void loadMCParams(rclcpp::Node& node, const BT::Blackboard::Ptr& blackboard);

// 注册武馆区所有行为树节点
void registerMCAreaNodes(BT::BehaviorTreeFactory& factory);

}  // namespace rc26_decision
