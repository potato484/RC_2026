#pragma once

#include <string>

#include <behaviortree_cpp/bt_factory.h>

namespace rc26_decision {

class CheckExitCondition : public BT::ConditionNode {
public:
  CheckExitCondition(const std::string &name, const BT::NodeConfig &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};

} // namespace rc26_decision
