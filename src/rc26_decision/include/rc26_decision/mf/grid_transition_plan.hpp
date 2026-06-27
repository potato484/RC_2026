#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>

#include "rc26_decision/mf/merlin_map.hpp"

namespace rc26_decision {

class PlanGridTransitionAction : public BT::SyncActionNode {
public:
  PlanGridTransitionAction(const std::string &name,
                           const BT::NodeConfig &config);

  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  static double normalizeAngle(double angle_rad);
  void setTransitionError(const std::string &reason);
};

} // namespace rc26_decision
