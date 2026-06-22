#pragma once

#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>

#include "rc26_decision/mf/merlin_map.hpp"

namespace rc26_decision {

class SelectNextGridAction : public BT::SyncActionNode {
public:
  SelectNextGridAction(const std::string &name,
                       const BT::NodeConfig &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  int findBestPathToExit(int current, int exit_grid,
                         std::shared_ptr<MerlinMapManager> map);
};

} // namespace rc26_decision
