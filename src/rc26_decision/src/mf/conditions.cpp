#include "rc26_decision/mf/conditions.hpp"

#include <memory>

#include "rc26_decision/mf/merlin_map.hpp"

namespace rc26_decision {

CheckExitCondition::CheckExitCondition(const std::string &name,
                                       const BT::NodeConfig &config)
    : BT::ConditionNode(name, config) {}

BT::PortsList CheckExitCondition::providedPorts() { return {}; }

BT::NodeStatus CheckExitCondition::tick() {
  int current = 0;
  (void)config().blackboard->get("current_grid", current);
  std::shared_ptr<MerlinMapManager> map;
  if (!config().blackboard->get("merlin_map", map) || !map) {
    return BT::NodeStatus::FAILURE;
  }
  const bool at_exit = map->isExitBlock(current);
  return at_exit ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

} // namespace rc26_decision
