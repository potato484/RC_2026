#include "rc26_decision/mf/select_next_grid.hpp"

#include <cmath>
#include <string>

#include "rc26_decision/decision_failure.hpp"

namespace {

bool isExecutableStepTransition(const rc26_decision::MerlinMapManager &map,
                                int from, int to) {
  const int from_depth = map.getDepth(from);
  const int to_depth = map.getDepth(to);
  if (from_depth < 0 || to_depth < 0) {
    return false;
  }
  return std::abs(to_depth - from_depth) == 1;
}

} // namespace

namespace rc26_decision {

SelectNextGridAction::SelectNextGridAction(const std::string &name,
                                           const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList SelectNextGridAction::providedPorts() {
  return {
      BT::OutputPort<std::string>("next_action"),
      BT::OutputPort<int>("target_grid"),
  };
}

BT::NodeStatus SelectNextGridAction::tick() {
  int current = 2, exit_grid = 10;
  (void)config().blackboard->get("current_grid", current);
  (void)config().blackboard->get("exit_grid", exit_grid);

  std::shared_ptr<MerlinMapManager> map;
  (void)config().blackboard->get("merlin_map", map);
  if (!map) {
    writeDecisionFailure(config().blackboard, "SelectNextGrid",
                         "黑板缺少 merlin_map，current_grid=" +
                             std::to_string(current) +
                             "，exit_grid=" + std::to_string(exit_grid));
    return BT::NodeStatus::FAILURE;
  }

  if (map->isExitBlock(current)) {
    exit_grid = current;
  }

  const int next = findBestPathToExit(current, exit_grid, map);
  if (next > 0) {
    config().blackboard->set("merlin_last_transition_reason",
                             std::string("select_move_target"));
    setOutput("next_action", std::string("MOVE"));
    setOutput("target_grid", next);
    return BT::NodeStatus::SUCCESS;
  }

  config().blackboard->set("merlin_last_transition_reason",
                           std::string("select_wait_no_legal_target"));
  setOutput("next_action", std::string("WAIT"));
  setOutput("target_grid", current);
  return BT::NodeStatus::SUCCESS;
}

int SelectNextGridAction::findBestPathToExit(
    int current, int exit_grid, std::shared_ptr<MerlinMapManager> map) {
  int best = -1, min_dist = 100;
  for (auto dir : {MFDirection::FRONT, MFDirection::LEFT, MFDirection::RIGHT}) {
    const int adj = map->getAdjacentGrid(current, dir);
    if (adj <= 0) {
      continue;
    }

    if (!isExecutableStepTransition(*map, current, adj)) {
      continue;
    }

    const int dist = std::abs((adj - 1) / 3 - (exit_grid - 1) / 3) +
                     std::abs((adj - 1) % 3 - (exit_grid - 1) % 3);
    if (dist < min_dist) {
      min_dist = dist;
      best = adj;
    }
  }
  return best;
}

} // namespace rc26_decision
