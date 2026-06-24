#include "rc26_decision/mf/grid_transition_plan.hpp"

#include <cmath>

namespace rc26_decision {

PlanGridTransitionAction::PlanGridTransitionAction(
    const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList PlanGridTransitionAction::providedPorts() {
  return {
      BT::InputPort<int>("target_grid", "目标格子 ID (1-12)"),
      BT::OutputPort<double>("target_yaw_rad", "本次格间动作目标 yaw"),
  };
}

BT::NodeStatus PlanGridTransitionAction::tick() {
  int target_grid = 0;
  if (!getInput("target_grid", target_grid)) {
    setTransitionError("missing_target_grid");
    return BT::NodeStatus::FAILURE;
  }

  int current_grid = 0;
  (void)config().blackboard->get("current_grid", current_grid);
  if (current_grid < 1 || current_grid > 12 || target_grid < 1 ||
      target_grid > 12) {
    setTransitionError("invalid_grid_id");
    return BT::NodeStatus::FAILURE;
  }

  std::shared_ptr<MerlinMapManager> map;
  if (!config().blackboard->get("merlin_map", map) || !map) {
    setTransitionError("missing_merlin_map");
    return BT::NodeStatus::FAILURE;
  }

  const int from_row = (current_grid - 1) / 3;
  const int from_col = (current_grid - 1) % 3;
  const int to_row = (target_grid - 1) / 3;
  const int to_col = (target_grid - 1) % 3;
  const int row_delta = to_row - from_row;
  const int col_delta = to_col - from_col;
  if (std::abs(row_delta) + std::abs(col_delta) != 1) {
    setTransitionError("non_adjacent_grid_transition");
    return BT::NodeStatus::FAILURE;
  }

  const int from_depth = map->getDepth(current_grid);
  const int to_depth = map->getDepth(target_grid);
  if (from_depth < 0 || to_depth < 0) {
    setTransitionError("invalid_grid_depth");
    return BT::NodeStatus::FAILURE;
  }

  const int height_delta = to_depth - from_depth;
  if (height_delta == 0) {
    setTransitionError("flat_transition_unsupported");
    return BT::NodeStatus::FAILURE;
  }
  if (std::abs(height_delta) > 1) {
    setTransitionError("height_delta_too_large");
    return BT::NodeStatus::FAILURE;
  }

  constexpr double kGridStepM = 1.2;
  const double dx = static_cast<double>(col_delta) * kGridStepM;
  const double dy = static_cast<double>(row_delta) * kGridStepM;
  const double edge_yaw = std::atan2(dy, dx);
  const bool climb = height_delta > 0;
  const double target_yaw = normalizeAngle(climb ? edge_yaw : edge_yaw + M_PI);

  setOutput("target_yaw_rad", target_yaw);
  config().blackboard->set("grid_transition_target_yaw", target_yaw);
  config().blackboard->set("grid_transition_planned_from_grid", current_grid);
  config().blackboard->set("grid_transition_planned_target_grid", target_grid);
  config().blackboard->set("grid_transition_planned_height_delta",
                           height_delta);
  setTransitionError("");
  return BT::NodeStatus::SUCCESS;
}

double PlanGridTransitionAction::normalizeAngle(double angle_rad) {
  while (angle_rad > M_PI) {
    angle_rad -= 2.0 * M_PI;
  }
  while (angle_rad < -M_PI) {
    angle_rad += 2.0 * M_PI;
  }
  return angle_rad;
}

void PlanGridTransitionAction::setTransitionError(const std::string &reason) {
  if (config().blackboard) {
    config().blackboard->set("mf_transition_error", reason);
  }
}

} // namespace rc26_decision
