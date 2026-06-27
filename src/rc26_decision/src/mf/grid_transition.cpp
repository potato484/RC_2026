#include "rc26_decision/mf/grid_transition.hpp"

#include <cmath>
#include <memory>

namespace {

double normalizeMfAngle(double angle_rad) {
  while (angle_rad > M_PI) {
    angle_rad -= 2.0 * M_PI;
  }
  while (angle_rad < -M_PI) {
    angle_rad += 2.0 * M_PI;
  }
  return angle_rad;
}

const char *transitionKindName(bool climb) {
  return climb ? "CLIMB" : "DESCEND";
}

} // namespace

namespace rc26_decision {

GridTransitionAction::GridTransitionAction(const std::string &name,
                                           const BT::NodeConfig &config)
    : StairActionBase(name, config) {}

BT::PortsList GridTransitionAction::providedPorts() {
  return {BT::InputPort<int>("target_grid", "目标格子 ID (1-12)")};
}

BT::NodeStatus GridTransitionAction::onStart() {
  if (!planTransition()) {
    return BT::NodeStatus::FAILURE;
  }
  if (!setupRuntime("梅林格间转移")) {
    setTransitionError("runtime_setup_failed");
    return BT::NodeStatus::FAILURE;
  }

  if (transition_kind_ == TransitionKind::CLIMB) {
    config().blackboard->set("stair_climb_done", false);
    setHeadingTarget(target_yaw_rad_);
    publishStop();
    phase_ = Phase::ClimbSendFrontExtend;
    beginCommand(CommandID::FRONT_PUSHROD_EXTEND, "FRONT_PUSHROD_EXTEND");
  } else {
    config().blackboard->set("stair_descend_done", false);
    setHeadingTarget(target_yaw_rad_);
    publishStop();
    phase_ = Phase::DescendDriveUntilRearEvent;
    beginEventWait(WheelEvent::Rear, params_.rear_event_timeout_s, "rear");
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GridTransitionAction::onRunning() {
  switch (phase_) {
  case Phase::ClimbSendFrontExtend:
    switch (tickCommand()) {
    case StepStatus::Success:
      phase_ = Phase::ClimbHoldAfterFrontExtend;
      beginZeroHold(params_.climb_front_extend_delay_s,
                    "front_extend_settle");
      break;
    case StepStatus::Failure:
      return failTransition("FRONT_PUSHROD_EXTEND failed");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::ClimbHoldAfterFrontExtend:
    switch (tickZeroHold()) {
    case StepStatus::Success:
      phase_ = Phase::ClimbDriveUntilFrontFirstEvent;
      beginEventWait(WheelEvent::FrontFirst, params_.front_event_timeout_s,
                     "front_first");
      break;
    case StepStatus::Failure:
      return failTransition("front extend zero hold failed");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::ClimbDriveUntilFrontFirstEvent:
    if (!headingReadyForMotion()) {
      publishStop();
      switch (tickEventWait()) {
      case StepStatus::Failure:
        return failTransition("heading odom stale before front first event");
      case StepStatus::Success:
      case StepStatus::Running:
        break;
      }
      break;
    }
    publishDrive(climbDriveSpeedMagnitude());
    switch (tickEventWait()) {
    case StepStatus::Success:
      publishStop();
      phase_ = Phase::ClimbSendFrontRetractAndRearExtend;
      beginCommandPair(CommandID::FRONT_PUSHROD_RETRACT,
                       "FRONT_PUSHROD_RETRACT",
                       CommandID::REAR_PUSHROD_EXTEND,
                       "REAR_PUSHROD_EXTEND");
      switch (tickCommandPair()) {
      case StepStatus::Success:
        phase_ = Phase::ClimbHoldAfterFrontRetractAndRearExtend;
        beginZeroHold(params_.climb_retract_rear_extend_delay_s,
                      "front_retract_rear_extend_settle");
        break;
      case StepStatus::Failure:
        return failTransition(
            "FRONT_PUSHROD_RETRACT + REAR_PUSHROD_EXTEND failed");
      case StepStatus::Running:
        break;
      }
      break;
    case StepStatus::Failure:
      return failTransition("front first laser event timeout");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::ClimbSendFrontRetractAndRearExtend:
    publishStop();
    switch (tickCommandPair()) {
    case StepStatus::Success:
      phase_ = Phase::ClimbHoldAfterFrontRetractAndRearExtend;
      beginZeroHold(params_.climb_retract_rear_extend_delay_s,
                    "front_retract_rear_extend_settle");
      break;
    case StepStatus::Failure:
      return failTransition(
          "FRONT_PUSHROD_RETRACT + REAR_PUSHROD_EXTEND failed");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::ClimbHoldAfterFrontRetractAndRearExtend:
    switch (tickZeroHold()) {
    case StepStatus::Success:
      phase_ = Phase::ClimbDriveUntilRearEvent;
      resetClimbRearDriveProfile();
      beginEventWait(WheelEvent::Rear, params_.rear_event_timeout_s, "rear");
      break;
    case StepStatus::Failure:
      return failTransition("front retract rear extend zero hold failed");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::ClimbDriveUntilRearEvent:
    if (!headingReadyForMotion()) {
      publishStop();
      switch (tickEventWait()) {
      case StepStatus::Failure:
        return failTransition("heading odom stale before rear event");
      case StepStatus::Success:
      case StepStatus::Running:
        break;
      }
      break;
    }
    publishDrive(climbRearDriveProfileSpeed());
    switch (tickEventWait()) {
    case StepStatus::Success:
      publishStop();
      phase_ = Phase::ClimbSendRearRetract;
      beginCommand(CommandID::REAR_PUSHROD_RETRACT, "REAR_PUSHROD_RETRACT");
      break;
    case StepStatus::Failure:
      return failTransition("rear laser event timeout");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::ClimbSendRearRetract:
    publishStop();
    switch (tickCommand()) {
    case StepStatus::Success:
      phase_ = Phase::ClimbHoldAfterRearRetract;
      beginZeroHold(params_.climb_rear_retract_delay_s,
                    "rear_retract_settle");
      break;
    case StepStatus::Failure:
      return failTransition("REAR_PUSHROD_RETRACT failed");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::ClimbHoldAfterRearRetract:
    switch (tickZeroHold()) {
    case StepStatus::Success:
      config().blackboard->set("stair_climb_done", true);
      commitTransition();
      phase_ = Phase::Done;
      releaseRuntime();
      return BT::NodeStatus::SUCCESS;
    case StepStatus::Failure:
      return failTransition("rear retract zero hold failed");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::DescendDriveUntilRearEvent:
    if (!headingReadyForMotion()) {
      publishStop();
      switch (tickEventWait()) {
      case StepStatus::Failure:
        return failTransition("heading odom stale before rear event");
      case StepStatus::Success:
      case StepStatus::Running:
        break;
      }
      break;
    }
    publishDrive(-descendDriveSpeedMagnitude());
    switch (tickEventWait()) {
    case StepStatus::Success:
      publishStop();
      phase_ = Phase::DescendSendRearExtend;
      beginCommand(CommandID::REAR_PUSHROD_EXTEND, "REAR_PUSHROD_EXTEND");
      break;
    case StepStatus::Failure:
      return failTransition("rear laser event timeout");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::DescendSendRearExtend:
    publishStop();
    switch (tickCommand()) {
    case StepStatus::Success:
      phase_ = Phase::DescendHoldAfterRearExtend;
      beginZeroHold(params_.descend_rear_extend_delay_s,
                    "rear_extend_settle");
      break;
    case StepStatus::Failure:
      return failTransition("REAR_PUSHROD_EXTEND failed");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::DescendHoldAfterRearExtend:
    switch (tickZeroHold()) {
    case StepStatus::Success:
      phase_ = Phase::DescendDriveUntilFrontSecondEvent;
      beginEventWait(WheelEvent::FrontSecond, params_.front_event_timeout_s,
                     "front_second");
      break;
    case StepStatus::Failure:
      return failTransition("rear extend zero hold failed");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::DescendDriveUntilFrontSecondEvent:
    if (!headingReadyForMotion()) {
      publishStop();
      switch (tickEventWait()) {
      case StepStatus::Failure:
        return failTransition("heading odom stale before front second event");
      case StepStatus::Success:
      case StepStatus::Running:
        break;
      }
      break;
    }
    publishDrive(-descendDriveSpeedMagnitude());
    switch (tickEventWait()) {
    case StepStatus::Success:
      publishStop();
      phase_ = Phase::DescendSendRearRetractAndFrontExtend;
      beginCommandPair(CommandID::REAR_PUSHROD_RETRACT,
                       "REAR_PUSHROD_RETRACT",
                       CommandID::FRONT_PUSHROD_EXTEND,
                       "FRONT_PUSHROD_EXTEND");
      switch (tickCommandPair()) {
      case StepStatus::Success:
        phase_ = Phase::DescendHoldAfterRearRetractAndFrontExtend;
        beginZeroHold(params_.descend_retract_front_extend_delay_s,
                      "rear_retract_front_extend_settle");
        break;
      case StepStatus::Failure:
        return failTransition(
            "REAR_PUSHROD_RETRACT + FRONT_PUSHROD_EXTEND failed");
      case StepStatus::Running:
        break;
      }
      break;
    case StepStatus::Failure:
      return failTransition("front second laser event timeout");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::DescendSendRearRetractAndFrontExtend:
    publishStop();
    switch (tickCommandPair()) {
    case StepStatus::Success:
      phase_ = Phase::DescendHoldAfterRearRetractAndFrontExtend;
      beginZeroHold(params_.descend_retract_front_extend_delay_s,
                    "rear_retract_front_extend_settle");
      break;
    case StepStatus::Failure:
      return failTransition(
          "REAR_PUSHROD_RETRACT + FRONT_PUSHROD_EXTEND failed");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::DescendHoldAfterRearRetractAndFrontExtend:
    switch (tickZeroHold()) {
    case StepStatus::Success:
      phase_ = Phase::DescendTimedDriveBeforeFrontRetract;
      beginTimedDrive(-params_.descend_front_retract_drive_speed_mps,
                      params_.descend_front_retract_drive_duration_s,
                      "front_retract_trigger_drive");
      break;
    case StepStatus::Failure:
      return failTransition("rear retract front extend zero hold failed");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::DescendTimedDriveBeforeFrontRetract:
    if (!headingReadyForMotion()) {
      return failTransition("heading odom stale before front retract");
    }
    switch (tickTimedDrive()) {
    case StepStatus::Success:
      publishStop();
      phase_ = Phase::DescendSendFrontRetract;
      beginCommand(CommandID::FRONT_PUSHROD_RETRACT,
                   "FRONT_PUSHROD_RETRACT");
      break;
    case StepStatus::Failure:
      return failTransition("front retract trigger drive failed");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::DescendSendFrontRetract:
    publishStop();
    switch (tickCommand()) {
    case StepStatus::Success:
      phase_ = Phase::DescendHoldAfterFrontRetract;
      beginZeroHold(params_.descend_front_retract_delay_s,
                    "front_retract_settle");
      break;
    case StepStatus::Failure:
      return failTransition("FRONT_PUSHROD_RETRACT failed");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::DescendHoldAfterFrontRetract:
    switch (tickZeroHold()) {
    case StepStatus::Success:
      config().blackboard->set("stair_descend_done", true);
      commitTransition();
      phase_ = Phase::Done;
      releaseRuntime();
      return BT::NodeStatus::SUCCESS;
    case StepStatus::Failure:
      return failTransition("front retract zero hold failed");
    case StepStatus::Running:
      break;
    }
    break;

  case Phase::Done:
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void GridTransitionAction::onHalted() {
  publishStop();
  releaseRuntime();
  phase_ = Phase::Done;
}

bool GridTransitionAction::planTransition() {
  if (!getInput("target_grid", target_grid_)) {
    setTransitionError("missing_target_grid");
    return false;
  }
  (void)config().blackboard->get("current_grid", from_grid_);
  if (from_grid_ < 1 || from_grid_ > 12 || target_grid_ < 1 ||
      target_grid_ > 12) {
    setTransitionError("invalid_grid_id");
    return false;
  }

  std::shared_ptr<MerlinMapManager> map;
  if (!config().blackboard->get("merlin_map", map) || !map) {
    setTransitionError("missing_merlin_map");
    return false;
  }

  const int from_row = (from_grid_ - 1) / 3;
  const int from_col = (from_grid_ - 1) % 3;
  const int to_row = (target_grid_ - 1) / 3;
  const int to_col = (target_grid_ - 1) % 3;
  const int row_delta = to_row - from_row;
  const int col_delta = to_col - from_col;
  if (std::abs(row_delta) + std::abs(col_delta) != 1) {
    setTransitionError("non_adjacent_grid_transition");
    return false;
  }

  const int from_depth = map->getDepth(from_grid_);
  const int to_depth = map->getDepth(target_grid_);
  if (from_depth < 0 || to_depth < 0) {
    setTransitionError("invalid_grid_depth");
    return false;
  }

  height_delta_ = to_depth - from_depth;
  if (height_delta_ == 0) {
    setTransitionError("flat_transition_unsupported");
    return false;
  }
  if (std::abs(height_delta_) > 1) {
    setTransitionError("height_delta_too_large");
    return false;
  }

  constexpr double kGridStepM = 1.2;
  // MF grid axis: row +1 maps to odom/map +X, col -1 maps to +Y.
  const double dx = static_cast<double>(row_delta) * kGridStepM;
  const double dy = static_cast<double>(-col_delta) * kGridStepM;
  const double edge_yaw = std::atan2(dy, dx);
  const bool climb = height_delta_ > 0;
  transition_kind_ = climb ? TransitionKind::CLIMB : TransitionKind::DESCEND;
  target_yaw_rad_ = normalizeMfAngle(climb ? edge_yaw : edge_yaw + M_PI);
  setTransitionError("");
  return true;
}

void GridTransitionAction::setTransitionError(const std::string &reason) {
  if (config().blackboard) {
    config().blackboard->set("mf_transition_error", reason);
  }
}

void GridTransitionAction::commitTransition() {
  const bool climb = transition_kind_ == TransitionKind::CLIMB;
  config().blackboard->set("last_grid", from_grid_);
  config().blackboard->set("current_grid", target_grid_);
  config().blackboard->set("last_transition_kind",
                           std::string(transitionKindName(climb)));
  config().blackboard->set("last_height_delta", height_delta_);
  config().blackboard->set("last_transition_target_yaw", target_yaw_rad_);
  setTransitionError("");
}

BT::NodeStatus GridTransitionAction::failTransition(const char *reason) {
  setTransitionError(reason ? reason : "transition_failed");
  return failWithStop(reason);
}

} // namespace rc26_decision
