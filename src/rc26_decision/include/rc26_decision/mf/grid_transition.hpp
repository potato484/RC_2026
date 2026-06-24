#pragma once

#include <string>

#include <behaviortree_cpp/bt_factory.h>

#include "rc26_decision/mf/merlin_map.hpp"
#include "rc26_decision/stair/stair_action_base.hpp"

namespace rc26_decision {

class GridTransitionAction : public StairActionBase {
public:
  GridTransitionAction(const std::string &name, const BT::NodeConfig &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class TransitionKind { CLIMB, DESCEND };
  enum class Phase {
    ClimbSendFrontExtend,
    ClimbHoldAfterFrontExtend,
    ClimbDriveUntilFrontFirstEvent,
    ClimbSendFrontRetractAndRearExtend,
    ClimbHoldAfterFrontRetractAndRearExtend,
    ClimbDriveUntilRearEvent,
    ClimbSendRearRetract,
    DescendDriveUntilRearEvent,
    DescendSendRearExtend,
    DescendHoldAfterRearExtend,
    DescendDriveUntilFrontSecondEvent,
    DescendSendRearRetractAndFrontExtend,
    DescendHoldAfterRearRetractAndFrontExtend,
    DescendTimedDriveBeforeFrontRetract,
    DescendSendFrontRetract,
    DescendHoldAfterFrontRetract,
    Done
  };

  bool planTransition();
  void setTransitionError(const std::string &reason);
  void commitTransition();
  BT::NodeStatus failTransition(const char *reason);

  Phase phase_{Phase::Done};
  TransitionKind transition_kind_{TransitionKind::CLIMB};
  int from_grid_{0};
  int target_grid_{0};
  int height_delta_{0};
  double target_yaw_rad_{0.0};
};

} // namespace rc26_decision
