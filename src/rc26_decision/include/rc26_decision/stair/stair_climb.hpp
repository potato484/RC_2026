#pragma once

#include "rc26_decision/stair/stair_action_base.hpp"

namespace rc26_decision {

class StairClimbAction : public StairActionBase {
public:
  StairClimbAction(const std::string &name, const BT::NodeConfig &config);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase {
    HeadingAlign,
    SendFrontExtend,
    HoldAfterFrontExtend,
    DriveUntilFrontFirstEvent,
    SendFrontRetractAndRearExtend,
    HoldAfterFrontRetractAndRearExtend,
    DriveUntilRearEvent,
    SendRearRetract,
    HoldAfterRearRetract,
    Done
  };

  Phase phase_{Phase::Done};
};

} // namespace rc26_decision
