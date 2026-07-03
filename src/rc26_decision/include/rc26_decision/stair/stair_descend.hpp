#pragma once

#include "rc26_decision/stair/stair_action_base.hpp"

namespace rc26_decision {

class StairDescendAction : public StairActionBase {
public:
  StairDescendAction(const std::string &name, const BT::NodeConfig &config);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase {
    HeadingAlign,
    DriveUntilRearEvent,
    SendRearExtend,
    HoldAfterRearExtend,
    DriveUntilFrontSecondEvent,
    SendRearRetractAndFrontExtend,
    HoldAfterRearRetractAndFrontExtend,
    TimedDriveBeforeFrontRetract,
    SendFrontRetract,
    HoldAfterFrontRetract,
    Done
  };

  Phase phase_{Phase::Done};
};

} // namespace rc26_decision
