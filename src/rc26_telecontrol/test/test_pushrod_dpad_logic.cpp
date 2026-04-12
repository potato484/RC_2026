#include <limits>

#include <gtest/gtest.h>

#include "rc26_telecontrol/pushrod_dpad_logic.hpp"

namespace rc26_telecontrol
{
namespace
{

TEST(PushrodDpadLogicTest, LeftPressEmitsExtendOnce)
{
  PushrodDpadLogic logic;
  EXPECT_FALSE(logic.update(0.0).has_value());
  EXPECT_EQ(logic.update(-1.0), PushrodCommand::kExtend);
  EXPECT_FALSE(logic.update(-1.0).has_value());
}

TEST(PushrodDpadLogicTest, RightPressEmitsRetractOnce)
{
  PushrodDpadLogic logic;
  EXPECT_EQ(logic.update(1.0), PushrodCommand::kRetract);
  EXPECT_FALSE(logic.update(1.0).has_value());
}

TEST(PushrodDpadLogicTest, ReturningToCenterRearmsTheSameDirection)
{
  PushrodDpadLogic logic;
  EXPECT_EQ(logic.update(-1.0), PushrodCommand::kExtend);
  EXPECT_FALSE(logic.update(0.0).has_value());
  EXPECT_EQ(logic.update(-1.0), PushrodCommand::kExtend);
}

TEST(PushrodDpadLogicTest, DirectDirectionSwitchEmitsOppositeCommand)
{
  PushrodDpadLogic logic;
  EXPECT_EQ(logic.update(-1.0), PushrodCommand::kExtend);
  EXPECT_EQ(logic.update(1.0), PushrodCommand::kRetract);
}

TEST(PushrodDpadLogicTest, NonFiniteAxisFallsBackToCenter)
{
  PushrodDpadLogic logic;
  EXPECT_EQ(logic.update(-1.0), PushrodCommand::kExtend);
  EXPECT_FALSE(logic.update(std::numeric_limits<double>::quiet_NaN()).has_value());
  EXPECT_EQ(logic.update(1.0), PushrodCommand::kRetract);
}

TEST(PushrodDpadLogicTest, CommandIdMatchesProtocol)
{
  EXPECT_EQ(
    commandIdForPushrodCommand(PushrodCommand::kExtend),
    static_cast<uint8_t>(rc26_serial::CommandID::PUSHROD_EXTEND));
  EXPECT_EQ(
    commandIdForPushrodCommand(PushrodCommand::kRetract),
    static_cast<uint8_t>(rc26_serial::CommandID::PUSHROD_RETRACT));
}

}  // namespace
}  // namespace rc26_telecontrol
