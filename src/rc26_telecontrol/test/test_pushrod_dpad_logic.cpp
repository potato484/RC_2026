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
  EXPECT_FALSE(logic.update(false, false).has_value());
  EXPECT_EQ(logic.update(true, false), PushrodCommand::kExtend);
  EXPECT_FALSE(logic.update(true, false).has_value());
}

TEST(PushrodDpadLogicTest, RightPressEmitsRetractOnce)
{
  PushrodDpadLogic logic;
  EXPECT_EQ(logic.update(false, true), PushrodCommand::kRetract);
  EXPECT_FALSE(logic.update(false, true).has_value());
}

TEST(PushrodDpadLogicTest, ReleasingButtonsRearmsTheSameDirection)
{
  PushrodDpadLogic logic;
  EXPECT_EQ(logic.update(true, false), PushrodCommand::kExtend);
  EXPECT_FALSE(logic.update(false, false).has_value());
  EXPECT_EQ(logic.update(true, false), PushrodCommand::kExtend);
}

TEST(PushrodDpadLogicTest, DirectDirectionSwitchEmitsOppositeCommand)
{
  PushrodDpadLogic logic;
  EXPECT_EQ(logic.update(true, false), PushrodCommand::kExtend);
  EXPECT_EQ(logic.update(false, true), PushrodCommand::kRetract);
}

TEST(PushrodDpadLogicTest, SimultaneousPressDoesNotEmitCommand)
{
  PushrodDpadLogic logic;
  EXPECT_FALSE(logic.update(true, true).has_value());
  EXPECT_FALSE(logic.update(true, true).has_value());
  EXPECT_FALSE(logic.update(false, false).has_value());
  EXPECT_EQ(logic.update(false, true), PushrodCommand::kRetract);
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
