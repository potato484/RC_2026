#include <limits>

#include <gtest/gtest.h>

#include "rc26_telecontrol/rear_pushrod_button_logic.hpp"

namespace rc26_telecontrol
{
namespace
{

TEST(RearPushrodButtonLogicTest, SelectPressEmitsExtendOnce)
{
  RearPushrodButtonLogic logic;
  EXPECT_FALSE(logic.update(false, false).has_value());
  EXPECT_EQ(logic.update(true, false), RearPushrodButtonCommand::kExtend);
  EXPECT_FALSE(logic.update(true, false).has_value());
}

TEST(RearPushrodButtonLogicTest, StartPressEmitsRetractOnce)
{
  RearPushrodButtonLogic logic;
  EXPECT_EQ(logic.update(false, true), RearPushrodButtonCommand::kRetract);
  EXPECT_FALSE(logic.update(false, true).has_value());
}

TEST(RearPushrodButtonLogicTest, ReleasingButtonsRearmsTheSameDirection)
{
  RearPushrodButtonLogic logic;
  EXPECT_EQ(logic.update(true, false), RearPushrodButtonCommand::kExtend);
  EXPECT_FALSE(logic.update(false, false).has_value());
  EXPECT_EQ(logic.update(true, false), RearPushrodButtonCommand::kExtend);
}

TEST(RearPushrodButtonLogicTest, DirectDirectionSwitchEmitsOppositeCommand)
{
  RearPushrodButtonLogic logic;
  EXPECT_EQ(logic.update(true, false), RearPushrodButtonCommand::kExtend);
  EXPECT_EQ(logic.update(false, true), RearPushrodButtonCommand::kRetract);
}

TEST(RearPushrodButtonLogicTest, SimultaneousPressDoesNotEmitCommand)
{
  RearPushrodButtonLogic logic;
  EXPECT_FALSE(logic.update(true, true).has_value());
  EXPECT_FALSE(logic.update(true, true).has_value());
  EXPECT_FALSE(logic.update(false, false).has_value());
  EXPECT_EQ(logic.update(false, true), RearPushrodButtonCommand::kRetract);
}

TEST(RearPushrodButtonLogicTest, CommandIdMatchesProtocol)
{
  EXPECT_EQ(
    commandIdForRearPushrodCommand(RearPushrodButtonCommand::kExtend),
    static_cast<uint8_t>(rc26_serial::CommandID::REAR_PUSHROD_EXTEND));
  EXPECT_EQ(
    commandIdForRearPushrodCommand(RearPushrodButtonCommand::kRetract),
    static_cast<uint8_t>(rc26_serial::CommandID::REAR_PUSHROD_RETRACT));
}

}  // namespace
}  // namespace rc26_telecontrol
