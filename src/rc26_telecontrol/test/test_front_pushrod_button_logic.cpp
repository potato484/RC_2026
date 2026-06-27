#include <gtest/gtest.h>

#include "rc26_telecontrol/front_pushrod_button_logic.hpp"

namespace rc26_telecontrol
{
namespace
{

TEST(FrontPushrodButtonLogicTest, YRisingEdgeTriggersSingleExtend)
{
  FrontPushrodButtonLogic logic;
  EXPECT_FALSE(logic.update(false, false).has_value());
  EXPECT_EQ(logic.update(true, false), FrontPushrodButtonCommand::kExtend);
  EXPECT_FALSE(logic.update(true, false).has_value());
}

TEST(FrontPushrodButtonLogicTest, ARisingEdgeTriggersSingleRetract)
{
  FrontPushrodButtonLogic logic;
  EXPECT_FALSE(logic.update(false, false).has_value());
  EXPECT_EQ(logic.update(false, true), FrontPushrodButtonCommand::kRetract);
  EXPECT_FALSE(logic.update(false, true).has_value());
}

TEST(FrontPushrodButtonLogicTest, SimultaneousPressReportsConflictOnce)
{
  FrontPushrodButtonLogic logic;
  EXPECT_EQ(logic.update(true, true), FrontPushrodButtonCommand::kConflict);
  EXPECT_FALSE(logic.update(true, true).has_value());
}

TEST(FrontPushrodButtonLogicTest, ReleaseStopsCommand)
{
  FrontPushrodButtonLogic logic;
  EXPECT_EQ(logic.update(true, false), FrontPushrodButtonCommand::kExtend);
  EXPECT_FALSE(logic.update(false, false).has_value());
}

TEST(FrontPushrodButtonLogicTest, ReleaseThenRepressTriggersAgain)
{
  FrontPushrodButtonLogic logic;
  EXPECT_EQ(logic.update(true, false), FrontPushrodButtonCommand::kExtend);
  EXPECT_FALSE(logic.update(true, false).has_value());
  EXPECT_FALSE(logic.update(false, false).has_value());
  EXPECT_EQ(logic.update(true, false), FrontPushrodButtonCommand::kExtend);
}

TEST(FrontPushrodButtonLogicTest, DirectionSwitchChangesCommandImmediately)
{
  FrontPushrodButtonLogic logic;
  EXPECT_EQ(logic.update(true, false), FrontPushrodButtonCommand::kExtend);
  EXPECT_EQ(logic.update(false, true), FrontPushrodButtonCommand::kRetract);
}

TEST(FrontPushrodButtonLogicTest, CommandIdMatchesProtocol)
{
  EXPECT_EQ(commandIdForFrontPushrodCommand(FrontPushrodButtonCommand::kExtend),
    static_cast<uint8_t>(rc26_serial::CommandID::FRONT_PUSHROD_EXTEND));
  EXPECT_EQ(commandIdForFrontPushrodCommand(FrontPushrodButtonCommand::kRetract),
    static_cast<uint8_t>(rc26_serial::CommandID::FRONT_PUSHROD_RETRACT));
  EXPECT_FALSE(commandIdForFrontPushrodCommand(FrontPushrodButtonCommand::kConflict).has_value());
}

}  // namespace
}  // namespace rc26_telecontrol
