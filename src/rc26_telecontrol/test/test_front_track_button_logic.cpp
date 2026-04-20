#include <gtest/gtest.h>

#include "rc26_telecontrol/front_track_button_logic.hpp"

namespace rc26_telecontrol
{
namespace
{

TEST(FrontTrackButtonLogicTest, YRisingEdgeTriggersSingleUp)
{
  FrontTrackButtonLogic logic;
  EXPECT_FALSE(logic.update(false, false).has_value());
  EXPECT_EQ(logic.update(true, false), FrontTrackButtonCommand::kFrontTrackUp);
  EXPECT_FALSE(logic.update(true, false).has_value());
}

TEST(FrontTrackButtonLogicTest, ARisingEdgeTriggersSingleDown)
{
  FrontTrackButtonLogic logic;
  EXPECT_FALSE(logic.update(false, false).has_value());
  EXPECT_EQ(logic.update(false, true), FrontTrackButtonCommand::kFrontTrackDown);
  EXPECT_FALSE(logic.update(false, true).has_value());
}

TEST(FrontTrackButtonLogicTest, SimultaneousPressReportsConflictOnce)
{
  FrontTrackButtonLogic logic;
  EXPECT_EQ(logic.update(true, true), FrontTrackButtonCommand::kConflict);
  EXPECT_FALSE(logic.update(true, true).has_value());
}

TEST(FrontTrackButtonLogicTest, ReleaseStopsCommand)
{
  FrontTrackButtonLogic logic;
  EXPECT_EQ(logic.update(true, false), FrontTrackButtonCommand::kFrontTrackUp);
  EXPECT_FALSE(logic.update(false, false).has_value());
}

TEST(FrontTrackButtonLogicTest, ReleaseThenRepressTriggersAgain)
{
  FrontTrackButtonLogic logic;
  EXPECT_EQ(logic.update(true, false), FrontTrackButtonCommand::kFrontTrackUp);
  EXPECT_FALSE(logic.update(true, false).has_value());
  EXPECT_FALSE(logic.update(false, false).has_value());
  EXPECT_EQ(logic.update(true, false), FrontTrackButtonCommand::kFrontTrackUp);
}

TEST(FrontTrackButtonLogicTest, DirectionSwitchChangesCommandImmediately)
{
  FrontTrackButtonLogic logic;
  EXPECT_EQ(logic.update(true, false), FrontTrackButtonCommand::kFrontTrackUp);
  EXPECT_EQ(logic.update(false, true), FrontTrackButtonCommand::kFrontTrackDown);
}

TEST(FrontTrackButtonLogicTest, CommandIdMatchesRenumberedProtocol)
{
  EXPECT_EQ(commandIdForCommand(FrontTrackButtonCommand::kFrontTrackUp),
    static_cast<uint8_t>(rc26_serial::CommandID::FRONT_TRACK_UP));
  EXPECT_EQ(commandIdForCommand(FrontTrackButtonCommand::kFrontTrackDown),
    static_cast<uint8_t>(rc26_serial::CommandID::FRONT_TRACK_DOWN));
  EXPECT_FALSE(commandIdForCommand(FrontTrackButtonCommand::kConflict).has_value());
}

}  // namespace
}  // namespace rc26_telecontrol
