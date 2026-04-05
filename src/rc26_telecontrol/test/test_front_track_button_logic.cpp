#include <gtest/gtest.h>

#include "rc26_telecontrol/front_track_button_logic.hpp"

namespace rc26_telecontrol
{
namespace
{

TEST(FrontTrackButtonLogicTest, YRisingEdgeTriggersUpOnlyOnce)
{
  FrontTrackButtonLogic logic;
  EXPECT_EQ(logic.update(false, false, false), FrontTrackButtonEvent::kNone);
  EXPECT_EQ(logic.update(true, false, false), FrontTrackButtonEvent::kFrontTrackUp);
  EXPECT_EQ(logic.update(true, false, false), FrontTrackButtonEvent::kNone);
}

TEST(FrontTrackButtonLogicTest, ARisingEdgeTriggersDownOnlyOnce)
{
  FrontTrackButtonLogic logic;
  EXPECT_EQ(logic.update(false, false, false), FrontTrackButtonEvent::kNone);
  EXPECT_EQ(logic.update(false, true, false), FrontTrackButtonEvent::kFrontTrackDown);
  EXPECT_EQ(logic.update(false, true, false), FrontTrackButtonEvent::kNone);
}

TEST(FrontTrackButtonLogicTest, SimultaneousRiseIsIgnoredAsConflict)
{
  FrontTrackButtonLogic logic;
  EXPECT_EQ(logic.update(true, true, false), FrontTrackButtonEvent::kConflict);
}

TEST(FrontTrackButtonLogicTest, BusyStateIgnoresNewTrigger)
{
  FrontTrackButtonLogic logic;
  EXPECT_EQ(logic.update(false, false, false), FrontTrackButtonEvent::kNone);
  EXPECT_EQ(logic.update(true, false, true), FrontTrackButtonEvent::kBusyIgnored);
}

TEST(FrontTrackButtonLogicTest, ReleaseAndPressAgainTriggersAgain)
{
  FrontTrackButtonLogic logic;
  EXPECT_EQ(logic.update(true, false, false), FrontTrackButtonEvent::kFrontTrackUp);
  EXPECT_EQ(logic.update(false, false, false), FrontTrackButtonEvent::kNone);
  EXPECT_EQ(logic.update(true, false, false), FrontTrackButtonEvent::kFrontTrackUp);
}

}  // namespace
}  // namespace rc26_telecontrol
