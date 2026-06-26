#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "rc26_decision/mf_preselection/mf_preselection_flow.hpp"

namespace {

TEST(MfPreselectionLogic, LabelMatchesExactAndPrefix) {
  const std::vector<std::string> exact{"R_R1", "B_R1"};
  const std::vector<std::string> prefixes{"T_", "F_"};

  EXPECT_TRUE(rc26_decision::MfPreselectionLogicResult::labelMatches(
      "R_R1", exact, prefixes));
  EXPECT_TRUE(rc26_decision::MfPreselectionLogicResult::labelMatches(
      "T_03", exact, prefixes));
  EXPECT_TRUE(rc26_decision::MfPreselectionLogicResult::labelMatches(
      "F_18", exact, prefixes));
  EXPECT_FALSE(rc26_decision::MfPreselectionLogicResult::labelMatches(
      "B_R2", exact, prefixes));
}

TEST(MfPreselectionLogic, PickupLimitUsesStrictMaximum) {
  EXPECT_TRUE(rc26_decision::MfPreselectionLogicResult::canPickup(0, 2));
  EXPECT_TRUE(rc26_decision::MfPreselectionLogicResult::canPickup(1, 2));
  EXPECT_FALSE(rc26_decision::MfPreselectionLogicResult::canPickup(2, 2));
  EXPECT_FALSE(rc26_decision::MfPreselectionLogicResult::canPickup(0, 0));
}

TEST(MfPreselectionLogic, FakeAvoidanceDirectionUsesPickupSource) {
  rc26_decision::MfPreselectionParams params;
  params.stair1_direction_yaw_rad = 1.0;
  params.stair3_direction_yaw_rad = -1.0;

  EXPECT_DOUBLE_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceYaw(
                       rc26_decision::MfPreselectionPickupSource::Stair1,
                       params),
                   1.0);
  EXPECT_DOUBLE_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceYaw(
                       rc26_decision::MfPreselectionPickupSource::Stair2,
                       params),
                   1.0);
  EXPECT_DOUBLE_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceYaw(
                       rc26_decision::MfPreselectionPickupSource::None, params),
                   1.0);
  EXPECT_DOUBLE_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceYaw(
                       rc26_decision::MfPreselectionPickupSource::Stair3,
                       params),
                   -1.0);
}

TEST(MfPreselectionLogic, GrabCommandFollowsHighSide) {
  rc26_decision::MfPreselectionParams params;
  params.grab_kfs_up_command_id = 0x03;
  params.grab_kfs_down_command_id = 0x02;

  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabCommandForHighSide(
                true, params),
            static_cast<uint8_t>(0x03));
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabCommandForHighSide(
                false, params),
            static_cast<uint8_t>(0x02));
}

} // namespace
