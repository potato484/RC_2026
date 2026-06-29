#include <cstdint>
#include <cmath>
#include <limits>
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

TEST(MfPreselectionLogic, KfsAlignVelocityUsesToleranceLimitAndDirection) {
  rc26_decision::MfPreselectionParams params;
  params.kfs_align_tolerance_px = 20;
  params.kfs_align_kp = 0.001;
  params.kfs_align_min_speed_mps = 0.015;
  params.kfs_align_max_speed_mps = 0.06;
  params.kfs_invert_lateral_direction = false;

  EXPECT_DOUBLE_EQ(
      rc26_decision::MfPreselectionLogicResult::kfsAlignVy(10, params), 0.0);
  EXPECT_DOUBLE_EQ(
      rc26_decision::MfPreselectionLogicResult::kfsAlignVy(30, params), -0.03);
  EXPECT_DOUBLE_EQ(
      rc26_decision::MfPreselectionLogicResult::kfsAlignVy(-30, params), 0.03);
  EXPECT_DOUBLE_EQ(
      rc26_decision::MfPreselectionLogicResult::kfsAlignVy(200, params), -0.06);
  EXPECT_DOUBLE_EQ(
      rc26_decision::MfPreselectionLogicResult::kfsAlignVy(21, params), -0.021);

  params.kfs_align_kp = 0.0001;
  EXPECT_DOUBLE_EQ(
      rc26_decision::MfPreselectionLogicResult::kfsAlignVy(30, params), -0.015);

  params.kfs_invert_lateral_direction = true;
  EXPECT_DOUBLE_EQ(
      rc26_decision::MfPreselectionLogicResult::kfsAlignVy(30, params), 0.015);
}

TEST(MfPreselectionLogic, KfsOpenLoopDistanceAndDurationUseArmReach) {
  EXPECT_DOUBLE_EQ(rc26_decision::MfPreselectionLogicResult::kfsOpenLoopDistance(
                       0.55, 0.40),
                   0.15);
  EXPECT_DOUBLE_EQ(rc26_decision::MfPreselectionLogicResult::kfsOpenLoopDuration(
                       0.15, 0.05),
                   3.0);
  EXPECT_DOUBLE_EQ(rc26_decision::MfPreselectionLogicResult::kfsOpenLoopDistance(
                       0.35, 0.40),
                   0.0);
  EXPECT_DOUBLE_EQ(rc26_decision::MfPreselectionLogicResult::kfsOpenLoopDuration(
                       0.0, 0.0),
                   0.0);
  EXPECT_TRUE(std::isinf(
      rc26_decision::MfPreselectionLogicResult::kfsOpenLoopDuration(0.10, 0.0)));
  EXPECT_GT(rc26_decision::MfPreselectionLogicResult::kfsOpenLoopDuration(
                0.90, 0.10),
            8.0);
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

TEST(MfPreselectionLogic, FakeAvoidanceTargetGridUsesSideColumns) {
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
                2, rc26_decision::MfPreselectionPickupSource::Stair1),
            1);
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
                5, rc26_decision::MfPreselectionPickupSource::Stair1),
            4);
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
                8, rc26_decision::MfPreselectionPickupSource::Stair1),
            7);

  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
                2, rc26_decision::MfPreselectionPickupSource::Stair3),
            3);
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
                5, rc26_decision::MfPreselectionPickupSource::Stair3),
            6);
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
                8, rc26_decision::MfPreselectionPickupSource::Stair3),
            9);

  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
          0, rc26_decision::MfPreselectionPickupSource::Stair1)
          .has_value());
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
          1, rc26_decision::MfPreselectionPickupSource::Stair1)
          .has_value());
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
          3, rc26_decision::MfPreselectionPickupSource::Stair3)
          .has_value());
}

TEST(MfPreselectionLogic, FinalExitCenterTargetUsesDescendDriveDirection) {
  constexpr double kHalfPi = 1.57079632679489661923;
  double target_x = 0.0;
  double target_y = 0.0;
  ASSERT_TRUE(rc26_decision::MfPreselectionLogicResult::finalExitCenterTarget(
      10.0, 20.0, 0.0, 1.2, target_x, target_y));
  EXPECT_NEAR(target_x, 8.8, 1e-9);
  EXPECT_NEAR(target_y, 20.0, 1e-9);

  ASSERT_TRUE(rc26_decision::MfPreselectionLogicResult::finalExitCenterTarget(
      10.0, 20.0, -kHalfPi, -2.0, target_x, target_y));
  EXPECT_NEAR(target_x, 10.0, 1e-9);
  EXPECT_NEAR(target_y, 22.0, 1e-9);

  EXPECT_FALSE(rc26_decision::MfPreselectionLogicResult::finalExitCenterTarget(
      std::numeric_limits<double>::quiet_NaN(), 20.0, 0.0, 1.2, target_x,
      target_y));
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

TEST(MfPreselectionLogic, BboxIouAndSameTargetUseLabelAndOverlap) {
  rc26_decision::MfPreselectionTargetSnapshot reference;
  reference.label = "T_03";
  reference.x1 = 10.0;
  reference.y1 = 10.0;
  reference.x2 = 50.0;
  reference.y2 = 50.0;

  rc26_decision::MfPreselectionTargetSnapshot close = reference;
  close.x1 = 15.0;
  close.y1 = 15.0;
  close.x2 = 55.0;
  close.y2 = 55.0;

  rc26_decision::MfPreselectionTargetSnapshot far = reference;
  far.x1 = 80.0;
  far.y1 = 80.0;
  far.x2 = 120.0;
  far.y2 = 120.0;

  rc26_decision::MfPreselectionTargetSnapshot wrong_label = close;
  wrong_label.label = "T_04";

  EXPECT_GT(rc26_decision::MfPreselectionLogicResult::bboxIou(reference, close),
            0.30);
  EXPECT_DOUBLE_EQ(
      rc26_decision::MfPreselectionLogicResult::bboxIou(reference, far), 0.0);
  EXPECT_TRUE(rc26_decision::MfPreselectionLogicResult::isSameVisualTarget(
      reference, close, 0.30));
  EXPECT_FALSE(rc26_decision::MfPreselectionLogicResult::isSameVisualTarget(
      reference, far, 0.30));
  EXPECT_FALSE(rc26_decision::MfPreselectionLogicResult::isSameVisualTarget(
      reference, wrong_label, 0.30));
}

TEST(MfPreselectionLogic, IgnoredTargetUsesSameVisualTargetRule) {
  rc26_decision::MfPreselectionTargetSnapshot ignored;
  ignored.label = "T_03";
  ignored.x1 = 10.0;
  ignored.y1 = 10.0;
  ignored.x2 = 50.0;
  ignored.y2 = 50.0;
  const std::vector<rc26_decision::MfPreselectionTargetSnapshot> ignored_targets{
      ignored};

  rc26_decision::MfPreselectionTargetSnapshot same = ignored;
  same.x1 = 14.0;
  same.y1 = 14.0;
  same.x2 = 54.0;
  same.y2 = 54.0;

  rc26_decision::MfPreselectionTargetSnapshot another_same_label = ignored;
  another_same_label.x1 = 120.0;
  another_same_label.y1 = 120.0;
  another_same_label.x2 = 160.0;
  another_same_label.y2 = 160.0;

  EXPECT_TRUE(rc26_decision::MfPreselectionLogicResult::isIgnoredTarget(
      same, ignored_targets, 0.30));
  EXPECT_FALSE(rc26_decision::MfPreselectionLogicResult::isIgnoredTarget(
      another_same_label, ignored_targets, 0.30));
}

} // namespace
