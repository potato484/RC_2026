#include <cstdint>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "rc26_decision/mf/merlin_map.hpp"
#include "rc26_decision/mf_preselection/mf_preselection_flow.hpp"
#include "rc26_decision/stair/stair_area.hpp"

namespace {

TEST(StairSpeedProfile, SamplesLinearFastToSlow) {
  const rc26_decision::StairSpeedProfile profile{0.10, 0.05, 1.0};

  EXPECT_DOUBLE_EQ(rc26_decision::sampleStairSpeedProfile(profile, 0.0), 0.10);
  EXPECT_DOUBLE_EQ(rc26_decision::sampleStairSpeedProfile(profile, 0.5), 0.075);
  EXPECT_DOUBLE_EQ(rc26_decision::sampleStairSpeedProfile(profile, 1.0), 0.05);
  EXPECT_DOUBLE_EQ(rc26_decision::sampleStairSpeedProfile(profile, 2.0), 0.05);
}

TEST(StairSpeedProfile, DurationZeroReturnsSlowSpeed) {
  const rc26_decision::StairSpeedProfile profile{0.40, 0.20, 0.0};

  EXPECT_DOUBLE_EQ(rc26_decision::sampleStairSpeedProfile(profile, 0.0), 0.20);
  EXPECT_DOUBLE_EQ(rc26_decision::sampleStairSpeedProfile(profile, 3.0), 0.20);
}

TEST(StairSpeedProfile, NormalizesSpeedsAndSlowUpperBound) {
  const auto profile = rc26_decision::normalizeStairSpeedProfile(
      rc26_decision::StairSpeedProfile{-0.05, -0.10, -1.0});

  EXPECT_DOUBLE_EQ(profile.fast_speed_mps, 0.05);
  EXPECT_DOUBLE_EQ(profile.slow_speed_mps, 0.05);
  EXPECT_DOUBLE_EQ(profile.slowdown_duration_s, 0.0);
}

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

TEST(MfPreselectionLogic, EntryInterruptWaitsForCenteredTarget) {
  rc26_decision::MfPreselectionParams params;
  params.entry_interrupt_max_offset_px = 180;

  EXPECT_TRUE(
      rc26_decision::MfPreselectionLogicResult::entryInterruptOffsetAcceptable(
          0, params));
  EXPECT_TRUE(
      rc26_decision::MfPreselectionLogicResult::entryInterruptOffsetAcceptable(
          -180, params));
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::entryInterruptOffsetAcceptable(
          -181, params));
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::entryInterruptOffsetAcceptable(
          240, params));
}

TEST(MfPreselectionLogic, EntryInterruptCompensatesMcuSineStopModel) {
  rc26_decision::MfPreselectionParams params;
  params.entry_interrupt_max_offset_px = 170;
  params.entry_interrupt_dynamic_comp_enable = true;
  params.entry_interrupt_latency_s = 0.15;
  params.entry_interrupt_fx_px = 450.0;
  params.entry_interrupt_extra_px_min = 20;
  params.entry_interrupt_extra_px_max = 80;
  params.entry_mcu_vy_acc_mps2 = 1.0;

  EXPECT_NEAR(
      rc26_decision::MfPreselectionLogicResult::mcuSineStopTime(0.30, 1.0),
      M_PI * 0.30 / 2.0, 1e-9);
  EXPECT_NEAR(
      rc26_decision::MfPreselectionLogicResult::mcuSineStopDistance(0.30, 1.0),
      M_PI * 0.30 * 0.30 / 4.0, 1e-9);

  const int extra =
      rc26_decision::MfPreselectionLogicResult::entryInterruptDynamicExtraPx(
          0.30, 1.0, params);
  EXPECT_EQ(extra, 52);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::
          entryInterruptEffectiveOffsetLimitPx(0.30, 1.0, params),
      222);
  EXPECT_TRUE(
      rc26_decision::MfPreselectionLogicResult::entryInterruptOffsetAcceptable(
          222, 0.30, 1.0, params));
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::entryInterruptOffsetAcceptable(
          223, 0.30, 1.0, params));
}

TEST(MfPreselectionLogic, EntryInterruptCompensationClampsAndCanDisable) {
  rc26_decision::MfPreselectionParams params;
  params.entry_interrupt_max_offset_px = 170;
  params.entry_interrupt_dynamic_comp_enable = true;
  params.entry_interrupt_latency_s = 0.15;
  params.entry_interrupt_fx_px = 450.0;
  params.entry_interrupt_extra_px_min = 20;
  params.entry_interrupt_extra_px_max = 80;
  params.entry_mcu_vy_acc_mps2 = 1.0;

  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::entryInterruptDynamicExtraPx(
          0.30, 10.0, params),
      20);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::entryInterruptDynamicExtraPx(
          0.80, 0.30, params),
      80);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::entryInterruptDynamicExtraPx(
          0.30, 0.0, params),
      0);

  params.entry_interrupt_dynamic_comp_enable = false;
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::
          entryInterruptEffectiveOffsetLimitPx(0.30, 1.0, params),
      170);
}

TEST(MfPreselectionLogic, EntryMcuStopSettleUsesStopTimeMarginAndCap) {
  rc26_decision::MfPreselectionParams params;
  params.entry_mcu_stop_settle_enable = true;
  params.entry_mcu_vy_acc_mps2 = 1.0;
  params.entry_mcu_stop_margin_s = 0.08;
  params.entry_mcu_stop_max_wait_s = 0.70;

  EXPECT_NEAR(
      rc26_decision::MfPreselectionLogicResult::entryMcuStopSettleDuration(
          0.30, params),
      M_PI * 0.30 / 2.0 + 0.08, 1e-9);

  params.entry_mcu_stop_max_wait_s = 0.40;
  EXPECT_DOUBLE_EQ(
      rc26_decision::MfPreselectionLogicResult::entryMcuStopSettleDuration(
          0.30, params),
      0.40);

  params.entry_mcu_stop_settle_enable = false;
  EXPECT_DOUBLE_EQ(
      rc26_decision::MfPreselectionLogicResult::entryMcuStopSettleDuration(
          0.30, params),
      0.0);
}

TEST(MfPreselectionLogic, AlignTimeoutPickupRequiresDepthAndLooseTolerance) {
  rc26_decision::MfPreselectionParams params;
  params.kfs_align_tolerance_px = 5;
  params.kfs_align_timeout_pickup_tolerance_px = 40;

  EXPECT_TRUE(
      rc26_decision::MfPreselectionLogicResult::kfsAlignTimeoutPickupAllowed(
          29, true, params));
  EXPECT_TRUE(
      rc26_decision::MfPreselectionLogicResult::kfsAlignTimeoutPickupAllowed(
          -40, true, params));
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::kfsAlignTimeoutPickupAllowed(
          -41, true, params));
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::kfsAlignTimeoutPickupAllowed(
          10, false, params));
}

TEST(MfPreselectionLogic, KfsAlignmentConfigUsesTipAlignmentVelocity) {
  rc26_decision::MfPreselectionParams params;
  params.kfs_align_tolerance_px = 20;
  params.kfs_align_max_jump_px = 80;
  params.kfs_lost_stop_frames = 4;
  params.kfs_align_kp = 0.001;
  params.kfs_align_min_speed_mps = 0.015;
  params.kfs_align_max_speed_mps = 0.06;
  params.kfs_invert_lateral_direction = false;
  params.heading_kp = 1.3;
  params.heading_max_speed_radps = 0.25;
  params.kfs_odom_yaw_tolerance_deg = 5.0;
  params.kfs_align_heading_gate_deg = 9.0;

  auto config =
      rc26_decision::MfPreselectionLogicResult::kfsAlignmentConfig(params, 1.2);

  EXPECT_TRUE(config.target_lock_enable);
  EXPECT_EQ(config.target_lock_max_jump_px, 80);
  EXPECT_EQ(config.lost_stop_frames, 4);
  EXPECT_EQ(config.tolerance_px, 20);
  EXPECT_DOUBLE_EQ(config.kp, 0.001);
  EXPECT_DOUBLE_EQ(config.min_speed_mps, 0.015);
  EXPECT_DOUBLE_EQ(config.max_speed_mps, 0.06);
  EXPECT_FALSE(config.invert_direction);
  EXPECT_TRUE(config.heading_hold_enable);
  EXPECT_DOUBLE_EQ(config.target_yaw_rad, 1.2);
  EXPECT_DOUBLE_EQ(config.heading_kp, 1.3);
  EXPECT_DOUBLE_EQ(config.heading_max_speed_radps, 0.25);
  EXPECT_NEAR(config.heading_tolerance_rad, 5.0 * M_PI / 180.0, 1e-9);
  EXPECT_NEAR(config.heading_gate_rad, 9.0 * M_PI / 180.0, 1e-9);

  EXPECT_DOUBLE_EQ(rc26_vision::computeTipAlignmentVy(10, config), 0.0);
  EXPECT_DOUBLE_EQ(rc26_vision::computeTipAlignmentVy(30, config), -0.03);
  EXPECT_DOUBLE_EQ(rc26_vision::computeTipAlignmentVy(-30, config), 0.03);
  EXPECT_DOUBLE_EQ(rc26_vision::computeTipAlignmentVy(200, config), -0.06);
  EXPECT_DOUBLE_EQ(rc26_vision::computeTipAlignmentVy(21, config), -0.021);

  params.kfs_align_kp = 0.0001;
  config =
      rc26_decision::MfPreselectionLogicResult::kfsAlignmentConfig(params, 1.2);
  EXPECT_DOUBLE_EQ(rc26_vision::computeTipAlignmentVy(30, config), -0.015);

  params.kfs_invert_lateral_direction = true;
  config =
      rc26_decision::MfPreselectionLogicResult::kfsAlignmentConfig(params, 1.2);
  EXPECT_DOUBLE_EQ(rc26_vision::computeTipAlignmentVy(30, config), 0.015);
}

TEST(MfPreselectionLogic, KfsApproachPlanUsesArmReachAndXAxisSign) {
  rc26_decision::MfPreselectionParams params;
  params.kfs_grab_distance_m = 0.40;
  params.kfs_approach_x_sign = 1;

  EXPECT_DOUBLE_EQ(rc26_decision::MfPreselectionLogicResult::kfsOpenLoopDistance(
                       0.55, 0.40),
                   0.15);
  EXPECT_DOUBLE_EQ(rc26_decision::MfPreselectionLogicResult::kfsOpenLoopDuration(
                       0.15, 0.05),
                   3.0);
  EXPECT_DOUBLE_EQ(
      rc26_decision::MfPreselectionLogicResult::kfsApproachOdomDistance(0.55,
                                                                        params),
      0.15);

  params.kfs_approach_x_sign = -1;
  EXPECT_DOUBLE_EQ(
      rc26_decision::MfPreselectionLogicResult::kfsApproachOdomDistance(0.55,
                                                                        params),
      -0.15);
  EXPECT_DOUBLE_EQ(rc26_decision::MfPreselectionLogicResult::kfsOpenLoopDistance(
                       0.35, 0.40),
                   0.0);
  EXPECT_DOUBLE_EQ(
      rc26_decision::MfPreselectionLogicResult::kfsApproachOdomDistance(0.35,
                                                                        params),
      -0.0);
  EXPECT_DOUBLE_EQ(rc26_decision::MfPreselectionLogicResult::kfsOpenLoopDuration(
                       0.0, 0.0),
                   0.0);
  EXPECT_TRUE(std::isinf(
      rc26_decision::MfPreselectionLogicResult::kfsOpenLoopDuration(0.10, 0.0)));
  EXPECT_GT(rc26_decision::MfPreselectionLogicResult::kfsOpenLoopDuration(
                0.90, 0.10),
            8.0);
}

TEST(MfPreselectionLogic, KfsOdomParamsNormalizeInvalidValues) {
  rc26_decision::MfPreselectionParams params;
  params.kfs_align_tolerance_px = -10;
  params.kfs_align_max_jump_px = -1;
  params.kfs_odom_xy_kp = -1.0;
  params.kfs_approach_odom_tolerance_m =
      std::numeric_limits<double>::quiet_NaN();
  params.kfs_odom_yaw_tolerance_deg =
      std::numeric_limits<double>::quiet_NaN();
  params.kfs_odom_stable_ticks = 0;
  params.kfs_approach_speed_mps = 0.02;
  params.kfs_approach_min_speed_mps = 0.05;

  rc26_decision::MfPreselectionLogicResult::normalizeKfsOdomParams(params);

  EXPECT_EQ(params.kfs_align_tolerance_px, 0);
  EXPECT_EQ(params.kfs_align_max_jump_px, 0);
  EXPECT_DOUBLE_EQ(params.kfs_odom_xy_kp, 0.8);
  EXPECT_DOUBLE_EQ(params.kfs_approach_odom_tolerance_m, 0.02);
  EXPECT_DOUBLE_EQ(params.kfs_odom_yaw_tolerance_deg, 3.0);
  EXPECT_EQ(params.kfs_odom_stable_ticks, 1);
  EXPECT_DOUBLE_EQ(params.kfs_approach_min_speed_mps, 0.02);
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

TEST(MfPreselectionLogic, FakeAvoidanceTargetGridCanBeClimbOrDescendByMap) {
  rc26_decision::MerlinMapManager red_map;
  ASSERT_TRUE(red_map.initRedMap());
  rc26_decision::MerlinMapManager blue_map;
  ASSERT_TRUE(blue_map.initBlueMap());

  const auto from_grid5_to_stair1 =
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
          5, rc26_decision::MfPreselectionPickupSource::Stair1);
  ASSERT_TRUE(from_grid5_to_stair1.has_value());
  EXPECT_EQ(*from_grid5_to_stair1, 4);
  EXPECT_EQ(red_map.getDepth(*from_grid5_to_stair1) - red_map.getDepth(5), 1);
  EXPECT_EQ(blue_map.getDepth(*from_grid5_to_stair1) - blue_map.getDepth(5), -1);

  const auto from_grid5_to_stair3 =
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceTargetGrid(
          5, rc26_decision::MfPreselectionPickupSource::Stair3);
  ASSERT_TRUE(from_grid5_to_stair3.has_value());
  EXPECT_EQ(*from_grid5_to_stair3, 6);
  EXPECT_EQ(red_map.getDepth(*from_grid5_to_stair3) - red_map.getDepth(5), -1);
  EXPECT_EQ(blue_map.getDepth(*from_grid5_to_stair3) - blue_map.getDepth(5), 1);
}

TEST(MfPreselectionLogic, FakeAvoidanceForwardTargetGridUsesSideColumns) {
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(1),
      4);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(4),
      7);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(7),
      10);

  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(3),
      6);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(6),
      9);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(9),
      12);

  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(0)
          .has_value());
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(2)
          .has_value());
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(5)
          .has_value());
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(8)
          .has_value());
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(10)
          .has_value());
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::fakeAvoidanceForwardTargetGrid(12)
          .has_value());
}

TEST(MfPreselectionLogic, EntryPickupSourceUsesLateralOffset) {
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::
          entryPickupSourceForLateralOffset(0.20, 0.03),
      rc26_decision::MfPreselectionPickupSource::Stair1);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::
          entryPickupSourceForLateralOffset(-0.20, 0.03),
      rc26_decision::MfPreselectionPickupSource::Stair3);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::
          entryPickupSourceForLateralOffset(0.02, 0.03),
      rc26_decision::MfPreselectionPickupSource::Stair2);
  EXPECT_EQ(
      rc26_decision::MfPreselectionLogicResult::
          entryPickupSourceForLateralOffset(
              std::numeric_limits<double>::quiet_NaN(), 0.03),
      rc26_decision::MfPreselectionPickupSource::Stair2);
}

TEST(MfPreselectionLogic, EntryReturnToCenterCommandUsesOffsetSign) {
  double vy = 0.0;
  double distance_m = 0.0;

  ASSERT_TRUE(rc26_decision::MfPreselectionLogicResult::
                  entryReturnToCenterCommand(0.30, 0.03, 0.20, vy,
                                             distance_m));
  EXPECT_DOUBLE_EQ(vy, -0.20);
  EXPECT_DOUBLE_EQ(distance_m, 0.30);

  ASSERT_TRUE(rc26_decision::MfPreselectionLogicResult::
                  entryReturnToCenterCommand(-0.40, 0.03, 0.20, vy,
                                             distance_m));
  EXPECT_DOUBLE_EQ(vy, 0.20);
  EXPECT_DOUBLE_EQ(distance_m, 0.40);

  ASSERT_TRUE(rc26_decision::MfPreselectionLogicResult::
                  entryReturnToCenterCommand(0.02, 0.03, 0.20, vy,
                                             distance_m));
  EXPECT_DOUBLE_EQ(vy, 0.0);
  EXPECT_DOUBLE_EQ(distance_m, 0.0);

  EXPECT_FALSE(rc26_decision::MfPreselectionLogicResult::
                   entryReturnToCenterCommand(
                       std::numeric_limits<double>::quiet_NaN(), 0.03, 0.20,
                       vy, distance_m));
}

TEST(MfPreselectionLogic, FinalExitCenterTargetUsesForwardEntryHeading) {
  constexpr double kHalfPi = 1.57079632679489661923;
  double target_x = 0.0;
  double target_y = 0.0;
  ASSERT_TRUE(rc26_decision::MfPreselectionLogicResult::finalExitCenterTarget(
      10.0, 20.0, 0.0, 1.2, target_x, target_y));
  EXPECT_NEAR(target_x, 11.2, 1e-9);
  EXPECT_NEAR(target_y, 20.0, 1e-9);

  ASSERT_TRUE(rc26_decision::MfPreselectionLogicResult::finalExitCenterTarget(
      10.0, 20.0, -kHalfPi, -2.0, target_x, target_y));
  EXPECT_NEAR(target_x, 10.0, 1e-9);
  EXPECT_NEAR(target_y, 18.0, 1e-9);

  EXPECT_FALSE(rc26_decision::MfPreselectionLogicResult::finalExitCenterTarget(
      std::numeric_limits<double>::quiet_NaN(), 20.0, 0.0, 1.2, target_x,
      target_y));
}

TEST(MfPreselectionLogic, GrabCommandFollowsHighSide) {
  rc26_decision::MfPreselectionParams params;
  params.grab_kfs_up_command_id =
      static_cast<int>(rc26_serial::CommandID::GRAB_KFS_UP);
  params.grab_kfs_down_command_id =
      static_cast<int>(rc26_serial::CommandID::GRAB_KFS_DOWN);
  params.entry_grab_kfs_up_command_id =
      static_cast<int>(rc26_serial::CommandID::ENTRY_GRAB_KFS_UP);
  params.entry_grab_kfs_up_done_feedback_id =
      static_cast<int>(rc26_serial::FeedbackID::ENTRY_GRAB_KFS_UP_DONE);

  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabCommandForHighSide(
                true, params),
            static_cast<uint8_t>(rc26_serial::CommandID::GRAB_KFS_UP));
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabCommandForHighSide(
                false, params),
            static_cast<uint8_t>(rc26_serial::CommandID::GRAB_KFS_DOWN));
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabCommandForPickup(
                true, rc26_decision::MfPreselectionPickupSource::None, false,
                params),
            static_cast<uint8_t>(rc26_serial::CommandID::GRAB_KFS_UP));
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabDoneFeedbackForPickup(
                true, rc26_decision::MfPreselectionPickupSource::None, false,
                params),
            -1);
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabCommandForPickup(
                false, rc26_decision::MfPreselectionPickupSource::Stair1, true,
                params),
            static_cast<uint8_t>(rc26_serial::CommandID::GRAB_KFS_DOWN));
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabDoneFeedbackForPickup(
                false, rc26_decision::MfPreselectionPickupSource::Stair1, true,
                params),
            -1);
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabCommandForPickup(
                true, rc26_decision::MfPreselectionPickupSource::Stair1, true,
                params),
            static_cast<uint8_t>(rc26_serial::CommandID::ENTRY_GRAB_KFS_UP));
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabDoneFeedbackForPickup(
                true, rc26_decision::MfPreselectionPickupSource::Stair1, true,
                params),
            static_cast<int>(rc26_serial::FeedbackID::ENTRY_GRAB_KFS_UP_DONE));
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabCommandForPickup(
                true, rc26_decision::MfPreselectionPickupSource::None, true,
                params),
            static_cast<uint8_t>(rc26_serial::CommandID::ENTRY_GRAB_KFS_UP));
  EXPECT_EQ(rc26_decision::MfPreselectionLogicResult::grabDoneFeedbackForPickup(
                true, rc26_decision::MfPreselectionPickupSource::None, true,
                params),
            static_cast<int>(rc26_serial::FeedbackID::ENTRY_GRAB_KFS_UP_DONE));
}

TEST(MfPreselectionLogic, PostGrabCenterAlignOnlyAppliesInsideMerlin) {
  EXPECT_TRUE(
      rc26_decision::MfPreselectionLogicResult::postGrabCenterAlignRequired(
          rc26_decision::MfPreselectionPickupSource::None, false));

  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::postGrabCenterAlignRequired(
          rc26_decision::MfPreselectionPickupSource::None, true));
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::postGrabCenterAlignRequired(
          rc26_decision::MfPreselectionPickupSource::Stair1, false));
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::postGrabCenterAlignRequired(
          rc26_decision::MfPreselectionPickupSource::Stair2, false));
  EXPECT_FALSE(
      rc26_decision::MfPreselectionLogicResult::postGrabCenterAlignRequired(
          rc26_decision::MfPreselectionPickupSource::Stair3, false));
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
