#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "rc26_vision/postprocess/alignment/tip_alignment.hpp"

namespace {

rc26_vision::Detection det(float x1, float y1, float x2, float y2, float score = 0.8F)
{
  rc26_vision::Detection detection;
  detection.x1 = x1;
  detection.y1 = y1;
  detection.x2 = x2;
  detection.y2 = y2;
  detection.score = score;
  detection.class_id = 0;
  detection.class_name = "JK";
  return detection;
}

rc26_vision::TipAlignmentConfig defaultConfig()
{
  rc26_vision::TipAlignmentConfig config;
  config.target_lock_enable = true;
  config.target_lock_max_jump_px = 80;
  config.lost_stop_frames = 3;
  config.tolerance_px = 20;
  config.kp = 0.001;
  config.min_speed_mps = 0.015;
  config.max_speed_mps = 0.06;
  return config;
}

}  // namespace

TEST(TipAlignment, SelectsClosestTargetToFrameCenterWhenUnlocked)
{
  auto config = defaultConfig();
  rc26_vision::TipTargetLockState state;
  const std::vector<int> target_ids{0};

  const auto selection = rc26_vision::updateTipAlignmentTarget(
    {det(30, 100, 90, 180), det(480, 100, 560, 180)}, 640, target_ids, state, config);

  ASSERT_TRUE(selection.has_target);
  EXPECT_EQ(selection.box_cx, 520);
  EXPECT_TRUE(selection.locked);
  EXPECT_EQ(selection.lock_lost_count, 0);
}

TEST(TipAlignment, KeepsLockedTargetEvenWhenAnotherTargetBecomesCloserToCenter)
{
  auto config = defaultConfig();
  rc26_vision::TipTargetLockState state;
  const std::vector<int> target_ids{0};

  auto selection = rc26_vision::updateTipAlignmentTarget(
    {det(470, 100, 550, 180)}, 640, target_ids, state, config);
  ASSERT_TRUE(selection.has_target);
  EXPECT_EQ(selection.box_cx, 510);

  selection = rc26_vision::updateTipAlignmentTarget(
    {det(455, 100, 535, 180), det(300, 100, 340, 180)}, 640, target_ids, state, config);

  ASSERT_TRUE(selection.has_target);
  EXPECT_EQ(selection.box_cx, 495);
  EXPECT_TRUE(selection.locked);
}

TEST(TipAlignment, DoesNotSwitchDuringShortLockedTargetMiss)
{
  auto config = defaultConfig();
  rc26_vision::TipTargetLockState state;
  const std::vector<int> target_ids{0};

  auto selection = rc26_vision::updateTipAlignmentTarget(
    {det(470, 100, 550, 180)}, 640, target_ids, state, config);
  ASSERT_TRUE(selection.has_target);

  selection = rc26_vision::updateTipAlignmentTarget(
    {det(300, 100, 340, 180)}, 640, target_ids, state, config);
  EXPECT_FALSE(selection.has_target);
  EXPECT_TRUE(selection.locked);
  EXPECT_EQ(selection.lock_lost_count, 1);

  selection = rc26_vision::updateTipAlignmentTarget(
    {det(300, 100, 340, 180)}, 640, target_ids, state, config);
  EXPECT_FALSE(selection.has_target);
  EXPECT_TRUE(selection.locked);
  EXPECT_EQ(selection.lock_lost_count, 2);

  selection = rc26_vision::updateTipAlignmentTarget(
    {det(300, 100, 340, 180)}, 640, target_ids, state, config);
  ASSERT_TRUE(selection.has_target);
  EXPECT_EQ(selection.box_cx, 320);
  EXPECT_TRUE(selection.locked);
  EXPECT_EQ(selection.lock_lost_count, 0);
}

TEST(TipAlignment, RearCameraDirectionUsesPositiveVyForPositiveOffset)
{
  auto config = defaultConfig();
  config.invert_direction = true;

  EXPECT_GT(rc26_vision::computeTipAlignmentVy(120, config), 0.0);
  EXPECT_LT(rc26_vision::computeTipAlignmentVy(-120, config), 0.0);
  EXPECT_DOUBLE_EQ(rc26_vision::computeTipAlignmentVy(10, config), 0.0);
}

TEST(TipAlignment, ApproachVelocityAlwaysUsesNegativeX)
{
  EXPECT_DOUBLE_EQ(rc26_vision::computeTipApproachVx(0.04), -0.04);
  EXPECT_DOUBLE_EQ(rc26_vision::computeTipApproachVx(-0.04), -0.04);
  EXPECT_DOUBLE_EQ(rc26_vision::computeTipApproachVx(0.0), 0.0);
}

TEST(TipAlignment, HeadingControlReturnsZeroWhenDisabled)
{
  auto config = defaultConfig();
  config.heading_hold_enable = false;
  config.target_yaw_rad = 1.0;

  const auto control = rc26_vision::computeTipHeadingControl(-1.0, config);

  EXPECT_DOUBLE_EQ(control.yaw_error_rad, 0.0);
  EXPECT_DOUBLE_EQ(control.angular_z_radps, 0.0);
  EXPECT_TRUE(control.aligned);
  EXPECT_TRUE(control.within_gate);
  EXPECT_TRUE(control.allow_lateral);
}

TEST(TipAlignment, HeadingControlUsesSignedShortestYawError)
{
  auto config = defaultConfig();
  config.heading_hold_enable = true;
  config.target_yaw_rad = 0.5;
  config.heading_tolerance_rad = 0.01;
  config.heading_gate_rad = 1.0;
  config.heading_kp = 1.0;
  config.heading_max_speed_radps = 2.0;

  auto control = rc26_vision::computeTipHeadingControl(0.2, config);
  EXPECT_NEAR(control.yaw_error_rad, 0.3, 1e-9);
  EXPECT_NEAR(control.angular_z_radps, 0.3, 1e-9);
  EXPECT_FALSE(control.aligned);
  EXPECT_TRUE(control.allow_lateral);

  control = rc26_vision::computeTipHeadingControl(0.8, config);
  EXPECT_NEAR(control.yaw_error_rad, -0.3, 1e-9);
  EXPECT_NEAR(control.angular_z_radps, -0.3, 1e-9);
}

TEST(TipAlignment, HeadingControlStopsInsideTolerance)
{
  auto config = defaultConfig();
  config.heading_hold_enable = true;
  config.target_yaw_rad = 1.0;
  config.heading_tolerance_rad = 0.05;
  config.heading_gate_rad = 0.2;
  config.heading_kp = 2.0;
  config.heading_max_speed_radps = 0.5;

  const auto control = rc26_vision::computeTipHeadingControl(0.98, config);

  EXPECT_NEAR(control.yaw_error_rad, 0.02, 1e-9);
  EXPECT_DOUBLE_EQ(control.angular_z_radps, 0.0);
  EXPECT_TRUE(control.aligned);
  EXPECT_TRUE(control.within_gate);
  EXPECT_TRUE(control.allow_lateral);
}

TEST(TipAlignment, HeadingControlClampsAngularSpeed)
{
  auto config = defaultConfig();
  config.heading_hold_enable = true;
  config.target_yaw_rad = 1.0;
  config.heading_tolerance_rad = 0.01;
  config.heading_gate_rad = 2.0;
  config.heading_kp = 10.0;
  config.heading_max_speed_radps = 0.3;

  const auto control = rc26_vision::computeTipHeadingControl(0.0, config);

  EXPECT_NEAR(control.yaw_error_rad, 1.0, 1e-9);
  EXPECT_NEAR(control.angular_z_radps, 0.3, 1e-9);
  EXPECT_FALSE(control.aligned);
  EXPECT_TRUE(control.allow_lateral);
}

TEST(TipAlignment, HeadingControlBlocksLateralOutsideGate)
{
  auto config = defaultConfig();
  config.heading_hold_enable = true;
  config.target_yaw_rad = 0.0;
  config.heading_tolerance_rad = 0.05;
  config.heading_gate_rad = 0.2;
  config.heading_kp = 1.0;
  config.heading_max_speed_radps = 0.5;

  const auto control = rc26_vision::computeTipHeadingControl(-0.5, config);

  EXPECT_NEAR(control.yaw_error_rad, 0.5, 1e-9);
  EXPECT_NEAR(control.angular_z_radps, 0.5, 1e-9);
  EXPECT_FALSE(control.aligned);
  EXPECT_FALSE(control.within_gate);
  EXPECT_FALSE(control.allow_lateral);
}
