#include <gtest/gtest.h>

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
