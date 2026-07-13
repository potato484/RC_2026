#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rc26_decision/navigation/bt_odom_relative_nav.hpp"
#include "rc26_decision/second_preselection/second_preselection.hpp"
#include "rc26_decision/stair/stair_area.hpp"

namespace {

std::string readTextFile(const std::filesystem::path &path) {
  std::ifstream input(path);
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

std::size_t countOccurrences(const std::string &text,
                             const std::string &needle) {
  std::size_t count = 0;
  std::size_t pos = 0;
  while ((pos = text.find(needle, pos)) != std::string::npos) {
    ++count;
    pos += needle.size();
  }
  return count;
}

std::string xmlNodeBlockByName(const std::string &xml,
                               const std::string &node_name) {
  const std::string name_attr = "name=\"" + node_name + "\"";
  const auto name_pos = xml.find(name_attr);
  if (name_pos == std::string::npos) {
    return {};
  }
  const auto tag_start = xml.rfind('<', name_pos);
  const auto tag_end = xml.find("/>", name_pos);
  if (tag_start == std::string::npos || tag_end == std::string::npos) {
    return {};
  }
  return xml.substr(tag_start, tag_end + 2 - tag_start);
}

bool isZeroTwist(const geometry_msgs::msg::Twist &msg) {
  constexpr double kTolerance = 1.0e-9;
  return std::abs(msg.linear.x) <= kTolerance &&
         std::abs(msg.linear.y) <= kTolerance &&
         std::abs(msg.linear.z) <= kTolerance &&
         std::abs(msg.angular.x) <= kTolerance &&
         std::abs(msg.angular.y) <= kTolerance &&
         std::abs(msg.angular.z) <= kTolerance;
}

rc26_vision::Detection makeDetection(float cx, float cy,
                                     const std::string &label = "T_KFS",
                                     float width = 40.0F,
                                     float height = 40.0F,
                                     float score = 0.9F) {
  return rc26_vision::Detection{cx - width * 0.5F, cy - height * 0.5F,
                               cx + width * 0.5F, cy + height * 0.5F,
                               score, 1, label};
}

std::filesystem::path secondPreselectionTreePath() {
  return std::filesystem::path(RC26_DECISION_SOURCE_DIR) / "behavior_trees" /
         "second_preselection_tree.xml";
}

std::filesystem::path secondPreselectionClimbPlaceTreePath() {
  return std::filesystem::path(RC26_DECISION_SOURCE_DIR) / "behavior_trees" /
         "second_preselection_climb_place_tree.xml";
}

} // namespace

TEST(SecondPreselectionLogic, DefaultsUseTotalXAndFixedPlacementRoute) {
  const rc26_decision::SecondPreselectionParams params;
  EXPECT_NEAR(params.post_pickup_forward_x_m, 1.5, 1.0e-9);
  EXPECT_NEAR(params.nav_y1_m, 0.75, 1.0e-9);
  EXPECT_NEAR(params.total_x_target_m, 4.2, 1.0e-9);
  EXPECT_NEAR(params.total_x_tolerance_m, 0.03, 1.0e-9);
  EXPECT_NEAR(params.ramp_forward_x_m, 5.0, 1.0e-9);
  EXPECT_NEAR(params.ramp_forward_timeout_s, 5.0, 1.0e-9);
  EXPECT_NEAR(params.place_fixed_forward_x_m, 1.8, 1.0e-9);
  EXPECT_NEAR(params.place_fixed_forward_timeout_s, 30.0, 1.0e-9);
  EXPECT_NEAR(params.place_observe_timeout_s, 5.0, 1.0e-9);
  EXPECT_NEAR(params.kfs_approach_timeout_s, 8.0, 1.0e-9);
  EXPECT_NEAR(params.place_occupied_middle_y_min_ratio, 0.12, 1.0e-9);
  EXPECT_NEAR(params.place_occupied_middle_y_max_ratio, 0.45, 1.0e-9);
  EXPECT_NEAR(params.place_occupied_lower_y_min_ratio, 0.45, 1.0e-9);
  EXPECT_NEAR(params.place_occupied_lower_y_max_ratio, 1.00, 1.0e-9);
  EXPECT_EQ(params.place_occupied_stable_frames, 2);
  EXPECT_NEAR(params.place_occupied_first_lateral_m, 0.56, 1.0e-9);
  EXPECT_NEAR(params.place_occupied_second_reverse_m, 1.10, 1.0e-9);
  EXPECT_EQ(params.place_kfs_command_id, 0x13);
  EXPECT_NEAR(params.post_place_retreat_x_m, -1.5, 1.0e-9);
}

TEST(SecondPreselectionLogic, ClimbPlaceDefaultsMatchIndependentRoute) {
  const rc26_decision::SecondPreselectionParams params;
  EXPECT_NEAR(params.climb_place_forward_x_m, 1.5, 1.0e-9);
  EXPECT_NEAR(params.climb_place_lateral_y_m, 0.3, 1.0e-9);
  EXPECT_EQ(params.climb_place_pre_climb_delay_msec, 20000);
  EXPECT_EQ(params.climb_place_front_pushrod_extend_command_id, 0x08);
  EXPECT_EQ(params.climb_place_manual_front_laser_feedback_id, 0x15);
  EXPECT_EQ(params.climb_place_front_pushrod_retract_command_id, 0x09);
  EXPECT_EQ(params.climb_place_rear_pushrod_extend_command_id, 0x0A);
  EXPECT_NEAR(params.climb_place_rear_forward_x_m, 0.6, 1.0e-9);
  EXPECT_NEAR(params.climb_place_rear_max_speed_mps, 0.40, 1.0e-9);
  EXPECT_NEAR(params.climb_place_rear_min_speed_mps, 0.10, 1.0e-9);
  EXPECT_NEAR(params.climb_place_rear_timeout_s, 20.0, 1.0e-9);
  EXPECT_EQ(params.climb_place_rear_pushrod_retract_command_id, 0x0B);
  EXPECT_EQ(params.climb_place_preload_pickup_command_id, 0x15);
  EXPECT_EQ(params.climb_place_preload_pickup_done_feedback_id, 0x14);
  EXPECT_NEAR(params.climb_place_final_delay_s, 25.0, 1.0e-9);
  EXPECT_EQ(params.climb_place_final_command_id, 0x13);
}

TEST(SecondPreselectionLogic, ClimbPlaceFinalGateNeedsDelayAndPickupDone) {
  EXPECT_FALSE(rc26_decision::secondPreselectionClimbPlaceReadyForFinal(
      24.99, 25.0, true));
  EXPECT_FALSE(rc26_decision::secondPreselectionClimbPlaceReadyForFinal(
      25.0, 25.0, false));
  EXPECT_TRUE(rc26_decision::secondPreselectionClimbPlaceReadyForFinal(
      25.0, 25.0, true));
  EXPECT_TRUE(rc26_decision::secondPreselectionClimbPlaceReadyForFinal(
      30.0, 25.0, true));
}

TEST(SecondPreselectionLogic, PickupApproachDistanceKeepsExistingSignRule) {
  auto params = rc26_decision::SecondPreselectionParams{};
  params.kfs_grab_distance_m = 0.45;
  params.kfs_approach_x_sign = 1;
  EXPECT_NEAR(rc26_decision::secondPreselectionKfsApproachDistance(0.95, params),
              0.50, 1.0e-6);

  params.kfs_approach_x_sign = -1;
  EXPECT_NEAR(rc26_decision::secondPreselectionKfsApproachDistance(0.95, params),
              -0.50, 1.0e-6);

  params.kfs_approach_x_sign = 1;
  EXPECT_NEAR(rc26_decision::secondPreselectionKfsApproachDistance(0.40, params),
              0.0, 1.0e-6);
}

TEST(SecondPreselectionLogic, TotalXProjectionCountsForwardSegmentsNotLateral) {
  const auto params = rc26_decision::SecondPreselectionParams{};
  const double origin_x = 10.0;
  const double origin_y = 20.0;
  const double yaw = 0.0;

  EXPECT_NEAR(rc26_decision::secondPreselectionProjectedX(
                  origin_x, origin_y, yaw, 10.6, 20.0),
              0.6, 1.0e-9);
  EXPECT_NEAR(rc26_decision::secondPreselectionProjectedX(
                  origin_x, origin_y, yaw, 11.1, 20.0),
              1.1, 1.0e-9);
  const double after_fixed_forward =
      rc26_decision::secondPreselectionProjectedX(origin_x, origin_y, yaw,
                                                  12.6, 20.0);
  EXPECT_NEAR(after_fixed_forward, 2.6, 1.0e-9);
  EXPECT_NEAR(rc26_decision::secondPreselectionProjectedX(
                  origin_x, origin_y, yaw, 12.6, 20.75),
              after_fixed_forward, 1.0e-9);
  EXPECT_NEAR(rc26_decision::secondPreselectionTotalXRemainingToDrive(
                  after_fixed_forward, params),
              1.6, 1.0e-9);
  EXPECT_NEAR(rc26_decision::secondPreselectionTotalXRemainingToDrive(
                  4.25, params),
              0.0, 1.0e-9);
}

TEST(SecondPreselectionLogic, RampForwardNeverRequestsNegativeRemaining) {
  auto params = rc26_decision::SecondPreselectionParams{};
  params.ramp_forward_x_m = 5.0;

  EXPECT_NEAR(rc26_decision::secondPreselectionRampRemainingToDrive(0.0, params),
              5.0, 1.0e-9);
  EXPECT_NEAR(rc26_decision::secondPreselectionRampRemainingToDrive(4.8, params),
              0.2, 1.0e-9);
  EXPECT_NEAR(rc26_decision::secondPreselectionRampRemainingToDrive(5.0, params),
              0.0, 1.0e-9);
  EXPECT_NEAR(rc26_decision::secondPreselectionRampRemainingToDrive(5.4, params),
              0.0, 1.0e-9);
}

TEST(SecondPreselectionLogic, RampForwardTimeoutTriggersAtConfiguredLimit) {
  EXPECT_FALSE(rc26_decision::secondPreselectionRampTimedOut(4.99, 5.0));
  EXPECT_TRUE(rc26_decision::secondPreselectionRampTimedOut(5.0, 5.0));
  EXPECT_TRUE(rc26_decision::secondPreselectionRampTimedOut(5.01, 5.0));
  EXPECT_FALSE(rc26_decision::secondPreselectionRampTimedOut(100.0, 0.0));
}

TEST(SecondPreselectionLogic, TotalXProjectionUsesSearchStartYaw) {
  EXPECT_NEAR(rc26_decision::secondPreselectionProjectedX(
                  0.0, 0.0, M_PI_2, 5.0, 3.0),
              3.0, 1.0e-9);
}

TEST(SecondPreselectionLogic, LayerClassificationSeparatesMiddleAndLowerKfs) {
  const auto params = rc26_decision::SecondPreselectionParams{};
  constexpr int width = 640;
  constexpr int height = 480;

  auto layers = rc26_decision::secondPreselectionFrameLayers(
      {makeDetection(320.0F, 120.0F)}, width, height, params);
  EXPECT_TRUE(layers.middle);
  EXPECT_FALSE(layers.lower);
  EXPECT_TRUE(rc26_decision::secondPreselectionFrameOccupied(
      {makeDetection(320.0F, 120.0F)}, width, height, params));

  layers = rc26_decision::secondPreselectionFrameLayers(
      {makeDetection(320.0F, 360.0F)}, width, height, params);
  EXPECT_FALSE(layers.middle);
  EXPECT_TRUE(layers.lower);
  EXPECT_FALSE(rc26_decision::secondPreselectionFrameOccupied(
      {makeDetection(320.0F, 360.0F)}, width, height, params));

  layers = rc26_decision::secondPreselectionFrameLayers(
      {makeDetection(300.0F, 120.0F), makeDetection(350.0F, 360.0F, "R1_KFS")},
      width, height, params);
  EXPECT_TRUE(layers.middle);
  EXPECT_TRUE(layers.lower);

  layers = rc26_decision::secondPreselectionFrameLayers(
      {makeDetection(320.0F, 40.0F), makeDetection(100.0F, 360.0F)},
      width, height, params);
  EXPECT_FALSE(layers.middle);
  EXPECT_FALSE(layers.lower);
  EXPECT_FALSE(rc26_decision::secondPreselectionFrameOccupied(
      {makeDetection(320.0F, 40.0F), makeDetection(100.0F, 360.0F)},
      width, height, params));
}

TEST(SecondPreselectionLogic, CenterKfsUsesMiddleAndLowerCentralRoi) {
  const auto params = rc26_decision::SecondPreselectionParams{};
  constexpr int width = 640;
  constexpr int height = 480;

  EXPECT_TRUE(rc26_decision::secondPreselectionFrameHasCenterKfs(
      {makeDetection(320.0F, 300.0F, "T_KFS")}, width, height, params));
  EXPECT_TRUE(rc26_decision::secondPreselectionFrameHasCenterKfs(
      {makeDetection(320.0F, 120.0F, "UNKNOWN_KFS")}, width, height, params));
  EXPECT_FALSE(rc26_decision::secondPreselectionFrameHasCenterKfs(
      {makeDetection(100.0F, 300.0F, "T_KFS")}, width, height, params));
  EXPECT_FALSE(rc26_decision::secondPreselectionFrameHasCenterKfs(
      {makeDetection(320.0F, 40.0F, "T_KFS")}, width, height, params));
  EXPECT_FALSE(rc26_decision::secondPreselectionFrameHasCenterKfs(
      {makeDetection(320.0F, 300.0F, "NOT_TARGET")}, width, height, params));
}

TEST(SecondPreselectionLogic, VisualFrameSequenceConsumesOnlyNewFrames) {
  int64_t last_sequence = 0;
  EXPECT_TRUE(
      rc26_decision::secondPreselectionConsumeNewFrameSequence(10,
                                                              last_sequence));
  EXPECT_EQ(last_sequence, 10);
  EXPECT_FALSE(
      rc26_decision::secondPreselectionConsumeNewFrameSequence(10,
                                                              last_sequence));
  EXPECT_TRUE(
      rc26_decision::secondPreselectionConsumeNewFrameSequence(11,
                                                              last_sequence));
  EXPECT_FALSE(
      rc26_decision::secondPreselectionConsumeNewFrameSequence(0,
                                                              last_sequence));
}

TEST(SecondPreselectionLogic, OccupancyAndClearNeedConsecutiveFrames) {
  using Decision = rc26_decision::SecondPreselectionOccupancyDecision;
  int occupied_count = 0;
  int clear_count = 0;

  EXPECT_EQ(rc26_decision::secondPreselectionUpdateOccupancyStability(
                true, 2, occupied_count, clear_count),
            Decision::Pending);
  EXPECT_EQ(rc26_decision::secondPreselectionUpdateOccupancyStability(
                true, 2, occupied_count, clear_count),
            Decision::Occupied);
  EXPECT_EQ(rc26_decision::secondPreselectionUpdateOccupancyStability(
                false, 2, occupied_count, clear_count),
            Decision::Pending);
  EXPECT_EQ(occupied_count, 0);
  EXPECT_EQ(rc26_decision::secondPreselectionUpdateOccupancyStability(
                false, 2, occupied_count, clear_count),
            Decision::Clear);
}

TEST(SecondPreselectionLogic, AvoidanceDistancesMirrorForRedAndBlue) {
  auto params = rc26_decision::SecondPreselectionParams{};
  params.team_mirror_sign = 1;
  EXPECT_NEAR(rc26_decision::secondPreselectionPlaceAvoidanceDistance(1, params),
              0.56, 1.0e-9);
  EXPECT_NEAR(rc26_decision::secondPreselectionPlaceAvoidanceDistance(2, params),
              -1.10, 1.0e-9);

  params.team_mirror_sign = -1;
  EXPECT_NEAR(rc26_decision::secondPreselectionPlaceAvoidanceDistance(1, params),
              -0.56, 1.0e-9);
  EXPECT_NEAR(rc26_decision::secondPreselectionPlaceAvoidanceDistance(2, params),
              1.10, 1.0e-9);
}

TEST(SecondPreselectionLogic, PlacementApproachUsesFixedForwardDistance) {
  auto params = rc26_decision::SecondPreselectionParams{};
  EXPECT_NEAR(params.place_fixed_forward_x_m, 1.8, 1.0e-9);
}

TEST(SecondPreselectionLogic, FixedForwardTimeoutKeepsStrictLimit) {
  EXPECT_FALSE(
      rc26_decision::secondPreselectionPlaceApproachTimedOut(29.99, 30.0));
  EXPECT_FALSE(
      rc26_decision::secondPreselectionPlaceApproachTimedOut(30.0, 30.0));
  EXPECT_TRUE(
      rc26_decision::secondPreselectionPlaceApproachTimedOut(30.01, 30.0));
  EXPECT_FALSE(
      rc26_decision::secondPreselectionPlaceApproachTimedOut(100.0, 0.0));
}

TEST(SecondPreselectionLogic, PlaceObserveTimeoutKeepsStrictLimit) {
  EXPECT_FALSE(
      rc26_decision::secondPreselectionPlaceObserveTimedOut(4.99, 5.0));
  EXPECT_FALSE(
      rc26_decision::secondPreselectionPlaceObserveTimedOut(5.0, 5.0));
  EXPECT_TRUE(
      rc26_decision::secondPreselectionPlaceObserveTimedOut(5.01, 5.0));
  EXPECT_FALSE(
      rc26_decision::secondPreselectionPlaceObserveTimedOut(100.0, 0.0));
}

TEST(SecondPreselectionLogic, OdomAxisDriveReachOrOvershootModeStopsPullback) {
  EXPECT_TRUE(rc26_decision::odomAxisDriveReachedOrOvershot(
      1.5, 0.02, 0.03, true));
  EXPECT_TRUE(rc26_decision::odomAxisDriveReachedOrOvershot(
      1.5, -0.10, 0.03, true));
  EXPECT_TRUE(rc26_decision::odomAxisDriveReachedOrOvershot(
      -1.5, 0.10, 0.03, true));

  EXPECT_FALSE(rc26_decision::odomAxisDriveReachedOrOvershot(
      1.5, 0.10, 0.03, true));
  EXPECT_FALSE(rc26_decision::odomAxisDriveReachedOrOvershot(
      -1.5, -0.10, 0.03, true));
  EXPECT_FALSE(rc26_decision::odomAxisDriveReachedOrOvershot(
      1.5, -0.10, 0.03, false));
  EXPECT_FALSE(rc26_decision::odomAxisDriveReachedOrOvershot(
      -1.5, 0.10, 0.03, false));
}

TEST(SecondPreselectionLogic, BehaviorTreeUsesNewPlacementSequence) {
  const std::string xml = readTextFile(secondPreselectionTreePath());

  const auto pickup_pos = xml.find("second_preselection_kfs_pickup");
  const auto fixed_x_pos = xml.find("second_preselection_post_pickup_forward");
  const auto lateral_pos = xml.find("second_preselection_nav_y1");
  const auto total_x_pos = xml.find("second_preselection_drive_to_total_x");
  const auto prepare_pos = xml.find("second_preselection_kfs_place_prepare");
  const auto approach_pos = xml.find("second_preselection_place_approach");
  const auto place_pos = xml.find("second_preselection_place_kfs");
  const auto retreat_pos = xml.find("second_preselection_post_place_retreat");
  const auto climb_pos = xml.find("second_preselection_post_place_climb");

  ASSERT_NE(pickup_pos, std::string::npos);
  ASSERT_NE(fixed_x_pos, std::string::npos);
  ASSERT_NE(lateral_pos, std::string::npos);
  ASSERT_NE(total_x_pos, std::string::npos);
  ASSERT_NE(prepare_pos, std::string::npos);
  ASSERT_NE(approach_pos, std::string::npos);
  ASSERT_NE(place_pos, std::string::npos);
  ASSERT_NE(retreat_pos, std::string::npos);
  ASSERT_NE(climb_pos, std::string::npos);
  EXPECT_LT(pickup_pos, fixed_x_pos);
  EXPECT_LT(fixed_x_pos, lateral_pos);
  EXPECT_LT(lateral_pos, total_x_pos);
  EXPECT_LT(total_x_pos, prepare_pos);
  EXPECT_LT(prepare_pos, approach_pos);
  EXPECT_LT(approach_pos, place_pos);
  EXPECT_LT(place_pos, retreat_pos);
  EXPECT_LT(retreat_pos, climb_pos);

  EXPECT_EQ(countOccurrences(
                xml,
                "<SecondPreselectionCommand name=\"second_preselection_place_kfs\""),
            1U);
  EXPECT_EQ(xml.find("SecondPreselectionR1KfsPlaceAlign"), std::string::npos);
  EXPECT_EQ(xml.find("second_preselect_nav_x2_m"), std::string::npos);
  EXPECT_EQ(xml.find("second_preselect_place_forward_x_m"), std::string::npos);
}

TEST(SecondPreselectionLogic, SecondFixedOdomSegmentsStopOnOvershoot) {
  const std::string xml = readTextFile(secondPreselectionTreePath());
  EXPECT_EQ(countOccurrences(xml, "succeed_on_reach_or_overshoot=\"true\""),
            3U);

  for (const char *node_name :
       {"second_preselection_post_pickup_forward", "second_preselection_nav_y1",
        "second_preselection_post_place_retreat"}) {
    const std::string block = xmlNodeBlockByName(xml, node_name);
    ASSERT_FALSE(block.empty()) << node_name;
    EXPECT_NE(block.find("succeed_on_reach_or_overshoot=\"true\""),
              std::string::npos)
        << node_name;
  }
}

TEST(SecondPreselectionLogic, BehaviorTreeXmlLoadsWithRegisteredNodes) {
  BT::BehaviorTreeFactory factory;
  rc26_decision::registerSecondPreselectionNodes(factory);
  rc26_decision::registerOdomNavigationNodes(factory);

  auto blackboard = BT::Blackboard::create();
  EXPECT_NO_THROW({
    auto tree =
        factory.createTreeFromFile(secondPreselectionTreePath().string(),
                                   blackboard);
    EXPECT_TRUE(tree.rootNode() != nullptr);
  });
}

TEST(SecondPreselectionLogic, RedAndBlueConfigsUseNewParameters) {
  const auto config_dir =
      std::filesystem::path(RC26_DECISION_SOURCE_DIR).parent_path() /
      "rc26_bringup" / "config";
  for (const char *filename : {"r2_red.yaml", "r2_blue.yaml"}) {
    const std::string yaml = readTextFile(config_dir / filename);
    EXPECT_NE(yaml.find("second_preselect_post_pickup_forward_x_m: 1.5"),
              std::string::npos);
    EXPECT_NE(yaml.find("second_preselect_total_x_target_m: 4.2"),
              std::string::npos);
    EXPECT_NE(yaml.find("second_preselect_place_fixed_forward_x_m: 1.8"),
              std::string::npos);
    EXPECT_NE(
        yaml.find("second_preselect_place_fixed_forward_timeout_s: 5.0"),
        std::string::npos);
    EXPECT_NE(
        yaml.find("second_preselect_place_observe_timeout_s: 5.0"),
        std::string::npos);
    EXPECT_NE(yaml.find("preselection_ramp_forward_x_m: 5.00"),
              std::string::npos);
    EXPECT_NE(yaml.find("preselection_ramp_forward_timeout_s: 5.0"),
              std::string::npos);
    EXPECT_NE(
        yaml.find("second_preselect_place_occupied_middle_y_min_ratio: 0.12"),
              std::string::npos);
    EXPECT_NE(
        yaml.find("second_preselect_place_occupied_middle_y_max_ratio: 0.45"),
              std::string::npos);
    EXPECT_NE(
        yaml.find("second_preselect_place_occupied_lower_y_min_ratio: 0.45"),
              std::string::npos);
    EXPECT_NE(
        yaml.find("second_preselect_place_occupied_lower_y_max_ratio: 1.00"),
              std::string::npos);
    EXPECT_EQ(yaml.find("second_preselect_place_approach_timeout_s"),
              std::string::npos);
    EXPECT_EQ(yaml.find("second_preselect_nav_x2_m"), std::string::npos);
    EXPECT_EQ(yaml.find("second_preselect_place_forward_x_m"),
              std::string::npos);
    EXPECT_EQ(yaml.find("preselection_ramp_approach_x_m"),
              std::string::npos);
    EXPECT_EQ(yaml.find("preselection_ramp_climb_x_m"), std::string::npos);
    EXPECT_EQ(yaml.find("preselection_ramp_timeout_s"), std::string::npos);
  }
}

TEST(SecondPreselectionLogic, PostPlaceClimbDebugTreeOnlyRunsPostPlaceAction) {
  BT::BehaviorTreeFactory factory;
  rc26_decision::registerSecondPreselectionNodes(factory);

  const auto tree_path =
      std::filesystem::path(RC26_DECISION_SOURCE_DIR) / "behavior_trees" /
      "second_preselection_post_place_climb_tree.xml";
  const std::string xml = readTextFile(tree_path);

  EXPECT_NE(xml.find("SecondPreselectionPostPlaceClimb"), std::string::npos);
  EXPECT_EQ(xml.find("OdomDriveX"), std::string::npos);
  EXPECT_EQ(xml.find("SecondPreselectionKfsPickup"), std::string::npos);
  EXPECT_EQ(xml.find("SecondPreselectionKfsPlacePrepare"), std::string::npos);

  auto blackboard = BT::Blackboard::create();
  EXPECT_NO_THROW({
    auto tree = factory.createTreeFromFile(tree_path.string(), blackboard);
    EXPECT_TRUE(tree.rootNode() != nullptr);
  });
}

TEST(SecondPreselectionLogic, ClimbPlaceTreeUsesIndependentOrderedSequence) {
  const std::string xml = readTextFile(secondPreselectionClimbPlaceTreePath());

  const auto forward_pos =
      xml.find("second_preselection_climb_place_forward");
  const auto lateral_pos =
      xml.find("second_preselection_climb_place_lateral");
  const auto delay_pos = xml.find("second_preselect_climb_place_pre_climb_delay_msec");
  const auto front_stage_pos =
      xml.find("second_preselection_climb_place_front_stage");
  const auto rear_forward_pos =
      xml.find("second_preselection_climb_place_rear_forward");
  const auto finish_pos = xml.find("second_preselection_climb_place_finish");

  ASSERT_NE(forward_pos, std::string::npos);
  ASSERT_NE(lateral_pos, std::string::npos);
  ASSERT_NE(delay_pos, std::string::npos);
  ASSERT_NE(front_stage_pos, std::string::npos);
  ASSERT_NE(rear_forward_pos, std::string::npos);
  ASSERT_NE(finish_pos, std::string::npos);
  EXPECT_LT(forward_pos, lateral_pos);
  EXPECT_LT(lateral_pos, delay_pos);
  EXPECT_LT(delay_pos, front_stage_pos);
  EXPECT_LT(front_stage_pos, rear_forward_pos);
  EXPECT_LT(rear_forward_pos, finish_pos);

  EXPECT_EQ(countOccurrences(xml, "succeed_on_reach_or_overshoot=\"true\""),
            3U);
  EXPECT_EQ(countOccurrences(xml, "succeed_on_timeout=\"true\""), 1U);
  EXPECT_NE(xml.find("SecondPreselectionClimbFrontStage"), std::string::npos);
  EXPECT_NE(xml.find("SecondPreselectionRearRetractPickupPlace"),
            std::string::npos);
  EXPECT_EQ(xml.find("WaitPreselectionBranchGate"), std::string::npos);
  EXPECT_EQ(xml.find("SecondPreselectionCommand"), std::string::npos);
  EXPECT_EQ(xml.find("StairClimb"), std::string::npos);
  EXPECT_EQ(xml.find("0x05"), std::string::npos);
  EXPECT_EQ(xml.find("second_preselect_start_command_id"), std::string::npos);
  EXPECT_EQ(xml.find("second_preselect_start_done_feedback_id"),
            std::string::npos);
  EXPECT_EQ(xml.find("SECOND_PRESELECTION_START"), std::string::npos);

  const std::string rear_block = xmlNodeBlockByName(
      xml, "second_preselection_climb_place_rear_forward");
  ASSERT_FALSE(rear_block.empty());
  EXPECT_NE(rear_block.find(
                "distance_m=\"{second_preselect_climb_place_rear_forward_x_m}\""),
            std::string::npos);
  EXPECT_NE(rear_block.find("succeed_on_reach_or_overshoot=\"true\""),
            std::string::npos);
  EXPECT_NE(rear_block.find("succeed_on_timeout=\"true\""),
            std::string::npos);
}

TEST(SecondPreselectionLogic, ClimbPlaceTreeXmlLoadsWithRegisteredNodes) {
  BT::BehaviorTreeFactory factory;
  rc26_decision::registerSecondPreselectionNodes(factory);
  rc26_decision::registerOdomNavigationNodes(factory);

  auto blackboard = BT::Blackboard::create();
  EXPECT_NO_THROW({
    auto tree = factory.createTreeFromFile(
        secondPreselectionClimbPlaceTreePath().string(), blackboard);
    EXPECT_TRUE(tree.rootNode() != nullptr);
  });
}

TEST(SecondPreselectionLogic, RedAndBlueConfigsExposeClimbPlaceParameters) {
  const auto config_dir =
      std::filesystem::path(RC26_DECISION_SOURCE_DIR).parent_path() /
      "rc26_bringup" / "config";
  for (const char *filename : {"r2_red.yaml", "r2_blue.yaml"}) {
    const std::string yaml = readTextFile(config_dir / filename);
    EXPECT_NE(yaml.find("second_preselect_climb_place_forward_x_m: 1.5"),
              std::string::npos);
    EXPECT_NE(yaml.find("second_preselect_climb_place_lateral_y_m: 0.3"),
              std::string::npos);
    EXPECT_NE(
        yaml.find("second_preselect_climb_place_pre_climb_delay_msec: 20000"),
        std::string::npos);
    EXPECT_NE(yaml.find("second_preselect_climb_place_rear_forward_x_m: 0.6"),
              std::string::npos);
    EXPECT_NE(yaml.find(
                  "second_preselect_climb_place_manual_front_laser_feedback_id: 21"),
              std::string::npos);
    EXPECT_NE(yaml.find(
                  "second_preselect_climb_place_preload_pickup_done_feedback_id: 20"),
              std::string::npos);
    EXPECT_NE(yaml.find("second_preselect_climb_place_final_delay_s: 25.0"),
              std::string::npos);
    EXPECT_NE(yaml.find("second_preselect_climb_place_final_command_id: 19"),
              std::string::npos);
  }
}

TEST(SecondPreselectionLogic,
     ClimbFrontStageStaysStoppedUntilManualFrontLaser) {
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }

  auto decision_node = std::make_shared<rclcpp::Node>(
      "second_preselection_climb_front_stage_test");
  auto fake_transport =
      std::make_shared<rclcpp::Node>("second_preselection_climb_front_fake");
  const std::string service_name = "/test/climb_front/send_command";
  const std::string feedback_topic = "/test/climb_front/command_feedback";
  const std::string cmd_vel_topic = "/test/climb_front/cmd_vel";

  std::mutex commands_mutex;
  std::vector<uint8_t> commands;
  std::atomic<int> next_seq{20};
  auto service = fake_transport->create_service<
      rc26_interfaces::srv::SendMechanismTransportCommand>(
      service_name,
      [&](const std::shared_ptr<
              rc26_interfaces::srv::SendMechanismTransportCommand::Request>
              request,
          std::shared_ptr<
              rc26_interfaces::srv::SendMechanismTransportCommand::Response>
              response) {
        {
          std::lock_guard<std::mutex> lock(commands_mutex);
          commands.push_back(request->command_id);
        }
        response->accepted = true;
        response->seq =
            static_cast<uint8_t>(next_seq.fetch_add(1) & 0xFF);
      });
  ASSERT_TRUE(service != nullptr);

  std::mutex twists_mutex;
  std::vector<geometry_msgs::msg::Twist> twists;
  auto cmd_vel_sub = fake_transport->create_subscription<
      geometry_msgs::msg::Twist>(
      cmd_vel_topic, rclcpp::QoS(100),
      [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (msg) {
          std::lock_guard<std::mutex> lock(twists_mutex);
          twists.push_back(*msg);
        }
      });
  ASSERT_TRUE(cmd_vel_sub != nullptr);
  auto feedback_pub = fake_transport->create_publisher<
      rc26_interfaces::msg::MechanismTransportFeedback>(feedback_topic,
                                                        rclcpp::QoS(10));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(decision_node);
  executor.add_node(fake_transport);
  std::thread spin_thread([&executor]() { executor.spin(); });

  BT::BehaviorTreeFactory factory;
  rc26_decision::registerSecondPreselectionNodes(factory);
  auto blackboard = BT::Blackboard::create();
  rclcpp::Node *decision_node_ptr = decision_node.get();
  blackboard->set("node", decision_node_ptr);

  auto stair_params = rc26_decision::StairParams{};
  stair_params.cmd_vel_topic = cmd_vel_topic;
  stair_params.send_command_service = service_name;
  stair_params.feedback_topic = feedback_topic;
  stair_params.heading_hold_enable = false;
  stair_params.command_timeout_s = 1.0;
  stair_params.front_event_timeout_s = 2.0;
  stair_params.climb_front_extend_delay_s = 0.0;
  stair_params.climb_retract_rear_extend_delay_s = 0.0;
  blackboard->set("stair_params", stair_params);

  auto second_params = rc26_decision::SecondPreselectionParams{};
  second_params.send_command_service = service_name;
  second_params.feedback_topic = feedback_topic;
  second_params.cmd_vel_topic = cmd_vel_topic;
  blackboard->set("second_preselection_params", second_params);

  auto tree = factory.createTreeFromText(
      R"(<root BTCPP_format="4" main_tree_to_execute="TestTree">
           <BehaviorTree ID="TestTree">
             <SecondPreselectionClimbFrontStage name="front_stage"/>
           </BehaviorTree>
         </root>)",
      blackboard);

  auto tick_for = [&](std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while (std::chrono::steady_clock::now() < deadline &&
           status != BT::NodeStatus::SUCCESS &&
           status != BT::NodeStatus::FAILURE) {
      status = tree.tickOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return status;
  };

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_EQ(tick_for(std::chrono::milliseconds(300)),
            BT::NodeStatus::RUNNING);
  {
    std::lock_guard<std::mutex> lock(commands_mutex);
    EXPECT_EQ(commands.size(), 1U);
    if (!commands.empty()) {
      EXPECT_EQ(commands[0], 0x08);
    }
  }
  {
    std::lock_guard<std::mutex> lock(twists_mutex);
    EXPECT_FALSE(twists.empty());
    if (!twists.empty()) {
      EXPECT_TRUE(std::all_of(twists.begin(), twists.end(), isZeroTwist));
    }
  }

  rc26_interfaces::msg::MechanismTransportFeedback ignored_rear_laser;
  ignored_rear_laser.feedback_id = 0x05;
  feedback_pub->publish(ignored_rear_laser);
  EXPECT_EQ(tick_for(std::chrono::milliseconds(100)),
            BT::NodeStatus::RUNNING);
  {
    std::lock_guard<std::mutex> lock(commands_mutex);
    EXPECT_EQ(commands.size(), 1U);
  }

  rc26_interfaces::msg::MechanismTransportFeedback manual_front_laser;
  manual_front_laser.feedback_id = 0x15;
  feedback_pub->publish(manual_front_laser);
  EXPECT_EQ(tick_for(std::chrono::milliseconds(500)),
            BT::NodeStatus::SUCCESS);
  {
    std::lock_guard<std::mutex> lock(commands_mutex);
    EXPECT_EQ(commands.size(), 3U);
    if (commands.size() >= 3U) {
      EXPECT_EQ(commands[0], 0x08);
      EXPECT_EQ(commands[1], 0x09);
      EXPECT_EQ(commands[2], 0x0A);
    }
  }
  {
    std::lock_guard<std::mutex> lock(twists_mutex);
    EXPECT_FALSE(twists.empty());
    if (!twists.empty()) {
      EXPECT_TRUE(std::all_of(twists.begin(), twists.end(), isZeroTwist));
    }
  }

  tree.haltTree();
  executor.cancel();
  spin_thread.join();
  executor.remove_node(decision_node);
  executor.remove_node(fake_transport);
}

TEST(SecondPreselectionLogic,
     ClimbRearOdomTimeoutStopsAndContinuesAsSuccess) {
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }

  auto decision_node =
      std::make_shared<rclcpp::Node>("second_preselection_rear_timeout_test");
  auto io_node =
      std::make_shared<rclcpp::Node>("second_preselection_rear_timeout_io");
  const std::string odom_topic = "/test/climb_rear_timeout/odom";
  const std::string cmd_vel_topic = "/test/climb_rear_timeout/cmd_vel";

  auto odom_pub = io_node->create_publisher<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(10));
  std::mutex twists_mutex;
  std::vector<geometry_msgs::msg::Twist> twists;
  auto cmd_vel_sub = io_node->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, rclcpp::QoS(100),
      [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (msg) {
          std::lock_guard<std::mutex> lock(twists_mutex);
          twists.push_back(*msg);
        }
      });
  ASSERT_TRUE(odom_pub != nullptr);
  ASSERT_TRUE(cmd_vel_sub != nullptr);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(decision_node);
  executor.add_node(io_node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  BT::BehaviorTreeFactory factory;
  rc26_decision::registerOdomNavigationNodes(factory);
  auto blackboard = BT::Blackboard::create();
  rclcpp::Node *decision_node_ptr = decision_node.get();
  blackboard->set("node", decision_node_ptr);
  auto tree = factory.createTreeFromText(
      R"(<root BTCPP_format="4" main_tree_to_execute="TestTree">
           <BehaviorTree ID="TestTree">
             <OdomDriveX name="rear_forward_timeout"
                         distance_m="0.6"
                         cmd_vel_topic="/test/climb_rear_timeout/cmd_vel"
                         odom_topic="/test/climb_rear_timeout/odom"
                         max_speed_mps="0.4"
                         min_speed_mps="0.1"
                         stable_ticks="1"
                         odom_timeout_s="0.5"
                         succeed_on_reach_or_overshoot="true"
                         succeed_on_timeout="true"
                         timeout_s="0.2"/>
           </BehaviorTree>
         </root>)",
      blackboard);

  BT::NodeStatus status = tree.tickOnce();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  while (std::chrono::steady_clock::now() < deadline &&
         status != BT::NodeStatus::SUCCESS &&
         status != BT::NodeStatus::FAILURE) {
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.orientation.w = 1.0;
    odom_pub->publish(odom);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    status = tree.tickOnce();
  }

  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  std::string exec_state;
  EXPECT_TRUE(blackboard->get("relative_nav_last_exec_state", exec_state));
  EXPECT_EQ(exec_state, "SUCCEEDED");
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  {
    std::lock_guard<std::mutex> lock(twists_mutex);
    EXPECT_FALSE(twists.empty());
    if (!twists.empty()) {
      EXPECT_TRUE(isZeroTwist(twists.back()));
    }
  }

  tree.haltTree();
  executor.cancel();
  spin_thread.join();
  executor.remove_node(decision_node);
  executor.remove_node(io_node);
}

TEST(SecondPreselectionLogic,
     RearRetractPickupWaitsForMatchingDoneBeforeFinalPlace) {
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }

  auto decision_node = std::make_shared<rclcpp::Node>(
      "second_preselection_climb_place_action_test");
  auto fake_transport =
      std::make_shared<rclcpp::Node>("second_preselection_fake_transport");
  const std::string service_name = "/test/climb_place/send_command";
  const std::string feedback_topic = "/test/climb_place/command_feedback";
  const std::string cmd_vel_topic = "/test/climb_place/cmd_vel";

  std::mutex commands_mutex;
  std::vector<uint8_t> commands;
  std::atomic<int> next_seq{40};
  std::atomic<int> pickup_seq{-1};
  auto service = fake_transport->create_service<
      rc26_interfaces::srv::SendMechanismTransportCommand>(
      service_name,
      [&](const std::shared_ptr<
              rc26_interfaces::srv::SendMechanismTransportCommand::Request>
              request,
          std::shared_ptr<
              rc26_interfaces::srv::SendMechanismTransportCommand::Response>
              response) {
        const int seq = next_seq.fetch_add(1);
        {
          std::lock_guard<std::mutex> lock(commands_mutex);
          commands.push_back(request->command_id);
        }
        if (request->command_id == 0x15) {
          pickup_seq.store(seq);
        }
        response->accepted = true;
        response->seq = static_cast<uint8_t>(seq & 0xFF);
      });
  ASSERT_TRUE(service != nullptr);
  auto feedback_pub = fake_transport->create_publisher<
      rc26_interfaces::msg::MechanismTransportFeedback>(feedback_topic,
                                                        rclcpp::QoS(10));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(decision_node);
  executor.add_node(fake_transport);
  std::thread spin_thread([&executor]() { executor.spin(); });

  BT::BehaviorTreeFactory factory;
  rc26_decision::registerSecondPreselectionNodes(factory);
  auto blackboard = BT::Blackboard::create();
  rclcpp::Node *decision_node_ptr = decision_node.get();
  blackboard->set("node", decision_node_ptr);
  auto params = rc26_decision::SecondPreselectionParams{};
  params.send_command_service = service_name;
  params.feedback_topic = feedback_topic;
  params.cmd_vel_topic = cmd_vel_topic;
  params.command_timeout_s = 1.0;
  params.done_timeout_s = 1.0;
  params.log_period_s = 0.05;
  params.climb_place_final_delay_s = 0.0;
  blackboard->set("second_preselection_params", params);

  auto tree = factory.createTreeFromText(
      R"(<root BTCPP_format="4" main_tree_to_execute="TestTree">
           <BehaviorTree ID="TestTree">
             <SecondPreselectionRearRetractPickupPlace name="finish"/>
           </BehaviorTree>
         </root>)",
      blackboard);

  auto tick_for = [&](std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    while (std::chrono::steady_clock::now() < deadline &&
           status != BT::NodeStatus::SUCCESS &&
           status != BT::NodeStatus::FAILURE) {
      status = tree.tickOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return status;
  };

  EXPECT_EQ(tick_for(std::chrono::milliseconds(250)),
            BT::NodeStatus::RUNNING);
  EXPECT_GE(pickup_seq.load(), 0);
  {
    std::lock_guard<std::mutex> lock(commands_mutex);
    EXPECT_EQ(commands.size(), 2U);
    EXPECT_NE(std::find(commands.begin(), commands.end(), 0x0B),
              commands.end());
    EXPECT_NE(std::find(commands.begin(), commands.end(), 0x15),
              commands.end());
  }

  rc26_interfaces::msg::MechanismTransportFeedback ignored_rear_laser;
  ignored_rear_laser.feedback_id = 0x05;
  feedback_pub->publish(ignored_rear_laser);
  rc26_interfaces::msg::MechanismTransportFeedback wrong_done;
  wrong_done.seq = static_cast<uint8_t>((pickup_seq.load() + 1) & 0xFF);
  wrong_done.feedback_id = 0x14;
  feedback_pub->publish(wrong_done);
  EXPECT_EQ(tick_for(std::chrono::milliseconds(150)),
            BT::NodeStatus::RUNNING);
  {
    std::lock_guard<std::mutex> lock(commands_mutex);
    EXPECT_EQ(commands.size(), 2U);
  }

  rc26_interfaces::msg::MechanismTransportFeedback correct_done;
  correct_done.seq = static_cast<uint8_t>(pickup_seq.load() & 0xFF);
  correct_done.feedback_id = 0x14;
  feedback_pub->publish(correct_done);
  EXPECT_EQ(tick_for(std::chrono::milliseconds(500)),
            BT::NodeStatus::SUCCESS);
  {
    std::lock_guard<std::mutex> lock(commands_mutex);
    EXPECT_EQ(commands.size(), 3U);
    if (commands.size() >= 3U) {
      EXPECT_EQ(commands[2], 0x13);
    }
  }

  tree.haltTree();
  executor.cancel();
  spin_thread.join();
  executor.remove_node(decision_node);
  executor.remove_node(fake_transport);
}
