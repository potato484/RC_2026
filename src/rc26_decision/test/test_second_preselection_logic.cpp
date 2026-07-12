#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>

#include "rc26_decision/navigation/bt_odom_relative_nav.hpp"
#include "rc26_decision/second_preselection/second_preselection.hpp"

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

} // namespace

TEST(SecondPreselectionLogic, DefaultsUseTotalXAndFixedPlacementRoute) {
  const rc26_decision::SecondPreselectionParams params;
  EXPECT_NEAR(params.post_pickup_forward_x_m, 1.5, 1.0e-9);
  EXPECT_NEAR(params.nav_y1_m, 0.75, 1.0e-9);
  EXPECT_NEAR(params.total_x_target_m, 4.2, 1.0e-9);
  EXPECT_NEAR(params.total_x_tolerance_m, 0.03, 1.0e-9);
  EXPECT_NEAR(params.place_fixed_forward_x_m, 1.8, 1.0e-9);
  EXPECT_NEAR(params.place_fixed_forward_timeout_s, 30.0, 1.0e-9);
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
        yaml.find("second_preselect_place_fixed_forward_timeout_s: 30.0"),
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
