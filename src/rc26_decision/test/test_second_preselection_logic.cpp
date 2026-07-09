#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

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

} // namespace

TEST(SecondPreselectionLogic, DefaultsUseDirectR1KfsPlaceRoute) {
  const rc26_decision::SecondPreselectionParams params;
  EXPECT_NEAR(params.nav_x2_m, 4.5, 1.0e-9);
  EXPECT_NEAR(params.place_forward_x_m, 0.8, 1.0e-9);
  EXPECT_EQ(params.place_kfs_command_id, 0x13);
  EXPECT_NEAR(params.post_place_retreat_x_m, -1.5, 1.0e-9);
  EXPECT_EQ(params.post_place_front_pushrod_extend_command_id, 0x08);
  EXPECT_NEAR(params.post_place_front_pushrod_extend_settle_s, 15.0, 1.0e-9);
  EXPECT_EQ(params.post_place_preload_pickup_command_id, 0x15);
  EXPECT_EQ(params.post_place_preload_pickup_done_feedback_id, 0x14);
  EXPECT_EQ(params.post_place_manual_front_laser_feedback_id, 0x15);
  EXPECT_NEAR(params.post_place_manual_front_laser_timeout_s, 1800.0, 1.0e-9);
  EXPECT_EQ(params.post_place_front_pushrod_retract_command_id, 0x09);
  EXPECT_EQ(params.post_place_rear_pushrod_extend_command_id, 0x0A);
  EXPECT_EQ(params.post_place_rear_laser_feedback_id, 0x05);
  EXPECT_EQ(params.post_place_rear_pushrod_retract_command_id, 0x0B);
  EXPECT_NEAR(params.post_place_final_delay_s, 25.0, 1.0e-9);
  EXPECT_EQ(params.post_place_final_command_id, 0x13);
}

TEST(SecondPreselectionLogic, KfsApproachDistanceUsesLockedDepthAndReach) {
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

TEST(SecondPreselectionLogic, BehaviorTreeUsesR1KfsAlignWithoutGridObserve) {
  const auto tree_path =
      std::filesystem::path(RC26_DECISION_SOURCE_DIR) / "behavior_trees" /
      "second_preselection_tree.xml";
  const std::string xml = readTextFile(tree_path);

  EXPECT_NE(xml.find("SecondPreselectionR1KfsPlaceAlign"), std::string::npos);
  EXPECT_NE(xml.find("second_preselect_nav_x2_m"), std::string::npos);
  EXPECT_NE(xml.find("second_preselect_place_forward_x_m"), std::string::npos);
  EXPECT_NE(xml.find("second_preselection_post_place_retreat"),
            std::string::npos);
  EXPECT_NE(xml.find("second_preselect_post_place_retreat_x_m"),
            std::string::npos);
  EXPECT_NE(xml.find("SecondPreselectionPostPlaceClimb"), std::string::npos);
  EXPECT_EQ(xml.find("SecondPreselectionObserve"), std::string::npos);
  EXPECT_EQ(xml.find("SecondPreselectionNoEmptyFailure"), std::string::npos);
  EXPECT_EQ(xml.find("second_preselection_selected_lateral_m"), std::string::npos);
  EXPECT_EQ(xml.find("second_preselect_retreat_x_m"), std::string::npos);
}

TEST(SecondPreselectionLogic, BehaviorTreeRunsPostPlaceRetreatBeforeClimb) {
  const auto tree_path =
      std::filesystem::path(RC26_DECISION_SOURCE_DIR) / "behavior_trees" /
      "second_preselection_tree.xml";
  const std::string xml = readTextFile(tree_path);

  const auto place_pos = xml.find("second_preselection_place_kfs");
  const auto retreat_pos = xml.find("second_preselection_post_place_retreat");
  const auto climb_pos = xml.find("second_preselection_post_place_climb");

  ASSERT_NE(place_pos, std::string::npos);
  ASSERT_NE(retreat_pos, std::string::npos);
  ASSERT_NE(climb_pos, std::string::npos);
  EXPECT_LT(place_pos, retreat_pos);
  EXPECT_LT(retreat_pos, climb_pos);
}

TEST(SecondPreselectionLogic, BehaviorTreeXmlLoadsWithRegisteredNodes) {
  BT::BehaviorTreeFactory factory;
  rc26_decision::registerSecondPreselectionNodes(factory);
  rc26_decision::registerOdomNavigationNodes(factory);

  const auto tree_path =
      std::filesystem::path(RC26_DECISION_SOURCE_DIR) / "behavior_trees" /
      "second_preselection_tree.xml";
  auto blackboard = BT::Blackboard::create();
  EXPECT_NO_THROW({
    auto tree = factory.createTreeFromFile(tree_path.string(), blackboard);
    EXPECT_TRUE(tree.rootNode() != nullptr);
  });
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
  EXPECT_EQ(xml.find("SecondPreselectionR1KfsPlaceAlign"), std::string::npos);

  auto blackboard = BT::Blackboard::create();
  EXPECT_NO_THROW({
    auto tree = factory.createTreeFromFile(tree_path.string(), blackboard);
    EXPECT_TRUE(tree.rootNode() != nullptr);
  });
}
