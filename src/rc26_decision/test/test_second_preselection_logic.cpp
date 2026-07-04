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
  EXPECT_EQ(xml.find("SecondPreselectionObserve"), std::string::npos);
  EXPECT_EQ(xml.find("SecondPreselectionNoEmptyFailure"), std::string::npos);
  EXPECT_EQ(xml.find("second_preselection_selected_lateral_m"), std::string::npos);
  EXPECT_EQ(xml.find("second_preselect_retreat_x_m"), std::string::npos);
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
