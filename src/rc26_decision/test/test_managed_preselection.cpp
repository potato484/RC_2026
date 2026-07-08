#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>

#include "rc26_decision/mc/mc_area.hpp"
#include "rc26_decision/mc_preselection_repeat_logic.hpp"
#include "rc26_decision/mf/mf_area.hpp"
#include "rc26_decision/mf_preselection/mf_preselection_flow.hpp"
#include "rc26_decision/navigation/bt_odom_relative_nav.hpp"
#include "rc26_decision/preselection_branch_gate_logic.hpp"
#include "rc26_decision/second_preselection/second_preselection.hpp"
#include "rc26_decision/stair/stair_area.hpp"
#include "rc26_decision/tree_switch_request.hpp"
#include "rc26_serial/protocol.hpp"

TEST(MfPreselectionTree, XmlLoadsWithRegisteredNodes) {
  BT::BehaviorTreeFactory factory;
  rc26_decision::registerMCAreaNodes(factory);
  rc26_decision::registerMFAreaNodes(factory);
  rc26_decision::registerMfPreselectionNodes(factory);
  rc26_decision::registerOdomNavigationNodes(factory);
  rc26_decision::registerStairNodes(factory);

  const auto tree_path =
      std::filesystem::path(RC26_DECISION_SOURCE_DIR) / "behavior_trees" /
      "mf_preselection_tree.xml";
  auto blackboard = BT::Blackboard::create();
  EXPECT_NO_THROW({
    auto tree = factory.createTreeFromFile(tree_path.string(), blackboard);
    EXPECT_TRUE(tree.rootNode() != nullptr);
  });
}

TEST(PreselectionBranchGateLogic, SelectsContinueSwitchAndDoneBySameSeq) {
  EXPECT_EQ(rc26_decision::selectPreselectionBranch(0x06, 0x06, 0x10),
            rc26_decision::PreselectionBranchSelection::ContinueFirst);
  EXPECT_EQ(rc26_decision::selectPreselectionBranch(0x10, 0x06, 0x10),
            rc26_decision::PreselectionBranchSelection::SwitchTarget);
  EXPECT_EQ(rc26_decision::selectPreselectionBranch(0x0C, 0x06, 0x10),
            rc26_decision::PreselectionBranchSelection::None);

  EXPECT_TRUE(rc26_decision::isSameSeqDoneFeedback(7, 0x0C, 7, 0x0C));
  EXPECT_FALSE(rc26_decision::isSameSeqDoneFeedback(8, 0x0C, 7, 0x0C));
  EXPECT_FALSE(rc26_decision::isSameSeqDoneFeedback(7, 0x0D, 7, 0x0C));

  EXPECT_EQ(rc26_decision::parsePreselectionStartProfile("mc"),
            rc26_decision::PreselectionStartProfile::Mc);
  EXPECT_EQ(rc26_decision::parsePreselectionStartProfile("second"),
            rc26_decision::PreselectionStartProfile::Second);
  EXPECT_EQ(rc26_decision::parsePreselectionStartProfile(""),
            rc26_decision::PreselectionStartProfile::Mc);
  EXPECT_EQ(rc26_decision::parsePreselectionBranchMode("continue_only"),
            rc26_decision::PreselectionBranchMode::ContinueOnly);
  EXPECT_EQ(rc26_decision::parsePreselectionBranchMode("switch_only"),
            rc26_decision::PreselectionBranchMode::SwitchOnly);
  EXPECT_EQ(rc26_decision::parsePreselectionBranchMode(""),
            rc26_decision::PreselectionBranchMode::Both);
  EXPECT_TRUE(rc26_decision::isPreselectionBranchAllowed(
      rc26_decision::PreselectionBranchSelection::ContinueFirst,
      rc26_decision::PreselectionBranchMode::ContinueOnly));
  EXPECT_FALSE(rc26_decision::isPreselectionBranchAllowed(
      rc26_decision::PreselectionBranchSelection::SwitchTarget,
      rc26_decision::PreselectionBranchMode::ContinueOnly));
  EXPECT_TRUE(rc26_decision::isPreselectionBranchAllowed(
      rc26_decision::PreselectionBranchSelection::SwitchTarget,
      rc26_decision::PreselectionBranchMode::SwitchOnly));
  EXPECT_FALSE(rc26_decision::isPreselectionBranchAllowed(
      rc26_decision::PreselectionBranchSelection::ContinueFirst,
      rc26_decision::PreselectionBranchMode::SwitchOnly));

  EXPECT_EQ(rc26_decision::selectPreselectionStartProfile(
                rc26_decision::PreselectionBranchSelection::ContinueFirst,
                "mc", "mc"),
            rc26_decision::PreselectionStartProfile::Mc);
  EXPECT_EQ(rc26_decision::selectPreselectionStartProfile(
                rc26_decision::PreselectionBranchSelection::SwitchTarget,
                "mc", "mc"),
            rc26_decision::PreselectionStartProfile::Mc);
  EXPECT_EQ(rc26_decision::selectPreselectionStartProfile(
                rc26_decision::PreselectionBranchSelection::ContinueFirst,
                "second", "second"),
            rc26_decision::PreselectionStartProfile::Second);
  EXPECT_EQ(rc26_decision::selectPreselectionStartProfile(
                rc26_decision::PreselectionBranchSelection::SwitchTarget,
                "second", "second"),
            rc26_decision::PreselectionStartProfile::Second);
  EXPECT_FALSE(rc26_decision::usesSecondPreselectionStart(
      rc26_decision::PreselectionStartProfile::Mc));
  EXPECT_TRUE(rc26_decision::usesSecondPreselectionStart(
      rc26_decision::PreselectionStartProfile::Second));
}

TEST(MCPreselectionRepeatLogic, ComputesSignedForwardDistance) {
  EXPECT_DOUBLE_EQ(rc26_decision::mcPreselectionEffectiveForwardX(0.05, 0.2, 0),
                   0.05);
  EXPECT_DOUBLE_EQ(rc26_decision::mcPreselectionEffectiveForwardX(0.05, 0.2, 1),
                   0.25);
  EXPECT_DOUBLE_EQ(rc26_decision::mcPreselectionEffectiveForwardX(0.05, 0.2, 2),
                   0.45);

  EXPECT_DOUBLE_EQ(rc26_decision::mcPreselectionEffectiveForwardX(1.05, 0.2, 0),
                   1.05);
  EXPECT_DOUBLE_EQ(rc26_decision::mcPreselectionEffectiveForwardX(1.05, 0.2, 1),
                   1.25);
  EXPECT_DOUBLE_EQ(rc26_decision::mcPreselectionEffectiveForwardX(1.05, 0.2, 2),
                   1.45);

  EXPECT_DOUBLE_EQ(rc26_decision::mcPreselectionEffectiveForwardX(-0.05, 0.2, 0),
                   -0.05);
  EXPECT_DOUBLE_EQ(rc26_decision::mcPreselectionEffectiveForwardX(-0.05, 0.2, 1),
                   -0.25);
  EXPECT_DOUBLE_EQ(rc26_decision::mcPreselectionEffectiveForwardX(-0.05, 0.2, 2),
                   -0.45);

  EXPECT_DOUBLE_EQ(rc26_decision::mcPreselectionEffectiveForwardX(0.05, 0.2, -1),
                   0.05);
}

TEST(TreeSwitchRequest, ConsumesRequestAndSuppressesDuplicateActiveTarget) {
  auto blackboard = BT::Blackboard::create();
  std::string tree_file;
  EXPECT_FALSE(
      rc26_decision::consumeBehaviorTreeSwitchRequest(blackboard, tree_file));

  rc26_decision::requestBehaviorTreeSwitch(blackboard,
                                           "mf_preselection_tree.xml");
  EXPECT_TRUE(
      rc26_decision::consumeBehaviorTreeSwitchRequest(blackboard, tree_file));
  EXPECT_EQ(tree_file, "mf_preselection_tree.xml");
  EXPECT_FALSE(
      rc26_decision::consumeBehaviorTreeSwitchRequest(blackboard, tree_file));

  const std::string target = "/tmp/rc26/mf_preselection_tree.xml";
  EXPECT_FALSE(
      rc26_decision::shouldSwitchToRequestedTree(target, target, true));
  EXPECT_TRUE(
      rc26_decision::shouldSwitchToRequestedTree(target, target, false));
}

TEST(ManagedPreselectionTrees, XmlFilesLoadAndLegacyStartTreeIsRemoved) {
  BT::BehaviorTreeFactory factory;
  rc26_decision::registerMCAreaNodes(factory);
  rc26_decision::registerMFAreaNodes(factory);
  rc26_decision::registerMfPreselectionNodes(factory);
  rc26_decision::registerSecondPreselectionNodes(factory);
  rc26_decision::registerOdomNavigationNodes(factory);
  rc26_decision::registerStairNodes(factory);

  const auto tree_dir =
      std::filesystem::path(RC26_DECISION_SOURCE_DIR) / "behavior_trees";
  const std::vector<std::string> expected_trees{
      "mc_repeat_preselection_tree.xml", "mc_mf_preselection_tree.xml", "mc_tree.xml",
      "preselection_ramp_forward_tree.xml", "second_preselection_combo_tree.xml",
      "second_preselection_tree.xml"};
  for (const auto &tree_name : expected_trees) {
    const auto tree_path = tree_dir / tree_name;
    auto blackboard = BT::Blackboard::create();
    EXPECT_NO_THROW({
      auto tree = factory.createTreeFromFile(tree_path.string(), blackboard);
      EXPECT_TRUE(tree.rootNode() != nullptr) << tree_name;
    }) << tree_name;
  }

  const auto combo_tree_path = tree_dir / "second_preselection_combo_tree.xml";
  std::ifstream combo_tree_stream(combo_tree_path);
  ASSERT_TRUE(combo_tree_stream.good());
  const std::string combo_tree_xml =
      std::string(std::istreambuf_iterator<char>(combo_tree_stream),
                  std::istreambuf_iterator<char>());
  const auto entry_gate_pos =
      combo_tree_xml.find("second_preselection_entry_branch_gate");
  const auto ramp_pos =
      combo_tree_xml.find("<SubTree ID=\"PreselectionRampForwardTree\"");
  const auto search_tree_pos =
      combo_tree_xml.find("<SubTree ID=\"SecondPreselectionTree\"");
  ASSERT_NE(entry_gate_pos, std::string::npos);
  ASSERT_NE(ramp_pos, std::string::npos);
  ASSERT_NE(search_tree_pos, std::string::npos);
  EXPECT_LT(entry_gate_pos, ramp_pos);
  EXPECT_LT(ramp_pos, search_tree_pos);
  EXPECT_NE(combo_tree_xml.find("continue_start_profile=\"second\""),
            std::string::npos);
  EXPECT_NE(combo_tree_xml.find("switch_start_profile=\"second\""),
            std::string::npos);
  EXPECT_NE(combo_tree_xml.find("switch_tree_file=\"second_preselection_tree.xml\""),
            std::string::npos);
  EXPECT_EQ(combo_tree_xml.find("accepted_branch=\"switch_only\""),
            std::string::npos);
  EXPECT_EQ(combo_tree_xml.find("second_preselect_after_ramp_turn"),
            std::string::npos);

  EXPECT_FALSE(std::filesystem::exists(tree_dir / "mf_preselection_start_tree.xml"));

  const auto repeat_tree_path = tree_dir / "mc_repeat_preselection_tree.xml";
  std::ifstream repeat_tree_stream(repeat_tree_path);
  ASSERT_TRUE(repeat_tree_stream.good());
  const std::string repeat_tree_xml =
      std::string(
      std::istreambuf_iterator<char>(repeat_tree_stream),
      std::istreambuf_iterator<char>());
  EXPECT_NE(repeat_tree_xml.find("MCPreselectionRepeatControl"), std::string::npos);
  EXPECT_EQ(repeat_tree_xml.find("base_forward_x_m"), std::string::npos);
  EXPECT_EQ(repeat_tree_xml.find("MFPreselectionAfterMCTree"), std::string::npos);
}
