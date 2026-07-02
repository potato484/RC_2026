#include <gtest/gtest.h>

#include <filesystem>

#include <behaviortree_cpp/bt_factory.h>

#include "rc26_decision/mc/mc_area.hpp"
#include "rc26_decision/mf/mf_area.hpp"
#include "rc26_decision/mf_preselection/mf_preselection_flow.hpp"
#include "rc26_decision/mf_preselection_trigger.hpp"
#include "rc26_decision/navigation/bt_odom_relative_nav.hpp"
#include "rc26_decision/stair/stair_area.hpp"
#include "rc26_serial/protocol.hpp"

namespace {

rc26_decision::MfPreselectionExternalTriggerConfig defaultConfig() {
  rc26_decision::MfPreselectionExternalTriggerConfig config;
  config.enable = true;
  config.feedback_id =
      static_cast<int>(rc26_serial::FeedbackID::MF_PRESELECTION_TRIGGER);
  config.tree_file = "mf_preselection_tree.xml";
  return config;
}

} // namespace

TEST(MfPreselectionTrigger, MatchesConfiguredUpstreamFeedback) {
  const auto config = defaultConfig();
  EXPECT_TRUE(rc26_decision::isMfPreselectionExternalTriggerFeedback(
      static_cast<uint8_t>(rc26_serial::FeedbackID::MF_PRESELECTION_TRIGGER),
      config));
}

TEST(MfPreselectionTrigger, IgnoresOtherFeedbackOrDisabledConfig) {
  auto config = defaultConfig();
  EXPECT_FALSE(rc26_decision::isMfPreselectionExternalTriggerFeedback(
      static_cast<uint8_t>(rc26_serial::FeedbackID::COMPETITION_START_DONE),
      config));

  config.enable = false;
  EXPECT_FALSE(rc26_decision::isMfPreselectionExternalTriggerFeedback(
      static_cast<uint8_t>(rc26_serial::FeedbackID::MF_PRESELECTION_TRIGGER),
      config));
}

TEST(MfPreselectionTrigger, ReloadPolicyHandlesRunningPendingAndTerminalStates) {
  const std::string target = "/tmp/rc26/mf_preselection_tree.xml";
  const std::string current = "/tmp/rc26/mc_mf_preselection_tree.xml";

  EXPECT_TRUE(rc26_decision::shouldReloadForMfPreselectionTrigger(
      current, target, false, true));
  EXPECT_FALSE(rc26_decision::shouldReloadForMfPreselectionTrigger(
      target, target, false, true));
  EXPECT_TRUE(rc26_decision::shouldReloadForMfPreselectionTrigger(
      target, target, false, false));
  EXPECT_FALSE(rc26_decision::shouldReloadForMfPreselectionTrigger(
      current, target, true, true));
}

TEST(MfPreselectionTrigger, MfPreselectionTreeXmlLoadsWithRegisteredNodes) {
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
