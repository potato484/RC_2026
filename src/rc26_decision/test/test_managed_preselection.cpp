#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "rc26_decision/mc/mc_area.hpp"
#include "rc26_decision/mc_preselection_repeat_logic.hpp"
#include "rc26_decision/mf/mf_area.hpp"
#include "rc26_decision/mf_preselection/mf_preselection_flow.hpp"
#include "rc26_decision/navigation/bt_odom_relative_nav.hpp"
#include "rc26_decision/preselection_branch_gate_logic.hpp"
#include "rc26_decision/second_preselection/second_preselection.hpp"
#include "rc26_decision/stair/stair_area.hpp"
#include "rc26_decision/decision_failure.hpp"
#include "rc26_decision/tree_switch_request.hpp"
#include "rc26_interfaces/msg/mechanism_transport_feedback.hpp"
#include "rc26_interfaces/srv/send_mechanism_transport_command.hpp"
#include "rc26_serial/protocol.hpp"

#include "../src/mc/mc_params.hpp"

namespace {

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

struct ManagedGateRunResult {
  BT::NodeStatus status{BT::NodeStatus::IDLE};
  std::vector<uint8_t> commands;
  bool switch_requested{false};
  std::string switch_tree_file;
  bool second_start_done{false};
  std::string failure_detail;
};

ManagedGateRunResult runManagedSecondGate(uint8_t branch_feedback_id,
                                          bool command_accepted,
                                          bool publish_done_feedback) {
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static std::atomic<int> run_id{0};
  const int id = run_id.fetch_add(1);
  const std::string suffix = std::to_string(id);
  const std::string service_name =
      "/test/managed_gate_" + suffix + "/send_command";
  const std::string feedback_topic =
      "/test/managed_gate_" + suffix + "/command_feedback";

  auto decision_node = std::make_shared<rclcpp::Node>(
      "managed_gate_decision_" + suffix);
  auto fake_transport = std::make_shared<rclcpp::Node>(
      "managed_gate_transport_" + suffix);

  std::mutex commands_mutex;
  std::vector<uint8_t> commands;
  using SendCommandSrv =
      rc26_interfaces::srv::SendMechanismTransportCommand;
  auto service = fake_transport->create_service<SendCommandSrv>(
      service_name,
      [&](const std::shared_ptr<SendCommandSrv::Request> request,
          std::shared_ptr<SendCommandSrv::Response> response) {
        {
          std::lock_guard<std::mutex> lock(commands_mutex);
          commands.push_back(request->command_id);
        }
        response->accepted = command_accepted;
        response->seq = 52;
      });
  (void)service;
  auto feedback_pub = fake_transport->create_publisher<
      rc26_interfaces::msg::MechanismTransportFeedback>(feedback_topic,
                                                        rclcpp::QoS(32).reliable());

  BT::BehaviorTreeFactory factory;
  rc26_decision::registerMCAreaNodes(factory);
  auto blackboard = BT::Blackboard::create();
  rc26_decision::clearDecisionFailure(blackboard);
  rc26_decision::clearBehaviorTreeSwitchRequest(blackboard);
  rclcpp::Node *decision_node_ptr = decision_node.get();
  blackboard->set("node", decision_node_ptr);

  rc26_decision::McParams mc_params;
  mc_params.start_signal_feedback_topic = feedback_topic;
  mc_params.start_signal_timeout_s = 0.0;
  mc_params.registration_gate_enable = false;
  blackboard->set("mc_params", mc_params);

  rc26_decision::SecondPreselectionParams second_params;
  second_params.send_command_service = service_name;
  second_params.command_timeout_s = 0.05;
  second_params.done_timeout_s = 0.05;
  second_params.log_period_s = 0.01;
  blackboard->set("second_preselection_params", second_params);

  const bool switch_branch = branch_feedback_id == 0x10;
  const std::string accepted_branch =
      switch_branch ? "switch_only" : "continue_only";
  const std::string switch_target =
      switch_branch ? "second_preselection_climb_place_tree.xml" : "";
  const std::string tree_xml =
      std::string(R"(<root BTCPP_format="4" main_tree_to_execute="TestTree">
           <BehaviorTree ID="TestTree">
             <WaitPreselectionBranchGate name="gate" accepted_branch=")") +
      accepted_branch + "\" continue_start_profile=\"second\" "
                        "switch_start_profile=\"second\" switch_tree_file=\"" +
      switch_target + R"("/>
           </BehaviorTree>
         </root>)";
  auto tree = factory.createTreeFromText(tree_xml, blackboard);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(decision_node);
  executor.add_node(fake_transport);
  std::thread spin_thread([&executor]() { executor.spin(); });

  ManagedGateRunResult result;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    result.status = tree.tickOnce();

    rc26_interfaces::msg::MechanismTransportFeedback branch_feedback;
    branch_feedback.feedback_id = branch_feedback_id;
    feedback_pub->publish(branch_feedback);

    if (publish_done_feedback && command_accepted) {
      rc26_interfaces::msg::MechanismTransportFeedback done_feedback;
      done_feedback.seq = 52;
      done_feedback.feedback_id = 0x0D;
      feedback_pub->publish(done_feedback);
    }

    std::string requested_tree;
    if (rc26_decision::consumeBehaviorTreeSwitchRequest(blackboard,
                                                        requested_tree)) {
      result.switch_requested = true;
      result.switch_tree_file = requested_tree;
      break;
    }
    if (result.status == BT::NodeStatus::SUCCESS ||
        result.status == BT::NodeStatus::FAILURE) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  (void)blackboard->get(rc26_decision::kPreselectionGateSecondStartDoneKey,
                        result.second_start_done);
  (void)blackboard->get(rc26_decision::kDecisionFailureDetailKey,
                        result.failure_detail);
  {
    std::lock_guard<std::mutex> lock(commands_mutex);
    result.commands = commands;
  }

  tree.haltTree();
  executor.cancel();
  spin_thread.join();
  executor.remove_node(decision_node);
  executor.remove_node(fake_transport);
  return result;
}

} // namespace

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

  EXPECT_EQ(rc26_decision::makePreselectionGateStateJson(
                true, "preselection_mc_start_gate", "continue_only", "mc", "second"),
            "{\"waiting\":true,\"gate\":\"preselection_mc_start_gate\","
            "\"accepted_branch\":\"continue_only\","
            "\"continue_start_profile\":\"mc\","
            "\"switch_start_profile\":\"second\"}");
  EXPECT_NE(rc26_decision::makePreselectionGateStateJson(
                false, "preselection_mc_start_gate", "continue_only", "mc", "second")
                .find("\"waiting\":false"),
            std::string::npos);
}

TEST(PreselectionBranchGateRuntime,
     RejectedSecondContinueHandshakeWarnsAndContinues) {
  const auto result = runManagedSecondGate(0x06, false, false);
  EXPECT_EQ(result.status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(result.commands.size(), 1U);
  EXPECT_EQ(result.commands.front(), 0x11);
  EXPECT_TRUE(result.second_start_done);
  EXPECT_FALSE(result.switch_requested);
  EXPECT_TRUE(result.failure_detail.empty());
}

TEST(PreselectionBranchGateRuntime,
     SecondSwitchDoneTimeoutStillRequestsTargetTree) {
  const auto result = runManagedSecondGate(0x10, true, false);
  EXPECT_EQ(result.status, BT::NodeStatus::RUNNING);
  ASSERT_EQ(result.commands.size(), 1U);
  EXPECT_EQ(result.commands.front(), 0x11);
  EXPECT_TRUE(result.second_start_done);
  EXPECT_TRUE(result.switch_requested);
  EXPECT_EQ(result.switch_tree_file,
            "second_preselection_climb_place_tree.xml");
  EXPECT_TRUE(result.failure_detail.empty());
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

  EXPECT_DOUBLE_EQ(rc26_decision::mcPreselectionEffectiveForwardX(1.05, -0.2, 0),
                   1.05);
  EXPECT_DOUBLE_EQ(rc26_decision::mcPreselectionEffectiveForwardX(1.05, -0.2, 1),
                   0.85);
  EXPECT_DOUBLE_EQ(rc26_decision::mcPreselectionEffectiveForwardX(1.05, -0.2, 2),
                   0.65);

  EXPECT_DOUBLE_EQ(rc26_decision::mcPreselectionEffectiveForwardX(-0.05, 0.2, 0),
                   -0.05);
  EXPECT_DOUBLE_EQ(rc26_decision::mcPreselectionEffectiveForwardX(-0.05, 0.2, 1),
                   0.15);
  EXPECT_DOUBLE_EQ(rc26_decision::mcPreselectionEffectiveForwardX(-0.05, 0.2, 2),
                   0.35);

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
      "second_preselection_climb_place_tree.xml", "second_preselection_tree.xml",
      "second_preselection_post_place_climb_tree.xml"};
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
  const auto after_ramp_gate_pos =
      combo_tree_xml.find("second_preselection_after_ramp_gate");
  ASSERT_NE(entry_gate_pos, std::string::npos);
  ASSERT_NE(ramp_pos, std::string::npos);
  ASSERT_NE(after_ramp_gate_pos, std::string::npos);
  EXPECT_LT(entry_gate_pos, ramp_pos);
  EXPECT_LT(ramp_pos, after_ramp_gate_pos);

  const std::string entry_gate_block = xmlNodeBlockByName(
      combo_tree_xml, "second_preselection_entry_branch_gate");
  const std::string after_ramp_gate_block = xmlNodeBlockByName(
      combo_tree_xml, "second_preselection_after_ramp_gate");
  ASSERT_FALSE(entry_gate_block.empty());
  ASSERT_FALSE(after_ramp_gate_block.empty());
  EXPECT_NE(entry_gate_block.find("continue_start_profile=\"second\""),
            std::string::npos);
  EXPECT_NE(entry_gate_block.find("switch_start_profile=\"second\""),
            std::string::npos);
  EXPECT_EQ(entry_gate_block.find("accepted_branch="), std::string::npos);
  EXPECT_NE(entry_gate_block.find(
                "switch_tree_file=\"second_preselection_climb_place_tree.xml\""),
            std::string::npos);
  EXPECT_NE(after_ramp_gate_block.find("accepted_branch=\"switch_only\""),
            std::string::npos);
  EXPECT_NE(after_ramp_gate_block.find("switch_start_profile=\"second\""),
            std::string::npos);
  EXPECT_NE(after_ramp_gate_block.find(
                "switch_tree_file=\"second_preselection_climb_place_tree.xml\""),
            std::string::npos);
  EXPECT_EQ(countOccurrences(
                combo_tree_xml,
                "switch_tree_file=\"second_preselection_climb_place_tree.xml\""),
            2U);
  EXPECT_EQ(combo_tree_xml.find(
                "switch_tree_file=\"second_preselection_tree.xml\""),
            std::string::npos);
  EXPECT_EQ(combo_tree_xml.find(
                "<include path=\"second_preselection_tree.xml\""),
            std::string::npos);
  EXPECT_EQ(combo_tree_xml.find("<SubTree ID=\"SecondPreselectionTree\""),
            std::string::npos);
  EXPECT_EQ(combo_tree_xml.find("SecondPreselectionTree"), std::string::npos);
  EXPECT_EQ(combo_tree_xml.find("second_preselect_after_ramp_turn"),
            std::string::npos);

  const auto ramp_tree_path = tree_dir / "preselection_ramp_forward_tree.xml";
  std::ifstream ramp_tree_stream(ramp_tree_path);
  ASSERT_TRUE(ramp_tree_stream.good());
  const std::string ramp_tree_xml =
      std::string(std::istreambuf_iterator<char>(ramp_tree_stream),
                  std::istreambuf_iterator<char>());
  EXPECT_NE(ramp_tree_xml.find("SecondPreselectionRampForward"),
            std::string::npos);
  EXPECT_EQ(ramp_tree_xml.find("OdomDriveX"), std::string::npos);
  EXPECT_EQ(ramp_tree_xml.find("preselection_ramp_approach_x_m"),
            std::string::npos);
  EXPECT_EQ(ramp_tree_xml.find("preselection_ramp_climb_x_m"),
            std::string::npos);
  EXPECT_EQ(ramp_tree_xml.find("preselection_ramp_timeout_s"),
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
