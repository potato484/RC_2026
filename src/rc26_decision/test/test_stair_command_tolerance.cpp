#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "rc26_decision/decision_failure.hpp"
#include "rc26_decision/stair/stair_action_base.hpp"
#include "rc26_interfaces/srv/send_mechanism_transport_command.hpp"

namespace {

class TestStairMechanismCommandAction
    : public rc26_decision::StairActionBase {
public:
  TestStairMechanismCommandAction(const std::string &name,
                                  const BT::NodeConfig &config)
      : StairActionBase(name, config) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<bool>("pair", false,
                                "Test a single command or a command pair")};
  }

  BT::NodeStatus onStart() override {
    if (!setupRuntime("TestStairMechanismCommand")) {
      return BT::NodeStatus::FAILURE;
    }
    (void)getInput("pair", pair_);
    if (pair_) {
      beginCommandPair(rc26_serial::CommandID::FRONT_PUSHROD_RETRACT,
                       "FRONT_PUSHROD_RETRACT",
                       rc26_serial::CommandID::REAR_PUSHROD_EXTEND,
                       "REAR_PUSHROD_EXTEND");
    } else {
      beginCommand(rc26_serial::CommandID::FRONT_PUSHROD_EXTEND,
                   "FRONT_PUSHROD_EXTEND");
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override {
    const auto status = pair_ ? tickCommandPair() : tickCommand();
    if (status == StepStatus::Running) {
      return BT::NodeStatus::RUNNING;
    }
    if (status == StepStatus::Failure) {
      return failWithStop("测试机构命令运行态损坏");
    }
    releaseRuntime();
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override { releaseRuntime(); }

private:
  bool pair_{false};
};

enum class ResponseMode { RejectAll, MixedPair };

struct StairCommandRunResult {
  BT::NodeStatus status{BT::NodeStatus::IDLE};
  std::vector<uint8_t> commands;
  std::string failure_detail;
};

StairCommandRunResult runStairCommand(bool pair, bool provide_service,
                                      ResponseMode response_mode) {
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static std::atomic<int> run_id{0};
  const int id = run_id.fetch_add(1);
  const std::string suffix = std::to_string(id);
  const std::string service_name =
      "/test/stair_command_" + suffix + "/send_command";

  auto decision_node =
      std::make_shared<rclcpp::Node>("stair_command_decision_" + suffix);
  auto fake_transport =
      std::make_shared<rclcpp::Node>("stair_command_transport_" + suffix);

  std::mutex commands_mutex;
  std::vector<uint8_t> commands;
  std::atomic<int> next_seq{20};
  using SendCommandSrv =
      rc26_interfaces::srv::SendMechanismTransportCommand;
  rclcpp::Service<SendCommandSrv>::SharedPtr service;
  if (provide_service) {
    service = fake_transport->create_service<SendCommandSrv>(
        service_name,
        [&](const std::shared_ptr<SendCommandSrv::Request> request,
            std::shared_ptr<SendCommandSrv::Response> response) {
          {
            std::lock_guard<std::mutex> lock(commands_mutex);
            commands.push_back(request->command_id);
          }
          const bool is_second_pair_command =
              request->command_id == static_cast<uint8_t>(
                                         rc26_serial::CommandID::REAR_PUSHROD_EXTEND);
          response->accepted =
              response_mode == ResponseMode::MixedPair &&
              !is_second_pair_command;
          response->seq = static_cast<uint8_t>(next_seq.fetch_add(1));
        });
  }

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<TestStairMechanismCommandAction>(
      "TestStairMechanismCommand");
  auto blackboard = BT::Blackboard::create();
  rc26_decision::clearDecisionFailure(blackboard);
  rclcpp::Node *decision_node_ptr = decision_node.get();
  blackboard->set("node", decision_node_ptr);
  rc26_decision::StairParams params;
  params.send_command_service = service_name;
  params.feedback_topic =
      "/test/stair_command_" + suffix + "/command_feedback";
  params.cmd_vel_topic = "/test/stair_command_" + suffix + "/cmd_vel";
  params.odom_topic = "/test/stair_command_" + suffix + "/odom";
  params.heading_hold_enable = false;
  params.command_timeout_s = provide_service ? 0.2 : 0.05;
  blackboard->set("stair_params", params);

  const std::string tree_xml =
      std::string(R"(<root BTCPP_format="4" main_tree_to_execute="TestTree">
           <BehaviorTree ID="TestTree">
             <TestStairMechanismCommand pair=")") +
      (pair ? "true" : "false") + R"("/>
           </BehaviorTree>
         </root>)";
  auto tree = factory.createTreeFromText(tree_xml, blackboard);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(decision_node);
  executor.add_node(fake_transport);
  std::thread spin_thread([&executor]() { executor.spin(); });

  StairCommandRunResult result;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline &&
         result.status != BT::NodeStatus::SUCCESS &&
         result.status != BT::NodeStatus::FAILURE) {
    result.status = tree.tickOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

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

TEST(StairCommandTolerance, RejectedSingleCommandContinues) {
  const auto result =
      runStairCommand(false, true, ResponseMode::RejectAll);
  EXPECT_EQ(result.status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(result.commands.size(), 1U);
  EXPECT_EQ(result.commands.front(), static_cast<uint8_t>(
                                         rc26_serial::CommandID::FRONT_PUSHROD_EXTEND));
  EXPECT_TRUE(result.failure_detail.empty());
}

TEST(StairCommandTolerance, MixedCommandPairContinuesAfterBothTerminal) {
  const auto result =
      runStairCommand(true, true, ResponseMode::MixedPair);
  EXPECT_EQ(result.status, BT::NodeStatus::SUCCESS);
  ASSERT_EQ(result.commands.size(), 2U);
  EXPECT_TRUE(result.failure_detail.empty());
}

TEST(StairCommandTolerance, ServiceUnavailableTimeoutContinues) {
  const auto result =
      runStairCommand(false, false, ResponseMode::RejectAll);
  EXPECT_EQ(result.status, BT::NodeStatus::SUCCESS);
  EXPECT_TRUE(result.commands.empty());
  EXPECT_TRUE(result.failure_detail.empty());
}
