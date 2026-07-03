// 预选入口分支 gate：等待 0x06/0x10，并按分支完成 MCU 握手。
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include "mc_params.hpp"
#include "rc26_decision/second_preselection/second_preselection.hpp"
#include "rc26_interfaces/msg/mechanism_transport_feedback.hpp"
#include "rc26_interfaces/srv/send_mechanism_transport_command.hpp"

namespace rc26_decision {

class PreselectionBranchGateAction : public BT::StatefulActionNode {
public:
  PreselectionBranchGateAction(const std::string &name,
                               const BT::NodeConfig &config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using FeedbackMsg = rc26_interfaces::msg::MechanismTransportFeedback;
  using SendCommandSrv = rc26_interfaces::srv::SendMechanismTransportCommand;

  enum class Branch { None, ContinueFirst, SwitchSecond };
  enum class Phase {
    WaitingBranch,
    SendingCommand,
    WaitingAck,
    WaitingDone,
    SwitchRequested
  };

  void handleFeedback(const FeedbackMsg::SharedPtr msg);
  void configureBranch(Branch branch);
  bool sendBranchCommand();
  BT::NodeStatus fail(const std::string &reason);
  void resetRuntimeHandles();
  static std::string byteHex(int value);

  McParams mc_params_;
  SecondPreselectionParams second_params_;
  rclcpp::Node *node_{nullptr};
  rclcpp::Subscription<FeedbackMsg>::SharedPtr feedback_sub_;
  rclcpp::Client<SendCommandSrv>::SharedPtr send_client_;
  std::chrono::steady_clock::time_point start_tp_{};
  std::chrono::steady_clock::time_point phase_tp_{};
  std::chrono::steady_clock::time_point last_log_tp_{};
  std::atomic<bool> branch_seen_{false};
  std::atomic<bool> command_response_seen_{false};
  std::atomic<bool> command_accepted_{false};
  std::atomic<bool> done_feedback_seen_{false};
  std::atomic<int> command_seq_{-1};
  std::atomic<uint64_t> generation_{0};
  Branch branch_{Branch::None};
  Phase phase_{Phase::WaitingBranch};
  std::string switch_tree_file_{"mf_preselection_tree.xml"};
  std::string command_service_;
  int branch_command_id_{0};
  int branch_done_feedback_id_{0};
  double branch_signal_timeout_s_{0.0};
  double branch_command_timeout_s_{5.0};
  double branch_done_timeout_s_{5.0};
  double branch_log_period_s_{1.0};
};

} // namespace rc26_decision
