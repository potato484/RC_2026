#include "preselection_branch_gate.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <exception>

#include "rc26_decision/decision_failure.hpp"
#include "rc26_decision/mechanism_error_diagnostic.hpp"
#include "rc26_decision/preselection_branch_gate_logic.hpp"
#include "rc26_decision/tree_switch_request.hpp"
#include "rc26_serial/protocol.hpp"

namespace rc26_decision {

namespace {

double elapsedSec(const std::chrono::steady_clock::time_point &since) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                       since)
      .count();
}

} // namespace

BT::PortsList PreselectionBranchGateAction::providedPorts() {
  return {
      BT::InputPort<std::string>(
          "switch_tree_file", "mf_preselection_tree.xml",
          "Target tree to load when feedback 0x10 selects the switch branch"),
      BT::InputPort<std::string>(
          "continue_start_profile", "mc",
          "Start profile used by the 0x06 branch: mc or second"),
      BT::InputPort<std::string>(
          "switch_start_profile", "mc",
          "Start profile used by the 0x10 branch: mc or second"),
      BT::InputPort<std::string>(
          "accepted_branch", "both",
          "Accepted branch: both, continue_only, or switch_only"),
      BT::InputPort<unsigned int>(
          "continue_pre_command_delay_msec", 0,
          "Delay after feedback 0x06 before sending the start command"),
      BT::InputPort<unsigned int>(
          "switch_pre_command_delay_msec", 0,
          "Delay after feedback 0x10 before sending the start command")};
}

PreselectionBranchGateAction::PreselectionBranchGateAction(
    const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

BT::NodeStatus PreselectionBranchGateAction::onStart() {
  if (!config().blackboard || !config().blackboard->get("node", node_) ||
      node_ == nullptr) {
    writeDecisionFailure(config().blackboard, "WaitPreselectionBranchGate",
                         "运行上下文缺失：blackboard 或 node 不可用");
    return BT::NodeStatus::FAILURE;
  }
  if (!config().blackboard->get("mc_params", mc_params_)) {
    writeDecisionFailure(config().blackboard, "WaitPreselectionBranchGate",
                         "黑板缺少 mc_params");
    return BT::NodeStatus::FAILURE;
  }
  if (!config().blackboard->get("second_preselection_params",
                                second_params_)) {
    writeDecisionFailure(config().blackboard, "WaitPreselectionBranchGate",
                         "黑板缺少 second_preselection_params");
    return BT::NodeStatus::FAILURE;
  }

  (void)getInput("switch_tree_file", switch_tree_file_);
  (void)getInput("continue_start_profile", continue_start_profile_text_);
  (void)getInput("switch_start_profile", switch_start_profile_text_);
  (void)getInput("accepted_branch", accepted_branch_text_);
  accepted_branch_ = parsePreselectionBranchMode(accepted_branch_text_);
  (void)getInput("continue_pre_command_delay_msec",
                 continue_pre_command_delay_msec_);
  (void)getInput("switch_pre_command_delay_msec",
                 switch_pre_command_delay_msec_);

  branch_seen_ = false;
  command_response_seen_ = false;
  command_accepted_ = false;
  done_feedback_seen_ = false;
  command_error_seen_ = false;
  command_busy_seen_ = false;
  command_seq_ = -1;
  command_error_detail_.clear();
  generation_.fetch_add(1, std::memory_order_relaxed);
  branch_ = Branch::None;
  branch_start_profile_ = StartProfile::Mc;
  branch_pre_command_delay_s_ = 0.0;
  phase_ = Phase::WaitingBranch;
  start_tp_ = std::chrono::steady_clock::now();
  phase_tp_ = start_tp_;
  last_log_tp_ = start_tp_;
  config().blackboard->set(kPreselectionGateSecondStartDoneKey, false);

  const std::string feedback_topic =
      mc_params_.start_signal_feedback_topic.empty()
          ? std::string("/mechanism/command_feedback")
          : mc_params_.start_signal_feedback_topic;
  feedback_sub_ = node_->create_subscription<FeedbackMsg>(
      feedback_topic, rclcpp::QoS(32).reliable(),
      [this](const FeedbackMsg::SharedPtr msg) { handleFeedback(msg); });
  gate_state_pub_ = node_->create_publisher<GateStateMsg>(
      "/decision/preselection_gate_state", rclcpp::QoS(1).reliable().transient_local());
  publishGateState(true);

  RCLCPP_INFO(
      node_->get_logger(),
      "预选入口 branch gate: 等待分支 accepted=%s 0x%02X继续 0x%02X切换/放下 topic=%s switch_tree=%s continue_profile=%s switch_profile=%s continue_pre_cmd_delay=%ums switch_pre_cmd_delay=%ums",
      branchModeName(accepted_branch_),
      static_cast<unsigned int>(mc_params_.start_signal_feedback_id & 0xFF),
      static_cast<unsigned int>(
          static_cast<int>(rc26_serial::FeedbackID::MANUAL_LIMIT_SWITCH_2_TRIGGERED) &
          0xFF),
      feedback_topic.c_str(),
      switch_tree_file_.empty() ? "<none>" : switch_tree_file_.c_str(),
      continue_start_profile_text_.c_str(), switch_start_profile_text_.c_str(),
      continue_pre_command_delay_msec_, switch_pre_command_delay_msec_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PreselectionBranchGateAction::onRunning() {
  const auto now = std::chrono::steady_clock::now();

  if (phase_ == Phase::WaitingBranch) {
    if (branch_seen_.load(std::memory_order_relaxed) &&
        branch_ != Branch::None) {
      configureBranch(branch_);
      phase_ = branch_pre_command_delay_s_ > 0.0
                   ? Phase::WaitingPreCommandDelay
                   : Phase::SendingCommand;
      phase_tp_ = now;
      last_log_tp_ = now;
      if (phase_ == Phase::WaitingPreCommandDelay) {
        RCLCPP_INFO(
            node_->get_logger(),
            "预选入口 branch gate: 收到分支后先等待 %.3fs，再发送 command=%s",
            branch_pre_command_delay_s_, byteHex(branch_command_id_).c_str());
        return BT::NodeStatus::RUNNING;
      }
    } else {
      if (branch_signal_timeout_s_ > 0.0 &&
          elapsedSec(start_tp_) > branch_signal_timeout_s_) {
        return fail("等待预选入口分支反馈超时");
      }
      if (elapsedSec(last_log_tp_) >= mc_params_.start_log_period_s) {
        RCLCPP_INFO(
            node_->get_logger(),
            "预选入口 branch gate: 等待 accepted=%s 0x%02X/0x%02X elapsed=%.1fs",
            branchModeName(accepted_branch_),
            static_cast<unsigned int>(mc_params_.start_signal_feedback_id &
                                      0xFF),
            static_cast<unsigned int>(
                static_cast<int>(
                    rc26_serial::FeedbackID::MANUAL_LIMIT_SWITCH_2_TRIGGERED) &
                0xFF),
            elapsedSec(start_tp_));
        last_log_tp_ = now;
      }
      return BT::NodeStatus::RUNNING;
    }
  }

  if (phase_ == Phase::WaitingPreCommandDelay) {
    const double elapsed = elapsedSec(phase_tp_);
    if (elapsed >= branch_pre_command_delay_s_) {
      phase_ = Phase::SendingCommand;
      phase_tp_ = now;
    } else {
      if (elapsedSec(last_log_tp_) >= branch_log_period_s_) {
        RCLCPP_INFO(
            node_->get_logger(),
            "预选入口 branch gate: 发 command=%s 前延时中 elapsed=%.1fs/%.1fs",
            byteHex(branch_command_id_).c_str(), elapsed,
            branch_pre_command_delay_s_);
        last_log_tp_ = now;
      }
      return BT::NodeStatus::RUNNING;
    }
  }

  if (phase_ == Phase::SendingCommand) {
    if (!sendBranchCommand()) {
      if (elapsedSec(phase_tp_) > branch_command_timeout_s_) {
        return completeBranchHandshake(
            "等待预选入口分支命令服务可用超时", true);
      }
      return BT::NodeStatus::RUNNING;
    }
    phase_ = Phase::WaitingAck;
    phase_tp_ = now;
    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == Phase::WaitingAck) {
    if (command_response_seen_.load(std::memory_order_relaxed)) {
      const int seq = command_seq_.load(std::memory_order_relaxed);
      if (!command_accepted_.load(std::memory_order_relaxed)) {
        return completeBranchHandshake(
            "预选入口分支命令未被接受，seq=" + std::to_string(seq), true);
      }
      RCLCPP_INFO(node_->get_logger(),
                  "预选入口 branch gate: command=%s ACK 成功 seq=%d，等待 done=%s",
                  byteHex(branch_command_id_).c_str(), seq,
                  byteHex(branch_done_feedback_id_).c_str());
      phase_ = Phase::WaitingDone;
      phase_tp_ = now;
      last_log_tp_ = now;
      return BT::NodeStatus::RUNNING;
    }
    if (elapsedSec(phase_tp_) > branch_command_timeout_s_) {
      return completeBranchHandshake("等待预选入口分支命令 ACK 超时",
                                     true);
    }
  }

  if (phase_ == Phase::WaitingDone) {
    const int seq = command_seq_.load(std::memory_order_relaxed);
    if (command_error_seen_.load(std::memory_order_relaxed)) {
      return completeBranchHandshake(
          command_error_detail_.empty()
              ? "预选入口分支命令收到 MCU 0xFE 最终错误，seq=" +
                    std::to_string(seq)
              : command_error_detail_,
          true);
    }
    if (done_feedback_seen_.load(std::memory_order_relaxed)) {
      RCLCPP_INFO(node_->get_logger(),
                  "预选入口 branch gate: 已收到 done=%s seq=%d",
                  byteHex(branch_done_feedback_id_).c_str(), seq);
      return completeBranchHandshake("", false);
    }
    if (elapsedSec(phase_tp_) > branch_done_timeout_s_) {
      return completeBranchHandshake(
          "等待预选入口分支 done 超时，seq=" + std::to_string(seq), true);
    }
    if (elapsedSec(last_log_tp_) >= branch_log_period_s_) {
      const bool busy_seen = command_busy_seen_.load(std::memory_order_relaxed);
      RCLCPP_INFO(node_->get_logger(),
                  "预选入口 branch gate: 等待 done=%s seq=%d elapsed=%.1fs busy=%s",
                  byteHex(branch_done_feedback_id_).c_str(), seq,
                  elapsedSec(phase_tp_), busy_seen ? "是" : "否");
      last_log_tp_ = now;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void PreselectionBranchGateAction::onHalted() {
  generation_.fetch_add(1, std::memory_order_relaxed);
  resetRuntimeHandles();
}

void PreselectionBranchGateAction::handleFeedback(
    const FeedbackMsg::SharedPtr msg) {
  if (!msg) {
    return;
  }

  const int seq = command_seq_.load(std::memory_order_relaxed);
  std::optional<MechanismErrorDiagnostic> diagnostic;
  if (isSameSeqMechanismError(*msg, seq, diagnostic)) {
    const std::string detail = mechanismErrorDiagnosticText(*diagnostic);
    if (diagnostic->busy) {
      command_busy_seen_.store(true, std::memory_order_relaxed);
      if (node_) {
        RCLCPP_INFO(node_->get_logger(),
                    "预选入口 branch gate: 分支命令仍在处理中：%s",
                    detail.c_str());
      }
    } else {
      command_error_detail_ = detail;
      command_error_seen_.store(true, std::memory_order_relaxed);
      if (node_) {
        RCLCPP_WARN(node_->get_logger(),
                    "预选入口 branch gate: 分支命令收到 MCU 最终错误，按机构容错处理：%s",
                    detail.c_str());
      }
    }
    return;
  }
  if (isSameSeqDoneFeedback(msg->seq, msg->feedback_id, seq,
                            branch_done_feedback_id_)) {
    done_feedback_seen_.store(true, std::memory_order_relaxed);
    return;
  }

  if (phase_ != Phase::WaitingBranch) {
    return;
  }

  const auto branch = selectPreselectionBranch(
      msg->feedback_id, mc_params_.start_signal_feedback_id,
      static_cast<int>(rc26_serial::FeedbackID::MANUAL_LIMIT_SWITCH_2_TRIGGERED));
  if (!isPreselectionBranchAllowed(branch, accepted_branch_)) {
    if (branch != PreselectionBranchSelection::None && node_) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "预选入口 branch gate: 收到当前 gate 不接受的分支 feedback=0x%02X accepted=%s，忽略",
          static_cast<unsigned int>(msg->feedback_id),
          branchModeName(accepted_branch_));
    }
    return;
  }
  if (branch == PreselectionBranchSelection::ContinueFirst) {
    branch_ = Branch::ContinueFirst;
    branch_seen_.store(true, std::memory_order_relaxed);
    return;
  }

  if (branch == PreselectionBranchSelection::SwitchTarget) {
    branch_ = Branch::SwitchTarget;
    branch_seen_.store(true, std::memory_order_relaxed);
  }
}

void PreselectionBranchGateAction::configureBranch(Branch branch) {
  if (branch == Branch::SwitchTarget) {
    branch_start_profile_ = toStartProfile(switch_start_profile_text_);
    configureStartProfile(branch_start_profile_);
    branch_pre_command_delay_s_ =
        static_cast<double>(switch_pre_command_delay_msec_) / 1000.0;
  RCLCPP_WARN(
      node_->get_logger(),
      "预选入口 branch gate: 收到 0x10 profile=%s，延时 %.3fs 后发送 0x%02X 等 0x%02X，成功后%s%s",
      profileName(branch_start_profile_),
      branch_pre_command_delay_s_,
      static_cast<unsigned int>(branch_command_id_ & 0xFF),
      static_cast<unsigned int>(branch_done_feedback_id_ & 0xFF),
      switch_tree_file_.empty() ? "继续当前树" : "切树 ",
      switch_tree_file_.empty() ? "" : switch_tree_file_.c_str());
    return;
  }

  branch_start_profile_ = toStartProfile(continue_start_profile_text_);
  configureStartProfile(branch_start_profile_);
  branch_pre_command_delay_s_ =
      static_cast<double>(continue_pre_command_delay_msec_) / 1000.0;
  RCLCPP_INFO(
      node_->get_logger(),
      "预选入口 branch gate: 收到 0x06 profile=%s，延时 %.3fs 后发送 0x%02X 等 0x%02X 后继续",
      profileName(branch_start_profile_),
      branch_pre_command_delay_s_,
      static_cast<unsigned int>(branch_command_id_ & 0xFF),
      static_cast<unsigned int>(branch_done_feedback_id_ & 0xFF));
}

void PreselectionBranchGateAction::configureStartProfile(StartProfile profile) {
  if (profile == StartProfile::Second) {
    command_service_ = second_params_.send_command_service.empty()
                           ? std::string("/mechanism/send_command")
                           : second_params_.send_command_service;
    branch_command_id_ = second_params_.start_command_id;
    branch_done_feedback_id_ = second_params_.start_done_feedback_id;
    branch_signal_timeout_s_ = 0.0;
    branch_command_timeout_s_ =
        std::max(0.001, second_params_.command_timeout_s);
    branch_done_timeout_s_ = std::max(0.001, second_params_.done_timeout_s);
    branch_log_period_s_ = std::max(0.1, second_params_.log_period_s);
    return;
  }

  command_service_ = mc_params_.start_command_service.empty()
                         ? std::string("/mechanism/send_command")
                         : mc_params_.start_command_service;
  branch_command_id_ = mc_params_.start_command_id;
  branch_done_feedback_id_ = mc_params_.start_done_feedback_id;
  branch_signal_timeout_s_ = std::max(0.0, mc_params_.start_signal_timeout_s);
  branch_command_timeout_s_ = std::max(0.001, mc_params_.start_command_timeout_s);
  branch_done_timeout_s_ = std::max(0.001, mc_params_.start_done_timeout_s);
  branch_log_period_s_ = std::max(0.1, mc_params_.start_log_period_s);
}

void PreselectionBranchGateAction::publishGateState(bool waiting) {
  if (!gate_state_pub_) {
    return;
  }
  GateStateMsg msg;
  msg.data = makePreselectionGateStateJson(
      waiting, name(), branchModeName(accepted_branch_),
      continue_start_profile_text_, switch_start_profile_text_);
  gate_state_pub_->publish(msg);
}

PreselectionBranchGateAction::StartProfile
PreselectionBranchGateAction::toStartProfile(const std::string &profile) {
  return usesSecondPreselectionStart(parsePreselectionStartProfile(profile))
             ? StartProfile::Second
             : StartProfile::Mc;
}

const char *PreselectionBranchGateAction::profileName(StartProfile profile) {
  return profile == StartProfile::Second ? "second" : "mc";
}

const char *PreselectionBranchGateAction::branchModeName(
    PreselectionBranchMode mode) {
  switch (mode) {
  case PreselectionBranchMode::ContinueOnly:
    return "continue_only";
  case PreselectionBranchMode::SwitchOnly:
    return "switch_only";
  case PreselectionBranchMode::Both:
  default:
    return "both";
  }
}

bool PreselectionBranchGateAction::sendBranchCommand() {
  if (!send_client_) {
    send_client_ = node_->create_client<SendCommandSrv>(command_service_);
  }
  if (!send_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "预选入口 branch gate: 等待服务 %s",
                         command_service_.c_str());
    return false;
  }

  auto request = std::make_shared<SendCommandSrv::Request>();
  request->command_id = static_cast<uint8_t>(branch_command_id_ & 0xFF);
  request->payload.clear();
  request->wait_ack = true;

  const uint64_t token = generation_.load(std::memory_order_relaxed);
  command_response_seen_ = false;
  command_accepted_ = false;
  command_seq_ = -1;
  try {
    send_client_->async_send_request(
        request, [this, token](rclcpp::Client<SendCommandSrv>::SharedFuture future) {
          if (token != generation_.load(std::memory_order_relaxed)) {
            return;
          }
          bool accepted = false;
          int seq = -1;
          try {
            const auto response = future.get();
            accepted = response && response->accepted;
            if (response) {
              seq = static_cast<int>(response->seq);
            }
          } catch (const std::exception &) {
            accepted = false;
          }
          command_seq_.store(seq, std::memory_order_relaxed);
          command_accepted_.store(accepted, std::memory_order_relaxed);
          command_response_seen_.store(true, std::memory_order_relaxed);
        });
  } catch (const std::exception &e) {
    RCLCPP_WARN(node_->get_logger(),
                "预选入口 branch gate: 命令发送异常，按机构容错继续：%s",
                e.what());
    command_response_seen_ = true;
    command_accepted_ = false;
    return true;
  }

  RCLCPP_INFO(node_->get_logger(),
              "预选入口 branch gate: 已发送 command=%s service=%s",
              byteHex(branch_command_id_).c_str(), command_service_.c_str());
  return true;
}

BT::NodeStatus PreselectionBranchGateAction::completeBranchHandshake(
    const std::string &reason, bool tolerated) {
  if (tolerated) {
    RCLCPP_WARN(
        node_->get_logger(),
        "预选入口 branch gate: 机构握手异常按容错完成，command=%s seq=%d reason=%s",
        byteHex(branch_command_id_).c_str(),
        command_seq_.load(std::memory_order_relaxed), reason.c_str());
  }
  if (branch_start_profile_ == StartProfile::Second) {
    // 该键表示逻辑启动 gate 已经放行，避免容错后目标树再次重复 0x11。
    config().blackboard->set(kPreselectionGateSecondStartDoneKey, true);
  }
  if (branch_ == Branch::ContinueFirst) {
    resetRuntimeHandles();
    return BT::NodeStatus::SUCCESS;
  }
  if (switch_tree_file_.empty()) {
    RCLCPP_INFO(node_->get_logger(),
                "预选入口 branch gate: 分支握手逻辑完成且未配置切树目标，继续当前树");
    resetRuntimeHandles();
    return BT::NodeStatus::SUCCESS;
  }
  requestBehaviorTreeSwitch(config().blackboard, switch_tree_file_);
  phase_ = Phase::SwitchRequested;
  resetRuntimeHandles();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PreselectionBranchGateAction::fail(
    const std::string &reason) {
  writeDecisionFailure(config().blackboard, "WaitPreselectionBranchGate",
                       reason);
  resetRuntimeHandles();
  return BT::NodeStatus::FAILURE;
}

void PreselectionBranchGateAction::resetRuntimeHandles() {
  publishGateState(false);
  feedback_sub_.reset();
  gate_state_pub_.reset();
  send_client_.reset();
}

std::string PreselectionBranchGateAction::byteHex(int value) {
  char buf[8];
  std::snprintf(buf, sizeof(buf), "0x%02X",
                static_cast<unsigned int>(value & 0xFF));
  return std::string(buf);
}

} // namespace rc26_decision
