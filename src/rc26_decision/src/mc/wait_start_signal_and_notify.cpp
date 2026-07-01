#include "wait_start_signal_and_notify.hpp"

#include <algorithm>
#include <chrono>
#include <exception>

#include "rc26_decision/decision_failure.hpp"

namespace rc26_decision {

namespace {

double elapsedSec(const std::chrono::steady_clock::time_point& since) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - since).count();
}

std::string byteHex(int value) {
    char buf[8];
    std::snprintf(buf, sizeof(buf), "0x%02X", static_cast<unsigned int>(value & 0xFF));
    return std::string(buf);
}

}  // namespace

WaitStartSignalAndNotifyAction::WaitStartSignalAndNotifyAction(
    const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::NodeStatus WaitStartSignalAndNotifyAction::onStart() {
    if (!config().blackboard->get("node", node_) || node_ == nullptr) {
        writeDecisionFailure(config().blackboard, "WaitStartSignalAndNotify",
                             "运行上下文缺失：node 不可用");
        return BT::NodeStatus::FAILURE;
    }
    if (!config().blackboard->get("mc_params", params_)) {
        RCLCPP_ERROR(node_->get_logger(), "组合树启动 gate: 黑板缺少 mc_params");
        writeDecisionFailure(config().blackboard, "WaitStartSignalAndNotify",
                             "黑板缺少 mc_params");
        return BT::NodeStatus::FAILURE;
    }

    signal_seen_ = false;
    command_response_seen_ = false;
    command_accepted_ = false;
    command_seq_ = -1;
    generation_.fetch_add(1, std::memory_order_relaxed);
    phase_ = Phase::WaitingSignal;
    start_tp_ = std::chrono::steady_clock::now();
    phase_tp_ = start_tp_;
    last_log_tp_ = start_tp_;

    feedback_sub_ = node_->create_subscription<FeedbackMsg>(
        params_.start_signal_feedback_topic, rclcpp::QoS(32).reliable(),
        [this](const FeedbackMsg::SharedPtr msg) { handleFeedback(msg); });
    send_client_ = node_->create_client<SendCommandSrv>(params_.start_command_service);

    RCLCPP_INFO(node_->get_logger(),
                "组合树启动 gate: 等待人工开始信号 feedback=%s topic=%s timeout=%.1fs，随后下发比赛开始 command=%s service=%s",
                byteHex(params_.start_signal_feedback_id).c_str(),
                params_.start_signal_feedback_topic.c_str(),
                params_.start_signal_timeout_s,
                byteHex(params_.start_command_id).c_str(),
                params_.start_command_service.c_str());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitStartSignalAndNotifyAction::onRunning() {
    if (node_ == nullptr) {
        return BT::NodeStatus::FAILURE;
    }

    const auto now = std::chrono::steady_clock::now();
    if (phase_ == Phase::WaitingSignal) {
        if (signal_seen_.load(std::memory_order_relaxed)) {
            RCLCPP_INFO(node_->get_logger(),
                        "组合树启动 gate: 已收到人工开始信号 %s，准备下发比赛开始命令 %s",
                        byteHex(params_.start_signal_feedback_id).c_str(),
                        byteHex(params_.start_command_id).c_str());
            phase_ = Phase::SendingCommand;
            phase_tp_ = now;
        } else {
            if (params_.start_signal_timeout_s > 0.0 &&
                elapsedSec(start_tp_) > params_.start_signal_timeout_s) {
                writeDecisionFailure(config().blackboard, "WaitStartSignalAndNotify",
                                     "等待人工开始信号超时");
                resetRuntimeHandles();
                return BT::NodeStatus::FAILURE;
            }
            if (elapsedSec(last_log_tp_) >= params_.start_log_period_s) {
                RCLCPP_INFO(node_->get_logger(),
                            "组合树启动 gate: 等待人工开始信号 %s，elapsed=%.1fs",
                            byteHex(params_.start_signal_feedback_id).c_str(),
                            elapsedSec(start_tp_));
                last_log_tp_ = now;
            }
            return BT::NodeStatus::RUNNING;
        }
    }

    if (phase_ == Phase::SendingCommand) {
        if (!sendStartCommand()) {
            return BT::NodeStatus::RUNNING;
        }
        phase_ = Phase::WaitingCommandAck;
        phase_tp_ = now;
        return BT::NodeStatus::RUNNING;
    }

    if (phase_ == Phase::WaitingCommandAck) {
        if (command_response_seen_.load(std::memory_order_relaxed)) {
            const int seq = command_seq_.load(std::memory_order_relaxed);
            if (command_accepted_.load(std::memory_order_relaxed)) {
                RCLCPP_INFO(node_->get_logger(),
                            "组合树启动 gate: 比赛开始命令 ACK 成功 command=%s seq=%d",
                            byteHex(params_.start_command_id).c_str(), seq);
                resetRuntimeHandles();
                return BT::NodeStatus::SUCCESS;
            }
            writeDecisionFailure(config().blackboard, "WaitStartSignalAndNotify",
                                 "比赛开始命令被拒绝，seq=" + std::to_string(seq));
            resetRuntimeHandles();
            return BT::NodeStatus::FAILURE;
        }
        if (elapsedSec(phase_tp_) > params_.start_command_timeout_s) {
            writeDecisionFailure(config().blackboard, "WaitStartSignalAndNotify",
                                 "等待比赛开始命令 ACK 超时");
            resetRuntimeHandles();
            return BT::NodeStatus::FAILURE;
        }
    }
    return BT::NodeStatus::RUNNING;
}

void WaitStartSignalAndNotifyAction::onHalted() {
    generation_.fetch_add(1, std::memory_order_relaxed);
    resetRuntimeHandles();
}

void WaitStartSignalAndNotifyAction::handleFeedback(const FeedbackMsg::SharedPtr msg) {
    if (!msg) {
        return;
    }
    if (msg->feedback_id != static_cast<uint8_t>(params_.start_signal_feedback_id & 0xFF)) {
        return;
    }
    signal_seen_.store(true, std::memory_order_relaxed);
}

bool WaitStartSignalAndNotifyAction::sendStartCommand() {
    if (!send_client_ || !send_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "组合树启动 gate: 等待服务 %s",
                             params_.start_command_service.c_str());
        return false;
    }

    auto request = std::make_shared<SendCommandSrv::Request>();
    request->command_id = static_cast<uint8_t>(params_.start_command_id & 0xFF);
    request->payload.clear();

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
                } catch (const std::exception&) {
                    accepted = false;
                }
                command_seq_.store(seq, std::memory_order_relaxed);
                command_accepted_.store(accepted, std::memory_order_relaxed);
                command_response_seen_.store(true, std::memory_order_relaxed);
            });
    } catch (const std::exception& e) {
        writeDecisionFailure(config().blackboard, "WaitStartSignalAndNotify",
                             std::string("比赛开始命令发送异常: ") + e.what());
        command_response_seen_ = true;
        command_accepted_ = false;
        return true;
    }

    RCLCPP_INFO(node_->get_logger(), "组合树启动 gate: 已下发比赛开始命令 command=%s",
                byteHex(params_.start_command_id).c_str());
    return true;
}

void WaitStartSignalAndNotifyAction::resetRuntimeHandles() {
    feedback_sub_.reset();
    send_client_.reset();
}

}  // namespace rc26_decision
