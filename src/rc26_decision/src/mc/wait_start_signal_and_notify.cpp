#include "wait_start_signal_and_notify.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <exception>
#include <thread>

#include "rc26_decision/decision_failure.hpp"
#include "rc26_decision/mechanism_error_diagnostic.hpp"

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
    done_feedback_seen_ = false;
    command_error_seen_ = false;
    command_busy_seen_ = false;
    command_seq_ = -1;
    command_error_detail_.clear();
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
                "组合树启动 gate: 等待人工开始信号 feedback=%s topic=%s timeout=%.1fs，随后下发比赛开始 command=%s service=%s，并等待完成反馈=%s",
                byteHex(params_.start_signal_feedback_id).c_str(),
                params_.start_signal_feedback_topic.c_str(),
                params_.start_signal_timeout_s,
                byteHex(params_.start_command_id).c_str(),
                params_.start_command_service.c_str(),
                byteHex(params_.start_done_feedback_id).c_str());
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
            if (elapsedSec(phase_tp_) > params_.start_command_timeout_s) {
                return completeStartHandshake("等待比赛开始命令服务可用超时", true);
            }
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
                            "组合树启动 gate: 比赛开始命令 ACK 成功 command=%s seq=%d，继续等待完成反馈 %s",
                            byteHex(params_.start_command_id).c_str(), seq,
                            byteHex(params_.start_done_feedback_id).c_str());
                phase_ = Phase::WaitingDoneFeedback;
                phase_tp_ = now;
                last_log_tp_ = now;
                return BT::NodeStatus::RUNNING;
            }
            return completeStartHandshake(
                "比赛开始命令未被接受，seq=" + std::to_string(seq), true);
        }
        if (elapsedSec(phase_tp_) > params_.start_command_timeout_s) {
            return completeStartHandshake("等待比赛开始命令 ACK 超时", true);
        }
    }

    if (phase_ == Phase::WaitingDoneFeedback) {
        const int seq = command_seq_.load(std::memory_order_relaxed);
        if (command_error_seen_.load(std::memory_order_relaxed)) {
            return completeStartHandshake(
                command_error_detail_.empty()
                    ? "比赛开始命令收到 MCU 0xFE 最终错误，seq=" +
                          std::to_string(seq)
                    : command_error_detail_,
                true);
        }
        if (done_feedback_seen_.load(std::memory_order_relaxed)) {
            RCLCPP_INFO(node_->get_logger(),
                        "组合树启动 gate: 已收到比赛开始完成反馈 %s seq=%d",
                        byteHex(params_.start_done_feedback_id).c_str(), seq);
            return completeStartHandshake("", false);
        }
        if (elapsedSec(phase_tp_) > params_.start_done_timeout_s) {
            return completeStartHandshake(
                "等待比赛开始完成反馈超时，seq=" + std::to_string(seq), true);
        }
        if (elapsedSec(last_log_tp_) >= params_.start_log_period_s) {
            RCLCPP_INFO(node_->get_logger(),
                        "组合树启动 gate: 等待比赛开始完成反馈 %s seq=%d elapsed=%.1fs busy=%s",
                        byteHex(params_.start_done_feedback_id).c_str(), seq,
                        elapsedSec(phase_tp_),
                        command_busy_seen_.load(std::memory_order_relaxed) ? "是" : "否");
            last_log_tp_ = now;
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
    const int seq = command_seq_.load(std::memory_order_relaxed);
    std::optional<MechanismErrorDiagnostic> diagnostic;
    if (isSameSeqMechanismError(*msg, seq, diagnostic)) {
        const std::string detail = mechanismErrorDiagnosticText(*diagnostic);
        if (diagnostic->busy) {
            command_busy_seen_.store(true, std::memory_order_relaxed);
            if (node_) {
                RCLCPP_INFO(node_->get_logger(),
                            "组合树启动 gate: 比赛开始命令仍在处理中：%s",
                            detail.c_str());
            }
        } else {
            command_error_detail_ = detail;
            command_error_seen_.store(true, std::memory_order_relaxed);
            if (node_) {
                RCLCPP_WARN(node_->get_logger(),
                            "组合树启动 gate: 比赛开始命令收到 MCU 最终错误，按机构容错处理：%s",
                            detail.c_str());
            }
        }
        return;
    }
    if (seq >= 0 &&
        msg->seq == static_cast<uint8_t>(seq & 0xFF) &&
        msg->feedback_id == static_cast<uint8_t>(params_.start_done_feedback_id & 0xFF)) {
        done_feedback_seen_.store(true, std::memory_order_relaxed);
        return;
    }
    if (msg->feedback_id == static_cast<uint8_t>(params_.start_signal_feedback_id & 0xFF) &&
        phase_ == Phase::WaitingSignal) {
        signal_seen_.store(true, std::memory_order_relaxed);
    }
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
                } catch (const std::exception&) {
                    accepted = false;
                }
                command_seq_.store(seq, std::memory_order_relaxed);
                command_accepted_.store(accepted, std::memory_order_relaxed);
                command_response_seen_.store(true, std::memory_order_relaxed);
            });
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(),
                    "组合树启动 gate: 比赛开始命令发送异常，按机构容错继续：%s",
                    e.what());
        command_response_seen_ = true;
        command_accepted_ = false;
        return true;
    }

    RCLCPP_INFO(node_->get_logger(), "组合树启动 gate: 已下发比赛开始命令 command=%s",
                byteHex(params_.start_command_id).c_str());
    return true;
}

BT::NodeStatus WaitStartSignalAndNotifyAction::completeStartHandshake(
    const std::string& reason, bool tolerated) {
    if (tolerated) {
        RCLCPP_WARN(
            node_->get_logger(),
            "组合树启动 gate: 机构握手异常按容错完成，command=%s seq=%d reason=%s",
            byteHex(params_.start_command_id).c_str(),
            command_seq_.load(std::memory_order_relaxed), reason.c_str());
    }
    if (params_.registration_gate_enable && !captureRegistrationReference()) {
        writeDecisionFailure(config().blackboard, "WaitStartSignalAndNotify",
                             "比赛开始后采集 MC 配准基准帧失败");
        resetRuntimeHandles();
        return BT::NodeStatus::FAILURE;
    }
    resetRuntimeHandles();
    return BT::NodeStatus::SUCCESS;
}

void WaitStartSignalAndNotifyAction::resetRuntimeHandles() {
    feedback_sub_.reset();
    send_client_.reset();
}

bool WaitStartSignalAndNotifyAction::openCamera(cv::VideoCapture& camera,
                                                int index,
                                                const std::string& path) const {
    const bool opened = path.empty() ? (camera.open(index, cv::CAP_V4L2) || camera.open(index))
                                     : (camera.open(path, cv::CAP_V4L2) || camera.open(path));
    if (!opened) {
        return false;
    }
    camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    camera.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(params_.width));
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(params_.height));
    camera.set(cv::CAP_PROP_FPS, static_cast<double>(params_.fps));
    return true;
}

bool WaitStartSignalAndNotifyAction::captureRegistrationReference() {
    if (node_ == nullptr) {
        return false;
    }
    if (params_.registration_capture_settle_s > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(
            params_.registration_capture_settle_s));
    }

    cv::VideoCapture camera;
    bool ok = params_.camera_device.empty() ? openCamera(camera, params_.camera_index, "")
                                             : openCamera(camera, -1, params_.camera_device);
    if (!ok && params_.auto_scan_camera) {
        for (int i = 0; i < 10 && !ok; ++i) {
            if (i == params_.camera_index) {
                continue;
            }
            ok = openCamera(camera, i, "");
        }
    }
    if (!ok) {
        RCLCPP_ERROR(node_->get_logger(),
                     "组合树启动 gate: 无法打开 MC 相机采集配准基准帧 (index=%d device='%s')",
                     params_.camera_index, params_.camera_device.c_str());
        return false;
    }

    cv::Mat frame;
    for (int i = 0; i < 5; ++i) {
        if (!camera.read(frame) || frame.empty()) {
            frame.release();
            continue;
        }
    }
    camera.release();
    if (frame.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "组合树启动 gate: MC 相机基准帧为空");
        return false;
    }

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    config().blackboard->set(params_.registration_reference_blackboard_key, gray.clone());
    RCLCPP_INFO(node_->get_logger(),
                "组合树启动 gate: 已采集 MC 配准基准帧 key=%s size=%dx%d",
                params_.registration_reference_blackboard_key.c_str(), gray.cols, gray.rows);
    return true;
}

}  // namespace rc26_decision
