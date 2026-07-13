// 组合树启动 gate：等待人工反馈信号后，通知下位机比赛开始。
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "mc_params.hpp"
#include "rc26_interfaces/msg/mechanism_transport_feedback.hpp"
#include "rc26_interfaces/srv/send_mechanism_transport_command.hpp"

namespace rc26_decision {

class WaitStartSignalAndNotifyAction : public BT::StatefulActionNode {
public:
    WaitStartSignalAndNotifyAction(const std::string& name,
                                   const BT::NodeConfig& config);

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    using FeedbackMsg = rc26_interfaces::msg::MechanismTransportFeedback;
    using SendCommandSrv = rc26_interfaces::srv::SendMechanismTransportCommand;

    enum class Phase { WaitingSignal, SendingCommand, WaitingCommandAck, WaitingDoneFeedback };

    void handleFeedback(const FeedbackMsg::SharedPtr msg);
    bool sendStartCommand();
    BT::NodeStatus completeStartHandshake(const std::string& reason,
                                          bool tolerated);
    void resetRuntimeHandles();
    bool captureRegistrationReference();
    bool openCamera(cv::VideoCapture& camera, int index, const std::string& path) const;

    McParams params_;
    rclcpp::Node* node_{nullptr};
    rclcpp::Subscription<FeedbackMsg>::SharedPtr feedback_sub_;
    rclcpp::Client<SendCommandSrv>::SharedPtr send_client_;
    std::chrono::steady_clock::time_point start_tp_{};
    std::chrono::steady_clock::time_point phase_tp_{};
    std::chrono::steady_clock::time_point last_log_tp_{};
    std::atomic<bool> signal_seen_{false};
    std::atomic<bool> command_response_seen_{false};
    std::atomic<bool> command_accepted_{false};
    std::atomic<bool> done_feedback_seen_{false};
    std::atomic<bool> command_error_seen_{false};
    std::atomic<bool> command_busy_seen_{false};
    std::atomic<int> command_seq_{-1};
    std::atomic<uint64_t> generation_{0};
    std::string command_error_detail_;
    Phase phase_{Phase::WaitingSignal};
};

}  // namespace rc26_decision
