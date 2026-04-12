#pragma once

#include <deque>
#include <memory>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "rc26_interfaces/msg/mechanism_transport_feedback.hpp"
#include "rc26_interfaces/srv/send_mechanism_transport_command.hpp"
#include "rc26_serial/serial_driver.hpp"

namespace rc26_merge_odom {

class MechanismTransportBridge {
public:
    using FeedbackMsg = rc26_interfaces::msg::MechanismTransportFeedback;
    using SendCommandSrv = rc26_interfaces::srv::SendMechanismTransportCommand;

    MechanismTransportBridge(rclcpp::Node& node, std::shared_ptr<rc26_decision::SerialDriver> target_serial);
    ~MechanismTransportBridge();

    MechanismTransportBridge(const MechanismTransportBridge&) = delete;
    MechanismTransportBridge& operator=(const MechanismTransportBridge&) = delete;

private:
    void handleSendCommand(const std::shared_ptr<SendCommandSrv::Request> request,
                           std::shared_ptr<SendCommandSrv::Response> response);
    void enqueueFeedback(uint8_t seq, uint8_t feedback_id, const std::vector<uint8_t>& payload);
    void flushFeedbackQueue();

    rclcpp::Node& node_;
    std::shared_ptr<rc26_decision::SerialDriver> target_serial_;
    rclcpp::Publisher<FeedbackMsg>::SharedPtr feedback_pub_;
    rclcpp::Service<SendCommandSrv>::SharedPtr send_command_srv_;
    rclcpp::TimerBase::SharedPtr flush_timer_;
    std::mutex queue_mutex_;
    std::deque<FeedbackMsg> pending_feedback_;
};

}  // namespace rc26_merge_odom
