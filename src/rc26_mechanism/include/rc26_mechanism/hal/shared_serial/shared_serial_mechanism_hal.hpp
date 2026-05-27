#pragma once

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rc26_interfaces/msg/mechanism_transport_feedback.hpp"
#include "rc26_interfaces/srv/send_mechanism_transport_command.hpp"

#include "rc26_mechanism/hal/contracts/i_mechanism_hal.hpp"

namespace rc26_mechanism {

class SharedSerialMechanismHAL : public IMechanismHAL {
public:
    explicit SharedSerialMechanismHAL(rclcpp_lifecycle::LifecycleNode& node);

    bool open() override;
    void close() override;
    bool isOpen() const override;
    bool sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload = {}) override;
    bool sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload, uint8_t& out_seq) override;
    void setFeedbackCallback(FeedbackCallback cb) override;
    CommHealthSnapshot commHealthSnapshot() const override;

private:
    using FeedbackMsg = rc26_interfaces::msg::MechanismTransportFeedback;
    using SendCommandSrv = rc26_interfaces::srv::SendMechanismTransportCommand;

    static constexpr std::chrono::milliseconds kServiceWaitTimeout{1000};
    static constexpr std::chrono::milliseconds kServiceCallTimeout{1000};

    void ensureInterfaces();
    void handleFeedback(const FeedbackMsg& feedback_msg);

    rclcpp_lifecycle::LifecycleNode& node_;
    rclcpp::Client<SendCommandSrv>::SharedPtr send_command_client_;
    rclcpp::Subscription<FeedbackMsg>::SharedPtr feedback_sub_;
    FeedbackCallback callback_;
    bool open_{false};
};

}  // namespace rc26_mechanism
