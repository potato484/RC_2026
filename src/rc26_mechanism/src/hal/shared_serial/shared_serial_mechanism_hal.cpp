#include "rc26_mechanism/hal/shared_serial/shared_serial_mechanism_hal.hpp"

#include <future>
#include <utility>

namespace rc26_mechanism {

namespace {

constexpr char kMechanismSendCommandService[] = "/mechanism/send_command";
constexpr char kMechanismCommandFeedbackTopic[] = "/mechanism/command_feedback";

}  // namespace

SharedSerialMechanismHAL::SharedSerialMechanismHAL(rclcpp_lifecycle::LifecycleNode& node) : node_(node) {}

bool SharedSerialMechanismHAL::open() {
    ensureInterfaces();
    if (!send_command_client_) {
        return false;
    }
    if (!send_command_client_->wait_for_service(kServiceWaitTimeout)) {
        RCLCPP_ERROR(node_.get_logger(), "shared_serial HAL cannot reach %s",
                     kMechanismSendCommandService);
        open_ = false;
        return false;
    }
    open_ = true;
    return true;
}

void SharedSerialMechanismHAL::close() {
    open_ = false;
}

bool SharedSerialMechanismHAL::isOpen() const {
    return open_;
}

bool SharedSerialMechanismHAL::sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload) {
    uint8_t seq = 0;
    return sendCommand(cmd_id, payload, seq);
}

bool SharedSerialMechanismHAL::sendCommand(uint8_t cmd_id, const std::vector<uint8_t>& payload, uint8_t& out_seq) {
    out_seq = 0;
    if (!isOpen()) {
        return false;
    }

    ensureInterfaces();
    if (!send_command_client_ || !send_command_client_->service_is_ready()) {
        RCLCPP_WARN(node_.get_logger(), "shared_serial HAL send rejected: transport service unavailable");
        return false;
    }

    auto request = std::make_shared<SendCommandSrv::Request>();
    request->command_id = cmd_id;
    request->payload = payload;
    request->wait_ack = true;

    auto future = send_command_client_->async_send_request(request);
    if (future.wait_for(kServiceCallTimeout) != std::future_status::ready) {
        RCLCPP_WARN(node_.get_logger(), "shared_serial HAL send timed out: cmd=0x%02X", cmd_id);
        return false;
    }

    const auto response = future.get();
    if (!response || !response->accepted) {
        RCLCPP_WARN(node_.get_logger(), "shared_serial HAL send failed: cmd=0x%02X", cmd_id);
        return false;
    }

    out_seq = response->seq;
    return true;
}

void SharedSerialMechanismHAL::setFeedbackCallback(FeedbackCallback cb) {
    callback_ = std::move(cb);
}

CommHealthSnapshot SharedSerialMechanismHAL::commHealthSnapshot() const {
    return {};
}

void SharedSerialMechanismHAL::ensureInterfaces() {
    if (!send_command_client_) {
        send_command_client_ = node_.create_client<SendCommandSrv>(kMechanismSendCommandService);
    }
    if (!feedback_sub_) {
        feedback_sub_ = node_.create_subscription<FeedbackMsg>(
            kMechanismCommandFeedbackTopic, rclcpp::QoS(32).reliable(),
            [this](const FeedbackMsg::SharedPtr msg) {
                if (msg) {
                    handleFeedback(*msg);
                }
            });
    }
}

void SharedSerialMechanismHAL::handleFeedback(const FeedbackMsg& feedback_msg) {
    if (!callback_) {
        return;
    }
    callback_(feedback_msg.seq, feedback_msg.feedback_id, feedback_msg.payload);
}

}  // namespace rc26_mechanism
