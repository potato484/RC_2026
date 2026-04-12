#include "rc26_merge_odom/transport/mechanism_transport_bridge.hpp"

#include <chrono>
#include <functional>
#include <utility>

#include "rc26_serial/protocol.hpp"

namespace rc26_merge_odom {

namespace {

constexpr char kMechanismTransportSendCommandService[] = "/mechanism/transport/send_command";
constexpr char kMechanismTransportFeedbackTopic[] = "/mechanism/transport/feedback";
constexpr auto kMechanismTransportFlushPeriod = std::chrono::milliseconds(10);

bool isContinuousTransportCommand(uint8_t command_id) {
    using CommandID = rc26_serial::CommandID;
    switch (static_cast<CommandID>(command_id)) {
    case CommandID::FRONT_TRACK_UP:
    case CommandID::FRONT_TRACK_DOWN:
        return true;
    default:
        return false;
    }
}

bool shouldPublishTransportFeedback(uint8_t feedback_id) {
    using FeedbackID = rc26_serial::FeedbackID;
    switch (static_cast<FeedbackID>(feedback_id)) {
    case FeedbackID::ACK:
    case FeedbackID::HEARTBEAT_ACK:
    case FeedbackID::ODOM_DATA:
        return false;
    default:
        return true;
    }
}

}  // namespace

MechanismTransportBridge::MechanismTransportBridge(
    rclcpp::Node& node, std::shared_ptr<rc26_decision::SerialDriver> target_serial)
    : node_(node), target_serial_(std::move(target_serial)) {
    feedback_pub_ =
        node_.create_publisher<FeedbackMsg>(kMechanismTransportFeedbackTopic, rclcpp::QoS(32).reliable());
    send_command_srv_ = node_.create_service<SendCommandSrv>(
        kMechanismTransportSendCommandService,
        std::bind(&MechanismTransportBridge::handleSendCommand, this, std::placeholders::_1,
                  std::placeholders::_2));
    flush_timer_ = node_.create_wall_timer(
        kMechanismTransportFlushPeriod, std::bind(&MechanismTransportBridge::flushFeedbackQueue, this));

    if (target_serial_) {
        target_serial_->setReceiveCallback([this](uint8_t seq, uint8_t feedback_id,
                                                  const std::vector<uint8_t>& payload) {
            enqueueFeedback(seq, feedback_id, payload);
        });
    }
}

MechanismTransportBridge::~MechanismTransportBridge() {
    if (target_serial_) {
        target_serial_->setReceiveCallback({});
    }
}

void MechanismTransportBridge::handleSendCommand(const std::shared_ptr<SendCommandSrv::Request> request,
                                                 std::shared_ptr<SendCommandSrv::Response> response) {
    response->accepted = false;
    response->seq = 0;

    if (!target_serial_ || !target_serial_->isOpen()) {
        RCLCPP_WARN(node_.get_logger(), "mechanism transport send rejected: target serial unavailable");
        return;
    }

    uint8_t seq = 0;
    const bool ok = isContinuousTransportCommand(request->command_id)
                        ? target_serial_->sendCommandNoAck(request->command_id, request->payload, seq)
                        : target_serial_->sendCommand(request->command_id, request->payload, seq);
    if (!ok) {
        RCLCPP_WARN(node_.get_logger(), "mechanism transport send failed: cmd=0x%02X err=%s",
                    request->command_id, target_serial_->lastError().c_str());
        return;
    }

    response->accepted = true;
    response->seq = seq;
}

void MechanismTransportBridge::enqueueFeedback(uint8_t seq, uint8_t feedback_id,
                                               const std::vector<uint8_t>& payload) {
    if (!shouldPublishTransportFeedback(feedback_id)) {
        return;
    }

    FeedbackMsg feedback;
    feedback.seq = seq;
    feedback.feedback_id = feedback_id;
    feedback.payload = payload;

    std::lock_guard<std::mutex> lock(queue_mutex_);
    pending_feedback_.push_back(std::move(feedback));
}

void MechanismTransportBridge::flushFeedbackQueue() {
    std::deque<FeedbackMsg> batch;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (pending_feedback_.empty()) {
            return;
        }
        batch.swap(pending_feedback_);
    }

    for (const auto& feedback : batch) {
        feedback_pub_->publish(feedback);
    }
}

}  // namespace rc26_merge_odom
