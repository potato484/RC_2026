#include "rc26_mcu_transport/mcu_transport_node.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <string>
#include <utility>

#include "rc26_serial/protocol.hpp"

namespace rc26_mcu_transport {

namespace {

constexpr char kDefaultSendCommandService[] = "/mechanism/send_command";
constexpr char kDefaultCommandFeedbackTopic[] = "/mechanism/command_feedback";
constexpr char kDiagnosticsTopic[] = "/mcu_transport/diagnostics";
constexpr auto kFlushPeriod = std::chrono::milliseconds(10);

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

std::chrono::milliseconds clampPeriodMs(int period_ms, int fallback_ms) {
    return std::chrono::milliseconds(std::max(period_ms > 0 ? period_ms : fallback_ms, 100));
}

}  // namespace

McuTransportNode::McuTransportNode(const rclcpp::NodeOptions& options)
    : Node("mcu_transport", options) {
    target_serial_port_ = declare_parameter<std::string>("target_serial_port", "/dev/ttyUSB0");
    target_baudrate_ = declare_parameter<int>("target_baudrate", rc26_decision::UART_BAUDRATE);
    open_retry_period_ms_ = declare_parameter<int>("open_retry_period_ms", 1000);
    diagnostics_period_ms_ = declare_parameter<int>("diagnostics_period_ms", 1000);
    send_command_service_ = declare_parameter<std::string>(
        "send_command_service", kDefaultSendCommandService);
    command_feedback_topic_ = declare_parameter<std::string>(
        "command_feedback_topic", kDefaultCommandFeedbackTopic);

    serial_ = std::make_shared<rc26_decision::SerialDriver>();
    serial_->setReceiveCallback([this](uint8_t seq, uint8_t feedback_id,
                                       const std::vector<uint8_t>& payload) {
        handleSerialFrame(seq, feedback_id, payload);
    });
    serial_->setReconnectStartCallback([this]() {
        RCLCPP_WARN(get_logger(), "target MCU serial reconnecting: %s", target_serial_port_.c_str());
    });
    serial_->setReconnectCallback([this]() {
        RCLCPP_INFO(get_logger(), "target MCU serial reconnected: %s", target_serial_port_.c_str());
    });
    serial_->setReconnectFailedCallback([this]() {
        RCLCPP_ERROR(get_logger(), "target MCU serial reconnect failed: %s", target_serial_port_.c_str());
    });

    feedback_pub_ = create_publisher<FeedbackMsg>(command_feedback_topic_, rclcpp::QoS(32).reliable());
    diagnostics_pub_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>(kDiagnosticsTopic, rclcpp::QoS(1).reliable());
    send_command_srv_ = create_service<SendCommandSrv>(
        send_command_service_,
        std::bind(&McuTransportNode::handleSendCommand, this, std::placeholders::_1, std::placeholders::_2));

    flush_timer_ = create_wall_timer(kFlushPeriod, std::bind(&McuTransportNode::flushFeedbackQueue, this));
    diagnostics_timer_ = create_wall_timer(
        clampPeriodMs(diagnostics_period_ms_, 1000), std::bind(&McuTransportNode::publishDiagnostics, this));
    open_retry_timer_ = create_wall_timer(
        clampPeriodMs(open_retry_period_ms_, 1000), std::bind(&McuTransportNode::tryOpenSerial, this));

    tryOpenSerial();

    RCLCPP_INFO(
        get_logger(),
        "mcu transport provider ready: service=%s feedback=%s serial=%s baud=%d",
        send_command_service_.c_str(), command_feedback_topic_.c_str(),
        target_serial_port_.c_str(), target_baudrate_);
}

McuTransportNode::~McuTransportNode() {
    if (serial_) {
        serial_->setReceiveCallback({});
        serial_->close();
    }
}

void McuTransportNode::tryOpenSerial() {
    if (!serial_ || initial_open_succeeded_.load(std::memory_order_acquire)) {
        return;
    }
    if (serial_->isOpen()) {
        initial_open_succeeded_.store(true, std::memory_order_release);
        return;
    }

    if (serial_->open(target_serial_port_, target_baudrate_)) {
        initial_open_succeeded_.store(true, std::memory_order_release);
        RCLCPP_INFO(
            get_logger(), "target MCU serial opened: %s @ %d",
            target_serial_port_.c_str(), target_baudrate_);
        return;
    }

    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "target MCU serial open failed: %s @ %d err=%s",
        target_serial_port_.c_str(), target_baudrate_, serial_->lastError().c_str());
}

void McuTransportNode::handleSerialFrame(uint8_t seq, uint8_t feedback_id,
                                         const std::vector<uint8_t>& payload) {
    enqueueFeedback(seq, feedback_id, payload);
}

void McuTransportNode::handleSendCommand(const std::shared_ptr<SendCommandSrv::Request> request,
                                         std::shared_ptr<SendCommandSrv::Response> response) {
    response->accepted = false;
    response->seq = 0;

    if (!request) {
        rejected_send_count_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_WARN(get_logger(), "mechanism transport send rejected: empty request");
        return;
    }

    if (!serial_ || !serial_->isOpen()) {
        rejected_send_count_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "mechanism transport send rejected: target MCU serial unavailable");
        return;
    }

    uint8_t seq = 0;
    const bool ok = serial_->sendCommand(request->command_id, request->payload, seq);
    if (!ok) {
        rejected_send_count_.fetch_add(1, std::memory_order_relaxed);
        RCLCPP_WARN(
            get_logger(), "mechanism transport send failed: cmd=0x%02X err=%s",
            request->command_id, serial_->lastError().c_str());
        return;
    }

    accepted_send_count_.fetch_add(1, std::memory_order_relaxed);
    response->accepted = true;
    response->seq = seq;
}

void McuTransportNode::enqueueFeedback(uint8_t seq, uint8_t feedback_id,
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

void McuTransportNode::flushFeedbackQueue() {
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
        feedback_publish_count_.fetch_add(1, std::memory_order_relaxed);
    }
}

diagnostic_msgs::msg::KeyValue McuTransportNode::makeKeyValue(
    const std::string& key, const std::string& value) const {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    return kv;
}

int McuTransportNode::diagnosticLevel() const {
    if (!serial_ || !serial_->isOpen()) {
        return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }

    const auto level = serial_->commHealth().level();
    using HealthLevel = rc26_decision::SerialDriver::CommHealth::Level;
    switch (level) {
    case HealthLevel::HEALTHY:
        return diagnostic_msgs::msg::DiagnosticStatus::OK;
    case HealthLevel::DEGRADED:
        return diagnostic_msgs::msg::DiagnosticStatus::WARN;
    case HealthLevel::CRITICAL:
    case HealthLevel::FAILED:
    default:
        return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }
}

std::string McuTransportNode::healthLevelText() const {
    if (!serial_) {
        return "missing_serial_driver";
    }

    const auto level = serial_->commHealth().level();
    using HealthLevel = rc26_decision::SerialDriver::CommHealth::Level;
    switch (level) {
    case HealthLevel::HEALTHY:
        return "healthy";
    case HealthLevel::DEGRADED:
        return "degraded";
    case HealthLevel::CRITICAL:
        return "critical";
    case HealthLevel::FAILED:
        return "failed";
    default:
        return "unknown";
    }
}

void McuTransportNode::publishDiagnostics() {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "rc26_mcu_transport";
    status.hardware_id = target_serial_port_;
    status.level = static_cast<uint8_t>(diagnosticLevel());
    status.message = (!serial_ || !serial_->isOpen()) ? "target MCU serial unavailable" : healthLevelText();

    if (serial_) {
        const auto& health = serial_->commHealth();
        status.values.push_back(makeKeyValue("serial_open", serial_->isOpen() ? "true" : "false"));
        status.values.push_back(makeKeyValue("port", target_serial_port_));
        status.values.push_back(makeKeyValue("baudrate", std::to_string(target_baudrate_)));
        status.values.push_back(makeKeyValue("health_level", healthLevelText()));
        status.values.push_back(makeKeyValue("avg_rtt_ms", std::to_string(serial_->avgRttMs())));
        status.values.push_back(makeKeyValue("last_error", serial_->lastError()));
        status.values.push_back(makeKeyValue("total_frames", std::to_string(health.total_frames.load())));
        status.values.push_back(makeKeyValue("parse_errors", std::to_string(health.parse_errors.load())));
        status.values.push_back(makeKeyValue("ack_timeouts", std::to_string(health.ack_timeouts.load())));
        status.values.push_back(makeKeyValue("reconnect_count", std::to_string(health.reconnect_count.load())));
        status.values.push_back(
            makeKeyValue("heartbeat_failures", std::to_string(health.heartbeat_failures.load())));
    }
    status.values.push_back(
        makeKeyValue("accepted_send_count", std::to_string(accepted_send_count_.load())));
    status.values.push_back(
        makeKeyValue("rejected_send_count", std::to_string(rejected_send_count_.load())));
    status.values.push_back(
        makeKeyValue("feedback_publish_count", std::to_string(feedback_publish_count_.load())));

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
}

}  // namespace rc26_mcu_transport

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rc26_mcu_transport::McuTransportNode>());
    rclcpp::shutdown();
    return 0;
}
