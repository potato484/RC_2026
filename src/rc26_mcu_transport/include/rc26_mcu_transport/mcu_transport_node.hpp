#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rc26_mcu_transport/chassis_cmd_vel_logic.hpp"
#include "rc26_interfaces/msg/mechanism_transport_feedback.hpp"
#include "rc26_interfaces/srv/send_mechanism_transport_command.hpp"
#include "rc26_serial/serial_driver.hpp"
#include "rc26_serial/protocol.hpp"

namespace rc26_mcu_transport {

inline bool shouldPublishTransportFeedback(uint8_t feedback_id,
                                           std::size_t payload_size) {
    using FeedbackID = rc26_serial::FeedbackID;
    switch (static_cast<FeedbackID>(feedback_id)) {
    case FeedbackID::ACK:
    case FeedbackID::HEARTBEAT_ACK:
    case FeedbackID::ODOM_DATA:
        return false;
    case FeedbackID::MCU_ERROR:
        return rc26_serial::isPlanarArmErrorPayloadSize(payload_size);
    default:
        return true;
    }
}

class McuTransportNode final : public rclcpp::Node {
public:
    using FeedbackMsg = rc26_interfaces::msg::MechanismTransportFeedback;
    using SendCommandSrv = rc26_interfaces::srv::SendMechanismTransportCommand;
    using TwistMsg = geometry_msgs::msg::Twist;

    explicit McuTransportNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~McuTransportNode() override;

private:
    void tryOpenSerial();
    void handleSerialFrame(uint8_t seq, uint8_t feedback_id, const std::vector<uint8_t>& payload);
    void handleSendCommand(const std::shared_ptr<SendCommandSrv::Request> request,
                           std::shared_ptr<SendCommandSrv::Response> response);
    void handleChassisCmdVel(const TwistMsg::SharedPtr msg);
    void sendChassisTarget();
    void enqueueFeedback(uint8_t seq, uint8_t feedback_id, const std::vector<uint8_t>& payload);
    void flushFeedbackQueue();
    void publishDiagnostics();

    std::chrono::milliseconds chassisTargetPeriod() const;
    diagnostic_msgs::msg::KeyValue makeKeyValue(const std::string& key, const std::string& value) const;
    int diagnosticLevel() const;
    std::string healthLevelText() const;

    std::shared_ptr<rc26_decision::SerialDriver> serial_;
    std::string target_serial_port_;
    int target_baudrate_{1000000};
    int open_retry_period_ms_{1000};
    int diagnostics_period_ms_{1000};
    std::string send_command_service_;
    std::string command_feedback_topic_;
    ChassisCmdVelConfig chassis_config_;
    ChassisCmdVelState chassis_state_;

    std::atomic<bool> initial_open_succeeded_{false};
    std::atomic<uint64_t> accepted_send_count_{0};
    std::atomic<uint64_t> rejected_send_count_{0};
    std::atomic<uint64_t> feedback_publish_count_{0};
    std::atomic<uint64_t> chassis_target_send_count_{0};
    std::atomic<uint64_t> chassis_target_send_fail_count_{0};
    std::atomic<uint64_t> chassis_target_zero_send_count_{0};

    rclcpp::Publisher<FeedbackMsg>::SharedPtr feedback_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp::Subscription<TwistMsg>::SharedPtr chassis_cmd_vel_sub_;
    rclcpp::Service<SendCommandSrv>::SharedPtr send_command_srv_;
    rclcpp::TimerBase::SharedPtr open_retry_timer_;
    rclcpp::TimerBase::SharedPtr flush_timer_;
    rclcpp::TimerBase::SharedPtr diagnostics_timer_;
    rclcpp::TimerBase::SharedPtr chassis_target_timer_;

    std::mutex queue_mutex_;
    std::deque<FeedbackMsg> pending_feedback_;
    std::mutex chassis_mutex_;
};

}  // namespace rc26_mcu_transport
