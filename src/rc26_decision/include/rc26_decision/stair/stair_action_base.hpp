#pragma once

#include <atomic>
#include <cstdint>
#include <cstddef>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rc26_decision/stair/stair_area.hpp"
#include "rc26_interfaces/msg/mechanism_transport_feedback.hpp"
#include "rc26_interfaces/srv/send_mechanism_transport_command.hpp"
#include "rc26_serial/protocol.hpp"

namespace rc26_decision {

class StairActionBase : public BT::StatefulActionNode {
public:
  StairActionBase(const std::string &name, const BT::NodeConfig &config);

  static BT::PortsList providedPorts();

  enum class StepStatus { Running, Success, Failure };
  enum class WheelEvent { FrontFirst, FrontSecond, Rear };

protected:
  bool setupRuntime(const char *action_label);
  void releaseRuntime();

  void publishDrive(double signed_speed_mps);
  void publishStop();
  double driveSpeedMagnitude() const;

  void beginCommand(CommandID command_id, const char *label);
  StepStatus tickCommand();
  void beginCommandPair(CommandID first_command_id, const char *first_label,
                        CommandID second_command_id, const char *second_label);
  StepStatus tickCommandPair();

  void beginEventWait(WheelEvent event, double timeout_s, const char *label);
  StepStatus tickEventWait();

  void beginTimedDrive(double signed_speed_mps, double duration_s,
                       const char *label);
  StepStatus tickTimedDrive();
  void beginZeroHold(double duration_s, const char *label);
  StepStatus tickZeroHold();

  BT::NodeStatus failWithStop(const char *reason);

  rclcpp::Node *node_{nullptr};
  StairParams params_;

private:
  using TwistMsg = geometry_msgs::msg::Twist;
  using FeedbackMsg = rc26_interfaces::msg::MechanismTransportFeedback;
  using SendCommandSrv = rc26_interfaces::srv::SendMechanismTransportCommand;

  double elapsedSinceStageStart() const;
  void markStageStart();
  void resetCommandState();
  void resetCommandPairState();
  bool sendPairCommand(std::size_t index);
  bool eventReceived() const;

  std::string action_label_;

  rclcpp::Publisher<TwistMsg>::SharedPtr cmd_pub_;
  rclcpp::Client<SendCommandSrv>::SharedPtr send_client_;
  rclcpp::Subscription<FeedbackMsg>::SharedPtr feedback_sub_;

  rclcpp::Time stage_start_;
  rclcpp::Time last_drive_publish_;
  bool has_last_drive_publish_{false};

  CommandID active_command_{CommandID::STOP};
  std::string active_command_label_;
  bool command_sent_{false};
  std::atomic<bool> command_response_seen_{false};
  std::atomic<bool> command_accepted_{false};
  std::atomic<bool> command_rejected_{false};
  std::atomic<uint64_t> command_generation_{0};

  struct CommandSlot {
    CommandID command_id{CommandID::STOP};
    std::string label;
    bool sent{false};
    std::atomic<bool> response_seen{false};
    std::atomic<bool> accepted{false};
    std::atomic<bool> rejected{false};
  };
  CommandSlot command_pair_[2];
  bool command_pair_active_{false};

  WheelEvent active_event_{WheelEvent::FrontFirst};
  std::string active_event_label_;
  double active_event_timeout_s_{0.0};
  uint64_t front_first_event_baseline_{0};
  uint64_t front_second_event_baseline_{0};
  uint64_t rear_event_baseline_{0};
  std::atomic<uint64_t> front_first_event_count_{0};
  std::atomic<uint64_t> front_second_event_count_{0};
  std::atomic<uint64_t> rear_event_count_{0};

  double timed_drive_speed_mps_{0.0};
  double timed_drive_duration_s_{0.0};
  std::string timed_drive_label_;

  double zero_hold_duration_s_{0.0};
  std::string zero_hold_label_;
};

} // namespace rc26_decision
