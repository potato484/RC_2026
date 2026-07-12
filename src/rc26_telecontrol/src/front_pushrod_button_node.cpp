#include <chrono>
#include <cstddef>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "rc26_interfaces/srv/send_mechanism_transport_command.hpp"
#include "rc26_telecontrol/front_pushrod_button_logic.hpp"

namespace rc26_telecontrol
{

namespace
{
constexpr auto k_front_pushrod_dispatch_period = std::chrono::milliseconds(20);
constexpr char k_mechanism_send_command_service[] = "/mechanism/send_command";
}  // namespace

class FrontPushrodButtonNode : public rclcpp::Node
{
public:
  using SendCommandSrv = rc26_interfaces::srv::SendMechanismTransportCommand;

  FrontPushrodButtonNode()
  : Node("rc26_telecontrol_front_pushrod_buttons")
  {
    send_command_client_ = create_client<SendCommandSrv>(k_mechanism_send_command_service);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&FrontPushrodButtonNode::joyCallback, this, std::placeholders::_1));
    dispatch_timer_ = create_wall_timer(
      k_front_pushrod_dispatch_period, std::bind(&FrontPushrodButtonNode::dispatchQueuedCommand, this));
    RCLCPP_INFO(
      get_logger(),
      "front pushrod sidecar ready: Y(button[%d])=extend(0x%02X), A(button[%d])=retract(0x%02X), edge-triggered reliable",
      k_front_pushrod_extend_button,
      static_cast<uint8_t>(rc26_serial::CommandID::FRONT_PUSHROD_EXTEND),
      k_front_pushrod_retract_button,
      static_cast<uint8_t>(rc26_serial::CommandID::FRONT_PUSHROD_RETRACT));
  }

private:
  static bool buttonPressed(const sensor_msgs::msg::Joy::ConstSharedPtr & msg, std::size_t index) noexcept
  {
    return msg && index < msg->buttons.size() && msg->buttons[index] != 0;
  }

  void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg)
  {
    const auto event = button_logic_.update(
      buttonPressed(msg, static_cast<std::size_t>(k_front_pushrod_extend_button)),
      buttonPressed(msg, static_cast<std::size_t>(k_front_pushrod_retract_button)));
    if (!event.has_value()) {
      return;
    }

    if (*event == FrontPushrodButtonCommand::kConflict) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ignore front pushrod command this cycle: Y and A pressed at the same time");
      return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    queued_commands_.push_back(*event);
  }

  void dispatchQueuedCommand()
  {
    std::optional<FrontPushrodButtonCommand> command;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (request_in_flight_ || queued_commands_.empty()) {
        return;
      }
      command = queued_commands_.front();
      queued_commands_.pop_front();
      request_in_flight_ = true;
    }

    const auto command_id = commandIdForFrontPushrodCommand(*command);
    if (!command_id.has_value()) {
      finishRemoteResponse();
      RCLCPP_ERROR(
        get_logger(), "front pushrod dispatch skipped: unsupported event=%u",
        static_cast<unsigned>(*command));
      return;
    }

    if (!send_command_client_ || !send_command_client_->service_is_ready()) {
      finishLocalFailure(*command);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "front pushrod command queued: %s unavailable", k_mechanism_send_command_service);
      return;
    }

    auto request = std::make_shared<SendCommandSrv::Request>();
    request->command_id = *command_id;
    request->payload.clear();
    request->wait_ack = true;

    try {
      send_command_client_->async_send_request(
        request,
        [this, command = *command, command_id = *command_id](
          rclcpp::Client<SendCommandSrv>::SharedFuture future) {
          finishRemoteResponse();
          try {
            const auto response = future.get();
            if (!response || !response->accepted) {
              RCLCPP_WARN(
                get_logger(), "front pushrod transport send rejected: %s cmd=0x%02X",
                frontPushrodCommandName(command), command_id);
              return;
            }
            RCLCPP_DEBUG(
              get_logger(),
              "front pushrod transport send accepted after transport ACK: %s cmd=0x%02X seq=%u",
              frontPushrodCommandName(command), command_id, response->seq);
          } catch (const std::exception & ex) {
            RCLCPP_ERROR(
              get_logger(), "front pushrod transport callback failed: %s cmd=0x%02X err=%s",
              frontPushrodCommandName(command), command_id, ex.what());
          }
        });
    } catch (const std::exception & ex) {
      finishLocalFailure(*command);
      RCLCPP_ERROR(
        get_logger(), "failed to send front pushrod transport request: %s cmd=0x%02X err=%s",
        frontPushrodCommandName(*command), *command_id, ex.what());
    }
  }

  void finishLocalFailure(FrontPushrodButtonCommand attempted_command)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    request_in_flight_ = false;
    queued_commands_.push_front(attempted_command);
  }

  void finishRemoteResponse()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    request_in_flight_ = false;
  }

  FrontPushrodButtonLogic button_logic_;
  std::mutex state_mutex_;
  std::deque<FrontPushrodButtonCommand> queued_commands_;
  bool request_in_flight_{false};
  rclcpp::Client<SendCommandSrv>::SharedPtr send_command_client_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr dispatch_timer_;
};

}  // namespace rc26_telecontrol

namespace
{

int runNode(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;

  try {
    rclcpp::spin(std::make_shared<rc26_telecontrol::FrontPushrodButtonNode>());
  } catch (const std::exception & ex) {
    exit_code = 1;
    RCLCPP_FATAL(
      rclcpp::get_logger("rc26_telecontrol_front_pushrod_buttons"),
      "Unhandled exception: %s", ex.what());
  } catch (...) {
    exit_code = 1;
    RCLCPP_FATAL(
      rclcpp::get_logger("rc26_telecontrol_front_pushrod_buttons"),
      "Unhandled exception: unknown error.");
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return exit_code;
}

}  // namespace

int main(int argc, char * argv[])
{
  return runNode(argc, argv);
}
