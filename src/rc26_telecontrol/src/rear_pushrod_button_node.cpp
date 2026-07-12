#include <chrono>
#include <cstddef>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "rc26_interfaces/srv/send_mechanism_transport_command.hpp"
#include "rc26_telecontrol/rear_pushrod_button_logic.hpp"

namespace rc26_telecontrol
{

namespace
{
constexpr auto k_rear_pushrod_dispatch_period = std::chrono::milliseconds(20);
constexpr char k_mechanism_send_command_service[] = "/mechanism/send_command";
constexpr std::size_t k_select_button = 6;
constexpr std::size_t k_start_button = 7;
}  // namespace

class RearPushrodButtonNode : public rclcpp::Node
{
public:
  using SendCommandSrv = rc26_interfaces::srv::SendMechanismTransportCommand;

  RearPushrodButtonNode()
  : Node("rc26_telecontrol_rear_pushrod_buttons")
  {
    send_command_client_ = create_client<SendCommandSrv>(k_mechanism_send_command_service);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&RearPushrodButtonNode::joyCallback, this, std::placeholders::_1));
    dispatch_timer_ = create_wall_timer(
      k_rear_pushrod_dispatch_period, std::bind(&RearPushrodButtonNode::dispatchQueuedCommand, this));

    RCLCPP_INFO(
      get_logger(),
      "rear pushrod sidecar ready: Select/Back(button[%zu])=extend(0x%02X), Start(button[%zu])=retract(0x%02X), edge-triggered reliable",
      k_select_button, commandIdForRearPushrodCommand(RearPushrodButtonCommand::kExtend),
      k_start_button, commandIdForRearPushrodCommand(RearPushrodButtonCommand::kRetract));
  }

private:
  static bool buttonPressed(const sensor_msgs::msg::Joy::ConstSharedPtr & msg, std::size_t index) noexcept
  {
    return msg && index < msg->buttons.size() && msg->buttons[index] != 0;
  }

  void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg)
  {
    const auto event = logic_.update(
      buttonPressed(msg, k_select_button), buttonPressed(msg, k_start_button));
    if (!event.has_value()) {
      return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    queued_command_ = *event;
  }

  void dispatchQueuedCommand()
  {
    std::optional<RearPushrodButtonCommand> command;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (request_in_flight_ || !queued_command_.has_value()) {
        return;
      }
      command = queued_command_;
      queued_command_.reset();
      request_in_flight_ = true;
    }

    if (!send_command_client_ || !send_command_client_->service_is_ready()) {
      finishLocalFailure(*command);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "rear pushrod command queued: %s unavailable", k_mechanism_send_command_service);
      return;
    }

    auto request = std::make_shared<SendCommandSrv::Request>();
    request->command_id = commandIdForRearPushrodCommand(*command);
    request->payload.clear();
    request->wait_ack = true;

    try {
      send_command_client_->async_send_request(
        request,
        [this, command = *command, command_id = request->command_id](
          rclcpp::Client<SendCommandSrv>::SharedFuture future) {
          finishRemoteResponse();
          try {
            const auto response = future.get();
            if (!response || !response->accepted) {
              RCLCPP_WARN(
                get_logger(), "rear pushrod transport send rejected: %s cmd=0x%02X",
                rearPushrodCommandName(command), command_id);
              return;
            }

            RCLCPP_DEBUG(
              get_logger(),
              "rear pushrod transport send accepted after transport ACK: %s cmd=0x%02X seq=%u",
              rearPushrodCommandName(command), command_id, response->seq);
          } catch (const std::exception & ex) {
            RCLCPP_ERROR(
              get_logger(), "rear pushrod transport callback failed: %s cmd=0x%02X err=%s",
              rearPushrodCommandName(command), command_id, ex.what());
          }
        });
    } catch (const std::exception & ex) {
      finishLocalFailure(*command);
      RCLCPP_ERROR(
        get_logger(), "failed to send rear pushrod transport request: %s cmd=0x%02X err=%s",
        rearPushrodCommandName(*command), request->command_id, ex.what());
    }
  }

  void finishLocalFailure(RearPushrodButtonCommand attempted_command)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    request_in_flight_ = false;
    if (!queued_command_.has_value()) {
      queued_command_ = attempted_command;
    }
  }

  void finishRemoteResponse()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    request_in_flight_ = false;
  }

  RearPushrodButtonLogic logic_;
  std::mutex state_mutex_;
  std::optional<RearPushrodButtonCommand> queued_command_;
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
    rclcpp::spin(std::make_shared<rc26_telecontrol::RearPushrodButtonNode>());
  } catch (const std::exception & ex) {
    exit_code = 1;
    RCLCPP_FATAL(
      rclcpp::get_logger("rc26_telecontrol_rear_pushrod_buttons"),
      "Unhandled exception: %s", ex.what());
  } catch (...) {
    exit_code = 1;
    RCLCPP_FATAL(
      rclcpp::get_logger("rc26_telecontrol_rear_pushrod_buttons"),
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
