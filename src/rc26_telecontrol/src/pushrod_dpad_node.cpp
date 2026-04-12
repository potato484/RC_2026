#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "rc26_interfaces/srv/send_mechanism_transport_command.hpp"
#include "rc26_telecontrol/pushrod_dpad_logic.hpp"

namespace rc26_telecontrol
{

namespace
{
constexpr auto k_pushrod_dispatch_period = std::chrono::milliseconds(20);
constexpr char k_mechanism_transport_send_command_service[] = "/mechanism/transport/send_command";
constexpr std::size_t k_dpad_x_axis = 6;
}  // namespace

class PushrodDpadNode : public rclcpp::Node
{
public:
  using SendCommandSrv = rc26_interfaces::srv::SendMechanismTransportCommand;

  PushrodDpadNode()
  : Node("rc26_telecontrol_pushrod_dpad")
  {
    send_command_client_ = create_client<SendCommandSrv>(k_mechanism_transport_send_command_service);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&PushrodDpadNode::joyCallback, this, std::placeholders::_1));
    dispatch_timer_ = create_wall_timer(
      k_pushrod_dispatch_period, std::bind(&PushrodDpadNode::dispatchQueuedCommand, this));

    RCLCPP_INFO(
      get_logger(),
      "pushrod dpad ready: axes[%zu] left=extend(0x%02X), right=retract(0x%02X), edge-triggered",
      k_dpad_x_axis, commandIdForPushrodCommand(PushrodCommand::kExtend),
      commandIdForPushrodCommand(PushrodCommand::kRetract));
  }

private:
  static double axisValue(const sensor_msgs::msg::Joy::ConstSharedPtr & msg, std::size_t index) noexcept
  {
    if (!msg || index >= msg->axes.size()) {
      return 0.0;
    }

    const double value = msg->axes[index];
    if (!std::isfinite(value)) {
      return 0.0;
    }

    return std::clamp(value, -1.0, 1.0);
  }

  void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg)
  {
    const auto event = logic_.update(axisValue(msg, k_dpad_x_axis));
    if (!event.has_value()) {
      return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    queued_command_ = *event;
  }

  void dispatchQueuedCommand()
  {
    std::optional<PushrodCommand> command;
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
        "pushrod command queued: %s unavailable", k_mechanism_transport_send_command_service);
      return;
    }

    auto request = std::make_shared<SendCommandSrv::Request>();
    request->command_id = commandIdForPushrodCommand(*command);
    request->payload.clear();

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
                get_logger(), "pushrod transport send rejected: %s cmd=0x%02X",
                pushrodCommandName(command), command_id);
              return;
            }

            RCLCPP_DEBUG(
              get_logger(), "pushrod transport send accepted: %s cmd=0x%02X seq=%u",
              pushrodCommandName(command), command_id, response->seq);
          } catch (const std::exception & ex) {
            RCLCPP_ERROR(
              get_logger(), "pushrod transport callback failed: %s cmd=0x%02X err=%s",
              pushrodCommandName(command), command_id, ex.what());
          }
        });
    } catch (const std::exception & ex) {
      finishLocalFailure(*command);
      RCLCPP_ERROR(
        get_logger(), "failed to send pushrod transport request: %s cmd=0x%02X err=%s",
        pushrodCommandName(*command), request->command_id, ex.what());
    }
  }

  void finishLocalFailure(PushrodCommand attempted_command)
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

  PushrodDpadLogic logic_;
  std::mutex state_mutex_;
  std::optional<PushrodCommand> queued_command_;
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
    rclcpp::spin(std::make_shared<rc26_telecontrol::PushrodDpadNode>());
  } catch (const std::exception & ex) {
    exit_code = 1;
    RCLCPP_FATAL(
      rclcpp::get_logger("rc26_telecontrol_pushrod_dpad"), "Unhandled exception: %s", ex.what());
  } catch (...) {
    exit_code = 1;
    RCLCPP_FATAL(
      rclcpp::get_logger("rc26_telecontrol_pushrod_dpad"),
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
