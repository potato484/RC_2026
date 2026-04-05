#include <atomic>
#include <chrono>
#include <cstddef>
#include <exception>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "rc26_interfaces/srv/send_mechanism_transport_command.hpp"
#include "rc26_telecontrol/front_track_button_logic.hpp"

namespace rc26_telecontrol
{

namespace
{
constexpr auto k_front_track_send_period = std::chrono::milliseconds(20);
constexpr char k_mechanism_transport_send_command_service[] = "/mechanism/transport/send_command";
}  // namespace

class FrontTrackButtonTestNode : public rclcpp::Node
{
public:
  using SendCommandSrv = rc26_interfaces::srv::SendMechanismTransportCommand;

  FrontTrackButtonTestNode()
  : Node("rc26_telecontrol_front_track_test")
  {
    send_command_client_ = create_client<SendCommandSrv>(k_mechanism_transport_send_command_service);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&FrontTrackButtonTestNode::joyCallback, this, std::placeholders::_1));
    send_timer_ = create_wall_timer(
      k_front_track_send_period, std::bind(&FrontTrackButtonTestNode::sendContinuousCommand, this));
    RCLCPP_INFO(
      get_logger(),
      "front-track button test ready: Y(button[%d])=up, A(button[%d])=down, send_period=%ldms",
      k_front_track_y_button, k_front_track_a_button,
      static_cast<long>(k_front_track_send_period.count()));
  }

private:
  static bool buttonPressed(const sensor_msgs::msg::Joy::ConstSharedPtr & msg, std::size_t index) noexcept
  {
    return msg && index < msg->buttons.size() && msg->buttons[index] != 0;
  }

  void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg)
  {
    const auto command = button_logic_.update(
      buttonPressed(msg, static_cast<std::size_t>(k_front_track_y_button)),
      buttonPressed(msg, static_cast<std::size_t>(k_front_track_a_button)));
    desired_command_.store(command, std::memory_order_relaxed);

    if (command == FrontTrackButtonCommand::kConflict) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ignore front-track command this cycle: Y and A pressed at the same time");
    }
  }

  void sendContinuousCommand()
  {
    const auto desired = desired_command_.load(std::memory_order_relaxed);
    const auto command_id = commandIdForCommand(desired);
    if (!command_id.has_value()) {
      return;
    }

    if (request_in_flight_.exchange(true, std::memory_order_relaxed)) {
      return;
    }

    if (!send_command_client_ || !send_command_client_->service_is_ready()) {
      request_in_flight_.store(false, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "front-track command skipped: %s unavailable", k_mechanism_transport_send_command_service);
      return;
    }

    auto request = std::make_shared<SendCommandSrv::Request>();
    request->command_id = *command_id;
    request->payload.clear();

    try {
      send_command_client_->async_send_request(
        request,
        [this, command_id = *command_id](rclcpp::Client<SendCommandSrv>::SharedFuture future) {
          request_in_flight_.store(false, std::memory_order_relaxed);
          try {
            const auto response = future.get();
            if (!response || !response->accepted) {
              RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "front-track transport send rejected: cmd=0x%02X", command_id);
              return;
            }
            RCLCPP_DEBUG(
              get_logger(), "front-track transport send accepted: cmd=0x%02X seq=%u", command_id,
              response->seq);
          } catch (const std::exception & ex) {
            RCLCPP_ERROR(
              get_logger(), "front-track transport callback failed cmd=0x%02X: %s", command_id,
              ex.what());
          }
        });
    } catch (const std::exception & ex) {
      request_in_flight_.store(false, std::memory_order_relaxed);
      RCLCPP_ERROR(get_logger(), "failed to send front-track transport request cmd=0x%02X: %s",
        *command_id, ex.what());
    }
  }

  FrontTrackButtonLogic button_logic_;
  std::atomic<FrontTrackButtonCommand> desired_command_{FrontTrackButtonCommand::kNone};
  std::atomic<bool> request_in_flight_{false};
  rclcpp::Client<SendCommandSrv>::SharedPtr send_command_client_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr send_timer_;
};

}  // namespace rc26_telecontrol

namespace
{

int runNode(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;

  try {
    rclcpp::spin(std::make_shared<rc26_telecontrol::FrontTrackButtonTestNode>());
  } catch (const std::exception & ex) {
    exit_code = 1;
    RCLCPP_FATAL(
      rclcpp::get_logger("rc26_telecontrol_front_track_test"), "Unhandled exception: %s", ex.what());
  } catch (...) {
    exit_code = 1;
    RCLCPP_FATAL(
      rclcpp::get_logger("rc26_telecontrol_front_track_test"),
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
