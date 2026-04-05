#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "rc26_interfaces/action/execute_mechanism.hpp"
#include "rc26_telecontrol/front_track_button_logic.hpp"

namespace rc26_telecontrol
{

class FrontTrackButtonTestNode : public rclcpp::Node
{
public:
  using ExecuteMechanism = rc26_interfaces::action::ExecuteMechanism;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteMechanism>;

  FrontTrackButtonTestNode()
  : Node("rc26_telecontrol_front_track_test")
  {
    action_client_ = rclcpp_action::create_client<ExecuteMechanism>(this, "/mechanism/execute");
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&FrontTrackButtonTestNode::joyCallback, this, std::placeholders::_1));
    RCLCPP_INFO(
      get_logger(),
      "front-track button test ready: Y(button[%d])=up, A(button[%d])=down",
      k_front_track_y_button, k_front_track_a_button);
  }

private:
  static bool buttonPressed(const sensor_msgs::msg::Joy::ConstSharedPtr & msg, std::size_t index) noexcept
  {
    return msg && index < msg->buttons.size() && msg->buttons[index] != 0;
  }

  void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr msg)
  {
    const auto event = button_logic_.update(
      buttonPressed(msg, static_cast<std::size_t>(k_front_track_y_button)),
      buttonPressed(msg, static_cast<std::size_t>(k_front_track_a_button)),
      goal_in_progress_.load(std::memory_order_relaxed));

    switch (event) {
      case FrontTrackButtonEvent::kNone:
        return;
      case FrontTrackButtonEvent::kConflict:
        RCLCPP_WARN(get_logger(), "ignore front-track trigger: Y and A pressed in the same frame");
        return;
      case FrontTrackButtonEvent::kBusyIgnored:
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "ignore front-track trigger: previous mechanism goal still running");
        return;
      case FrontTrackButtonEvent::kFrontTrackUp:
      case FrontTrackButtonEvent::kFrontTrackDown:
        dispatchGoal(commandIdForEvent(event));
        return;
    }
  }

  void dispatchGoal(uint8_t command_id)
  {
    if (!action_client_->wait_for_action_server(std::chrono::milliseconds(100))) {
      RCLCPP_WARN(
        get_logger(),
        "front-track trigger rejected: /mechanism/execute action server unavailable (cmd=0x%02X)",
        command_id);
      return;
    }

    ExecuteMechanism::Goal goal;
    goal.command_id = command_id;
    goal.payload.clear();
    goal.timeout_sec = k_front_track_timeout_sec;

    goal_in_progress_.store(true, std::memory_order_relaxed);

    rclcpp_action::Client<ExecuteMechanism>::SendGoalOptions options;
    options.goal_response_callback =
      [this, command_id](GoalHandle::SharedPtr goal_handle) {
        if (!goal_handle) {
          goal_in_progress_.store(false, std::memory_order_relaxed);
          RCLCPP_WARN(get_logger(), "front-track goal rejected: cmd=0x%02X", command_id);
          return;
        }
        RCLCPP_INFO(get_logger(), "front-track goal accepted: cmd=0x%02X", command_id);
      };
    options.result_callback =
      [this, command_id](const GoalHandle::WrappedResult & result) {
        goal_in_progress_.store(false, std::memory_order_relaxed);
        const uint16_t error_code = result.result ? result.result->error_code : 0U;
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            if (result.result && result.result->success) {
              RCLCPP_INFO(get_logger(), "front-track command succeeded: cmd=0x%02X", command_id);
            } else {
              RCLCPP_WARN(
                get_logger(), "front-track command failed: cmd=0x%02X error=0x%04X", command_id,
                error_code);
            }
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(
              get_logger(), "front-track command aborted: cmd=0x%02X error=0x%04X", command_id,
              error_code);
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "front-track command canceled: cmd=0x%02X", command_id);
            break;
          default:
            RCLCPP_WARN(get_logger(), "front-track command ended with unknown result: cmd=0x%02X", command_id);
            break;
        }
      };

    try {
      action_client_->async_send_goal(goal, options);
    } catch (const std::exception & ex) {
      goal_in_progress_.store(false, std::memory_order_relaxed);
      RCLCPP_ERROR(get_logger(), "failed to send front-track goal cmd=0x%02X: %s", command_id, ex.what());
    }
  }

  FrontTrackButtonLogic button_logic_;
  std::atomic<bool> goal_in_progress_{false};
  rclcpp_action::Client<ExecuteMechanism>::SharedPtr action_client_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
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
