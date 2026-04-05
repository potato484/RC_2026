#pragma once

#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rc26_interfaces/msg/xhu_motion_mode_state.hpp"
#include "rc26_interfaces/srv/set_xhu_motion_mode.hpp"

namespace rc26_nav_mode_manager {

struct XhuProfile {
  std::string name;
  std::string fallback_profile;
  bool require_stopped{false};
  double watchdog_timeout_sec{0.0};
  bool stop_required_on_timeout{false};
  double max_linear_speed{0.4};
  double max_angular_speed{0.6};
  double max_linear_accel{0.4};
  double max_angular_accel{0.6};
};

class XhuMotionModeManager : public rclcpp::Node {
public:
  explicit XhuMotionModeManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  struct VelocitySample {
    rclcpp::Time stamp;
    double linear{0.0};
    double angular{0.0};
  };

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  bool isRobotStopped() const;
  void handleSetMode(
      const rc26_interfaces::srv::SetXhuMotionMode::Request::SharedPtr request,
      rc26_interfaces::srv::SetXhuMotionMode::Response::SharedPtr response);
  void armWatchdogForMode(const std::string &mode, std::optional<double> timeout_override);
  void checkWatchdog();
  void publishState();
  void pruneOdomWindowLocked(const rclcpp::Time &now);

  std::unordered_map<std::string, XhuProfile> profiles_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<rc26_interfaces::msg::XhuMotionModeState>::SharedPtr mode_state_pub_;
  rclcpp::Service<rc26_interfaces::srv::SetXhuMotionMode>::SharedPtr set_mode_srv_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  mutable std::mutex odom_mutex_;
  mutable std::mutex state_mutex_;
  std::deque<VelocitySample> odom_window_;
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  std::optional<rclcpp::Time> watchdog_deadline_;

  std::string current_mode_;
  std::string current_reason_;
  bool stop_required_{false};
  bool timed_out_{false};

  double stop_linear_eps_mps_{0.05};
  double stop_angular_eps_rps_{0.05};
  double odom_stale_timeout_sec_{0.25};
  double stop_window_sec_{0.35};
};

}  // namespace rc26_nav_mode_manager
