#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <mutex>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace rc26_telecontrol
{

class TelecontrolNodeBase : public rclcpp::Node
{
public:
  explicit TelecontrolNodeBase(const std::string & node_name, bool use_deadzone_hysteresis);
  ~TelecontrolNodeBase() override;

protected:
  using JoyMsg = sensor_msgs::msg::Joy;
  using JoyMsgConstSharedPtr = JoyMsg::ConstSharedPtr;

  void initialize(const std::string & control_mode_description);

  double linear_speed_limit() const noexcept;
  double angular_speed_limit() const noexcept;
  double joy_deadzone() const noexcept;
  double deadzone_hyst() const noexcept;

  bool button_pressed(const JoyMsgConstSharedPtr & joy_msg, int button_index) const noexcept;
  double axis_value(const JoyMsgConstSharedPtr & joy_msg, std::size_t axis_index) const noexcept;

  virtual geometry_msgs::msg::Twist compute_target_twist(const JoyMsgConstSharedPtr & joy_msg) = 0;
  virtual void reset_control_state() noexcept;

  template<typename T>
  void log_normalized_parameter(const char * name, const T & before, const T & after) const
  {
    if (before == after) {
      return;
    }

    std::ostringstream before_stream;
    before_stream << before;
    std::ostringstream after_stream;
    after_stream << after;
    RCLCPP_WARN(
      get_logger(), "Parameter %s invalid/out of range, normalized from %s to %s.", name,
      before_stream.str().c_str(), after_stream.str().c_str());
  }

private:
  static constexpr double k_nominal_control_dt = 0.02;
  static constexpr double k_min_control_dt = 0.001;
  static constexpr double k_max_control_dt = 0.1;
  static constexpr double k_zero_epsilon = 1e-6;

  void declare_parameters();
  void load_parameters();
  void normalize_parameters();
  void create_interfaces();
  void log_startup() const;
  void joy_callback(JoyMsgConstSharedPtr msg);
  void timer_callback();
  void publish_zero_now() noexcept;
  void clamp_target_twist(geometry_msgs::msg::Twist & twist) const noexcept;
  bool deadman_pressed(const JoyMsgConstSharedPtr & joy_msg) const noexcept;
  double control_dt_seconds() noexcept;

  static bool is_zero_twist(
    const geometry_msgs::msg::Twist & twist,
    double tolerance = k_zero_epsilon) noexcept;
  static double rate_limit(double current, double target, double max_rate, double dt) noexcept;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<JoyMsg>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  const bool use_deadzone_hysteresis_;
  std::string control_mode_description_;
  std::string cmd_vel_topic_{"cmd_vel_teleop"};
  double v_linear_{0.2};
  double v_angular_{0.5};
  double joy_deadzone_{0.15};
  double joy_timeout_s_{0.3};
  double max_accel_{1.5};
  double max_alpha_{3.0};
  int stop_repeat_n_{10};
  bool require_deadman_{false};
  int deadman_button_{4};
  double deadzone_hyst_{0.02};

  JoyMsgConstSharedPtr latest_joy_;
  std::mutex joy_mutex_;
  std::chrono::steady_clock::time_point last_joy_time_{};
  bool joy_ever_received_{false};

  std::chrono::steady_clock::time_point last_control_time_{};
  bool control_time_initialized_{false};

  geometry_msgs::msg::Twist current_output_twist_{};
  int stop_repeat_count_{0};
};

class StickTelecontrolNode final : public TelecontrolNodeBase
{
public:
  StickTelecontrolNode();

protected:
  geometry_msgs::msg::Twist compute_target_twist(const JoyMsgConstSharedPtr & joy_msg) override;
  void reset_control_state() noexcept override;

private:
  double apply_deadzone_hysteresis(double value, std::size_t axis_index) noexcept;

  std::array<bool, 4> axis_active_{};
};

class DpadTelecontrolNode final : public TelecontrolNodeBase
{
public:
  DpadTelecontrolNode();

protected:
  geometry_msgs::msg::Twist compute_target_twist(const JoyMsgConstSharedPtr & joy_msg) override;
};

}  // namespace rc26_telecontrol
