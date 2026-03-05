#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <string>

class rc26_telecontrol : public rclcpp::Node
{
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

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

  sensor_msgs::msg::Joy::SharedPtr latest_joy_;
  std::mutex joy_mutex_;
  std::chrono::steady_clock::time_point last_joy_time_{};
  bool joy_ever_received_{false};

  geometry_msgs::msg::Twist current_output_twist_{};
  int stop_repeat_count_{0};
  bool axis_active_[4]{false, false, false, false};

  static constexpr double EPS = 1e-6;

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joy_mutex_);
    latest_joy_ = msg;
    last_joy_time_ = std::chrono::steady_clock::now();
    joy_ever_received_ = true;
  }

  static bool is_zero_twist(const geometry_msgs::msg::Twist & t, double tol = EPS)
  {
    return std::abs(t.linear.x) < tol && std::abs(t.linear.y) < tol && std::abs(t.angular.z) < tol;
  }

  static double rate_limit(double cur, double tgt, double max_rate, double dt)
  {
    return cur + std::clamp(tgt - cur, -max_rate * dt, max_rate * dt);
  }

  double apply_deadzone_hysteresis(double value, int axis_idx)
  {
    if (axis_idx < 0 || axis_idx >= 4) {
      return 0.0;
    }

    const double enter = joy_deadzone_;
    const double exit = std::max(joy_deadzone_ - deadzone_hyst_, 0.0);
    bool & active = axis_active_[axis_idx];

    if (!active && std::abs(value) >= enter) {
      active = true;
    }
    if (active && std::abs(value) < exit) {
      active = false;
    }
    if (!active) {
      return 0.0;
    }

    if (enter >= 1.0) {
      return 0.0;
    }
    return std::copysign((std::abs(value) - enter) / (1.0 - enter), value);
  }

  void timer_callback()
  {
    sensor_msgs::msg::Joy::SharedPtr joy_copy;
    std::chrono::steady_clock::time_point last_joy_time_copy{};
    bool joy_ever_received_copy = false;
    {
      std::lock_guard<std::mutex> lock(joy_mutex_);
      joy_copy = latest_joy_;
      last_joy_time_copy = last_joy_time_;
      joy_ever_received_copy = joy_ever_received_;
    }

    if (joy_timeout_s_ > 0.0 && joy_ever_received_copy) {
      const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_joy_time_copy).count();
      if (elapsed > joy_timeout_s_) {
        current_output_twist_ = geometry_msgs::msg::Twist{};
        pub_->publish(current_output_twist_);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "Joy timeout (%.2fs), holding zero.", elapsed);
        return;
      }
    }

    geometry_msgs::msg::Twist target_twist{};
    if (joy_copy) {
      if (joy_copy->axes.size() > 1) {
        const double left_stick_forward = apply_deadzone_hysteresis(joy_copy->axes[1], 1);
        target_twist.linear.x = left_stick_forward * v_linear_;
      }
      if (joy_copy->axes.size() > 0) {
        const double left_stick_right = apply_deadzone_hysteresis(joy_copy->axes[0], 0);
        target_twist.linear.y = left_stick_right * v_linear_;
      }
      if (joy_copy->axes.size() > 3) {
        const double right_stick_left = apply_deadzone_hysteresis(joy_copy->axes[3], 3);
        target_twist.angular.z = right_stick_left * v_angular_;
      }
    }

    if (require_deadman_) {
      const bool pressed =
        joy_copy && deadman_button_ >= 0 &&
        deadman_button_ < static_cast<int>(joy_copy->buttons.size()) &&
        joy_copy->buttons[deadman_button_] != 0;
      if (!pressed) {
        target_twist = geometry_msgs::msg::Twist{};
      }
    }

    target_twist.linear.x = std::clamp(target_twist.linear.x, -v_linear_, v_linear_);
    target_twist.linear.y = std::clamp(target_twist.linear.y, -v_linear_, v_linear_);
    target_twist.angular.z = std::clamp(target_twist.angular.z, -v_angular_, v_angular_);

    constexpr double DT = 0.02;
    current_output_twist_.linear.x =
      rate_limit(current_output_twist_.linear.x, target_twist.linear.x, max_accel_, DT);
    current_output_twist_.linear.y =
      rate_limit(current_output_twist_.linear.y, target_twist.linear.y, max_accel_, DT);
    current_output_twist_.angular.z =
      rate_limit(current_output_twist_.angular.z, target_twist.angular.z, max_alpha_, DT);

    if (is_zero_twist(current_output_twist_)) {
      if (stop_repeat_count_ < stop_repeat_n_) {
        pub_->publish(current_output_twist_);
        ++stop_repeat_count_;
      }
    } else {
      stop_repeat_count_ = 0;
      pub_->publish(current_output_twist_);
    }
  }

public:
  rc26_telecontrol()
  : Node("rc26_telecontrol")
  {
    declare_parameter<std::string>("cmd_vel_topic", "cmd_vel_teleop");
    declare_parameter<double>("v_linear", 0.2);
    declare_parameter<double>("v_angular", 0.5);
    declare_parameter<double>("joy_deadzone", 0.15);
    declare_parameter<double>("joy_timeout_s", 0.3);
    declare_parameter<double>("max_accel", 1.5);
    declare_parameter<double>("max_alpha", 3.0);
    declare_parameter<int>("stop_repeat_n", 10);
    declare_parameter<bool>("require_deadman", false);
    declare_parameter<int>("deadman_button", 4);
    declare_parameter<double>("deadzone_hyst", 0.02);

    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    get_parameter("v_linear", v_linear_);
    get_parameter("v_angular", v_angular_);
    get_parameter("joy_deadzone", joy_deadzone_);
    get_parameter("joy_timeout_s", joy_timeout_s_);
    get_parameter("max_accel", max_accel_);
    get_parameter("max_alpha", max_alpha_);
    get_parameter("stop_repeat_n", stop_repeat_n_);
    get_parameter("require_deadman", require_deadman_);
    get_parameter("deadman_button", deadman_button_);
    get_parameter("deadzone_hyst", deadzone_hyst_);

    joy_deadzone_ = std::clamp(joy_deadzone_, 0.0, 0.95);
    deadzone_hyst_ = std::clamp(deadzone_hyst_, 0.0, joy_deadzone_);
    max_accel_ = std::max(max_accel_, 0.0);
    max_alpha_ = std::max(max_alpha_, 0.0);
    stop_repeat_n_ = std::max(stop_repeat_n_, 0);
    deadman_button_ = std::max(deadman_button_, 0);

    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, pub_qos);

    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", sub_qos, std::bind(&rc26_telecontrol::joy_callback, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&rc26_telecontrol::timer_callback, this));

    RCLCPP_INFO(get_logger(), "rc26_telecontrol (dual-stick joystick) ready.");
    RCLCPP_INFO(
      get_logger(),
      "v_linear=%.2f, v_angular=%.2f, deadzone=%.2f, deadzone_hyst=%.2f, timeout=%.2fs",
      v_linear_, v_angular_, joy_deadzone_, deadzone_hyst_, joy_timeout_s_);
    RCLCPP_INFO(
      get_logger(),
      "max_accel=%.2f, max_alpha=%.2f, stop_repeat_n=%d, require_deadman=%s, deadman_button=%d",
      max_accel_, max_alpha_, stop_repeat_n_, require_deadman_ ? "true" : "false", deadman_button_);
    RCLCPP_INFO(get_logger(), "publish cmd_vel topic: %s", cmd_vel_topic_.c_str());
  }

  ~rc26_telecontrol()
  {
    pub_->publish(geometry_msgs::msg::Twist{});
    RCLCPP_INFO(get_logger(), "rc26_telecontrol stopped.");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rc26_telecontrol>());
  rclcpp::shutdown();
  return 0;
}
