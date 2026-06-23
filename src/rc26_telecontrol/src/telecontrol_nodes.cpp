#include "rc26_telecontrol/telecontrol_nodes.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <utility>

namespace rc26_telecontrol
{

namespace
{
constexpr std::chrono::milliseconds k_control_period{20};
constexpr std::size_t k_left_stick_x_axis = 0;
constexpr std::size_t k_left_stick_y_axis = 1;
constexpr std::size_t k_right_stick_yaw_axis = 3;
constexpr std::size_t k_dpad_x_axis = 6;
constexpr std::size_t k_dpad_y_axis = 7;
constexpr int k_x_button = 2;
constexpr int k_b_button = 1;
}  // namespace

TelecontrolNodeBase::TelecontrolNodeBase(
  const std::string & node_name,
  bool use_deadzone_hysteresis)
: Node(node_name),
  use_deadzone_hysteresis_(use_deadzone_hysteresis)
{
}

TelecontrolNodeBase::~TelecontrolNodeBase()
{
  try {
    publish_zero_now();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Failed to publish stop command during shutdown: %s", ex.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Failed to publish stop command during shutdown: unknown error.");
  }
}

void TelecontrolNodeBase::initialize(const std::string & control_mode_description)
{
  control_mode_description_ = control_mode_description;
  declare_parameters();
  load_parameters();
  normalize_parameters();
  create_interfaces();
  log_startup();
}

double TelecontrolNodeBase::linear_speed_limit() const noexcept
{
  return v_linear_;
}

double TelecontrolNodeBase::angular_speed_limit() const noexcept
{
  return v_angular_;
}

double TelecontrolNodeBase::joy_deadzone() const noexcept
{
  return joy_deadzone_;
}

double TelecontrolNodeBase::deadzone_hyst() const noexcept
{
  return deadzone_hyst_;
}

bool TelecontrolNodeBase::button_pressed(
  const JoyMsgConstSharedPtr & joy_msg,
  int button_index) const noexcept
{
  if (!joy_msg || button_index < 0) {
    return false;
  }

  return static_cast<std::size_t>(button_index) < joy_msg->buttons.size() &&
         joy_msg->buttons[static_cast<std::size_t>(button_index)] != 0;
}

double TelecontrolNodeBase::axis_value(
  const JoyMsgConstSharedPtr & joy_msg,
  std::size_t axis_index) const noexcept
{
  if (!joy_msg || axis_index >= joy_msg->axes.size()) {
    return 0.0;
  }

  const double raw_value = joy_msg->axes[axis_index];
  if (!std::isfinite(raw_value)) {
    return 0.0;
  }

  return std::clamp(raw_value, -1.0, 1.0);
}

void TelecontrolNodeBase::reset_control_state() noexcept
{
}

void TelecontrolNodeBase::declare_parameters()
{
  declare_parameter<std::string>("cmd_vel_topic", cmd_vel_topic_);
  declare_parameter<double>("v_linear", v_linear_);
  declare_parameter<double>("v_angular", v_angular_);
  if (use_deadzone_hysteresis_) {
    declare_parameter<double>("joy_deadzone", joy_deadzone_);
    declare_parameter<double>("deadzone_hyst", deadzone_hyst_);
  }
  declare_parameter<double>("joy_timeout_s", joy_timeout_s_);
  declare_parameter<double>("max_accel", max_accel_);
  declare_parameter<double>("max_alpha", max_alpha_);
  declare_parameter<int>("stop_repeat_n", stop_repeat_n_);
  declare_parameter<bool>("require_deadman", require_deadman_);
  declare_parameter<int>("deadman_button", deadman_button_);
}

void TelecontrolNodeBase::load_parameters()
{
  get_parameter("cmd_vel_topic", cmd_vel_topic_);
  get_parameter("v_linear", v_linear_);
  get_parameter("v_angular", v_angular_);
  if (use_deadzone_hysteresis_) {
    get_parameter("joy_deadzone", joy_deadzone_);
    get_parameter("deadzone_hyst", deadzone_hyst_);
  }
  get_parameter("joy_timeout_s", joy_timeout_s_);
  get_parameter("max_accel", max_accel_);
  get_parameter("max_alpha", max_alpha_);
  get_parameter("stop_repeat_n", stop_repeat_n_);
  get_parameter("require_deadman", require_deadman_);
  get_parameter("deadman_button", deadman_button_);
}

void TelecontrolNodeBase::normalize_parameters()
{
  const auto normalize_abs_double = [this](const char * name, double & value, double fallback) {
      const double before = value;
      if (!std::isfinite(value)) {
        value = fallback;
      } else {
        value = std::fabs(value);
      }
      log_normalized_parameter(name, before, value);
    };

  const auto normalize_positive_double = [this](const char * name, double & value, double fallback) {
      const double before = value;
      if (!std::isfinite(value) || value <= 0.0) {
        value = fallback;
      } else {
        value = std::fabs(value);
      }
      log_normalized_parameter(name, before, value);
    };

  const auto normalize_bounded_double =
    [this](const char * name, double & value, double fallback, double low, double high) {
      const double before = value;
      if (!std::isfinite(value)) {
        value = fallback;
      } else {
        value = std::clamp(value, low, high);
      }
      log_normalized_parameter(name, before, value);
    };

  if (cmd_vel_topic_.empty()) {
    const std::string before = std::move(cmd_vel_topic_);
    cmd_vel_topic_ = "cmd_vel_teleop";
    log_normalized_parameter("cmd_vel_topic", before, cmd_vel_topic_);
  }

  normalize_abs_double("v_linear", v_linear_, 2.0);
  normalize_abs_double("v_angular", v_angular_, 2.0);
  normalize_positive_double("max_accel", max_accel_, 1.5);
  normalize_positive_double("max_alpha", max_alpha_, 3.0);

  const double timeout_before = joy_timeout_s_;
  if (!std::isfinite(joy_timeout_s_)) {
    joy_timeout_s_ = 0.3;
  } else if (joy_timeout_s_ <= 0.0) {
    joy_timeout_s_ = 0.0;
  }
  log_normalized_parameter("joy_timeout_s", timeout_before, joy_timeout_s_);

  const int stop_repeat_before = stop_repeat_n_;
  stop_repeat_n_ = std::max(stop_repeat_n_, 0);
  log_normalized_parameter("stop_repeat_n", stop_repeat_before, stop_repeat_n_);

  const int deadman_button_before = deadman_button_;
  deadman_button_ = std::max(deadman_button_, 0);
  log_normalized_parameter("deadman_button", deadman_button_before, deadman_button_);

  if (use_deadzone_hysteresis_) {
    normalize_bounded_double("joy_deadzone", joy_deadzone_, 0.15, 0.0, 0.95);
    normalize_bounded_double("deadzone_hyst", deadzone_hyst_, 0.02, 0.0, joy_deadzone_);
  } else {
    deadzone_hyst_ = 0.0;
  }
}

void TelecontrolNodeBase::create_interfaces()
{
  auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, pub_qos);

  auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  joy_sub_ = create_subscription<JoyMsg>(
    "/joy", sub_qos,
    std::bind(&TelecontrolNodeBase::joy_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(k_control_period, std::bind(&TelecontrolNodeBase::timer_callback, this));
}

void TelecontrolNodeBase::log_startup() const
{
  RCLCPP_INFO(get_logger(), "%s ready.", control_mode_description_.c_str());
  if (use_deadzone_hysteresis_) {
    RCLCPP_INFO(
      get_logger(),
      "v_linear=%.2f, v_angular=%.2f, deadzone=%.2f, deadzone_hyst=%.2f, timeout=%.2fs",
      v_linear_, v_angular_, joy_deadzone_, deadzone_hyst_, joy_timeout_s_);
  } else {
    RCLCPP_INFO(
      get_logger(), "v_linear=%.2f, v_angular=%.2f, timeout=%.2fs", v_linear_, v_angular_,
      joy_timeout_s_);
  }
  RCLCPP_INFO(
    get_logger(),
    "max_accel=%.2f, max_alpha=%.2f, stop_repeat_n=%d, require_deadman=%s, deadman_button=%d",
    max_accel_, max_alpha_, stop_repeat_n_, require_deadman_ ? "true" : "false",
    deadman_button_);
  RCLCPP_INFO(get_logger(), "publish cmd_vel topic: %s", cmd_vel_topic_.c_str());
  if (joy_timeout_s_ == 0.0) {
    RCLCPP_WARN(get_logger(), "Watchdog disabled by joy_timeout_s<=0.");
  }
}

void TelecontrolNodeBase::joy_callback(JoyMsgConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(joy_mutex_);
  latest_joy_ = std::move(msg);
  last_joy_time_ = std::chrono::steady_clock::now();
  joy_ever_received_ = true;
}

void TelecontrolNodeBase::timer_callback()
{
  JoyMsgConstSharedPtr joy_copy;
  std::chrono::steady_clock::time_point last_joy_time_copy{};
  bool joy_ever_received_copy = false;

  {
    std::lock_guard<std::mutex> lock(joy_mutex_);
    joy_copy = latest_joy_;
    last_joy_time_copy = last_joy_time_;
    joy_ever_received_copy = joy_ever_received_;
  }

  if (joy_timeout_s_ > 0.0 && joy_ever_received_copy) {
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_joy_time_copy).count();
    if (elapsed > joy_timeout_s_) {
      control_time_initialized_ = false;
      reset_control_state();
      publish_zero_now();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Joy timeout (%.2fs), holding zero.", elapsed);
      return;
    }
  }

  geometry_msgs::msg::Twist target_twist{};
  if (joy_copy) {
    target_twist = compute_target_twist(joy_copy);
  } else {
    reset_control_state();
  }

  if (require_deadman_ && !deadman_pressed(joy_copy)) {
    control_time_initialized_ = false;
    reset_control_state();
    publish_zero_now();
    return;
  }

  clamp_target_twist(target_twist);

  const double dt = control_dt_seconds();
  current_output_twist_.linear.x =
    rate_limit(current_output_twist_.linear.x, target_twist.linear.x, max_accel_, dt);
  current_output_twist_.linear.y =
    rate_limit(current_output_twist_.linear.y, target_twist.linear.y, max_accel_, dt);
  current_output_twist_.angular.z =
    rate_limit(current_output_twist_.angular.z, target_twist.angular.z, max_alpha_, dt);

  if (is_zero_twist(current_output_twist_)) {
    if (stop_repeat_count_ < stop_repeat_n_) {
      pub_->publish(current_output_twist_);
      ++stop_repeat_count_;
    }
    return;
  }

  stop_repeat_count_ = 0;
  pub_->publish(current_output_twist_);
}

void TelecontrolNodeBase::publish_zero_now() noexcept
{
  current_output_twist_ = geometry_msgs::msg::Twist{};
  stop_repeat_count_ = 0;
  if (pub_ && rclcpp::ok()) {
    pub_->publish(current_output_twist_);
  }
}

void TelecontrolNodeBase::clamp_target_twist(geometry_msgs::msg::Twist & twist) const noexcept
{
  twist.linear.x = std::clamp(twist.linear.x, -v_linear_, v_linear_);
  twist.linear.y = std::clamp(twist.linear.y, -v_linear_, v_linear_);
  twist.angular.z = std::clamp(twist.angular.z, -v_angular_, v_angular_);
}

bool TelecontrolNodeBase::deadman_pressed(const JoyMsgConstSharedPtr & joy_msg) const noexcept
{
  return button_pressed(joy_msg, deadman_button_);
}

double TelecontrolNodeBase::control_dt_seconds() noexcept
{
  const auto now = std::chrono::steady_clock::now();
  if (!control_time_initialized_) {
    last_control_time_ = now;
    control_time_initialized_ = true;
    return k_nominal_control_dt;
  }

  const double dt = std::chrono::duration<double>(now - last_control_time_).count();
  last_control_time_ = now;
  if (!std::isfinite(dt) || dt <= 0.0) {
    return k_nominal_control_dt;
  }

  return std::clamp(dt, k_min_control_dt, k_max_control_dt);
}

bool TelecontrolNodeBase::is_zero_twist(
  const geometry_msgs::msg::Twist & twist,
  double tolerance) noexcept
{
  return std::abs(twist.linear.x) < tolerance && std::abs(twist.linear.y) < tolerance &&
         std::abs(twist.angular.z) < tolerance;
}

double TelecontrolNodeBase::rate_limit(
  double current,
  double target,
  double max_rate,
  double dt) noexcept
{
  return current + std::clamp(target - current, -max_rate * dt, max_rate * dt);
}

StickTelecontrolNode::StickTelecontrolNode()
: TelecontrolNodeBase("rc26_telecontrol", true)
{
  initialize("rc26_telecontrol (dual-stick joystick)");
}

geometry_msgs::msg::Twist StickTelecontrolNode::compute_target_twist(
  const JoyMsgConstSharedPtr & joy_msg)
{
  geometry_msgs::msg::Twist target_twist{};
  target_twist.linear.x =
    apply_deadzone_hysteresis(axis_value(joy_msg, k_left_stick_y_axis), k_left_stick_y_axis) *
    linear_speed_limit();
  target_twist.linear.y =
    apply_deadzone_hysteresis(axis_value(joy_msg, k_left_stick_x_axis), k_left_stick_x_axis) *
    linear_speed_limit();
  target_twist.angular.z =
    -apply_deadzone_hysteresis(axis_value(joy_msg, k_right_stick_yaw_axis), k_right_stick_yaw_axis) *
    angular_speed_limit();
  return target_twist;
}

void StickTelecontrolNode::reset_control_state() noexcept
{
  axis_active_.fill(false);
}

double StickTelecontrolNode::apply_deadzone_hysteresis(double value, std::size_t axis_index) noexcept
{
  if (axis_index >= axis_active_.size()) {
    return 0.0;
  }

  const double magnitude = std::abs(value);
  const double enter = joy_deadzone();
  const double exit = std::max(joy_deadzone() - deadzone_hyst(), 0.0);
  bool & active = axis_active_[axis_index];

  if (!active && magnitude >= enter) {
    active = true;
  } else if (active && magnitude < exit) {
    active = false;
  }

  if (!active) {
    return 0.0;
  }

  const double active_threshold = exit;
  const double denominator = std::max(1.0 - active_threshold, 1e-6);
  const double scaled = std::clamp((magnitude - active_threshold) / denominator, 0.0, 1.0);
  return std::copysign(scaled, value);
}

DpadTelecontrolNode::DpadTelecontrolNode()
: TelecontrolNodeBase("rc26_telecontrol_dpad", false)
{
  initialize("rc26_telecontrol_dpad (D-Pad + Button control)");
}

geometry_msgs::msg::Twist DpadTelecontrolNode::compute_target_twist(
  const JoyMsgConstSharedPtr & joy_msg)
{
  geometry_msgs::msg::Twist target_twist{};
  target_twist.linear.x = axis_value(joy_msg, k_dpad_y_axis) * linear_speed_limit();
  target_twist.linear.y = axis_value(joy_msg, k_dpad_x_axis) * linear_speed_limit();

  const bool x_pressed = button_pressed(joy_msg, k_x_button);
  const bool b_pressed = button_pressed(joy_msg, k_b_button);
  if (x_pressed && !b_pressed) {
    target_twist.angular.z = angular_speed_limit();
  } else if (b_pressed && !x_pressed) {
    target_twist.angular.z = -angular_speed_limit();
  }

  return target_twist;
}

}  // namespace rc26_telecontrol
