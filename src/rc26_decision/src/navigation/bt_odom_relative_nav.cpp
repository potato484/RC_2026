#include "rc26_decision/navigation/bt_odom_relative_nav.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include "rc26_decision/decision_failure.hpp"

namespace rc26_decision {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDefaultOdomTimeoutSec = 0.5;
constexpr double kDefaultRelativeYawCaptureTimeoutSec = 2.0;
constexpr double kDefaultOdomRelativeMaxSpeedMps = 0.20;
constexpr double kDefaultOdomRelativeMinSpeedMps = 0.03;
constexpr double kDefaultOdomRelativeXyKp = 0.8;
constexpr double kDefaultOdomRelativeHeadingKp = 1.2;
constexpr double kDefaultOdomRelativeHeadingMaxSpeedRadps = 0.30;
constexpr double kDefaultOdomRelativeXyToleranceM = 0.03;
constexpr double kDefaultOdomRelativeYawToleranceDeg = 3.0;
constexpr int kDefaultOdomRelativeStableTicks = 3;
constexpr double kDefaultOdomRelativeTimeoutSec = 10.0;

double normalizeAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double finitePositiveOr(double value, double fallback) {
  return (std::isfinite(value) && value > 0.0) ? value : fallback;
}

double finiteNonNegativeOr(double value, double fallback) {
  return (std::isfinite(value) && value >= 0.0) ? value : fallback;
}

std::string nonEmptyOr(std::string value, const char *fallback) {
  return value.empty() ? std::string(fallback) : value;
}

void resetRelativeNavBlackboard(const BT::Blackboard::Ptr &blackboard,
                                const std::string &state) {
  if (!blackboard) {
    return;
  }
  blackboard->set("relative_nav_last_exec_state", state);
  blackboard->set("relative_nav_last_failure_reason", std::string(""));
  blackboard->set("relative_nav_last_distance_remaining", 0.0);
}

} // namespace

bool odomAxisDriveReachedOrOvershot(double distance_m, double axis_remaining_m,
                                    double xy_tolerance_m,
                                    bool succeed_on_reach_or_overshoot) {
  if (!succeed_on_reach_or_overshoot || !std::isfinite(distance_m) ||
      !std::isfinite(axis_remaining_m) || !std::isfinite(xy_tolerance_m)) {
    return false;
  }
  const double tolerance = std::abs(xy_tolerance_m);
  if (std::abs(axis_remaining_m) <= tolerance) {
    return true;
  }
  if (std::abs(distance_m) <= tolerance) {
    return true;
  }
  return distance_m * axis_remaining_m <= 0.0;
}

void loadOdomRelativeNavParams(rclcpp::Node &node,
                               const BT::Blackboard::Ptr &blackboard) {
  std::string cmd_vel_topic = node.declare_parameter<std::string>(
      "odom_relative_nav_cmd_vel_topic", "cmd_vel");
  std::string odom_topic = node.declare_parameter<std::string>(
      "odom_relative_nav_odom_topic", "odom");
  double max_speed_mps = node.declare_parameter<double>(
      "odom_relative_nav_max_speed_mps", kDefaultOdomRelativeMaxSpeedMps);
  double min_speed_mps = node.declare_parameter<double>(
      "odom_relative_nav_min_speed_mps", kDefaultOdomRelativeMinSpeedMps);
  double xy_kp = node.declare_parameter<double>(
      "odom_relative_nav_xy_kp", kDefaultOdomRelativeXyKp);
  double heading_kp = node.declare_parameter<double>(
      "odom_relative_nav_heading_kp", kDefaultOdomRelativeHeadingKp);
  double heading_max_speed_radps = node.declare_parameter<double>(
      "odom_relative_nav_heading_max_speed_radps",
      kDefaultOdomRelativeHeadingMaxSpeedRadps);
  double xy_tolerance_m = node.declare_parameter<double>(
      "odom_relative_nav_xy_tolerance_m", kDefaultOdomRelativeXyToleranceM);
  double yaw_tolerance_deg = node.declare_parameter<double>(
      "odom_relative_nav_yaw_tolerance_deg", kDefaultOdomRelativeYawToleranceDeg);
  int stable_ticks = node.declare_parameter<int>(
      "odom_relative_nav_stable_ticks", kDefaultOdomRelativeStableTicks);
  double odom_timeout_s = node.declare_parameter<double>(
      "odom_relative_nav_odom_timeout_s", kDefaultOdomTimeoutSec);
  double timeout_s = node.declare_parameter<double>(
      "odom_relative_nav_timeout_s", kDefaultOdomRelativeTimeoutSec);

  cmd_vel_topic = nonEmptyOr(cmd_vel_topic, "cmd_vel");
  odom_topic = nonEmptyOr(odom_topic, "odom");
  max_speed_mps =
      std::abs(finitePositiveOr(max_speed_mps, kDefaultOdomRelativeMaxSpeedMps));
  min_speed_mps = std::min(
      std::abs(finiteNonNegativeOr(min_speed_mps,
                                   kDefaultOdomRelativeMinSpeedMps)),
      max_speed_mps);
  xy_kp = finitePositiveOr(xy_kp, kDefaultOdomRelativeXyKp);
  heading_kp = finiteNonNegativeOr(heading_kp, kDefaultOdomRelativeHeadingKp);
  heading_max_speed_radps = std::abs(finiteNonNegativeOr(
      heading_max_speed_radps, kDefaultOdomRelativeHeadingMaxSpeedRadps));
  xy_tolerance_m =
      finitePositiveOr(xy_tolerance_m, kDefaultOdomRelativeXyToleranceM);
  yaw_tolerance_deg =
      finiteNonNegativeOr(yaw_tolerance_deg, kDefaultOdomRelativeYawToleranceDeg);
  stable_ticks = std::max(1, stable_ticks);
  odom_timeout_s = finitePositiveOr(odom_timeout_s, kDefaultOdomTimeoutSec);
  timeout_s = finitePositiveOr(timeout_s, kDefaultOdomRelativeTimeoutSec);

  blackboard->set("odom_relative_nav_cmd_vel_topic", cmd_vel_topic);
  blackboard->set("odom_relative_nav_odom_topic", odom_topic);
  blackboard->set("odom_relative_nav_max_speed_mps", max_speed_mps);
  blackboard->set("odom_relative_nav_min_speed_mps", min_speed_mps);
  blackboard->set("odom_relative_nav_xy_kp", xy_kp);
  blackboard->set("odom_relative_nav_heading_kp", heading_kp);
  blackboard->set("odom_relative_nav_heading_max_speed_radps",
                  heading_max_speed_radps);
  blackboard->set("odom_relative_nav_xy_tolerance_m", xy_tolerance_m);
  blackboard->set("odom_relative_nav_yaw_tolerance_deg", yaw_tolerance_deg);
  blackboard->set("odom_relative_nav_stable_ticks", stable_ticks);
  blackboard->set("odom_relative_nav_odom_timeout_s", odom_timeout_s);
  blackboard->set("odom_relative_nav_timeout_s", timeout_s);
  resetRelativeNavBlackboard(blackboard, "IDLE");

  RCLCPP_INFO(node.get_logger(),
              "odom 单轴分段导航参数已加载: cmd_vel=%s odom=%s max=%.2fm/s xy_tol=%.3fm yaw_tol=%.1fdeg timeout=%.1fs",
              cmd_vel_topic.c_str(), odom_topic.c_str(), max_speed_mps,
              xy_tolerance_m, yaw_tolerance_deg, timeout_s);
}

OdomAxisDriveAction::OdomAxisDriveAction(const std::string &name,
                                         const BT::NodeConfig &config,
                                         Axis axis,
                                         const char *action_label)
    : BT::StatefulActionNode(name, config), axis_(axis),
      action_label_(action_label ? action_label : "odom axis drive") {}

BT::PortsList OdomAxisDriveAction::providedPorts() {
  return {
      BT::InputPort<std::string>("cmd_vel_topic", "cmd_vel",
                                 "Velocity command topic"),
      BT::InputPort<std::string>("odom_topic", "odom", "Odometry topic"),
      BT::InputPort<double>("distance_m",
                            "Signed relative displacement along this axis"),
      BT::InputPort<double>("max_speed_mps", kDefaultOdomRelativeMaxSpeedMps,
                            "Maximum axis speed in m/s"),
      BT::InputPort<double>("min_speed_mps", kDefaultOdomRelativeMinSpeedMps,
                            "Minimum axis speed before position tolerance"),
      BT::InputPort<double>("xy_kp", kDefaultOdomRelativeXyKp,
                            "Axis position error to speed gain"),
      BT::InputPort<double>("heading_kp", kDefaultOdomRelativeHeadingKp,
                            "Yaw error to angular.z gain"),
      BT::InputPort<double>("heading_max_speed_radps",
                            kDefaultOdomRelativeHeadingMaxSpeedRadps,
                            "Maximum heading correction angular speed"),
      BT::InputPort<double>("xy_tolerance_m", kDefaultOdomRelativeXyToleranceM,
                            "Position success tolerance in meters"),
      BT::InputPort<double>("yaw_tolerance_deg",
                            kDefaultOdomRelativeYawToleranceDeg,
                            "Yaw hold success tolerance in degrees"),
      BT::InputPort<int>("stable_ticks", kDefaultOdomRelativeStableTicks,
                         "Consecutive in-tolerance ticks required"),
      BT::InputPort<double>("odom_timeout_s", kDefaultOdomTimeoutSec,
                            "Maximum accepted odom age"),
      BT::InputPort<double>("timeout_s", kDefaultOdomRelativeTimeoutSec,
                            "Action timeout in seconds"),
      BT::InputPort<bool>(
          "succeed_on_reach_or_overshoot", false,
          "Stop and succeed when the axis target is reached or overshot"),
      BT::InputPort<bool>(
          "succeed_on_timeout", false,
          "Stop and succeed on action timeout after capturing an odom target"),
  };
}

BT::NodeStatus OdomAxisDriveAction::onStart() {
  if (!config().blackboard || !config().blackboard->get("node", node_) ||
      !node_) {
    writeDecisionFailure(config().blackboard, action_label_,
                         "运行上下文缺失：blackboard 或 node 不可用");
    return BT::NodeStatus::FAILURE;
  }

  (void)getInput("cmd_vel_topic", cmd_vel_topic_);
  (void)getInput("odom_topic", odom_topic_);
  cmd_vel_topic_ = nonEmptyOr(cmd_vel_topic_, "cmd_vel");
  odom_topic_ = nonEmptyOr(odom_topic_, "odom");
  cmd_pub_ =
      node_->create_publisher<TwistMsg>(cmd_vel_topic_, rclcpp::QoS(10));

  if (!getInput("distance_m", distance_m_) || !std::isfinite(distance_m_)) {
    return failWithStop("相对单轴距离 distance_m 非法");
  }
  (void)getInput("max_speed_mps", max_speed_mps_);
  (void)getInput("min_speed_mps", min_speed_mps_);
  (void)getInput("xy_kp", xy_kp_);
  (void)getInput("heading_kp", heading_kp_);
  (void)getInput("heading_max_speed_radps", heading_max_speed_radps_);
  (void)getInput("xy_tolerance_m", xy_tolerance_m_);
  double yaw_tolerance_deg = kDefaultOdomRelativeYawToleranceDeg;
  (void)getInput("yaw_tolerance_deg", yaw_tolerance_deg);
  (void)getInput("stable_ticks", stable_ticks_required_);
  (void)getInput("odom_timeout_s", odom_timeout_s_);
  (void)getInput("timeout_s", timeout_s_);
  succeed_on_reach_or_overshoot_ = false;
  (void)getInput("succeed_on_reach_or_overshoot",
                 succeed_on_reach_or_overshoot_);
  succeed_on_timeout_ = false;
  (void)getInput("succeed_on_timeout", succeed_on_timeout_);

  max_speed_mps_ =
      std::abs(finitePositiveOr(max_speed_mps_, kDefaultOdomRelativeMaxSpeedMps));
  min_speed_mps_ = std::min(
      std::abs(finiteNonNegativeOr(min_speed_mps_,
                                   kDefaultOdomRelativeMinSpeedMps)),
      max_speed_mps_);
  xy_kp_ = finitePositiveOr(xy_kp_, kDefaultOdomRelativeXyKp);
  heading_kp_ =
      finiteNonNegativeOr(heading_kp_, kDefaultOdomRelativeHeadingKp);
  heading_max_speed_radps_ = std::abs(finiteNonNegativeOr(
      heading_max_speed_radps_, kDefaultOdomRelativeHeadingMaxSpeedRadps));
  xy_tolerance_m_ =
      finitePositiveOr(xy_tolerance_m_, kDefaultOdomRelativeXyToleranceM);
  yaw_tolerance_deg = finiteNonNegativeOr(
      yaw_tolerance_deg, kDefaultOdomRelativeYawToleranceDeg);
  yaw_tolerance_rad_ = std::abs(yaw_tolerance_deg) * kPi / 180.0;
  stable_ticks_required_ = std::max(1, stable_ticks_required_);
  odom_timeout_s_ = finitePositiveOr(odom_timeout_s_, kDefaultOdomTimeoutSec);
  timeout_s_ = finitePositiveOr(timeout_s_, kDefaultOdomRelativeTimeoutSec);

  odom_sub_ = node_->create_subscription<OdomMsg>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const OdomMsg::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
        has_odom_ = true;
        last_odom_tp_ = std::chrono::steady_clock::now();
      });

  has_odom_ = false;
  target_ready_ = false;
  stable_ticks_ = 0;
  start_time_ = node_->now();
  writeState("RUNNING");
  writeDistanceRemaining(std::abs(distance_m_));
  RCLCPP_INFO(node_->get_logger(),
              "%s 启动: cmd_vel=%s odom=%s distance=%.3fm max=%.3fm/s tol=%.3fm yaw_tol=%.1fdeg timeout=%.2fs overshoot_success=%s timeout_success=%s",
              action_label_, cmd_vel_topic_.c_str(), odom_topic_.c_str(), distance_m_,
              max_speed_mps_, xy_tolerance_m_, yaw_tolerance_deg, timeout_s_,
              succeed_on_reach_or_overshoot_ ? "true" : "false",
              succeed_on_timeout_ ? "true" : "false");
  publishStop();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OdomAxisDriveAction::onRunning() {
  return tickTowardTarget();
}

void OdomAxisDriveAction::onHalted() {
  writeState("HALTED");
  publishStop();
  releaseRuntime();
}

void OdomAxisDriveAction::publishStop() {
  if (cmd_pub_) {
    cmd_pub_->publish(TwistMsg{});
  }
}

void OdomAxisDriveAction::releaseRuntime() {
  odom_sub_.reset();
  cmd_pub_.reset();
  node_ = nullptr;
  has_odom_ = false;
  target_ready_ = false;
  stable_ticks_ = 0;
  succeed_on_reach_or_overshoot_ = false;
  succeed_on_timeout_ = false;
}

bool OdomAxisDriveAction::odomReady() const {
  if (!has_odom_) {
    return false;
  }
  const auto age_s = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - last_odom_tp_)
                         .count();
  return age_s <= odom_timeout_s_;
}

bool OdomAxisDriveAction::timedOut() const {
  if (!node_) {
    return false;
  }
  return (node_->now() - start_time_).seconds() > timeout_s_;
}

bool OdomAxisDriveAction::prepareTargetFromCurrentOdom() {
  if (!odomReady()) {
    return false;
  }
  start_x_ = current_x_;
  start_y_ = current_y_;
  target_yaw_ = current_yaw_;
  const double c = std::cos(target_yaw_);
  const double s = std::sin(target_yaw_);
  const double x_m = axis_ == Axis::X ? distance_m_ : 0.0;
  const double y_m = axis_ == Axis::Y ? distance_m_ : 0.0;
  target_x_ = start_x_ + x_m * c - y_m * s;
  target_y_ = start_y_ + x_m * s + y_m * c;
  target_ready_ = true;
  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
                "%s 目标已捕获: start=(%.3f, %.3f, %.3f) body_delta=(%.3f, %.3f) target=(%.3f, %.3f)",
                action_label_, start_x_, start_y_, target_yaw_, x_m, y_m, target_x_,
                target_y_);
  }
  return true;
}

BT::NodeStatus OdomAxisDriveAction::tickTowardTarget() {
  if (!node_ || !cmd_pub_) {
    writeDecisionFailure(config().blackboard, action_label_,
                         "运行上下文缺失：node 或 cmd_vel 发布器不可用，cmd_vel=" +
                             cmd_vel_topic_ + "，odom=" + odom_topic_);
    return BT::NodeStatus::FAILURE;
  }
  if (timedOut()) {
    if (succeed_on_timeout_ && target_ready_) {
      publishStop();
      writeState("SUCCEEDED");
      if (node_) {
        RCLCPP_WARN(node_->get_logger(),
                    "%s 动作超时，按配置停车并继续后续行为: timeout=%.2fs",
                    action_label_, timeout_s_);
      }
      releaseRuntime();
      return BT::NodeStatus::SUCCESS;
    }
    return failWithStop("相对行驶超时");
  }
  if (!target_ready_) {
    if (!odomReady()) {
      writeState("WAITING_FOR_ODOM");
      publishStop();
      return BT::NodeStatus::RUNNING;
    }
    if (!prepareTargetFromCurrentOdom()) {
      return failWithStop("相对行驶目标捕获失败");
    }
    writeState("RUNNING");
  }
  if (!odomReady()) {
    stable_ticks_ = 0;
    writeState("WAITING_FOR_ODOM");
    publishStop();
    return BT::NodeStatus::RUNNING;
  }

  const double start_c = std::cos(target_yaw_);
  const double start_s = std::sin(target_yaw_);
  const double traveled_x = current_x_ - start_x_;
  const double traveled_y = current_y_ - start_y_;
  const double axis_progress =
      axis_ == Axis::X ? traveled_x * start_c + traveled_y * start_s
                       : -traveled_x * start_s + traveled_y * start_c;
  const double axis_remaining = distance_m_ - axis_progress;
  const double distance = std::abs(axis_remaining);
  const double yaw_error = normalizeAngle(target_yaw_ - current_yaw_);
  writeDistanceRemaining(distance);

  if (odomAxisDriveReachedOrOvershot(distance_m_, axis_remaining,
                                     xy_tolerance_m_,
                                     succeed_on_reach_or_overshoot_)) {
    publishStop();
    writeState("SUCCEEDED");
    RCLCPP_INFO(node_->get_logger(),
                "%s 到达或超调完成: remaining=%.3fm yaw_error=%.3frad",
                action_label_, axis_remaining, yaw_error);
    releaseRuntime();
    return BT::NodeStatus::SUCCESS;
  }

  if (distance <= xy_tolerance_m_ &&
      std::abs(yaw_error) <= yaw_tolerance_rad_) {
    ++stable_ticks_;
    publishStop();
    if (stable_ticks_ >= stable_ticks_required_) {
      writeState("SUCCEEDED");
      RCLCPP_INFO(node_->get_logger(),
                  "%s 完成: remaining=%.3fm yaw_error=%.3frad",
                  action_label_, distance, yaw_error);
      releaseRuntime();
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  stable_ticks_ = 0;
  writeState("RUNNING");
  TwistMsg cmd;
  if (distance > xy_tolerance_m_) {
    double axis_speed = xy_kp_ * axis_remaining;
    const double abs_speed = std::abs(axis_speed);
    if (abs_speed > max_speed_mps_) {
      axis_speed = std::copysign(max_speed_mps_, axis_speed);
    }
    if (abs_speed < min_speed_mps_ && abs_speed > 1e-9) {
      axis_speed = std::copysign(min_speed_mps_, axis_speed);
    }
    if (axis_ == Axis::X) {
      cmd.linear.x = axis_speed;
    } else {
      cmd.linear.y = axis_speed;
    }
  }
  cmd.angular.z = std::clamp(heading_kp_ * yaw_error,
                             -heading_max_speed_radps_,
                             heading_max_speed_radps_);
  cmd_pub_->publish(cmd);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OdomAxisDriveAction::failWithStop(
    const std::string &reason) {
  const std::string detail =
      reason + "，cmd_vel=" + cmd_vel_topic_ + "，odom=" + odom_topic_ +
      "，距离_m=" + std::to_string(distance_m_) +
      "，动作超时_s=" + std::to_string(timeout_s_) +
      "，odom超时_s=" + std::to_string(odom_timeout_s_);
  if (node_) {
    RCLCPP_WARN(node_->get_logger(), "%s 失败: %s", action_label_,
                detail.c_str());
  }
  writeFailure(detail);
  writeDecisionFailure(config().blackboard, action_label_, detail);
  publishStop();
  releaseRuntime();
  return BT::NodeStatus::FAILURE;
}

void OdomAxisDriveAction::writeState(const std::string &state) {
  if (!config().blackboard) {
    return;
  }
  config().blackboard->set("relative_nav_last_exec_state", state);
  if (state != "FAILED") {
    config().blackboard->set("relative_nav_last_failure_reason",
                             std::string(""));
  }
}

void OdomAxisDriveAction::writeFailure(const std::string &reason) {
  if (!config().blackboard) {
    return;
  }
  config().blackboard->set("relative_nav_last_exec_state",
                           std::string("FAILED"));
  config().blackboard->set("relative_nav_last_failure_reason", reason);
}

void OdomAxisDriveAction::writeDistanceRemaining(double distance) {
  if (config().blackboard) {
    config().blackboard->set("relative_nav_last_distance_remaining", distance);
  }
}

OdomDriveXAction::OdomDriveXAction(const std::string &name,
                                   const BT::NodeConfig &config)
    : OdomAxisDriveAction(name, config, Axis::X, "odom X 相对行驶") {}

BT::PortsList OdomDriveXAction::providedPorts() {
  return OdomAxisDriveAction::providedPorts();
}

OdomDriveYAction::OdomDriveYAction(const std::string &name,
                                   const BT::NodeConfig &config)
    : OdomAxisDriveAction(name, config, Axis::Y, "odom Y 相对横移") {}

BT::PortsList OdomDriveYAction::providedPorts() {
  return OdomAxisDriveAction::providedPorts();
}

OdomDriveXTurnXAction::OdomDriveXTurnXAction(const std::string &name,
                                             const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList OdomDriveXTurnXAction::providedPorts() {
  return {
      BT::InputPort<std::string>("cmd_vel_topic", "cmd_vel",
                                 "Velocity command topic"),
      BT::InputPort<std::string>("odom_topic", "odom", "Odometry topic"),
      BT::InputPort<double>("first_x_m",
                            "First signed X displacement in the start frame"),
      BT::InputPort<double>("yaw_delta_rad",
                            "Relative yaw delta from the start odom yaw"),
      BT::InputPort<double>("second_x_m",
                            "Second signed X displacement in the target frame"),
      BT::InputPort<double>("max_speed_mps", kDefaultOdomRelativeMaxSpeedMps,
                            "Maximum planar speed in m/s"),
      BT::InputPort<double>("min_speed_mps", kDefaultOdomRelativeMinSpeedMps,
                            "Minimum planar speed before position tolerance"),
      BT::InputPort<double>("xy_kp", kDefaultOdomRelativeXyKp,
                            "Planar position error to speed gain"),
      BT::InputPort<double>("heading_kp", kDefaultOdomRelativeHeadingKp,
                            "Yaw error to angular.z gain"),
      BT::InputPort<double>("heading_max_speed_radps",
                            kDefaultOdomRelativeHeadingMaxSpeedRadps,
                            "Maximum heading correction angular speed"),
      BT::InputPort<double>("xy_tolerance_m", kDefaultOdomRelativeXyToleranceM,
                            "Planar success tolerance in meters"),
      BT::InputPort<double>("yaw_tolerance_deg",
                            kDefaultOdomRelativeYawToleranceDeg,
                            "Yaw success tolerance in degrees"),
      BT::InputPort<int>("stable_ticks", kDefaultOdomRelativeStableTicks,
                         "Consecutive in-tolerance ticks required"),
      BT::InputPort<double>("odom_timeout_s", kDefaultOdomTimeoutSec,
                            "Maximum accepted odom age"),
      BT::InputPort<double>("timeout_s", kDefaultOdomRelativeTimeoutSec,
                            "Action timeout in seconds"),
      BT::OutputPort<double>("target_yaw_rad", "Computed absolute target yaw"),
  };
}

BT::NodeStatus OdomDriveXTurnXAction::onStart() {
  if (!config().blackboard || !config().blackboard->get("node", node_) ||
      !node_) {
    writeDecisionFailure(config().blackboard, "OdomDriveXTurnX",
                         "运行上下文缺失：blackboard 或 node 不可用");
    return BT::NodeStatus::FAILURE;
  }

  (void)getInput("cmd_vel_topic", cmd_vel_topic_);
  (void)getInput("odom_topic", odom_topic_);
  cmd_vel_topic_ = nonEmptyOr(cmd_vel_topic_, "cmd_vel");
  odom_topic_ = nonEmptyOr(odom_topic_, "odom");
  cmd_pub_ =
      node_->create_publisher<TwistMsg>(cmd_vel_topic_, rclcpp::QoS(10));

  if (!getInput("first_x_m", first_x_m_) || !std::isfinite(first_x_m_)) {
    return failWithStop("复合导航 first_x_m 非法");
  }
  if (!getInput("yaw_delta_rad", yaw_delta_rad_) ||
      !std::isfinite(yaw_delta_rad_)) {
    return failWithStop("复合导航 yaw_delta_rad 非法");
  }
  if (!getInput("second_x_m", second_x_m_) || !std::isfinite(second_x_m_)) {
    return failWithStop("复合导航 second_x_m 非法");
  }
  (void)getInput("max_speed_mps", max_speed_mps_);
  (void)getInput("min_speed_mps", min_speed_mps_);
  (void)getInput("xy_kp", xy_kp_);
  (void)getInput("heading_kp", heading_kp_);
  (void)getInput("heading_max_speed_radps", heading_max_speed_radps_);
  (void)getInput("xy_tolerance_m", xy_tolerance_m_);
  double yaw_tolerance_deg = kDefaultOdomRelativeYawToleranceDeg;
  (void)getInput("yaw_tolerance_deg", yaw_tolerance_deg);
  (void)getInput("stable_ticks", stable_ticks_required_);
  (void)getInput("odom_timeout_s", odom_timeout_s_);
  (void)getInput("timeout_s", timeout_s_);

  max_speed_mps_ =
      std::abs(finitePositiveOr(max_speed_mps_, kDefaultOdomRelativeMaxSpeedMps));
  min_speed_mps_ = std::min(
      std::abs(finiteNonNegativeOr(min_speed_mps_,
                                   kDefaultOdomRelativeMinSpeedMps)),
      max_speed_mps_);
  xy_kp_ = finitePositiveOr(xy_kp_, kDefaultOdomRelativeXyKp);
  heading_kp_ =
      finiteNonNegativeOr(heading_kp_, kDefaultOdomRelativeHeadingKp);
  heading_max_speed_radps_ = std::abs(finiteNonNegativeOr(
      heading_max_speed_radps_, kDefaultOdomRelativeHeadingMaxSpeedRadps));
  xy_tolerance_m_ =
      finitePositiveOr(xy_tolerance_m_, kDefaultOdomRelativeXyToleranceM);
  yaw_tolerance_deg = finiteNonNegativeOr(
      yaw_tolerance_deg, kDefaultOdomRelativeYawToleranceDeg);
  yaw_tolerance_rad_ = std::abs(yaw_tolerance_deg) * kPi / 180.0;
  stable_ticks_required_ = std::max(1, stable_ticks_required_);
  odom_timeout_s_ = finitePositiveOr(odom_timeout_s_, kDefaultOdomTimeoutSec);
  timeout_s_ = finitePositiveOr(timeout_s_, kDefaultOdomRelativeTimeoutSec);

  odom_sub_ = node_->create_subscription<OdomMsg>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const OdomMsg::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
        has_odom_ = true;
        last_odom_tp_ = std::chrono::steady_clock::now();
      });

  has_odom_ = false;
  target_ready_ = false;
  stable_ticks_ = 0;
  line_progress_ = 0.0;
  line_lookahead_m_ =
      std::max(xy_tolerance_m_, min_speed_mps_ / std::max(xy_kp_, 1.0e-9));
  start_time_ = node_->now();
  writeState("RUNNING");
  writeDistanceRemaining(std::abs(first_x_m_) + std::abs(second_x_m_));
  RCLCPP_INFO(node_->get_logger(),
              "odom X-turn-X 复合导航启动: cmd_vel=%s odom=%s first_x=%.3fm yaw_delta=%.4frad second_x=%.3fm max=%.3fm/s tol=%.3fm yaw_tol=%.1fdeg timeout=%.2fs",
              cmd_vel_topic_.c_str(), odom_topic_.c_str(), first_x_m_,
              yaw_delta_rad_, second_x_m_, max_speed_mps_, xy_tolerance_m_,
              yaw_tolerance_deg, timeout_s_);
  publishStop();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OdomDriveXTurnXAction::onRunning() {
  return tickTowardTarget();
}

void OdomDriveXTurnXAction::onHalted() {
  writeState("HALTED");
  publishStop();
  releaseRuntime();
}

void OdomDriveXTurnXAction::publishStop() {
  if (cmd_pub_) {
    cmd_pub_->publish(TwistMsg{});
  }
}

void OdomDriveXTurnXAction::releaseRuntime() {
  odom_sub_.reset();
  cmd_pub_.reset();
  node_ = nullptr;
  has_odom_ = false;
  target_ready_ = false;
  stable_ticks_ = 0;
}

bool OdomDriveXTurnXAction::odomReady() const {
  if (!has_odom_) {
    return false;
  }
  const auto age_s = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - last_odom_tp_)
                         .count();
  return age_s <= odom_timeout_s_;
}

bool OdomDriveXTurnXAction::timedOut() const {
  if (!node_) {
    return false;
  }
  return (node_->now() - start_time_).seconds() > timeout_s_;
}

bool OdomDriveXTurnXAction::prepareTargetFromCurrentOdom() {
  if (!odomReady()) {
    return false;
  }
  start_x_ = current_x_;
  start_y_ = current_y_;
  start_yaw_ = current_yaw_;
  const navigation::StraightLinePose start{start_x_, start_y_, start_yaw_};
  const auto target =
      navigation::xTurnXTarget(start, first_x_m_, yaw_delta_rad_, second_x_m_);
  target_x_ = target.x;
  target_y_ = target.y;
  target_yaw_ = target.yaw;
  line_progress_ = 0.0;
  target_ready_ = true;
  (void)setOutput("target_yaw_rad", target_yaw_);
  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
                "odom X-turn-X 直线轨迹目标已捕获: start=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f) lookahead=%.3fm",
                start_x_, start_y_, start_yaw_, target_x_, target_y_,
                target_yaw_, line_lookahead_m_);
  }
  return true;
}

navigation::StraightLineReference OdomDriveXTurnXAction::currentLineReference() {
  const navigation::StraightLinePose start{start_x_, start_y_, start_yaw_};
  const navigation::StraightLinePose target{target_x_, target_y_, target_yaw_};
  line_progress_ = navigation::projectProgressOnLine(
      start, target, current_x_, current_y_, line_progress_);
  return navigation::straightLineReference(start, target, line_progress_,
                                           line_lookahead_m_);
}

BT::NodeStatus OdomDriveXTurnXAction::tickTowardTarget() {
  if (!node_ || !cmd_pub_) {
    writeDecisionFailure(config().blackboard, "OdomDriveXTurnX",
                         "运行上下文缺失：node 或 cmd_vel 发布器不可用，cmd_vel=" +
                             cmd_vel_topic_ + "，odom=" + odom_topic_);
    return BT::NodeStatus::FAILURE;
  }
  if (timedOut()) {
    return failWithStop("复合导航超时");
  }
  if (!target_ready_) {
    if (!odomReady()) {
      writeState("WAITING_FOR_ODOM");
      publishStop();
      return BT::NodeStatus::RUNNING;
    }
    if (!prepareTargetFromCurrentOdom()) {
      return failWithStop("复合导航目标捕获失败");
    }
    writeState("RUNNING");
  }
  if (!odomReady()) {
    stable_ticks_ = 0;
    writeState("WAITING_FOR_ODOM");
    publishStop();
    return BT::NodeStatus::RUNNING;
  }

  const double error_world_x = target_x_ - current_x_;
  const double error_world_y = target_y_ - current_y_;
  const double distance = std::hypot(error_world_x, error_world_y);
  const double yaw_error = normalizeAngle(target_yaw_ - current_yaw_);
  writeDistanceRemaining(distance);

  if (distance <= xy_tolerance_m_ &&
      std::abs(yaw_error) <= yaw_tolerance_rad_) {
    ++stable_ticks_;
    publishStop();
    if (stable_ticks_ >= stable_ticks_required_) {
      writeState("SUCCEEDED");
      RCLCPP_INFO(node_->get_logger(),
                  "odom X-turn-X 复合导航完成: remaining=%.3fm yaw_error=%.3frad",
                  distance, yaw_error);
      releaseRuntime();
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  stable_ticks_ = 0;
  writeState("RUNNING");
  TwistMsg cmd;
  if (distance > xy_tolerance_m_) {
    const auto reference = currentLineReference();
    const double reference_error_world_x = reference.pose.x - current_x_;
    const double reference_error_world_y = reference.pose.y - current_y_;
    const double c = std::cos(current_yaw_);
    const double s = std::sin(current_yaw_);
    double body_x = reference_error_world_x * c + reference_error_world_y * s;
    double body_y = -reference_error_world_x * s + reference_error_world_y * c;
    double speed_x = xy_kp_ * body_x;
    double speed_y = xy_kp_ * body_y;
    double speed_norm = std::hypot(speed_x, speed_y);
    if (speed_norm > max_speed_mps_ && speed_norm > 1e-9) {
      const double scale = max_speed_mps_ / speed_norm;
      speed_x *= scale;
      speed_y *= scale;
      speed_norm = max_speed_mps_;
    }
    if (speed_norm < min_speed_mps_ && speed_norm > 1e-9) {
      const double scale = min_speed_mps_ / speed_norm;
      speed_x *= scale;
      speed_y *= scale;
    }
    cmd.linear.x = speed_x;
    cmd.linear.y = speed_y;
  }
  const auto reference = currentLineReference();
  const double reference_yaw_error =
      normalizeAngle(reference.pose.yaw - current_yaw_);
  cmd.angular.z = std::clamp(heading_kp_ * reference_yaw_error,
                             -heading_max_speed_radps_,
                             heading_max_speed_radps_);
  cmd_pub_->publish(cmd);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OdomDriveXTurnXAction::failWithStop(
    const std::string &reason) {
  const std::string detail =
      reason + "，cmd_vel=" + cmd_vel_topic_ + "，odom=" + odom_topic_ +
      "，first_x_m=" + std::to_string(first_x_m_) +
      "，yaw_delta_rad=" + std::to_string(yaw_delta_rad_) +
      "，second_x_m=" + std::to_string(second_x_m_) +
      "，动作超时_s=" + std::to_string(timeout_s_) +
      "，odom超时_s=" + std::to_string(odom_timeout_s_);
  if (node_) {
    RCLCPP_WARN(node_->get_logger(), "odom X-turn-X 复合导航失败: %s",
                detail.c_str());
  }
  writeFailure(detail);
  writeDecisionFailure(config().blackboard, "OdomDriveXTurnX", detail);
  publishStop();
  releaseRuntime();
  return BT::NodeStatus::FAILURE;
}

void OdomDriveXTurnXAction::writeState(const std::string &state) {
  if (!config().blackboard) {
    return;
  }
  config().blackboard->set("relative_nav_last_exec_state", state);
  if (state != "FAILED") {
    config().blackboard->set("relative_nav_last_failure_reason",
                             std::string(""));
  }
}

void OdomDriveXTurnXAction::writeFailure(const std::string &reason) {
  if (!config().blackboard) {
    return;
  }
  config().blackboard->set("relative_nav_last_exec_state",
                           std::string("FAILED"));
  config().blackboard->set("relative_nav_last_failure_reason", reason);
}

void OdomDriveXTurnXAction::writeDistanceRemaining(double distance) {
  if (config().blackboard) {
    config().blackboard->set("relative_nav_last_distance_remaining", distance);
  }
}

OdomTurnToYawAction::OdomTurnToYawAction(const std::string &name,
                                         const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList OdomTurnToYawAction::providedPorts() {
  return {
      BT::InputPort<std::string>("cmd_vel_topic", "cmd_vel",
                                 "Velocity command topic"),
      BT::InputPort<std::string>("odom_topic", "odom", "Odometry topic"),
      BT::InputPort<double>("target_yaw_rad", "Absolute odom yaw target"),
      BT::InputPort<double>("kp", kDefaultOdomRelativeHeadingKp,
                            "Yaw error to angular.z gain"),
      BT::InputPort<double>("max_speed_radps",
                            kDefaultOdomRelativeHeadingMaxSpeedRadps,
                            "Maximum turn angular speed"),
      BT::InputPort<double>("yaw_tolerance_deg",
                            kDefaultOdomRelativeYawToleranceDeg,
                            "Yaw success tolerance in degrees"),
      BT::InputPort<int>("stable_ticks", kDefaultOdomRelativeStableTicks,
                         "Consecutive in-tolerance ticks required"),
      BT::InputPort<double>("odom_timeout_s", kDefaultOdomTimeoutSec,
                            "Maximum accepted odom age"),
      BT::InputPort<double>("timeout_s", kDefaultOdomRelativeTimeoutSec,
                            "Action timeout in seconds"),
  };
}

BT::NodeStatus OdomTurnToYawAction::onStart() {
  if (!config().blackboard || !config().blackboard->get("node", node_) ||
      !node_) {
    writeDecisionFailure(config().blackboard, "OdomTurnToYaw",
                         "运行上下文缺失：blackboard 或 node 不可用");
    return BT::NodeStatus::FAILURE;
  }

  (void)getInput("cmd_vel_topic", cmd_vel_topic_);
  (void)getInput("odom_topic", odom_topic_);
  cmd_vel_topic_ = nonEmptyOr(cmd_vel_topic_, "cmd_vel");
  odom_topic_ = nonEmptyOr(odom_topic_, "odom");
  cmd_pub_ =
      node_->create_publisher<TwistMsg>(cmd_vel_topic_, rclcpp::QoS(10));
  if (!getInput("target_yaw_rad", target_yaw_rad_) ||
      !std::isfinite(target_yaw_rad_)) {
    return failWithStop("目标 yaw 参数 target_yaw_rad 非法");
  }
  target_yaw_rad_ = normalizeAngle(target_yaw_rad_);
  (void)getInput("kp", kp_);
  (void)getInput("max_speed_radps", max_speed_radps_);
  double yaw_tolerance_deg = kDefaultOdomRelativeYawToleranceDeg;
  (void)getInput("yaw_tolerance_deg", yaw_tolerance_deg);
  (void)getInput("stable_ticks", stable_ticks_required_);
  (void)getInput("odom_timeout_s", odom_timeout_s_);
  (void)getInput("timeout_s", timeout_s_);

  kp_ = finiteNonNegativeOr(kp_, kDefaultOdomRelativeHeadingKp);
  max_speed_radps_ = std::abs(finiteNonNegativeOr(
      max_speed_radps_, kDefaultOdomRelativeHeadingMaxSpeedRadps));
  yaw_tolerance_deg =
      finiteNonNegativeOr(yaw_tolerance_deg, kDefaultOdomRelativeYawToleranceDeg);
  yaw_tolerance_rad_ = std::abs(yaw_tolerance_deg) * kPi / 180.0;
  stable_ticks_required_ = std::max(1, stable_ticks_required_);
  odom_timeout_s_ = finitePositiveOr(odom_timeout_s_, kDefaultOdomTimeoutSec);
  timeout_s_ = finitePositiveOr(timeout_s_, kDefaultOdomRelativeTimeoutSec);

  odom_sub_ = node_->create_subscription<OdomMsg>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const OdomMsg::SharedPtr msg) {
        current_yaw_rad_ = yawFromQuaternion(msg->pose.pose.orientation);
        has_odom_ = true;
        last_odom_tp_ = std::chrono::steady_clock::now();
      });

  has_odom_ = false;
  stable_ticks_ = 0;
  start_time_ = node_->now();
  writeState("RUNNING");
  RCLCPP_INFO(node_->get_logger(),
              "odom 绝对转向启动: cmd_vel=%s odom=%s target_yaw=%.4frad max=%.3frad/s tol=%.1fdeg timeout=%.2fs",
              cmd_vel_topic_.c_str(), odom_topic_.c_str(), target_yaw_rad_,
              max_speed_radps_, yaw_tolerance_deg, timeout_s_);
  publishStop();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OdomTurnToYawAction::onRunning() { return tickTurn(); }

void OdomTurnToYawAction::onHalted() {
  writeState("HALTED");
  publishStop();
  releaseRuntime();
}

void OdomTurnToYawAction::publishStop() {
  if (cmd_pub_) {
    cmd_pub_->publish(TwistMsg{});
  }
}

void OdomTurnToYawAction::releaseRuntime() {
  odom_sub_.reset();
  cmd_pub_.reset();
  node_ = nullptr;
  has_odom_ = false;
  stable_ticks_ = 0;
}

bool OdomTurnToYawAction::odomReady() const {
  if (!has_odom_) {
    return false;
  }
  const auto age_s = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - last_odom_tp_)
                         .count();
  return age_s <= odom_timeout_s_;
}

bool OdomTurnToYawAction::timedOut() const {
  if (!node_) {
    return false;
  }
  return (node_->now() - start_time_).seconds() > timeout_s_;
}

BT::NodeStatus OdomTurnToYawAction::tickTurn() {
  if (!node_ || !cmd_pub_) {
    writeDecisionFailure(config().blackboard, "OdomTurnToYaw",
                         "运行上下文缺失：node 或 cmd_vel 发布器不可用，cmd_vel=" +
                             cmd_vel_topic_ + "，odom=" + odom_topic_);
    return BT::NodeStatus::FAILURE;
  }
  if (timedOut()) {
    return failWithStop("odom 绝对转向超时");
  }
  if (!odomReady()) {
    stable_ticks_ = 0;
    writeState("WAITING_FOR_ODOM");
    publishStop();
    return BT::NodeStatus::RUNNING;
  }

  const double yaw_error = normalizeAngle(target_yaw_rad_ - current_yaw_rad_);
  if (std::abs(yaw_error) <= yaw_tolerance_rad_) {
    ++stable_ticks_;
    publishStop();
    if (stable_ticks_ >= stable_ticks_required_) {
      writeState("SUCCEEDED");
      RCLCPP_INFO(node_->get_logger(),
                  "odom 绝对转向完成: target=%.4frad current=%.4frad error=%.4frad",
                  target_yaw_rad_, current_yaw_rad_, yaw_error);
      releaseRuntime();
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  stable_ticks_ = 0;
  writeState("RUNNING");
  TwistMsg cmd;
  cmd.angular.z =
      std::clamp(kp_ * yaw_error, -max_speed_radps_, max_speed_radps_);
  cmd_pub_->publish(cmd);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OdomTurnToYawAction::failWithStop(
    const std::string &reason) {
  const std::string detail =
      reason + "，cmd_vel=" + cmd_vel_topic_ + "，odom=" + odom_topic_ +
      "，目标yaw_rad=" + std::to_string(target_yaw_rad_) +
      "，当前yaw_rad=" + std::to_string(current_yaw_rad_) +
      "，动作超时_s=" + std::to_string(timeout_s_) +
      "，odom超时_s=" + std::to_string(odom_timeout_s_);
  if (node_) {
    RCLCPP_WARN(node_->get_logger(), "odom 绝对转向失败: %s",
                detail.c_str());
  }
  writeFailure(detail);
  writeDecisionFailure(config().blackboard, "OdomTurnToYaw", detail);
  publishStop();
  releaseRuntime();
  return BT::NodeStatus::FAILURE;
}

void OdomTurnToYawAction::writeState(const std::string &state) {
  if (!config().blackboard) {
    return;
  }
  config().blackboard->set("relative_nav_last_exec_state", state);
  if (state != "FAILED") {
    config().blackboard->set("relative_nav_last_failure_reason",
                             std::string(""));
  }
}

void OdomTurnToYawAction::writeFailure(const std::string &reason) {
  if (!config().blackboard) {
    return;
  }
  config().blackboard->set("relative_nav_last_exec_state",
                           std::string("FAILED"));
  config().blackboard->set("relative_nav_last_failure_reason", reason);
}

RelativeYawTargetAction::RelativeYawTargetAction(
    const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList RelativeYawTargetAction::providedPorts() {
  return {
      BT::InputPort<std::string>("odom_topic", "odom", "Odometry topic"),
      BT::InputPort<double>("yaw_delta_rad",
                            "Relative yaw delta added to current yaw"),
      BT::InputPort<double>("timeout_s", kDefaultRelativeYawCaptureTimeoutSec,
                            "Fresh odom wait timeout"),
      BT::InputPort<double>("odom_timeout_s", kDefaultOdomTimeoutSec,
                            "Maximum accepted odom age"),
      BT::OutputPort<double>("target_yaw_rad", "Computed target yaw"),
  };
}

BT::NodeStatus RelativeYawTargetAction::onStart() {
  if (!config().blackboard || !config().blackboard->get("node", node_) ||
      !node_) {
    writeDecisionFailure(config().blackboard, "RelativeYawTarget",
                         "运行上下文缺失：blackboard 或 node 不可用");
    return BT::NodeStatus::FAILURE;
  }

  (void)getInput("odom_topic", odom_topic_);
  odom_topic_ = nonEmptyOr(odom_topic_, "odom");
  if (!getInput("yaw_delta_rad", yaw_delta_rad_) ||
      !std::isfinite(yaw_delta_rad_)) {
    RCLCPP_ERROR(node_->get_logger(), "相对 yaw 目标参数非法: yaw_delta_rad");
    writeDecisionFailure(config().blackboard, "RelativeYawTarget",
                         "相对 yaw 增量 yaw_delta_rad 非法，odom=" + odom_topic_);
    releaseRuntime();
    return BT::NodeStatus::FAILURE;
  }
  (void)getInput("timeout_s", timeout_s_);
  (void)getInput("odom_timeout_s", odom_timeout_s_);
  timeout_s_ =
      finitePositiveOr(timeout_s_, kDefaultRelativeYawCaptureTimeoutSec);
  odom_timeout_s_ = finitePositiveOr(odom_timeout_s_, kDefaultOdomTimeoutSec);

  has_yaw_ = false;
  last_odom_tp_ = {};
  start_time_ = node_->now();
  odom_sub_ = node_->create_subscription<OdomMsg>(
      odom_topic_, rclcpp::QoS(rclcpp::KeepLast(10)),
      [this](const OdomMsg::SharedPtr msg) {
        current_yaw_rad_ = yawFromQuaternion(msg->pose.pose.orientation);
        has_yaw_ = true;
        last_odom_tp_ = std::chrono::steady_clock::now();
      });

  RCLCPP_INFO(node_->get_logger(),
              "相对 yaw 目标捕获启动: odom=%s delta=%.4frad timeout=%.2fs",
              odom_topic_.c_str(), yaw_delta_rad_, timeout_s_);
  return tryCaptureTarget();
}

BT::NodeStatus RelativeYawTargetAction::onRunning() {
  return tryCaptureTarget();
}

void RelativeYawTargetAction::onHalted() { releaseRuntime(); }

BT::NodeStatus RelativeYawTargetAction::tryCaptureTarget() {
  if (!node_) {
    writeDecisionFailure(config().blackboard, "RelativeYawTarget",
                         "运行上下文缺失：node 不可用，odom=" + odom_topic_);
    return BT::NodeStatus::FAILURE;
  }
  if (has_yaw_) {
    const auto age_s = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - last_odom_tp_)
                           .count();
    if (age_s <= odom_timeout_s_) {
      const double target_yaw =
          normalizeAngle(current_yaw_rad_ + yaw_delta_rad_);
      (void)setOutput("target_yaw_rad", target_yaw);
      RCLCPP_INFO(node_->get_logger(),
                  "相对 yaw 目标已捕获: current=%.4frad delta=%.4frad target=%.4frad",
                  current_yaw_rad_, yaw_delta_rad_, target_yaw);
      releaseRuntime();
      return BT::NodeStatus::SUCCESS;
    }
  }

  if ((node_->now() - start_time_).seconds() > timeout_s_) {
    RCLCPP_WARN(node_->get_logger(), "相对 yaw 目标捕获超时: odom=%s",
                odom_topic_.c_str());
    writeDecisionFailure(
        config().blackboard, "RelativeYawTarget",
        "相对 yaw 目标捕获超时，odom=" + odom_topic_ +
            "，捕获超时_s=" + std::to_string(timeout_s_) +
            "，odom超时_s=" + std::to_string(odom_timeout_s_));
    releaseRuntime();
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void RelativeYawTargetAction::releaseRuntime() {
  odom_sub_.reset();
  node_ = nullptr;
  has_yaw_ = false;
}

void registerOdomNavigationNodes(BT::BehaviorTreeFactory &factory) {
  factory.registerNodeType<OdomDriveXAction>("OdomDriveX");
  factory.registerNodeType<OdomDriveYAction>("OdomDriveY");
  factory.registerNodeType<OdomDriveXTurnXAction>("OdomDriveXTurnX");
  factory.registerNodeType<OdomTurnToYawAction>("OdomTurnToYaw");
  factory.registerNodeType<RelativeYawTargetAction>("RelativeYawTarget");
}

} // namespace rc26_decision
