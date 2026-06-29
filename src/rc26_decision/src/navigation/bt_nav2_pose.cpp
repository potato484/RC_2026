#include "rc26_decision/navigation/bt_nav2_pose.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <string>

#include <tf2/exceptions.h>

namespace rc26_decision {

namespace {

constexpr uint16_t kErrorActionFailed = 1;
constexpr uint16_t kErrorActionAborted = 120;
constexpr uint16_t kErrorActionCanceled = 121;
constexpr uint16_t kErrorActionUnknown = 122;
constexpr uint16_t kErrorGoalPoseMismatch = 130;
constexpr double kMaxGoalFrameTfAgeSec = 0.5;
constexpr double kDefaultSuccessXyTolerance = 0.20;
constexpr double kDefaultSuccessYawTolerance = 0.25;
constexpr double kDefaultPoseCaptureTimeoutSec = 5.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDefaultRelativeYawCaptureTimeoutSec = 2.0;
constexpr double kDefaultOdomTimeoutSec = 0.5;
constexpr double kDefaultOdomRelativeDriveDistanceM = 0.40;
constexpr double kDefaultOdomRelativeDriveMaxSpeedMps = 0.20;
constexpr double kDefaultOdomRelativeDriveMinSpeedMps = 0.03;
constexpr double kDefaultOdomRelativeDriveKp = 0.8;
constexpr double kDefaultOdomRelativeDriveHeadingKp = 1.2;
constexpr double kDefaultOdomRelativeDriveHeadingMaxSpeedRadps = 0.30;
constexpr double kDefaultOdomRelativeDriveXyToleranceM = 0.03;
constexpr double kDefaultOdomRelativeDriveYawToleranceDeg = 3.0;
constexpr int kDefaultOdomRelativeDriveStableTicks = 3;
constexpr double kDefaultOdomRelativeDriveTimeoutSec = 10.0;

BT::Blackboard::Ptr blackboardOf(const BT::TreeNode &node) {
  return node.config().blackboard;
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

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

void resetNavBlackboard(const BT::Blackboard::Ptr &blackboard) {
  if (!blackboard) {
    return;
  }
  blackboard->set("nav_last_exec_state", std::string("PENDING"));
  blackboard->set("nav_last_failure_code", std::string(""));
  blackboard->set("nav_last_failure_reason", std::string(""));
  blackboard->set("nav_last_distance_remaining", 0.0);
  blackboard->set("nav_last_recovery_count", static_cast<int>(0));
}

void writeFailure(const BT::Blackboard::Ptr &blackboard,
                  const std::string &failure_code,
                  const std::string &failure_reason) {
  if (!blackboard) {
    return;
  }
  blackboard->set("nav_last_exec_state", std::string("FAILED"));
  blackboard->set("nav_last_failure_code", failure_code);
  blackboard->set("nav_last_failure_reason", failure_reason);
}

std::chrono::milliseconds toTimeout(double timeout_sec,
                                    double fallback_sec) {
  if (!std::isfinite(timeout_sec) || timeout_sec <= 0.0) {
    timeout_sec = fallback_sec;
  }
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(timeout_sec));
}

} // namespace

void loadOdomRightTurnNavParams(rclcpp::Node &node,
                                const BT::Blackboard::Ptr &blackboard) {
  const std::string cmd_vel_topic = node.declare_parameter<std::string>(
      "odom_right_turn_nav_cmd_vel_topic", "cmd_vel");
  const std::string odom_topic =
      node.declare_parameter<std::string>("odom_right_turn_nav_odom_topic",
                                          "odom");
  const double forward_distance_m = node.declare_parameter<double>(
      "odom_right_turn_nav_forward_distance_m",
      kDefaultOdomRelativeDriveDistanceM);
  const double reverse_distance_m = node.declare_parameter<double>(
      "odom_right_turn_nav_reverse_distance_m",
      -kDefaultOdomRelativeDriveDistanceM);
  const double right_turn_delta_rad = node.declare_parameter<double>(
      "odom_right_turn_nav_right_turn_delta_rad", -0.5 * kPi);
  const double odom_capture_timeout_s = node.declare_parameter<double>(
      "odom_right_turn_nav_odom_capture_timeout_s",
      kDefaultRelativeYawCaptureTimeoutSec);
  const double odom_timeout_s = node.declare_parameter<double>(
      "odom_right_turn_nav_odom_timeout_s", kDefaultOdomTimeoutSec);
  const double drive_max_speed_mps = node.declare_parameter<double>(
      "odom_right_turn_nav_drive_max_speed_mps",
      kDefaultOdomRelativeDriveMaxSpeedMps);
  const double drive_min_speed_mps = node.declare_parameter<double>(
      "odom_right_turn_nav_drive_min_speed_mps",
      kDefaultOdomRelativeDriveMinSpeedMps);
  const double drive_xy_kp = node.declare_parameter<double>(
      "odom_right_turn_nav_drive_xy_kp", kDefaultOdomRelativeDriveKp);
  const double drive_heading_kp = node.declare_parameter<double>(
      "odom_right_turn_nav_drive_heading_kp",
      kDefaultOdomRelativeDriveHeadingKp);
  const double drive_heading_max_speed_radps =
      node.declare_parameter<double>(
          "odom_right_turn_nav_drive_heading_max_speed_radps",
          kDefaultOdomRelativeDriveHeadingMaxSpeedRadps);
  const double drive_xy_tolerance_m = node.declare_parameter<double>(
      "odom_right_turn_nav_drive_xy_tolerance_m",
      kDefaultOdomRelativeDriveXyToleranceM);
  const double drive_yaw_tolerance_deg = node.declare_parameter<double>(
      "odom_right_turn_nav_drive_yaw_tolerance_deg",
      kDefaultOdomRelativeDriveYawToleranceDeg);
  const int drive_stable_ticks = node.declare_parameter<int>(
      "odom_right_turn_nav_drive_stable_ticks",
      kDefaultOdomRelativeDriveStableTicks);
  const double drive_timeout_s = node.declare_parameter<double>(
      "odom_right_turn_nav_drive_timeout_s",
      kDefaultOdomRelativeDriveTimeoutSec);

  blackboard->set("odom_right_turn_nav_cmd_vel_topic", cmd_vel_topic);
  blackboard->set("odom_right_turn_nav_odom_topic", odom_topic);
  blackboard->set("odom_right_turn_nav_forward_distance_m",
                  forward_distance_m);
  blackboard->set("odom_right_turn_nav_reverse_distance_m",
                  reverse_distance_m);
  blackboard->set("odom_right_turn_nav_right_turn_delta_rad",
                  right_turn_delta_rad);
  blackboard->set("odom_right_turn_nav_odom_capture_timeout_s",
                  odom_capture_timeout_s);
  blackboard->set("odom_right_turn_nav_odom_timeout_s", odom_timeout_s);
  blackboard->set("odom_right_turn_nav_drive_max_speed_mps",
                  drive_max_speed_mps);
  blackboard->set("odom_right_turn_nav_drive_min_speed_mps",
                  drive_min_speed_mps);
  blackboard->set("odom_right_turn_nav_drive_xy_kp", drive_xy_kp);
  blackboard->set("odom_right_turn_nav_drive_heading_kp", drive_heading_kp);
  blackboard->set("odom_right_turn_nav_drive_heading_max_speed_radps",
                  drive_heading_max_speed_radps);
  blackboard->set("odom_right_turn_nav_drive_xy_tolerance_m",
                  drive_xy_tolerance_m);
  blackboard->set("odom_right_turn_nav_drive_yaw_tolerance_deg",
                  drive_yaw_tolerance_deg);
  blackboard->set("odom_right_turn_nav_drive_stable_ticks",
                  drive_stable_ticks);
  blackboard->set("odom_right_turn_nav_drive_timeout_s", drive_timeout_s);

  RCLCPP_INFO(node.get_logger(),
              "odom 右转导航参数已加载: cmd_vel=%s odom=%s forward=%.2fm right_turn=%.4frad reverse=%.2fm max_speed=%.2fm/s",
              cmd_vel_topic.c_str(), odom_topic.c_str(), forward_distance_m,
              right_turn_delta_rad, reverse_distance_m, drive_max_speed_mps);
}

CaptureCurrentPoseAction::CaptureCurrentPoseAction(
    const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList CaptureCurrentPoseAction::providedPorts() {
  return {
      BT::InputPort<std::string>("frame_id", "map", "Pose frame"),
      BT::InputPort<std::string>("base_frame", "base_footprint",
                                 "Robot base frame"),
      BT::InputPort<double>("timeout_sec", kDefaultPoseCaptureTimeoutSec,
                            "Fresh TF wait timeout"),
      BT::OutputPort<double>("x", "Current x in frame_id"),
      BT::OutputPort<double>("y", "Current y in frame_id"),
      BT::OutputPort<double>("yaw", "Current yaw in frame_id"),
  };
}

BT::NodeStatus CaptureCurrentPoseAction::onStart() {
  if (!config().blackboard || !config().blackboard->get("node", node_) ||
      !node_) {
    return BT::NodeStatus::FAILURE;
  }

  (void)getInput("frame_id", frame_id_);
  (void)getInput("base_frame", base_frame_);
  double timeout_sec = kDefaultPoseCaptureTimeoutSec;
  (void)getInput("timeout_sec", timeout_sec);
  if (frame_id_.empty()) {
    frame_id_ = "map";
  }
  if (base_frame_.empty()) {
    base_frame_ = "base_footprint";
  }
  timeout_ = toTimeout(timeout_sec, kDefaultPoseCaptureTimeoutSec);
  start_time_ = std::chrono::steady_clock::now();

  if (!tf_buffer_) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ =
        std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, node_, true);
  }

  return tryCapture();
}

BT::NodeStatus CaptureCurrentPoseAction::onRunning() { return tryCapture(); }

void CaptureCurrentPoseAction::onHalted() {}

BT::NodeStatus CaptureCurrentPoseAction::tryCapture() {
  if (!node_ || !tf_buffer_) {
    return BT::NodeStatus::FAILURE;
  }

  try {
    const auto transform = tf_buffer_->lookupTransform(
        frame_id_, base_frame_, tf2::TimePointZero);
    const rclcpp::Time stamp(transform.header.stamp);
    const auto age = node_->now() - stamp;
    if (age >= rclcpp::Duration::from_seconds(0.0) &&
        age <= rclcpp::Duration::from_seconds(kMaxGoalFrameTfAgeSec)) {
      const double x = transform.transform.translation.x;
      const double y = transform.transform.translation.y;
      const double yaw = yawFromQuaternion(transform.transform.rotation);
      (void)setOutput("x", x);
      (void)setOutput("y", y);
      (void)setOutput("yaw", yaw);
      RCLCPP_INFO(node_->get_logger(),
                  "捕获当前位姿: %s->%s x=%.3f y=%.3f yaw=%.3f",
                  frame_id_.c_str(), base_frame_.c_str(), x, y, yaw);
      return BT::NodeStatus::SUCCESS;
    }
  } catch (const tf2::TransformException &) {
  }

  if (std::chrono::steady_clock::now() - start_time_ > timeout_) {
    RCLCPP_WARN(node_->get_logger(), "捕获当前位姿超时: %s->%s",
                frame_id_.c_str(), base_frame_.c_str());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

OdomRelativeDriveAction::OdomRelativeDriveAction(
    const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList OdomRelativeDriveAction::providedPorts() {
  return {
      BT::InputPort<std::string>("cmd_vel_topic", "cmd_vel",
                                 "Velocity command topic"),
      BT::InputPort<std::string>("odom_topic", "odom", "Odometry topic"),
      BT::InputPort<double>(
          "distance_m",
          "Signed distance along the start yaw direction. Positive means x+"),
      BT::InputPort<double>("max_speed_mps", 0.20,
                            "Maximum planar speed in m/s"),
      BT::InputPort<double>("min_speed_mps", 0.03,
                            "Minimum planar speed before xy tolerance"),
      BT::InputPort<double>("xy_kp", 0.8,
                            "Position error to planar speed gain"),
      BT::InputPort<double>("heading_kp", 1.2,
                            "Yaw error to angular.z gain"),
      BT::InputPort<double>("heading_max_speed_radps", 0.30,
                            "Maximum heading correction angular speed"),
      BT::InputPort<double>("xy_tolerance_m", 0.03,
                            "Position success tolerance in meters"),
      BT::InputPort<double>("yaw_tolerance_deg", 3.0,
                            "Yaw hold success tolerance in degrees"),
      BT::InputPort<int>("stable_ticks", 3,
                         "Consecutive in-tolerance ticks required"),
      BT::InputPort<double>("odom_timeout_s", 0.5,
                            "Maximum accepted odom age"),
      BT::InputPort<double>("timeout_s",
                            kDefaultOdomRelativeDriveTimeoutSec,
                            "Action timeout in seconds"),
  };
}

BT::NodeStatus OdomRelativeDriveAction::onStart() {
  if (!config().blackboard || !config().blackboard->get("node", node_) ||
      !node_) {
    return BT::NodeStatus::FAILURE;
  }

  (void)getInput("cmd_vel_topic", cmd_vel_topic_);
  (void)getInput("odom_topic", odom_topic_);
  if (cmd_vel_topic_.empty()) {
    cmd_vel_topic_ = "cmd_vel";
  }
  if (odom_topic_.empty()) {
    odom_topic_ = "odom";
  }

  (void)getInput("max_speed_mps", max_speed_mps_);
  (void)getInput("min_speed_mps", min_speed_mps_);
  (void)getInput("xy_kp", xy_kp_);
  (void)getInput("heading_kp", heading_kp_);
  (void)getInput("heading_max_speed_radps", heading_max_speed_radps_);
  (void)getInput("xy_tolerance_m", xy_tolerance_m_);
  double yaw_tolerance_deg = kDefaultOdomRelativeDriveYawToleranceDeg;
  (void)getInput("yaw_tolerance_deg", yaw_tolerance_deg);
  (void)getInput("stable_ticks", stable_ticks_required_);
  (void)getInput("odom_timeout_s", odom_timeout_s_);
  (void)getInput("timeout_s", timeout_s_);

  if (!getInput("distance_m", distance_m_) || !std::isfinite(distance_m_)) {
    RCLCPP_ERROR(node_->get_logger(), "odom 相对行驶参数非法: distance_m");
    releaseRuntime();
    return BT::NodeStatus::FAILURE;
  }
  if (!std::isfinite(max_speed_mps_) || max_speed_mps_ <= 0.0) {
    max_speed_mps_ = 0.20;
  }
  if (!std::isfinite(min_speed_mps_) || min_speed_mps_ < 0.0) {
    min_speed_mps_ = 0.03;
  }
  max_speed_mps_ = std::abs(max_speed_mps_);
  min_speed_mps_ = std::min(std::abs(min_speed_mps_), max_speed_mps_);
  if (!std::isfinite(xy_kp_) || xy_kp_ <= 0.0) {
    xy_kp_ = kDefaultOdomRelativeDriveKp;
  }
  if (!std::isfinite(heading_kp_) || heading_kp_ < 0.0) {
    heading_kp_ = kDefaultOdomRelativeDriveHeadingKp;
  }
  if (!std::isfinite(heading_max_speed_radps_) ||
      heading_max_speed_radps_ < 0.0) {
    heading_max_speed_radps_ =
        kDefaultOdomRelativeDriveHeadingMaxSpeedRadps;
  }
  heading_max_speed_radps_ = std::abs(heading_max_speed_radps_);
  if (!std::isfinite(xy_tolerance_m_) || xy_tolerance_m_ <= 0.0) {
    xy_tolerance_m_ = kDefaultOdomRelativeDriveXyToleranceM;
  }
  if (!std::isfinite(yaw_tolerance_deg) || yaw_tolerance_deg < 0.0) {
    yaw_tolerance_deg = kDefaultOdomRelativeDriveYawToleranceDeg;
  }
  yaw_tolerance_rad_ = std::abs(yaw_tolerance_deg) * kPi / 180.0;
  stable_ticks_required_ = std::max(1, stable_ticks_required_);
  if (!std::isfinite(odom_timeout_s_) || odom_timeout_s_ <= 0.0) {
    odom_timeout_s_ = kDefaultOdomTimeoutSec;
  }
  if (!std::isfinite(timeout_s_) || timeout_s_ <= 0.0) {
    timeout_s_ = kDefaultOdomRelativeDriveTimeoutSec;
  }

  cmd_pub_ =
      node_->create_publisher<TwistMsg>(cmd_vel_topic_, rclcpp::QoS(10));
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
  RCLCPP_INFO(node_->get_logger(),
              "odom 相对行驶启动: cmd_vel=%s odom=%s distance=%.3fm max=%.3fm/s tol=%.3fm yaw_tol=%.1fdeg timeout=%.2fs",
              cmd_vel_topic_.c_str(), odom_topic_.c_str(), distance_m_,
              max_speed_mps_, xy_tolerance_m_, yaw_tolerance_deg, timeout_s_);
  publishStop();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OdomRelativeDriveAction::onRunning() {
  return tickTowardTarget();
}

void OdomRelativeDriveAction::onHalted() {
  publishStop();
  releaseRuntime();
}

void OdomRelativeDriveAction::publishStop() {
  if (cmd_pub_) {
    cmd_pub_->publish(TwistMsg{});
  }
}

void OdomRelativeDriveAction::releaseRuntime() {
  odom_sub_.reset();
  cmd_pub_.reset();
  node_ = nullptr;
  has_odom_ = false;
  target_ready_ = false;
  stable_ticks_ = 0;
}

bool OdomRelativeDriveAction::odomReady() const {
  if (!has_odom_) {
    return false;
  }
  const auto age_s = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - last_odom_tp_)
                         .count();
  return age_s <= odom_timeout_s_;
}

bool OdomRelativeDriveAction::timedOut() const {
  if (!node_) {
    return false;
  }
  return (node_->now() - start_time_).seconds() > timeout_s_;
}

bool OdomRelativeDriveAction::prepareTargetFromCurrentOdom() {
  if (!odomReady()) {
    return false;
  }
  target_yaw_ = current_yaw_;
  target_x_ = current_x_ + distance_m_ * std::cos(target_yaw_);
  target_y_ = current_y_ + distance_m_ * std::sin(target_yaw_);
  target_ready_ = true;
  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
                "odom 相对行驶目标已捕获: start=(%.3f, %.3f, %.3f) target=(%.3f, %.3f) distance=%.3fm",
                current_x_, current_y_, target_yaw_, target_x_, target_y_,
                distance_m_);
  }
  return true;
}

BT::NodeStatus OdomRelativeDriveAction::tickTowardTarget() {
  if (!node_ || !cmd_pub_) {
    return BT::NodeStatus::FAILURE;
  }
  if (timedOut()) {
    return failWithStop("relative drive timeout");
  }
  if (!target_ready_) {
    if (!odomReady()) {
      publishStop();
      return BT::NodeStatus::RUNNING;
    }
    if (!prepareTargetFromCurrentOdom()) {
      return failWithStop("relative drive target prepare failed");
    }
  }
  if (!odomReady()) {
    stable_ticks_ = 0;
    publishStop();
    return BT::NodeStatus::RUNNING;
  }

  const double error_x = target_x_ - current_x_;
  const double error_y = target_y_ - current_y_;
  const double distance = std::hypot(error_x, error_y);
  const double yaw_error = normalizeAngle(target_yaw_ - current_yaw_);

  if (distance <= xy_tolerance_m_ &&
      std::abs(yaw_error) <= yaw_tolerance_rad_) {
    ++stable_ticks_;
    publishStop();
    if (stable_ticks_ >= stable_ticks_required_) {
      RCLCPP_INFO(node_->get_logger(),
                  "odom 相对行驶完成: remaining=%.3fm yaw_error=%.3frad",
                  distance, yaw_error);
      releaseRuntime();
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  stable_ticks_ = 0;
  TwistMsg cmd;
  if (distance > xy_tolerance_m_) {
    const double world_vx = xy_kp_ * error_x;
    const double world_vy = xy_kp_ * error_y;
    const double c = std::cos(current_yaw_);
    const double s = std::sin(current_yaw_);
    double body_vx = c * world_vx + s * world_vy;
    double body_vy = -s * world_vx + c * world_vy;
    double body_speed = std::hypot(body_vx, body_vy);

    if (body_speed > max_speed_mps_ && body_speed > 0.0) {
      const double scale = max_speed_mps_ / body_speed;
      body_vx *= scale;
      body_vy *= scale;
      body_speed = max_speed_mps_;
    }
    if (body_speed < min_speed_mps_ && body_speed > 1e-9) {
      const double scale = min_speed_mps_ / body_speed;
      body_vx *= scale;
      body_vy *= scale;
    }
    cmd.linear.x = body_vx;
    cmd.linear.y = body_vy;
  }
  cmd.angular.z = std::clamp(heading_kp_ * yaw_error,
                             -heading_max_speed_radps_,
                             heading_max_speed_radps_);
  cmd_pub_->publish(cmd);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OdomRelativeDriveAction::failWithStop(const char *reason) {
  if (node_) {
    RCLCPP_WARN(node_->get_logger(), "odom 相对行驶失败: %s",
                reason ? reason : "unknown");
  }
  publishStop();
  releaseRuntime();
  return BT::NodeStatus::FAILURE;
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
    return BT::NodeStatus::FAILURE;
  }

  (void)getInput("odom_topic", odom_topic_);
  if (odom_topic_.empty()) {
    odom_topic_ = "odom";
  }
  if (!getInput("yaw_delta_rad", yaw_delta_rad_) ||
      !std::isfinite(yaw_delta_rad_)) {
    RCLCPP_ERROR(node_->get_logger(), "相对 yaw 目标参数非法: yaw_delta_rad");
    releaseRuntime();
    return BT::NodeStatus::FAILURE;
  }
  (void)getInput("timeout_s", timeout_s_);
  (void)getInput("odom_timeout_s", odom_timeout_s_);
  if (!std::isfinite(timeout_s_) || timeout_s_ <= 0.0) {
    timeout_s_ = kDefaultRelativeYawCaptureTimeoutSec;
  }
  if (!std::isfinite(odom_timeout_s_) || odom_timeout_s_ <= 0.0) {
    odom_timeout_s_ = kDefaultOdomTimeoutSec;
  }

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
    return BT::NodeStatus::FAILURE;
  }
  if (has_yaw_) {
    const auto age_s = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - last_odom_tp_)
                           .count();
    if (age_s <= odom_timeout_s_) {
      const double target_yaw = normalizeAngle(current_yaw_rad_ + yaw_delta_rad_);
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

NavToPoseAction::NavToPoseAction(const std::string &name,
                                 const BT::NodeConfig &config)
    : BtActionNode<NavigateToPose>(name, config, "navigate_to_pose",
                                   std::chrono::seconds(60)) {}

bool NavToPoseAction::isActionReady(rclcpp::Node &node) {
  if (!bt_navigator_state_client_) {
    bt_navigator_state_client_ =
        node.create_client<GetLifecycleState>("/bt_navigator/get_state");
  }

  if (!bt_navigator_state_client_->service_is_ready()) {
    bt_navigator_state_future_ = {};
    return false;
  }

  if (!bt_navigator_state_future_.valid()) {
    auto request = std::make_shared<GetLifecycleState::Request>();
    bt_navigator_state_future_ =
        bt_navigator_state_client_->async_send_request(request).future.share();
    return false;
  }

  if (bt_navigator_state_future_.wait_for(std::chrono::milliseconds(0)) !=
      std::future_status::ready) {
    return false;
  }

  const auto response = bt_navigator_state_future_.get();
  bt_navigator_state_future_ = {};
  if (!response) {
    return false;
  }

  return response->current_state.id ==
             lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
         hasFreshGoalFrameTf(node);
}

bool NavToPoseAction::hasFreshGoalFrameTf(rclcpp::Node &node) {
  if (!tf_buffer_) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node.get_clock());
    tf_listener_ =
        std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, &node, true);
    return false;
  }

  std::string frame_id = "map";
  (void)getInput("frame_id", frame_id);
  if (frame_id.empty()) {
    frame_id = "map";
  }

  try {
    const auto transform = tf_buffer_->lookupTransform(
        frame_id, "base_footprint", tf2::TimePointZero);
    const rclcpp::Time stamp(transform.header.stamp);
    const auto age = node.now() - stamp;
    return age >= rclcpp::Duration::from_seconds(0.0) &&
           age <= rclcpp::Duration::from_seconds(kMaxGoalFrameTfAgeSec);
  } catch (const tf2::TransformException &) {
    return false;
  }
}

bool NavToPoseAction::isAtRequestedGoal(rclcpp::Node &node,
                                        std::string &failure_reason) {
  if (!tf_buffer_) {
    failure_reason = "missing TF listener for reached-pose verification";
    return false;
  }

  double target_x = 0.0;
  double target_y = 0.0;
  double target_yaw = 0.0;
  double xy_tolerance = kDefaultSuccessXyTolerance;
  double yaw_tolerance = kDefaultSuccessYawTolerance;
  std::string frame_id = "map";

  if (!getInput("x", target_x) || !std::isfinite(target_x) ||
      !getInput("y", target_y) || !std::isfinite(target_y)) {
    failure_reason = "invalid target pose while verifying Nav2 success";
    return false;
  }
  if (!getInput("yaw", target_yaw) || !std::isfinite(target_yaw)) {
    target_yaw = 0.0;
  }
  (void)getInput("frame_id", frame_id);
  (void)getInput("success_xy_tolerance", xy_tolerance);
  (void)getInput("success_yaw_tolerance", yaw_tolerance);
  if (frame_id.empty()) {
    frame_id = "map";
  }
  if (!std::isfinite(xy_tolerance) || xy_tolerance <= 0.0) {
    xy_tolerance = kDefaultSuccessXyTolerance;
  }
  if (!std::isfinite(yaw_tolerance) || yaw_tolerance <= 0.0) {
    yaw_tolerance = kDefaultSuccessYawTolerance;
  }

  try {
    const auto transform = tf_buffer_->lookupTransform(
        frame_id, "base_footprint", tf2::TimePointZero);
    const rclcpp::Time stamp(transform.header.stamp);
    const auto age = node.now() - stamp;
    if (age < rclcpp::Duration::from_seconds(0.0) ||
        age > rclcpp::Duration::from_seconds(kMaxGoalFrameTfAgeSec)) {
      failure_reason = "stale TF while verifying Nav2 success";
      return false;
    }

    const double current_x = transform.transform.translation.x;
    const double current_y = transform.transform.translation.y;
    const double position_error =
        std::hypot(current_x - target_x, current_y - target_y);
    const double current_yaw =
        yawFromQuaternion(transform.transform.rotation);
    const double yaw_error =
        std::abs(normalizeAngle(current_yaw - target_yaw));

    if (const auto blackboard = blackboardOf(*this)) {
      blackboard->set("nav_last_distance_remaining", position_error);
    }

    if (position_error > xy_tolerance) {
      failure_reason = "Nav2 reported success but pose is still " +
                       std::to_string(position_error) + "m from target";
      return false;
    }
    if (yaw_error > yaw_tolerance) {
      failure_reason = "Nav2 reported success but yaw error is " +
                       std::to_string(yaw_error) + "rad";
      return false;
    }
    return true;
  } catch (const tf2::TransformException &ex) {
    failure_reason =
        std::string("failed to verify Nav2 success pose: ") + ex.what();
    return false;
  }
}

BT::PortsList NavToPoseAction::providedPorts() {
  auto ports = BtActionNode<NavigateToPose>::basePorts(60.0);
  ports.insert(BT::InputPort<std::string>("frame_id", "map", "Goal frame"));
  ports.insert(BT::InputPort<double>("x", "Goal x in frame_id"));
  ports.insert(BT::InputPort<double>("y", "Goal y in frame_id"));
  ports.insert(BT::InputPort<double>("yaw", 0.0, "Goal yaw in radians"));
  ports.insert(BT::InputPort<std::string>("behavior_tree", "",
                                          "Optional Nav2 BT XML path"));
  ports.insert(BT::InputPort<double>(
      "success_xy_tolerance", kDefaultSuccessXyTolerance,
      "Decision-side success verification xy tolerance"));
  ports.insert(BT::InputPort<double>(
      "success_yaw_tolerance", kDefaultSuccessYawTolerance,
      "Decision-side success verification yaw tolerance"));
  return ports;
}

bool NavToPoseAction::buildGoal(Goal &goal) {
  rclcpp::Node *node = nullptr;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  std::string frame_id = "map";
  std::string behavior_tree;

  if (!config().blackboard || !config().blackboard->get("node", node) ||
      !node) {
    return false;
  }
  if (!getInput("x", x) || !std::isfinite(x) || !getInput("y", y) ||
      !std::isfinite(y)) {
    return false;
  }
  if (!getInput("yaw", yaw) || !std::isfinite(yaw)) {
    yaw = 0.0;
  }
  (void)getInput("frame_id", frame_id);
  (void)getInput("behavior_tree", behavior_tree);
  if (frame_id.empty()) {
    frame_id = "map";
  }

  if (auto blackboard = blackboardOf(*this)) {
    resetNavBlackboard(blackboard);
  }

  goal.pose.header.frame_id = frame_id;
  goal.pose.header.stamp = node->get_clock()->now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.position.z = 0.0;
  goal.pose.pose.orientation = yawToQuaternion(yaw);
  goal.behavior_tree = behavior_tree;
  return true;
}

void NavToPoseAction::onFeedback(
    const std::shared_ptr<const Feedback> &feedback) {
  if (!feedback) {
    return;
  }
  const auto blackboard = blackboardOf(*this);
  if (!blackboard) {
    return;
  }
  blackboard->set("nav_last_exec_state", std::string("RUNNING"));
  blackboard->set("nav_last_distance_remaining",
                  static_cast<double>(feedback->distance_remaining));
  blackboard->set("nav_last_recovery_count",
                  static_cast<int>(feedback->number_of_recoveries));
}

void NavToPoseAction::onGoalAccepted() {
  if (const auto blackboard = blackboardOf(*this)) {
    blackboard->set("nav_last_exec_state", std::string("RUNNING"));
    blackboard->set("nav_last_failure_code", std::string(""));
    blackboard->set("nav_last_failure_reason", std::string(""));
  }
}

void NavToPoseAction::onActionFailure(uint16_t error_code,
                                      const std::string &failure_code,
                                      const std::string &failure_reason) {
  (void)error_code;
  writeFailure(blackboardOf(*this), failure_code, failure_reason);
}

void NavToPoseAction::onHaltHook() { bt_navigator_state_future_ = {}; }

BT::NodeStatus NavToPoseAction::handleResult(const WrappedResult &result,
                                             uint16_t &error_code) {
  const auto blackboard = blackboardOf(*this);
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    rclcpp::Node *node = nullptr;
    std::string failure_reason;
    if (!config().blackboard || !config().blackboard->get("node", node) ||
        !node || !isAtRequestedGoal(*node, failure_reason)) {
      if (failure_reason.empty()) {
        failure_reason = "Nav2 reported success but reached pose was not verified";
      }
      writeFailure(blackboard, "SUCCESS_POSE_MISMATCH", failure_reason);
      error_code = kErrorGoalPoseMismatch;
      return BT::NodeStatus::FAILURE;
    }
    if (blackboard) {
      blackboard->set("nav_last_exec_state", std::string("SUCCEEDED"));
      blackboard->set("nav_last_failure_code", std::string(""));
      blackboard->set("nav_last_failure_reason", std::string(""));
    }
    error_code = 0;
    return BT::NodeStatus::SUCCESS;
  }

  switch (result.code) {
  case rclcpp_action::ResultCode::ABORTED:
    writeFailure(blackboard, "ACTION_ABORTED", "navigate_to_pose aborted");
    error_code = kErrorActionAborted;
    break;
  case rclcpp_action::ResultCode::CANCELED:
    writeFailure(blackboard, "ACTION_CANCELED", "navigate_to_pose canceled");
    error_code = kErrorActionCanceled;
    break;
  case rclcpp_action::ResultCode::UNKNOWN:
    writeFailure(blackboard, "ACTION_UNKNOWN", "navigate_to_pose failed");
    error_code = kErrorActionUnknown;
    break;
  default:
    writeFailure(blackboard, "ACTION_FAILED", "navigate_to_pose failed");
    error_code = kErrorActionFailed;
    break;
  }
  return BT::NodeStatus::FAILURE;
}

void registerNav2PoseNodes(BT::BehaviorTreeFactory &factory) {
  factory.registerNodeType<CaptureCurrentPoseAction>("CaptureCurrentPose");
  factory.registerNodeType<OdomRelativeDriveAction>("OdomRelativeDrive");
  factory.registerNodeType<RelativeYawTargetAction>("RelativeYawTarget");
  factory.registerNodeType<NavToPoseAction>("NavToPose");
}

} // namespace rc26_decision
