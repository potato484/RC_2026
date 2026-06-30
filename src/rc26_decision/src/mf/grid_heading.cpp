#include "rc26_decision/mf/grid_heading.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "rc26_decision/decision_failure.hpp"

namespace rc26_decision {

namespace {

constexpr double kDeg2Rad = M_PI / 180.0;

double selectDirectionYaw(const GridHeadingParams &params) {
  if (params.direction == "forward") {
    return params.forward_yaw_rad;
  }
  if (params.direction == "left") {
    return params.left_yaw_rad;
  }
  if (params.direction == "right") {
    return params.right_yaw_rad;
  }
  if (params.direction == "backward") {
    return params.backward_yaw_rad;
  }
  throw std::invalid_argument(
      "grid_heading_direction must be one of forward | left | right | backward");
}

} // namespace

void loadGridHeadingParams(rclcpp::Node &node,
                           const BT::Blackboard::Ptr &blackboard) {
  GridHeadingParams p;
  p.direction =
      node.declare_parameter<std::string>("grid_heading_direction", p.direction);
  p.forward_yaw_rad = node.declare_parameter<double>(
      "grid_heading_forward_yaw_rad", p.forward_yaw_rad);
  p.left_yaw_rad = node.declare_parameter<double>("grid_heading_left_yaw_rad",
                                                  p.left_yaw_rad);
  p.right_yaw_rad = node.declare_parameter<double>(
      "grid_heading_right_yaw_rad", p.right_yaw_rad);
  p.backward_yaw_rad = node.declare_parameter<double>(
      "grid_heading_backward_yaw_rad", p.backward_yaw_rad);
  p.cmd_vel_topic = node.declare_parameter<std::string>(
      "grid_heading_cmd_vel_topic", p.cmd_vel_topic);
  p.odom_topic = node.declare_parameter<std::string>("grid_heading_odom_topic",
                                                     p.odom_topic);
  p.kp = node.declare_parameter<double>("grid_heading_kp", p.kp);
  p.turn_max_speed_radps = node.declare_parameter<double>(
      "grid_heading_turn_max_speed_radps", p.turn_max_speed_radps);
  p.align_max_speed_radps = node.declare_parameter<double>(
      "grid_heading_align_max_speed_radps", p.align_max_speed_radps);
  p.turn_gate_deg = node.declare_parameter<double>("grid_heading_turn_gate_deg",
                                                   p.turn_gate_deg);
  p.align_tolerance_deg = node.declare_parameter<double>(
      "grid_heading_align_tolerance_deg", p.align_tolerance_deg);
  p.align_stable_ticks = node.declare_parameter<int>(
      "grid_heading_align_stable_ticks", p.align_stable_ticks);
  p.odom_timeout_s = node.declare_parameter<double>(
      "grid_heading_odom_timeout_s", p.odom_timeout_s);
  p.turn_timeout_s = node.declare_parameter<double>(
      "grid_heading_turn_timeout_s", p.turn_timeout_s);
  p.align_timeout_s = node.declare_parameter<double>(
      "grid_heading_align_timeout_s", p.align_timeout_s);

  p.kp = std::max(0.0, p.kp);
  p.turn_max_speed_radps = std::max(0.0, std::abs(p.turn_max_speed_radps));
  p.align_max_speed_radps = std::max(0.0, std::abs(p.align_max_speed_radps));
  p.align_tolerance_deg = std::max(0.0, p.align_tolerance_deg);
  p.turn_gate_deg = std::max(p.align_tolerance_deg, p.turn_gate_deg);
  p.align_stable_ticks = std::max(1, p.align_stable_ticks);
  p.odom_timeout_s = std::max(0.001, p.odom_timeout_s);
  p.turn_timeout_s = std::max(0.001, p.turn_timeout_s);
  p.align_timeout_s = std::max(0.001, p.align_timeout_s);

  const double selected_yaw_rad = selectDirectionYaw(p);
  blackboard->set("grid_heading_params", p);
  blackboard->set("grid_heading_direction", p.direction);
  blackboard->set("grid_heading_target_yaw_rad", selected_yaw_rad);

  RCLCPP_INFO(node.get_logger(),
              "Grid heading 参数已加载: direction=%s target_yaw=%.4frad cmd_vel=%s odom=%s kp=%.2f turn_max=%.2frad/s align_max=%.2frad/s gate=%.1fdeg tol=%.1fdeg",
              p.direction.c_str(), selected_yaw_rad, p.cmd_vel_topic.c_str(),
              p.odom_topic.c_str(), p.kp, p.turn_max_speed_radps,
              p.align_max_speed_radps, p.turn_gate_deg,
              p.align_tolerance_deg);
}

GridHeadingActionBase::GridHeadingActionBase(const std::string &name,
                                             const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config), start_time_(0, 0, RCL_ROS_TIME) {}

BT::PortsList GridHeadingActionBase::providedPorts() {
  return {BT::InputPort<double>("target_yaw_rad", "目标 yaw，单位 rad")};
}

bool GridHeadingActionBase::setupRuntime(const char *action_label) {
  action_label_ = action_label ? action_label : "GridHeading";
  if (!config().blackboard->get("node", node_) || node_ == nullptr) {
    writeDecisionFailure(config().blackboard, action_label_,
                         "运行上下文缺失：node 不可用");
    return false;
  }
  if (!config().blackboard->get("grid_heading_params", params_)) {
    RCLCPP_ERROR(node_->get_logger(), "%s: 黑板缺少 grid_heading_params",
                 action_label_.c_str());
    writeDecisionFailure(config().blackboard, action_label_,
                         "黑板缺少 grid_heading_params");
    return false;
  }
  if (!readTargetYaw()) {
    RCLCPP_ERROR(node_->get_logger(), "%s: 缺少 target_yaw_rad 输入",
                 action_label_.c_str());
    writeDecisionFailure(config().blackboard, action_label_,
                         "缺少 target_yaw_rad 输入");
    return false;
  }

  cmd_pub_ =
      node_->create_publisher<TwistMsg>(params_.cmd_vel_topic, rclcpp::QoS(10));
  if (!params_.odom_topic.empty()) {
    odom_sub_ = node_->create_subscription<OdomMsg>(
        params_.odom_topic, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const OdomMsg::SharedPtr msg) {
          current_yaw_rad_ = yawFromQuaternion(msg->pose.pose.orientation);
          has_yaw_ = true;
          last_odom_tp_ = std::chrono::steady_clock::now();
        });
  }
  if (node_) {
    start_time_ = node_->now();
  }
  RCLCPP_INFO(node_->get_logger(), "%s 启动: target_yaw=%.4frad odom=%s",
              action_label_.c_str(), target_yaw_rad_,
              params_.odom_topic.c_str());
  return true;
}

bool GridHeadingActionBase::readTargetYaw() {
  double target_yaw = 0.0;
  if (!getInput("target_yaw_rad", target_yaw)) {
    return false;
  }
  target_yaw_rad_ = normalizeAngle(target_yaw);
  return true;
}

void GridHeadingActionBase::releaseRuntime() {
  odom_sub_.reset();
  cmd_pub_.reset();
  node_ = nullptr;
  has_yaw_ = false;
}

void GridHeadingActionBase::publishStop() {
  if (cmd_pub_) {
    cmd_pub_->publish(TwistMsg{});
  }
}

void GridHeadingActionBase::publishAngular(double angular_z_radps) {
  if (!cmd_pub_) {
    return;
  }
  TwistMsg msg;
  msg.angular.z = angular_z_radps;
  cmd_pub_->publish(msg);
}

bool GridHeadingActionBase::odomReady() const {
  if (!has_yaw_) {
    return false;
  }
  const auto age_s = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - last_odom_tp_)
                         .count();
  return age_s <= params_.odom_timeout_s;
}

double GridHeadingActionBase::headingError() const {
  if (!has_yaw_) {
    return 0.0;
  }
  return normalizeAngle(target_yaw_rad_ - current_yaw_rad_);
}

double GridHeadingActionBase::headingAngularZ(double max_speed_radps) const {
  const double raw = params_.kp * headingError();
  const double limit = std::abs(max_speed_radps);
  return std::clamp(raw, -limit, limit);
}

double GridHeadingActionBase::elapsedSinceStart() const {
  if (!node_) {
    return 0.0;
  }
  return (node_->now() - start_time_).seconds();
}

BT::NodeStatus GridHeadingActionBase::failWithStop(const char *reason) {
  const std::string detail =
      std::string(reason ? reason : "未知原因") +
      "，目标yaw_rad=" + std::to_string(target_yaw_rad_) +
      "，当前yaw_rad=" + std::to_string(current_yaw_rad_) +
      "，odom=" + params_.odom_topic +
      "，转向超时_s=" + std::to_string(params_.turn_timeout_s) +
      "，对齐超时_s=" + std::to_string(params_.align_timeout_s);
  if (node_) {
    RCLCPP_WARN(node_->get_logger(), "%s 失败: %s", action_label_.c_str(),
                detail.c_str());
  }
  writeDecisionFailure(config().blackboard, action_label_, detail);
  publishStop();
  releaseRuntime();
  return BT::NodeStatus::FAILURE;
}

double GridHeadingActionBase::normalizeAngle(double angle_rad) {
  while (angle_rad > M_PI) {
    angle_rad -= 2.0 * M_PI;
  }
  while (angle_rad < -M_PI) {
    angle_rad += 2.0 * M_PI;
  }
  return angle_rad;
}

double GridHeadingActionBase::yawFromQuaternion(
    const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

GridTurnAction::GridTurnAction(const std::string &name,
                               const BT::NodeConfig &config)
    : GridHeadingActionBase(name, config) {}

BT::NodeStatus GridTurnAction::onStart() {
  if (!setupRuntime("GridTurn")) {
    return BT::NodeStatus::FAILURE;
  }
  publishStop();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GridTurnAction::onRunning() {
  if (elapsedSinceStart() > params_.turn_timeout_s) {
    return failWithStop("转向超时");
  }
  if (!odomReady()) {
    publishStop();
    return BT::NodeStatus::RUNNING;
  }

  const double gate_rad = params_.turn_gate_deg * kDeg2Rad;
  if (std::abs(headingError()) <= gate_rad) {
    publishStop();
    releaseRuntime();
    return BT::NodeStatus::SUCCESS;
  }

  publishAngular(headingAngularZ(params_.turn_max_speed_radps));
  return BT::NodeStatus::RUNNING;
}

void GridTurnAction::onHalted() {
  publishStop();
  releaseRuntime();
}

GridHeadingAlignAction::GridHeadingAlignAction(const std::string &name,
                                               const BT::NodeConfig &config)
    : GridHeadingActionBase(name, config) {}

BT::NodeStatus GridHeadingAlignAction::onStart() {
  stable_ticks_ = 0;
  if (!setupRuntime("GridHeadingAlign")) {
    return BT::NodeStatus::FAILURE;
  }
  publishStop();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GridHeadingAlignAction::onRunning() {
  if (elapsedSinceStart() > params_.align_timeout_s) {
    return failWithStop("对齐超时");
  }
  if (!odomReady()) {
    stable_ticks_ = 0;
    publishStop();
    return BT::NodeStatus::RUNNING;
  }

  const double tolerance_rad = params_.align_tolerance_deg * kDeg2Rad;
  if (std::abs(headingError()) <= tolerance_rad) {
    ++stable_ticks_;
    publishStop();
    if (stable_ticks_ >= params_.align_stable_ticks) {
      releaseRuntime();
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  stable_ticks_ = 0;
  publishAngular(headingAngularZ(params_.align_max_speed_radps));
  return BT::NodeStatus::RUNNING;
}

void GridHeadingAlignAction::onHalted() {
  publishStop();
  releaseRuntime();
  stable_ticks_ = 0;
}

void registerGridHeadingNodes(BT::BehaviorTreeFactory &factory) {
  factory.registerNodeType<GridTurnAction>("GridTurn");
  factory.registerNodeType<GridHeadingAlignAction>("GridHeadingAlign");
}

} // namespace rc26_decision
