#include "rc26_decision/mf/grid_center.hpp"

#include <algorithm>
#include <cmath>

namespace rc26_decision {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kMinPositive = 0.001;

bool validGrid(int grid_id) { return grid_id >= 1 && grid_id <= 12; }

int gridRow(int grid_id) { return (grid_id - 1) / 3; }

int gridCol(int grid_id) { return (grid_id - 1) % 3; }

double absFiniteOr(double value, double fallback) {
  return std::isfinite(value) ? std::abs(value) : fallback;
}

} // namespace

void loadGridCenterParams(rclcpp::Node &node,
                          const BT::Blackboard::Ptr &blackboard) {
  GridCenterParams p;
  p.cmd_vel_topic = node.declare_parameter<std::string>(
      "mf_center_cmd_vel_topic", p.cmd_vel_topic);
  p.odom_topic =
      node.declare_parameter<std::string>("mf_center_odom_topic", p.odom_topic);
  p.grid_step_m =
      node.declare_parameter<double>("mf_center_grid_step_m", p.grid_step_m);
  p.entry_forward_offset_m = node.declare_parameter<double>(
      "mf_center_entry_forward_offset_m", p.entry_forward_offset_m);
  p.entry_forward_speed_mps = node.declare_parameter<double>(
      "mf_center_entry_forward_speed_mps", p.entry_forward_speed_mps);
  p.xy_kp = node.declare_parameter<double>("mf_center_xy_kp", p.xy_kp);
  p.min_speed_mps = node.declare_parameter<double>(
      "mf_center_min_speed_mps", p.min_speed_mps);
  p.max_speed_mps = node.declare_parameter<double>(
      "mf_center_max_speed_mps", p.max_speed_mps);
  p.xy_tolerance_m = node.declare_parameter<double>(
      "mf_center_xy_tolerance_m", p.xy_tolerance_m);
  p.yaw_kp =
      node.declare_parameter<double>("mf_center_yaw_kp", p.yaw_kp);
  p.yaw_max_speed_radps = node.declare_parameter<double>(
      "mf_center_yaw_max_speed_radps", p.yaw_max_speed_radps);
  p.yaw_tolerance_deg = node.declare_parameter<double>(
      "mf_center_yaw_tolerance_deg", p.yaw_tolerance_deg);
  p.stable_ticks =
      node.declare_parameter<int>("mf_center_stable_ticks", p.stable_ticks);
  p.odom_timeout_s = node.declare_parameter<double>(
      "mf_center_odom_timeout_s", p.odom_timeout_s);
  p.align_timeout_s = node.declare_parameter<double>(
      "mf_center_align_timeout_s", p.align_timeout_s);

  p.grid_step_m = std::max(kMinPositive, absFiniteOr(p.grid_step_m, 1.2));
  p.entry_forward_offset_m =
      std::max(0.0, std::isfinite(p.entry_forward_offset_m)
                        ? p.entry_forward_offset_m
                        : 0.25);
  p.entry_forward_speed_mps =
      std::max(kMinPositive, absFiniteOr(p.entry_forward_speed_mps, 0.04));
  p.xy_kp = std::max(0.0, std::isfinite(p.xy_kp) ? p.xy_kp : 0.8);
  p.min_speed_mps = std::max(0.0, absFiniteOr(p.min_speed_mps, 0.010));
  p.max_speed_mps = std::max(p.min_speed_mps,
                             absFiniteOr(p.max_speed_mps, 0.050));
  p.xy_tolerance_m =
      std::max(kMinPositive, absFiniteOr(p.xy_tolerance_m, 0.035));
  p.yaw_kp = std::max(0.0, std::isfinite(p.yaw_kp) ? p.yaw_kp : 1.2);
  p.yaw_max_speed_radps =
      std::max(0.0, absFiniteOr(p.yaw_max_speed_radps, 0.30));
  p.yaw_tolerance_deg =
      std::max(0.0, absFiniteOr(p.yaw_tolerance_deg, 3.0));
  p.stable_ticks = std::max(1, p.stable_ticks);
  p.odom_timeout_s =
      std::max(kMinPositive, absFiniteOr(p.odom_timeout_s, 0.5));
  p.align_timeout_s =
      std::max(kMinPositive, absFiniteOr(p.align_timeout_s, 8.0));

  blackboard->set("mf_center_params", p);
  blackboard->set("mf_center_reference_grid", 0);
  blackboard->set("mf_center_reference_x", 0.0);
  blackboard->set("mf_center_reference_y", 0.0);
  blackboard->set("mf_center_reference_yaw", 0.0);
  blackboard->set("mf_center_target_grid", 0);
  blackboard->set("mf_center_target_x", 0.0);
  blackboard->set("mf_center_target_y", 0.0);
  blackboard->set("mf_center_error_x", 0.0);
  blackboard->set("mf_center_error_y", 0.0);
  blackboard->set("mf_center_error_distance", 0.0);
  blackboard->set("mf_center_error", std::string(""));

  RCLCPP_INFO(node.get_logger(),
              "MF 格中心参数已加载: cmd_vel=%s odom=%s step=%.3fm entry_offset=%.3fm entry_speed=%.3fm/s xy_tol=%.3fm yaw_tol=%.1fdeg",
              p.cmd_vel_topic.c_str(), p.odom_topic.c_str(), p.grid_step_m,
              p.entry_forward_offset_m, p.entry_forward_speed_mps,
              p.xy_tolerance_m, p.yaw_tolerance_deg);
}

GridCenterActionBase::GridCenterActionBase(const std::string &name,
                                           const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config) {}

bool GridCenterActionBase::setupRuntime(const char *action_label) {
  action_label_ = action_label ? action_label : "GridCenter";
  if (!config().blackboard || !config().blackboard->get("node", node_) ||
      !node_) {
    return false;
  }
  if (!config().blackboard->get("mf_center_params", params_)) {
    RCLCPP_ERROR(node_->get_logger(), "%s: 黑板缺少 mf_center_params",
                 action_label_.c_str());
    return false;
  }

  params_.grid_step_m =
      std::max(kMinPositive, absFiniteOr(params_.grid_step_m, 1.2));
  params_.entry_forward_offset_m =
      std::max(0.0, std::isfinite(params_.entry_forward_offset_m)
                        ? params_.entry_forward_offset_m
                        : 0.25);
  params_.entry_forward_speed_mps = std::max(
      kMinPositive, absFiniteOr(params_.entry_forward_speed_mps, 0.04));
  params_.xy_kp =
      std::max(0.0, std::isfinite(params_.xy_kp) ? params_.xy_kp : 0.8);
  params_.min_speed_mps =
      std::max(0.0, absFiniteOr(params_.min_speed_mps, 0.010));
  params_.max_speed_mps = std::max(
      params_.min_speed_mps, absFiniteOr(params_.max_speed_mps, 0.050));
  params_.xy_tolerance_m =
      std::max(kMinPositive, absFiniteOr(params_.xy_tolerance_m, 0.035));
  params_.yaw_kp =
      std::max(0.0, std::isfinite(params_.yaw_kp) ? params_.yaw_kp : 1.2);
  params_.yaw_max_speed_radps =
      std::max(0.0, absFiniteOr(params_.yaw_max_speed_radps, 0.30));
  params_.yaw_tolerance_deg =
      std::max(0.0, absFiniteOr(params_.yaw_tolerance_deg, 3.0));
  params_.stable_ticks = std::max(1, params_.stable_ticks);
  params_.odom_timeout_s =
      std::max(kMinPositive, absFiniteOr(params_.odom_timeout_s, 0.5));
  params_.align_timeout_s =
      std::max(kMinPositive, absFiniteOr(params_.align_timeout_s, 8.0));

  cmd_pub_ =
      node_->create_publisher<TwistMsg>(params_.cmd_vel_topic, rclcpp::QoS(10));
  if (!params_.odom_topic.empty()) {
    odom_sub_ = node_->create_subscription<OdomMsg>(
        params_.odom_topic, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const OdomMsg::SharedPtr msg) {
          current_x_ = msg->pose.pose.position.x;
          current_y_ = msg->pose.pose.position.y;
          current_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
          has_odom_ = true;
          last_odom_tp_ = std::chrono::steady_clock::now();
        });
  }
  stable_ticks_ = 0;
  has_odom_ = false;
  markStart();
  setCenterError("");

  RCLCPP_INFO(node_->get_logger(), "%s 启动: cmd_vel=%s odom=%s",
              action_label_.c_str(), params_.cmd_vel_topic.c_str(),
              params_.odom_topic.c_str());
  return true;
}

void GridCenterActionBase::releaseRuntime() {
  odom_sub_.reset();
  cmd_pub_.reset();
  node_ = nullptr;
  has_odom_ = false;
  stable_ticks_ = 0;
}

void GridCenterActionBase::publishStop() {
  if (cmd_pub_) {
    cmd_pub_->publish(TwistMsg{});
  }
}

bool GridCenterActionBase::odomReady() const {
  if (!has_odom_) {
    return false;
  }
  const auto age_s = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - last_odom_tp_)
                         .count();
  return age_s <= params_.odom_timeout_s;
}

bool GridCenterActionBase::timedOut() const {
  if (!node_) {
    return false;
  }
  return (node_->now() - start_time_).seconds() > params_.align_timeout_s;
}

void GridCenterActionBase::markStart() {
  if (node_) {
    start_time_ = node_->now();
  }
}

void GridCenterActionBase::writeReferenceGrid(int grid_id, double x, double y,
                                              double yaw) {
  if (!config().blackboard) {
    return;
  }
  config().blackboard->set("mf_center_reference_grid", grid_id);
  config().blackboard->set("mf_center_reference_x", x);
  config().blackboard->set("mf_center_reference_y", y);
  config().blackboard->set("mf_center_reference_yaw", normalizeAngle(yaw));
  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
                "%s: 记录 grid%d 中心参考 x=%.3f y=%.3f yaw=%.3f",
                action_label_.c_str(), grid_id, x, y, normalizeAngle(yaw));
  }
}

bool GridCenterActionBase::readReferenceGrid(int &grid_id, double &x, double &y,
                                             double &yaw) const {
  if (!config().blackboard) {
    return false;
  }
  if (!config().blackboard->get("mf_center_reference_grid", grid_id) ||
      !config().blackboard->get("mf_center_reference_x", x) ||
      !config().blackboard->get("mf_center_reference_y", y) ||
      !config().blackboard->get("mf_center_reference_yaw", yaw)) {
    return false;
  }
  return validGrid(grid_id) && std::isfinite(x) && std::isfinite(y) &&
         std::isfinite(yaw);
}

bool GridCenterActionBase::computeGridCenterFromReference(
    int target_grid, double &target_x, double &target_y) const {
  int reference_grid = 0;
  double reference_x = 0.0;
  double reference_y = 0.0;
  double reference_yaw = 0.0;
  if (!validGrid(target_grid) ||
      !readReferenceGrid(reference_grid, reference_x, reference_y,
                         reference_yaw)) {
    return false;
  }

  const int row_delta = gridRow(target_grid) - gridRow(reference_grid);
  const int col_delta = gridCol(target_grid) - gridCol(reference_grid);
  target_x = reference_x + static_cast<double>(row_delta) * params_.grid_step_m;
  target_y = reference_y - static_cast<double>(col_delta) * params_.grid_step_m;
  return true;
}

BT::NodeStatus GridCenterActionBase::tickTowardTarget(double target_x,
                                                      double target_y,
                                                      double target_yaw) {
  if (!node_ || !cmd_pub_) {
    return BT::NodeStatus::FAILURE;
  }
  if (timedOut()) {
    return failWithStop("center align timeout");
  }
  if (!odomReady()) {
    stable_ticks_ = 0;
    publishStop();
    return BT::NodeStatus::RUNNING;
  }

  const double error_x = target_x - current_x_;
  const double error_y = target_y - current_y_;
  const double distance = std::hypot(error_x, error_y);
  const double yaw_error = normalizeAngle(target_yaw - current_yaw_);

  if (config().blackboard) {
    config().blackboard->set("mf_center_target_x", target_x);
    config().blackboard->set("mf_center_target_y", target_y);
    config().blackboard->set("mf_center_error_x", error_x);
    config().blackboard->set("mf_center_error_y", error_y);
    config().blackboard->set("mf_center_error_distance", distance);
  }

  const double yaw_tolerance_rad = params_.yaw_tolerance_deg * kDeg2Rad;
  if (distance <= params_.xy_tolerance_m &&
      std::abs(yaw_error) <= yaw_tolerance_rad) {
    ++stable_ticks_;
    publishStop();
    if (stable_ticks_ >= params_.stable_ticks) {
      setCenterError("");
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  stable_ticks_ = 0;
  TwistMsg cmd;
  if (distance > params_.xy_tolerance_m) {
    const double world_vx = params_.xy_kp * error_x;
    const double world_vy = params_.xy_kp * error_y;
    const double c = std::cos(current_yaw_);
    const double s = std::sin(current_yaw_);
    double body_vx = c * world_vx + s * world_vy;
    double body_vy = -s * world_vx + c * world_vy;
    double body_speed = std::hypot(body_vx, body_vy);

    if (body_speed > params_.max_speed_mps && body_speed > 0.0) {
      const double scale = params_.max_speed_mps / body_speed;
      body_vx *= scale;
      body_vy *= scale;
      body_speed = params_.max_speed_mps;
    }
    if (body_speed < params_.min_speed_mps && body_speed > 1e-9) {
      const double scale = params_.min_speed_mps / body_speed;
      body_vx *= scale;
      body_vy *= scale;
    }
    cmd.linear.x = body_vx;
    cmd.linear.y = body_vy;
  }

  const double raw_wz = params_.yaw_kp * yaw_error;
  cmd.angular.z = std::clamp(raw_wz, -params_.yaw_max_speed_radps,
                             params_.yaw_max_speed_radps);
  cmd_pub_->publish(cmd);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GridCenterActionBase::failWithStop(const char *reason) {
  if (node_) {
    RCLCPP_WARN(node_->get_logger(), "%s 失败: %s", action_label_.c_str(),
                reason ? reason : "unknown");
  }
  setCenterError(reason ? reason : "center_align_failed");
  publishStop();
  releaseRuntime();
  return BT::NodeStatus::FAILURE;
}

void GridCenterActionBase::setCenterError(const std::string &reason) const {
  if (!config().blackboard) {
    return;
  }
  config().blackboard->set("mf_center_error", reason);
  config().blackboard->set("mf_transition_error", reason);
}

double GridCenterActionBase::normalizeAngle(double angle_rad) {
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

double GridCenterActionBase::yawFromQuaternion(
    const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

CaptureGridCenterReferenceAction::CaptureGridCenterReferenceAction(
    const std::string &name, const BT::NodeConfig &config)
    : GridCenterActionBase(name, config) {}

BT::PortsList CaptureGridCenterReferenceAction::providedPorts() {
  return {BT::InputPort<int>("reference_grid", 2,
                             "当前位姿对应的 MF 格中心 ID")};
}

BT::NodeStatus CaptureGridCenterReferenceAction::onStart() {
  if (!setupRuntime("MF格中心参考捕获")) {
    return BT::NodeStatus::FAILURE;
  }
  (void)getInput("reference_grid", reference_grid_);
  if (!validGrid(reference_grid_)) {
    return failWithStop("invalid reference_grid");
  }
  return tryCapture();
}

BT::NodeStatus CaptureGridCenterReferenceAction::onRunning() {
  return tryCapture();
}

void CaptureGridCenterReferenceAction::onHalted() {
  publishStop();
  releaseRuntime();
}

BT::NodeStatus CaptureGridCenterReferenceAction::tryCapture() {
  if (timedOut()) {
    return failWithStop("capture center reference timeout");
  }
  if (!odomReady()) {
    publishStop();
    return BT::NodeStatus::RUNNING;
  }
  writeReferenceGrid(reference_grid_, current_x_, current_y_, current_yaw_);
  publishStop();
  releaseRuntime();
  return BT::NodeStatus::SUCCESS;
}

MFEntryCenterAdvanceAction::MFEntryCenterAdvanceAction(
    const std::string &name, const BT::NodeConfig &config)
    : GridCenterActionBase(name, config) {}

BT::PortsList MFEntryCenterAdvanceAction::providedPorts() {
  return {
      BT::InputPort<int>("reference_grid", 2,
                         "入口前进结束后记录为哪个 MF 格中心"),
      BT::InputPort<double>("target_yaw_rad", 0.0,
                            "入口前进时保持的目标 yaw"),
  };
}

BT::NodeStatus MFEntryCenterAdvanceAction::onStart() {
  if (!setupRuntime("MF入口格中心前进")) {
    return BT::NodeStatus::FAILURE;
  }
  (void)getInput("reference_grid", reference_grid_);
  (void)getInput("target_yaw_rad", target_yaw_rad_);
  if (!std::isfinite(target_yaw_rad_)) {
    target_yaw_rad_ = 0.0;
  }
  target_yaw_rad_ = normalizeAngle(target_yaw_rad_);
  if (!validGrid(reference_grid_)) {
    return failWithStop("invalid reference_grid");
  }
  params_.max_speed_mps =
      std::min(params_.max_speed_mps, params_.entry_forward_speed_mps);
  params_.min_speed_mps = std::min(params_.min_speed_mps, params_.max_speed_mps);
  target_ready_ = false;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MFEntryCenterAdvanceAction::onRunning() {
  if (!target_ready_) {
    if (timedOut()) {
      return failWithStop("entry center odom timeout");
    }
    if (!odomReady()) {
      publishStop();
      return BT::NodeStatus::RUNNING;
    }
    if (!prepareTargetFromCurrentOdom()) {
      return failWithStop("entry center target prepare failed");
    }
  }

  const BT::NodeStatus status =
      tickTowardTarget(target_x_, target_y_, target_yaw_rad_);
  if (status == BT::NodeStatus::SUCCESS) {
    writeReferenceGrid(reference_grid_, target_x_, target_y_, target_yaw_rad_);
    publishStop();
    releaseRuntime();
  }
  return status;
}

void MFEntryCenterAdvanceAction::onHalted() {
  publishStop();
  releaseRuntime();
  target_ready_ = false;
}

bool MFEntryCenterAdvanceAction::prepareTargetFromCurrentOdom() {
  if (!odomReady()) {
    return false;
  }
  target_x_ =
      current_x_ + params_.entry_forward_offset_m * std::cos(target_yaw_rad_);
  target_y_ =
      current_y_ + params_.entry_forward_offset_m * std::sin(target_yaw_rad_);
  target_ready_ = true;
  if (config().blackboard) {
    config().blackboard->set("mf_center_target_grid", reference_grid_);
    config().blackboard->set("mf_center_target_x", target_x_);
    config().blackboard->set("mf_center_target_y", target_y_);
  }
  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
                "%s: 从当前位姿前进 %.3fm 到 grid%d 中心参考 x=%.3f y=%.3f",
                action_label_.c_str(), params_.entry_forward_offset_m,
                reference_grid_, target_x_, target_y_);
  }
  return true;
}

GridCenterAlignAction::GridCenterAlignAction(const std::string &name,
                                             const BT::NodeConfig &config)
    : GridCenterActionBase(name, config) {}

BT::PortsList GridCenterAlignAction::providedPorts() {
  return {
      BT::InputPort<int>("target_grid", "目标 MF 格中心 ID"),
      BT::InputPort<double>("target_yaw_rad", 0.0,
                            "归位时保持的目标 yaw"),
  };
}

BT::NodeStatus GridCenterAlignAction::onStart() {
  if (!setupRuntime("MF格中心归位")) {
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("target_grid", target_grid_) || !validGrid(target_grid_)) {
    return failWithStop("invalid target_grid");
  }
  (void)getInput("target_yaw_rad", target_yaw_rad_);
  if (!std::isfinite(target_yaw_rad_)) {
    target_yaw_rad_ = 0.0;
  }
  target_yaw_rad_ = normalizeAngle(target_yaw_rad_);
  if (!computeGridCenterFromReference(target_grid_, target_x_, target_y_)) {
    return failWithStop("missing center reference");
  }
  if (config().blackboard) {
    config().blackboard->set("mf_center_target_grid", target_grid_);
    config().blackboard->set("mf_center_target_x", target_x_);
    config().blackboard->set("mf_center_target_y", target_y_);
  }
  if (node_) {
    RCLCPP_INFO(node_->get_logger(),
                "%s: 目标 grid%d 中心 x=%.3f y=%.3f yaw=%.3f",
                action_label_.c_str(), target_grid_, target_x_, target_y_,
                target_yaw_rad_);
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GridCenterAlignAction::onRunning() {
  const BT::NodeStatus status =
      tickTowardTarget(target_x_, target_y_, target_yaw_rad_);
  if (status == BT::NodeStatus::SUCCESS) {
    publishStop();
    releaseRuntime();
  }
  return status;
}

void GridCenterAlignAction::onHalted() {
  publishStop();
  releaseRuntime();
}

void registerGridCenterNodes(BT::BehaviorTreeFactory &factory) {
  factory.registerNodeType<CaptureGridCenterReferenceAction>(
      "CaptureGridCenterReference");
  factory.registerNodeType<MFEntryCenterAdvanceAction>("MFEntryCenterAdvance");
  factory.registerNodeType<GridCenterAlignAction>("GridCenterAlign");
}

} // namespace rc26_decision
