#include "rc26_decision/navigation/bt_nav2_pose.hpp"

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
  factory.registerNodeType<NavToPoseAction>("NavToPose");
}

} // namespace rc26_decision
