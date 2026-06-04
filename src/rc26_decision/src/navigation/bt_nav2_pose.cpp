#include "rc26_decision/navigation/bt_nav2_pose.hpp"

#include <cmath>
#include <string>

namespace rc26_decision {

namespace {

constexpr uint16_t kErrorActionFailed = 1;
constexpr uint16_t kErrorActionAborted = 120;
constexpr uint16_t kErrorActionCanceled = 121;
constexpr uint16_t kErrorActionUnknown = 122;

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

} // namespace

NavToPoseAction::NavToPoseAction(const std::string &name,
                                 const BT::NodeConfig &config)
    : BtActionNode<NavigateToPose>(name, config, "navigate_to_pose",
                                   std::chrono::seconds(60)) {}

BT::PortsList NavToPoseAction::providedPorts() {
  auto ports = BtActionNode<NavigateToPose>::basePorts(60.0);
  ports.insert(BT::InputPort<std::string>("frame_id", "map", "Goal frame"));
  ports.insert(BT::InputPort<double>("x", "Goal x in frame_id"));
  ports.insert(BT::InputPort<double>("y", "Goal y in frame_id"));
  ports.insert(BT::InputPort<double>("yaw", 0.0, "Goal yaw in radians"));
  ports.insert(BT::InputPort<std::string>("behavior_tree", "",
                                          "Optional Nav2 BT XML path"));
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

BT::NodeStatus NavToPoseAction::handleResult(const WrappedResult &result,
                                             uint16_t &error_code) {
  const auto blackboard = blackboardOf(*this);
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
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
  factory.registerNodeType<NavToPoseAction>("NavToPose");
}

} // namespace rc26_decision
