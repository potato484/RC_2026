#pragma once

#include "rc26_decision/common/bt_action_node.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <string>

namespace rc26_decision {

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GetLifecycleState = lifecycle_msgs::srv::GetState;

void loadOdomRightTurnNavParams(rclcpp::Node &node,
                                const BT::Blackboard::Ptr &blackboard);

class NavToPoseAction : public BtActionNode<NavigateToPose> {
public:
  NavToPoseAction(const std::string &name, const BT::NodeConfig &config);
  static BT::PortsList providedPorts();

protected:
  bool isActionReady(rclcpp::Node &node) override;
  bool buildGoal(Goal &goal) override;
  void onFeedback(const std::shared_ptr<const Feedback> &feedback) override;
  BT::NodeStatus handleResult(const WrappedResult &result,
                              uint16_t &error_code) override;
  void onGoalAccepted() override;
  void onActionFailure(uint16_t error_code, const std::string &failure_code,
                       const std::string &failure_reason) override;
  void onHaltHook() override;

private:
  bool hasFreshGoalFrameTf(rclcpp::Node &node);
  bool isAtRequestedGoal(rclcpp::Node &node, std::string &failure_reason);

  rclcpp::Client<GetLifecycleState>::SharedPtr bt_navigator_state_client_;
  rclcpp::Client<GetLifecycleState>::SharedFuture bt_navigator_state_future_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

class CaptureCurrentPoseAction : public BT::StatefulActionNode {
public:
  CaptureCurrentPoseAction(const std::string &name,
                           const BT::NodeConfig &config);
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  BT::NodeStatus tryCapture();

  rclcpp::Node *node_{nullptr};
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::milliseconds timeout_{std::chrono::milliseconds(5000)};
  std::string frame_id_{"map"};
  std::string base_frame_{"base_footprint"};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

class OdomRelativeDriveAction : public BT::StatefulActionNode {
public:
  OdomRelativeDriveAction(const std::string &name,
                          const BT::NodeConfig &config);
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using TwistMsg = geometry_msgs::msg::Twist;
  using OdomMsg = nav_msgs::msg::Odometry;

  void publishStop();
  void releaseRuntime();
  bool odomReady() const;
  bool timedOut() const;
  bool prepareTargetFromCurrentOdom();
  BT::NodeStatus tickTowardTarget();
  BT::NodeStatus failWithStop(const char *reason);

  rclcpp::Node *node_{nullptr};
  rclcpp::Publisher<TwistMsg>::SharedPtr cmd_pub_;
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_odom_tp_{};
  std::string cmd_vel_topic_{"cmd_vel"};
  std::string odom_topic_{"odom"};
  double distance_m_{0.0};
  double max_speed_mps_{0.20};
  double min_speed_mps_{0.03};
  double xy_kp_{0.8};
  double heading_kp_{1.2};
  double heading_max_speed_radps_{0.30};
  double xy_tolerance_m_{0.03};
  double yaw_tolerance_rad_{0.05235987756};
  int stable_ticks_required_{3};
  double odom_timeout_s_{0.5};
  double timeout_s_{10.0};
  double current_x_{0.0};
  double current_y_{0.0};
  double current_yaw_{0.0};
  double target_x_{0.0};
  double target_y_{0.0};
  double target_yaw_{0.0};
  bool has_odom_{false};
  bool target_ready_{false};
  int stable_ticks_{0};
};

class RelativeYawTargetAction : public BT::StatefulActionNode {
public:
  RelativeYawTargetAction(const std::string &name,
                          const BT::NodeConfig &config);
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using OdomMsg = nav_msgs::msg::Odometry;

  BT::NodeStatus tryCaptureTarget();
  void releaseRuntime();

  rclcpp::Node *node_{nullptr};
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_odom_tp_{};
  std::string odom_topic_{"odom"};
  double yaw_delta_rad_{0.0};
  double timeout_s_{2.0};
  double odom_timeout_s_{0.5};
  double current_yaw_rad_{0.0};
  bool has_yaw_{false};
};

void registerNav2PoseNodes(BT::BehaviorTreeFactory &factory);

} // namespace rc26_decision
