#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <string>

#include "rc26_decision/navigation/straight_line_trajectory.hpp"

namespace rc26_decision {

void loadOdomRelativeNavParams(rclcpp::Node &node,
                               const BT::Blackboard::Ptr &blackboard);

class OdomAxisDriveAction : public BT::StatefulActionNode {
public:
  enum class Axis { X, Y };

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

protected:
  OdomAxisDriveAction(const std::string &name, const BT::NodeConfig &config,
                      Axis axis, const char *action_label);

private:
  using TwistMsg = geometry_msgs::msg::Twist;
  using OdomMsg = nav_msgs::msg::Odometry;

  void publishStop();
  void releaseRuntime();
  bool odomReady() const;
  bool timedOut() const;
  bool prepareTargetFromCurrentOdom();
  BT::NodeStatus tickTowardTarget();
  BT::NodeStatus failWithStop(const std::string &reason);
  void writeState(const std::string &state);
  void writeFailure(const std::string &reason);
  void writeDistanceRemaining(double distance);

  Axis axis_{Axis::X};
  const char *action_label_{"odom axis drive"};
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
  double start_x_{0.0};
  double start_y_{0.0};
  double target_x_{0.0};
  double target_y_{0.0};
  double target_yaw_{0.0};
  bool has_odom_{false};
  bool target_ready_{false};
  int stable_ticks_{0};
};

class OdomDriveXAction final : public OdomAxisDriveAction {
public:
  OdomDriveXAction(const std::string &name, const BT::NodeConfig &config);
  static BT::PortsList providedPorts();
};

class OdomDriveYAction final : public OdomAxisDriveAction {
public:
  OdomDriveYAction(const std::string &name, const BT::NodeConfig &config);
  static BT::PortsList providedPorts();
};

class OdomDriveXTurnXAction final : public BT::StatefulActionNode {
public:
  OdomDriveXTurnXAction(const std::string &name,
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
  navigation::StraightLineReference currentLineReference();
  BT::NodeStatus tickTowardTarget();
  BT::NodeStatus failWithStop(const std::string &reason);
  void writeState(const std::string &state);
  void writeFailure(const std::string &reason);
  void writeDistanceRemaining(double distance);

  rclcpp::Node *node_{nullptr};
  rclcpp::Publisher<TwistMsg>::SharedPtr cmd_pub_;
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_odom_tp_{};
  std::string cmd_vel_topic_{"cmd_vel"};
  std::string odom_topic_{"odom"};
  double first_x_m_{0.0};
  double yaw_delta_rad_{0.0};
  double second_x_m_{0.0};
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
  double start_x_{0.0};
  double start_y_{0.0};
  double start_yaw_{0.0};
  double target_x_{0.0};
  double target_y_{0.0};
  double target_yaw_{0.0};
  double line_progress_{0.0};
  double line_lookahead_m_{0.0};
  bool has_odom_{false};
  bool target_ready_{false};
  int stable_ticks_{0};
};

class OdomTurnToYawAction : public BT::StatefulActionNode {
public:
  OdomTurnToYawAction(const std::string &name, const BT::NodeConfig &config);
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
  BT::NodeStatus tickTurn();
  BT::NodeStatus failWithStop(const std::string &reason);
  void writeState(const std::string &state);
  void writeFailure(const std::string &reason);

  rclcpp::Node *node_{nullptr};
  rclcpp::Publisher<TwistMsg>::SharedPtr cmd_pub_;
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_odom_tp_{};
  std::string cmd_vel_topic_{"cmd_vel"};
  std::string odom_topic_{"odom"};
  double target_yaw_rad_{0.0};
  double kp_{1.2};
  double max_speed_radps_{0.30};
  double yaw_tolerance_rad_{0.05235987756};
  int stable_ticks_required_{3};
  double odom_timeout_s_{0.5};
  double timeout_s_{10.0};
  double current_yaw_rad_{0.0};
  bool has_odom_{false};
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

void registerOdomNavigationNodes(BT::BehaviorTreeFactory &factory);

} // namespace rc26_decision
