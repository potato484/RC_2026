#pragma once

#include <chrono>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rc26_decision {

struct GridHeadingParams {
  std::string direction{"left"};
  double forward_yaw_rad{0.0};
  double left_yaw_rad{1.5708};
  double right_yaw_rad{-1.5708};
  double backward_yaw_rad{3.1416};
  std::string cmd_vel_topic{"cmd_vel"};
  std::string odom_topic{"odom"};
  double kp{1.2};
  double max_speed_radps{0.30};
  double turn_gate_deg{8.0};
  double align_tolerance_deg{3.0};
  int align_stable_ticks{3};
  double odom_timeout_s{0.5};
  double turn_timeout_s{8.0};
  double align_timeout_s{3.0};
};

void loadGridHeadingParams(rclcpp::Node &node,
                           const BT::Blackboard::Ptr &blackboard);
void registerGridHeadingNodes(BT::BehaviorTreeFactory &factory);

class GridHeadingActionBase : public BT::StatefulActionNode {
public:
  GridHeadingActionBase(const std::string &name,
                        const BT::NodeConfig &config);

  static BT::PortsList providedPorts();

protected:
  bool setupRuntime(const char *action_label);
  bool readTargetYaw();
  void releaseRuntime();
  void publishStop();
  void publishAngular(double angular_z_radps);
  bool odomReady() const;
  double headingError() const;
  double headingAngularZ() const;
  double elapsedSinceStart() const;
  BT::NodeStatus failWithStop(const char *reason);
  static double normalizeAngle(double angle_rad);

  rclcpp::Node *node_{nullptr};
  GridHeadingParams params_;
  double target_yaw_rad_{0.0};

private:
  using TwistMsg = geometry_msgs::msg::Twist;
  using OdomMsg = nav_msgs::msg::Odometry;

  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q);

  std::string action_label_;
  rclcpp::Publisher<TwistMsg>::SharedPtr cmd_pub_;
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
  rclcpp::Time start_time_;
  double current_yaw_rad_{0.0};
  bool has_yaw_{false};
  std::chrono::steady_clock::time_point last_odom_tp_{};
};

class GridTurnAction : public GridHeadingActionBase {
public:
  GridTurnAction(const std::string &name, const BT::NodeConfig &config);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class GridHeadingAlignAction : public GridHeadingActionBase {
public:
  GridHeadingAlignAction(const std::string &name,
                         const BT::NodeConfig &config);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  int stable_ticks_{0};
};

} // namespace rc26_decision
