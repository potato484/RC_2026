#pragma once

#include <chrono>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rc26_decision {

struct GridCenterParams {
  std::string cmd_vel_topic{"cmd_vel"};
  std::string odom_topic{"odom"};
  double grid_step_m{1.2};
  double entry_forward_offset_m{0.25};
  double entry_forward_speed_mps{0.04};
  double xy_kp{0.8};
  double min_speed_mps{0.010};
  double max_speed_mps{0.050};
  double xy_tolerance_m{0.035};
  double yaw_kp{1.2};
  double yaw_max_speed_radps{0.30};
  double yaw_tolerance_deg{3.0};
  int stable_ticks{3};
  double odom_timeout_s{0.5};
  double align_timeout_s{8.0};
};

void loadGridCenterParams(rclcpp::Node &node,
                          const BT::Blackboard::Ptr &blackboard);
void registerGridCenterNodes(BT::BehaviorTreeFactory &factory);

class GridCenterActionBase : public BT::StatefulActionNode {
public:
  GridCenterActionBase(const std::string &name, const BT::NodeConfig &config);

protected:
  using TwistMsg = geometry_msgs::msg::Twist;
  using OdomMsg = nav_msgs::msg::Odometry;

  bool setupRuntime(const char *action_label);
  void releaseRuntime();
  void publishStop();
  bool odomReady() const;
  bool timedOut() const;
  void markStart();
  void writeReferenceGrid(int grid_id, double x, double y, double yaw);
  bool readReferenceGrid(int &grid_id, double &x, double &y, double &yaw) const;
  bool computeGridCenterFromReference(int target_grid, double &target_x,
                                      double &target_y) const;
  BT::NodeStatus tickTowardTarget(double target_x, double target_y,
                                  double target_yaw);
  BT::NodeStatus failWithStop(const char *reason);
  void setCenterError(const std::string &reason) const;

  static double normalizeAngle(double angle_rad);
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion &q);

  rclcpp::Node *node_{nullptr};
  GridCenterParams params_;
  std::string action_label_;
  rclcpp::Publisher<TwistMsg>::SharedPtr cmd_pub_;
  rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  double current_x_{0.0};
  double current_y_{0.0};
  double current_yaw_{0.0};
  bool has_odom_{false};
  std::chrono::steady_clock::time_point last_odom_tp_{};
  int stable_ticks_{0};
};

class CaptureGridCenterReferenceAction : public GridCenterActionBase {
public:
  CaptureGridCenterReferenceAction(const std::string &name,
                                   const BT::NodeConfig &config);
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  BT::NodeStatus tryCapture();

  int reference_grid_{2};
};

class MFEntryCenterAdvanceAction : public GridCenterActionBase {
public:
  MFEntryCenterAdvanceAction(const std::string &name,
                             const BT::NodeConfig &config);
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool prepareTargetFromCurrentOdom();

  int reference_grid_{2};
  double target_yaw_rad_{0.0};
  double target_x_{0.0};
  double target_y_{0.0};
  bool target_ready_{false};
};

class GridCenterAlignAction : public GridCenterActionBase {
public:
  GridCenterAlignAction(const std::string &name,
                        const BT::NodeConfig &config);
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  int target_grid_{0};
  double target_yaw_rad_{0.0};
  double target_x_{0.0};
  double target_y_{0.0};
};

} // namespace rc26_decision
