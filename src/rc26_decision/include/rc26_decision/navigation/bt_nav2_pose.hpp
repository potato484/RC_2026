#pragma once

#include "rc26_decision/common/bt_action_node.hpp"

#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <string>

namespace rc26_decision {

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GetLifecycleState = lifecycle_msgs::srv::GetState;

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

void registerNav2PoseNodes(BT::BehaviorTreeFactory &factory);

} // namespace rc26_decision
