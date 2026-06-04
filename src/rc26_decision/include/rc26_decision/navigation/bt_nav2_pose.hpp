#pragma once

#include "rc26_decision/common/bt_action_node.hpp"

#include <nav2_msgs/action/navigate_to_pose.hpp>

namespace rc26_decision {

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class NavToPoseAction : public BtActionNode<NavigateToPose> {
public:
  NavToPoseAction(const std::string &name, const BT::NodeConfig &config);
  static BT::PortsList providedPorts();

protected:
  bool buildGoal(Goal &goal) override;
  void onFeedback(const std::shared_ptr<const Feedback> &feedback) override;
  BT::NodeStatus handleResult(const WrappedResult &result,
                              uint16_t &error_code) override;
  void onGoalAccepted() override;
  void onActionFailure(uint16_t error_code, const std::string &failure_code,
                       const std::string &failure_reason) override;
};

void registerNav2PoseNodes(BT::BehaviorTreeFactory &factory);

} // namespace rc26_decision
