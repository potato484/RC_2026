#pragma once

#include <future>
#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <nav2_msgs/action/spin.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace rc26_decision {

class LocalizationObserveSpin : public BT::StatefulActionNode {
public:
    LocalizationObserveSpin(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    using SpinAction = nav2_msgs::action::Spin;
    using GoalHandleSpin = rclcpp_action::ClientGoalHandle<SpinAction>;

    bool sendGoal(double yaw_rad);
    bool pollGoalResult(bool& goal_success);
    bool checkRecovered() const;

    rclcpp::Node* node_{nullptr};
    std::string action_name_{"spin"};
    double spin_angle_deg_{15.0};
    double spin_time_allowance_sec_{3.0};

    rclcpp_action::Client<SpinAction>::SharedPtr spin_client_;
    std::shared_future<typename GoalHandleSpin::SharedPtr> goal_handle_future_;
    typename GoalHandleSpin::SharedPtr goal_handle_;
    std::shared_future<typename GoalHandleSpin::WrappedResult> result_future_;
    int stage_{0};  // 0=first(+), 1=second(-)
};

}  // namespace rc26_decision

