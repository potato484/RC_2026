#include "rc26_decision/navigation/bt_localization_observe_spin.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "rc26_interfaces/msg/localization_health.hpp"

namespace rc26_decision {

namespace {
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
}

LocalizationObserveSpin::LocalizationObserveSpin(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config) {}

BT::PortsList LocalizationObserveSpin::providedPorts() {
    return {
        BT::InputPort<double>("angle_deg", 15.0, "单次扫描角度（度）"),
        BT::InputPort<double>("time_allowance_sec", 3.0, "单次扫描时间上限（秒）"),
        BT::InputPort<std::string>("action_name", "spin", "Nav2 Spin action 名称"),
    };
}

BT::NodeStatus LocalizationObserveSpin::onStart() {
    node_ = nullptr;
    if (!config().blackboard->get("node", node_) || !node_) {
        return BT::NodeStatus::FAILURE;
    }

    std::string action_name = "spin";
    (void)getInput("action_name", action_name);
    std::string blackboard_action_name;
    if (config().blackboard->get("loc_spin_action_name", blackboard_action_name) && !blackboard_action_name.empty()) {
        action_name = blackboard_action_name;
    }
    if (action_name.empty()) {
        action_name = "spin";
    }
    action_name_ = action_name;

    (void)getInput("angle_deg", spin_angle_deg_);
    (void)getInput("time_allowance_sec", spin_time_allowance_sec_);
    double bb_angle_deg = spin_angle_deg_;
    double bb_time_allowance = spin_time_allowance_sec_;
    if (config().blackboard->get("loc_spin_angle_deg", bb_angle_deg)) {
        spin_angle_deg_ = bb_angle_deg;
    }
    if (config().blackboard->get("loc_spin_time_allowance_sec", bb_time_allowance)) {
        spin_time_allowance_sec_ = bb_time_allowance;
    }

    spin_angle_deg_ = std::clamp(spin_angle_deg_, 1.0, 90.0);
    spin_time_allowance_sec_ = std::clamp(spin_time_allowance_sec_, 0.5, 10.0);

    spin_client_ = rclcpp_action::create_client<SpinAction>(
        node_->get_node_base_interface(), node_->get_node_graph_interface(), node_->get_node_logging_interface(),
        node_->get_node_waitables_interface(), action_name_);
    if (!spin_client_ || !spin_client_->wait_for_action_server(std::chrono::milliseconds(300))) {
        return BT::NodeStatus::FAILURE;
    }

    stage_ = 0;
    if (!sendGoal(spin_angle_deg_ * kDegToRad)) {
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LocalizationObserveSpin::onRunning() {
    bool goal_success = false;
    if (!pollGoalResult(goal_success)) {
        return BT::NodeStatus::RUNNING;
    }
    if (!goal_success) {
        return BT::NodeStatus::FAILURE;
    }

    // 每半次扫描后检查健康度是否已经恢复（<=YELLOW）。
    if (checkRecovered()) {
        return BT::NodeStatus::SUCCESS;
    }

    if (stage_ == 0) {
        stage_ = 1;
        if (!sendGoal(-spin_angle_deg_ * kDegToRad)) {
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::FAILURE;
}

void LocalizationObserveSpin::onHalted() {
    if (spin_client_ && goal_handle_) {
        (void)spin_client_->async_cancel_goal(goal_handle_);
    }
    goal_handle_.reset();
    goal_handle_future_ = {};
    result_future_ = {};
    stage_ = 0;
}

bool LocalizationObserveSpin::sendGoal(const double yaw_rad) {
    if (!spin_client_) {
        return false;
    }
    SpinAction::Goal goal;
    goal.target_yaw = static_cast<float>(yaw_rad);
    goal.time_allowance = rclcpp::Duration::from_seconds(spin_time_allowance_sec_);
    typename rclcpp_action::Client<SpinAction>::SendGoalOptions options;
    goal_handle_future_ = spin_client_->async_send_goal(goal, options);
    goal_handle_.reset();
    result_future_ = {};
    return true;
}

bool LocalizationObserveSpin::pollGoalResult(bool& goal_success) {
    goal_success = false;
    if (!spin_client_) {
        return true;
    }

    if (!goal_handle_ && goal_handle_future_.valid()) {
        if (goal_handle_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            return false;
        }
        goal_handle_ = goal_handle_future_.get();
        if (!goal_handle_) {
            return true;
        }
        result_future_ = spin_client_->async_get_result(goal_handle_);
    }

    if (!result_future_.valid()) {
        return false;
    }
    if (result_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return false;
    }

    const auto wrapped = result_future_.get();
    goal_handle_.reset();
    goal_handle_future_ = {};
    result_future_ = {};
    goal_success = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED;
    return true;
}

bool LocalizationObserveSpin::checkRecovered() const {
    int loc_level = static_cast<int>(rc26_interfaces::msg::LocalizationHealth::RED);
    (void)config().blackboard->get("loc_level", loc_level);
    return loc_level <= static_cast<int>(rc26_interfaces::msg::LocalizationHealth::YELLOW);
}

}  // namespace rc26_decision
