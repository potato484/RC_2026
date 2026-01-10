#pragma once

#include <cstdint>
#include <future>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include "rc26_serial/serial_driver.hpp"

namespace rc26_decision
{

class WaypointNavigator
{
public:
    enum class Status
    {
        Idle,
        Running,
        Succeeded,
        Failed,
        Canceled,
    };

    WaypointNavigator(
        rclcpp::Node& node,
        std::shared_ptr<SerialDriver> cmd_serial,
        std::string nav2_action_name = "navigate_to_pose",
        std::string goal_frame = "map");

    bool start(uint8_t waypoint_id);
    Status tick();
    void cancelAndStop();

    uint8_t activeWaypointId() const { return active_waypoint_id_; }
    Status status() const { return status_; }

private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    rclcpp::Node& node_;
    std::shared_ptr<SerialDriver> cmd_serial_;
    std::string nav2_action_name_;
    std::string goal_frame_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;

    uint8_t active_waypoint_id_{0};
    Status status_{Status::Idle};
    bool cancel_requested_{false};

    std::shared_future<typename GoalHandleNavigateToPose::SharedPtr> goal_handle_future_;
    typename GoalHandleNavigateToPose::SharedPtr goal_handle_;
    std::shared_future<typename GoalHandleNavigateToPose::WrappedResult> result_future_;

    bool sendNavModeAndSpeedToMcu(uint8_t waypoint_id);
    bool sendNav2Goal(uint8_t waypoint_id);
};

}  // namespace rc26_decision
