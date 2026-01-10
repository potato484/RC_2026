#include "rc26_decision/navigation/waypoint_navigator.hpp"

#include <chrono>
#include <cstring>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "rc26_decision/navigation/waypoint_catalog.hpp"
#include "rc26_serial/protocol.hpp"

namespace rc26_decision
{

namespace
{
CommandID navModeToCommandId(NavMode mode)
{
    switch (mode)
    {
        case NavMode::Normal: return CommandID::NAV_NORMAL;
        case NavMode::StairUp: return CommandID::NAV_STAIR_UP;
        case NavMode::StairDown: return CommandID::NAV_STAIR_DOWN;
        default: return CommandID::NAV_NORMAL;
    }
}

std::vector<uint8_t> floatToPayload(float value)
{
    std::vector<uint8_t> payload(sizeof(float));
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(float));
    // 显式小端序列化
    payload[0] = static_cast<uint8_t>(bits & 0xFF);
    payload[1] = static_cast<uint8_t>((bits >> 8) & 0xFF);
    payload[2] = static_cast<uint8_t>((bits >> 16) & 0xFF);
    payload[3] = static_cast<uint8_t>((bits >> 24) & 0xFF);
    return payload;
}
}  // namespace

WaypointNavigator::WaypointNavigator(
    rclcpp::Node& node,
    std::shared_ptr<SerialDriver> cmd_serial,
    std::string nav2_action_name,
    std::string goal_frame)
    : node_(node),
      cmd_serial_(std::move(cmd_serial)),
      nav2_action_name_(std::move(nav2_action_name)),
      goal_frame_(std::move(goal_frame))
{
    nav2_client_ = rclcpp_action::create_client<NavigateToPose>(
        node_.get_node_base_interface(),
        node_.get_node_graph_interface(),
        node_.get_node_logging_interface(),
        node_.get_node_waitables_interface(),
        nav2_action_name_);
}

bool WaypointNavigator::start(uint8_t waypoint_id)
{
    if (status_ == Status::Running)
    {
        cancelAndStop();
    }

    const Waypoint* wp = findWaypoint(waypoint_id);
    if (!wp)
    {
        RCLCPP_ERROR(node_.get_logger(), "WaypointNavigator: invalid waypoint_id=%u", waypoint_id);
        status_ = Status::Failed;
        return false;
    }

    active_waypoint_id_ = waypoint_id;
    status_ = Status::Idle;
    cancel_requested_ = false;

    if (!sendNavModeAndSpeedToMcu(waypoint_id))
    {
        RCLCPP_WARN(node_.get_logger(),
            "WaypointNavigator: MCU command failed (id=%u), continuing to Nav2", waypoint_id);
    }

    if (!sendNav2Goal(waypoint_id))
    {
        status_ = Status::Failed;
        return false;
    }

    status_ = Status::Running;
    return true;
}

WaypointNavigator::Status WaypointNavigator::tick()
{
    if (status_ != Status::Running)
    {
        return status_;
    }

    if (!goal_handle_ && goal_handle_future_.valid())
    {
        const auto ready = goal_handle_future_.wait_for(std::chrono::seconds(0));
        if (ready == std::future_status::ready)
        {
            goal_handle_ = goal_handle_future_.get();
            if (!goal_handle_)
            {
                RCLCPP_ERROR(node_.get_logger(), "WaypointNavigator: Nav2 rejected goal");
                status_ = Status::Failed;
                return status_;
            }

            // 如果在等待期间收到取消请求，立即取消目标
            if (cancel_requested_)
            {
                (void)nav2_client_->async_cancel_goal(goal_handle_);
                goal_handle_.reset();
                status_ = Status::Canceled;
                return status_;
            }

            result_future_ = nav2_client_->async_get_result(goal_handle_);
        }
    }

    if (result_future_.valid())
    {
        const auto ready = result_future_.wait_for(std::chrono::seconds(0));
        if (ready == std::future_status::ready)
        {
            const auto wrapped = result_future_.get();
            switch (wrapped.code)
            {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    status_ = Status::Succeeded;
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    status_ = Status::Canceled;
                    break;
                default:
                    status_ = Status::Failed;
                    break;
            }
        }
    }

    return status_;
}

void WaypointNavigator::cancelAndStop()
{
    cancel_requested_ = true;

    if (nav2_client_ && goal_handle_)
    {
        (void)nav2_client_->async_cancel_goal(goal_handle_);
    }

    goal_handle_.reset();
    goal_handle_future_ = {};
    result_future_ = {};

    if (cmd_serial_ && cmd_serial_->isOpen())
    {
        (void)cmd_serial_->sendStop();
    }

    status_ = Status::Canceled;
    active_waypoint_id_ = 0;
}

bool WaypointNavigator::sendNavModeAndSpeedToMcu(uint8_t waypoint_id)
{
    const Waypoint* wp = findWaypoint(waypoint_id);
    if (!wp)
    {
        return false;
    }

    if (!cmd_serial_ || !cmd_serial_->isOpen())
    {
        RCLCPP_WARN(node_.get_logger(), "WaypointNavigator: cmd_serial not available");
        return false;
    }

    const auto cmd = navModeToCommandId(wp->mode);
    const auto payload = floatToPayload(wp->speed_mps);
    return cmd_serial_->sendCommand(cmd, payload);
}

bool WaypointNavigator::sendNav2Goal(uint8_t waypoint_id)
{
    if (!nav2_client_)
    {
        RCLCPP_ERROR(node_.get_logger(), "WaypointNavigator: nav2 client not initialized");
        return false;
    }

    // 等待 Nav2 action server 就绪（最多等待 5 秒）
    if (!nav2_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(node_.get_logger(),
            "WaypointNavigator: Nav2 action server not ready after 5s: %s", nav2_action_name_.c_str());
        return false;
    }

    const Waypoint* wp = findWaypoint(waypoint_id);
    if (!wp)
    {
        return false;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = goal_frame_;
    goal.pose.header.stamp = node_.now();
    goal.pose.pose.position.x = wp->x;
    goal.pose.pose.position.y = wp->y;
    goal.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, wp->yaw);
    goal.pose.pose.orientation.x = q.x();
    goal.pose.pose.orientation.y = q.y();
    goal.pose.pose.orientation.z = q.z();
    goal.pose.pose.orientation.w = q.w();

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    goal_handle_future_ = nav2_client_->async_send_goal(goal, options);
    goal_handle_.reset();
    result_future_ = {};
    return true;
}

}  // namespace rc26_decision
