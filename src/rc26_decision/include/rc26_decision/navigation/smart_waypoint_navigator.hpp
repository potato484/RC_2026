#pragma once

#include <atomic>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/msg/costmap_filter_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/bool.hpp>

#include "rc26_decision/navigation/smart_waypoint_types.hpp"
#include "rc26_interfaces/msg/nav_safety_state.hpp"
#include "rc26_interfaces/srv/set_nav_mode.hpp"

namespace rc26_decision {

class SmartWaypointNavigator {
public:
    using NavSafetyStateMsg = rc26_interfaces::msg::NavSafetyState;
    using SetNavMode = rc26_interfaces::srv::SetNavMode;

    enum class Status {
        Idle,
        Running,
        Succeeded,
        Failed,
        Canceled,
    };

    SmartWaypointNavigator(rclcpp::Node& node, std::string nav2_action_name = "navigate_to_pose",
                           std::string goal_frame = "map",
                           std::string controller_server_node = "controller_server", std::string odom_topic = "odometry",
                           double stop_linear_eps_mps = 0.05, double stop_angular_eps_rps = 0.05);

    bool start(const SmartWaypointSpec& waypoint);
    Status tick();
    void cancelAndStop();

    Status status() const { return status_; }

private:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using CostmapFilterInfo = nav2_msgs::msg::CostmapFilterInfo;
    using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using SetNavModeFuture = rclcpp::Client<SetNavMode>::SharedFuture;
    using GetParamsFuture = std::shared_future<std::vector<rclcpp::Parameter>>;
    using SetParamsFuture = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;

    enum class ExecState {
        Idle,
        SetMode,
        ApplyParams,
        SendGoal,
        WaitGoalHandle,
        WaitResult,
        Cleanup,
    };

    rclcpp::Node& node_;
    rclcpp::Logger logger_;
    std::string nav2_action_name_;
    std::string goal_frame_;
    std::string controller_server_node_;
    std::string odom_topic_;

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
    rclcpp::Client<SetNavMode>::SharedPtr nav_mode_client_;
    std::shared_ptr<rclcpp::AsyncParametersClient> controller_params_client_;

    rclcpp::Subscription<NavSafetyStateMsg>::SharedPtr nav_safety_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<CostmapFilterInfo>::SharedPtr costmap_filter_info_sub_;
    rclcpp::Subscription<OccupancyGrid>::SharedPtr kfs_filter_mask_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kfs_heartbeat_sub_;

    std::atomic<bool> nav_safety_stop_required_{false};
    std::atomic<bool> nav_safety_timed_out_{false};
    std::atomic<double> last_linear_speed_mps_{0.0};
    std::atomic<double> last_angular_speed_rps_{0.0};
    std::atomic<int64_t> last_odom_recv_ns_{0};
    std::atomic<int64_t> last_filter_info_stamp_ns_{0};
    std::atomic<int64_t> last_mask_stamp_ns_{0};
    std::atomic<int64_t> last_heartbeat_ns_{0};
    std::atomic<bool> heartbeat_enabled_{false};

    double stop_linear_eps_mps_{0.05};
    double stop_angular_eps_rps_{0.05};
    bool keepout_gate_enable_{true};
    double keepout_gate_max_age_ms_{300.0};
    double keepout_gate_timeout_sec_{3.0};
    std::string keepout_gate_mode_{"legacy"};
    std::string keepout_gate_heartbeat_topic_{"/kfs_keepout_heartbeat"};
    rclcpp::Time keepout_gate_wait_start_;
    bool keepout_gate_wait_started_{false};

    // Global, configurable speed-profile scales (applied as multipliers to controller defaults).
    // Parameter names on rc26_decision node:
    // - speed_profile_scales.fast  (default 1.0)
    // - speed_profile_scales.slow  (default 0.4)
    // - speed_profile_scales.creep (default 0.2)
    double speed_profile_fast_scale_{1.0};
    double speed_profile_slow_scale_{0.4};
    double speed_profile_creep_scale_{0.2};

    Status status_{Status::Idle};
    ExecState exec_state_{ExecState::Idle};
    std::optional<Status> final_status_;

    SmartWaypointSpec active_;
    rclcpp::Time state_start_time_;
    rclcpp::Time deadline_;
    bool has_deadline_{false};

    bool cancel_requested_{false};

    std::shared_future<typename GoalHandleNavigateToPose::SharedPtr> goal_handle_future_;
    typename GoalHandleNavigateToPose::SharedPtr goal_handle_;
    std::shared_future<typename GoalHandleNavigateToPose::WrappedResult> result_future_;

    SetNavModeFuture set_mode_future_;
    bool set_mode_requested_{false};
    SetNavModeFuture cleanup_set_mode_future_;
    bool cleanup_set_mode_requested_{false};
    SetNavModeFuture keepout_gate_set_mode_future_;
    bool keepout_gate_set_mode_requested_{false};

    GetParamsFuture get_defaults_future_;
    bool get_defaults_requested_{false};
    SetParamsFuture set_params_future_;
    bool set_params_requested_{false};
    SetParamsFuture restore_params_future_;
    bool restore_params_requested_{false};

    struct Nav2Defaults {
        bool loaded = false;
        double xy_goal_tolerance = 0.25;
        double yaw_goal_tolerance = 0.25;
        double v_linear_max = 0.0;
        double v_angular_max = 0.0;
    } defaults_;

    bool params_modified_{false};

    bool shouldAbort() const;
    bool haveRecentOdom(double max_age_sec) const;
    bool robotStopped() const;

    void loadSpeedProfileScales();
    void loadKeepoutGateConfig();
    bool isKeepoutReady(std::string& reason) const;
    void requestSafeModeForKeepoutGate();
    void pollKeepoutGateSafeMode();

    void requestSetMode(const std::string& profile, float timeout, const std::string& reason, SetNavModeFuture& future, bool& requested_flag);
    bool pollSetMode(SetNavModeFuture& future, bool& requested_flag, const char* ctx);

    void requestControllerDefaults();
    bool pollControllerDefaults();

    void requestSetControllerParams(const std::vector<rclcpp::Parameter>& params, SetParamsFuture& future, bool& requested_flag);
    bool pollSetControllerParams(SetParamsFuture& future, bool& requested_flag, const char* ctx);

    std::vector<rclcpp::Parameter> buildControllerParamsForActive() const;
    std::vector<rclcpp::Parameter> buildControllerParamsRestoreDefaults() const;

    bool sendNav2Goal();

    void abortWithFailure(const std::string& reason);
};

}  // namespace rc26_decision
