#include "rc26_decision/navigation/smart_waypoint_navigator.hpp"

#include <chrono>
#include <cmath>
#include <cstring>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "rc26_serial/protocol.hpp"

namespace rc26_decision {

namespace {

std::vector<uint8_t> floatToPayload(float value) {
    std::vector<uint8_t> payload(sizeof(float));
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(float));
    payload[0] = static_cast<uint8_t>(bits & 0xFF);
    payload[1] = static_cast<uint8_t>((bits >> 8) & 0xFF);
    payload[2] = static_cast<uint8_t>((bits >> 16) & 0xFF);
    payload[3] = static_cast<uint8_t>((bits >> 24) & 0xFF);
    return payload;
}

CommandID mcuModeToCommandId(McuNavMode mode) {
    switch (mode) {
    case McuNavMode::Normal:
        return CommandID::NAV_NORMAL;
    case McuNavMode::StairUp:
        return CommandID::NAV_STAIR_UP;
    case McuNavMode::StairDown:
        return CommandID::NAV_STAIR_DOWN;
    default:
        return CommandID::NAV_NORMAL;
    }
}

}  // namespace

SmartWaypointNavigator::SmartWaypointNavigator(rclcpp::Node& node, std::shared_ptr<SerialDriver> cmd_serial,
                                               std::string nav2_action_name, std::string goal_frame,
                                               std::string controller_server_node, std::string odom_topic,
                                               double stop_linear_eps_mps, double stop_angular_eps_rps)
    : node_(node),
      logger_(node.get_logger()),
      cmd_serial_(std::move(cmd_serial)),
      nav2_action_name_(std::move(nav2_action_name)),
      goal_frame_(std::move(goal_frame)),
      controller_server_node_(std::move(controller_server_node)),
      odom_topic_(std::move(odom_topic)),
      stop_linear_eps_mps_(stop_linear_eps_mps),
      stop_angular_eps_rps_(stop_angular_eps_rps) {
    nav2_client_ = rclcpp_action::create_client<NavigateToPose>(
        node_.get_node_base_interface(), node_.get_node_graph_interface(), node_.get_node_logging_interface(),
        node_.get_node_waitables_interface(), nav2_action_name_);

    nav_mode_client_ = node_.create_client<SetNavMode>("nav_safety/set_mode");

    controller_params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(&node_, controller_server_node_);

    nav_safety_state_sub_ = node_.create_subscription<NavSafetyModeMsg>(
        "nav_safety/state", 10, [this](const NavSafetyModeMsg::SharedPtr msg) {
            nav_safety_stop_required_.store(msg->stop_required);
            nav_safety_timed_out_.store(msg->timed_out);
        });

    odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(), [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            const double vx = msg->twist.twist.linear.x;
            const double vy = msg->twist.twist.linear.y;
            const double wz = msg->twist.twist.angular.z;
            last_linear_speed_mps_.store(std::sqrt(vx * vx + vy * vy));
            last_angular_speed_rps_.store(std::abs(wz));
            last_odom_recv_ns_.store(node_.now().nanoseconds());
        });
}

bool SmartWaypointNavigator::start(const SmartWaypointSpec& waypoint) {
    if (status_ == Status::Running) {
        cancelAndStop();
    }

    active_ = waypoint;
    status_ = Status::Running;
    exec_state_ = ExecState::SetMode;
    final_status_.reset();
    cancel_requested_ = false;
    params_modified_ = false;

    set_mode_requested_ = false;
    cleanup_set_mode_requested_ = false;
    get_defaults_requested_ = false;
    set_params_requested_ = false;
    restore_params_requested_ = false;

    set_mode_future_ = {};
    cleanup_set_mode_future_ = {};
    get_defaults_future_ = {};
    set_params_future_ = {};
    restore_params_future_ = {};

    goal_handle_future_ = {};
    goal_handle_.reset();
    result_future_ = {};

    state_start_time_ = node_.now();
    if (active_.timeout_sec > 0.0f) {
        has_deadline_ = true;
        deadline_ = state_start_time_ + rclcpp::Duration::from_seconds(active_.timeout_sec);
    } else {
        has_deadline_ = false;
    }

    return true;
}

bool SmartWaypointNavigator::shouldAbort() const {
    return nav_safety_stop_required_.load() || nav_safety_timed_out_.load();
}

bool SmartWaypointNavigator::haveRecentOdom(double max_age_sec) const {
    const int64_t last_ns = last_odom_recv_ns_.load();
    if (last_ns == 0) {
        return false;
    }
    const int64_t now_ns = node_.now().nanoseconds();
    const double age_sec = static_cast<double>(now_ns - last_ns) * 1e-9;
    return age_sec <= max_age_sec;
}

bool SmartWaypointNavigator::robotStopped() const {
    if (!haveRecentOdom(1.0)) {
        return false;
    }
    const double lin = last_linear_speed_mps_.load();
    const double ang = last_angular_speed_rps_.load();
    return (lin < stop_linear_eps_mps_) && (ang < stop_angular_eps_rps_);
}

void SmartWaypointNavigator::requestSetMode(uint8_t mode, float timeout, const std::string& reason,
                                            SetNavModeFuture& future, bool& requested_flag) {
    if (requested_flag) {
        return;
    }
    auto request = std::make_shared<SetNavMode::Request>();
    request->mode = mode;
    request->timeout = timeout;
    request->reason = reason;
    future = nav_mode_client_->async_send_request(request);
    requested_flag = true;
}

bool SmartWaypointNavigator::pollSetMode(SetNavModeFuture& future, bool& requested_flag, const char* ctx) {
    if (!requested_flag) {
        return false;
    }
    if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return false;
    }

    try {
        auto response = future.get();
        requested_flag = false;
        future = {};

        if (!response || !response->success) {
            const std::string msg = response ? response->message : "no response";
            abortWithFailure(std::string(ctx) + ": set_mode failed: " + msg);
            return false;
        }
        return true;
    } catch (const std::exception& ex) {
        requested_flag = false;
        future = {};
        abortWithFailure(std::string(ctx) + ": exception: " + ex.what());
        return false;
    }
}

void SmartWaypointNavigator::requestControllerDefaults() {
    if (defaults_.loaded || get_defaults_requested_) {
        return;
    }
    if (!controller_params_client_ || !controller_params_client_->wait_for_service(std::chrono::seconds(0))) {
        return;
    }
    get_defaults_future_ = controller_params_client_->get_parameters({
        "general_goal_checker.xy_goal_tolerance",
        "general_goal_checker.yaw_goal_tolerance",
        "FollowPath.v_linear_max",
        "FollowPath.v_angular_max",
    });
    get_defaults_requested_ = true;
}

bool SmartWaypointNavigator::pollControllerDefaults() {
    if (defaults_.loaded) {
        return true;
    }
    if (!get_defaults_requested_) {
        return false;
    }
    if (get_defaults_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return false;
    }

    try {
        const auto params = get_defaults_future_.get();
        get_defaults_requested_ = false;
        get_defaults_future_ = {};

        for (const auto& p : params) {
            if (p.get_name() == "general_goal_checker.xy_goal_tolerance" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                defaults_.xy_goal_tolerance = p.as_double();
            } else if (p.get_name() == "general_goal_checker.yaw_goal_tolerance" &&
                       p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                defaults_.yaw_goal_tolerance = p.as_double();
            } else if (p.get_name() == "FollowPath.v_linear_max" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                defaults_.v_linear_max = p.as_double();
            } else if (p.get_name() == "FollowPath.v_angular_max" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                defaults_.v_angular_max = p.as_double();
            }
        }

        defaults_.loaded = true;
        return true;
    } catch (const std::exception& ex) {
        RCLCPP_WARN(logger_, "pollControllerDefaults exception: %s", ex.what());
        get_defaults_requested_ = false;
        get_defaults_future_ = {};
        return false;
    }
}

void SmartWaypointNavigator::requestSetControllerParams(const std::vector<rclcpp::Parameter>& params, SetParamsFuture& future,
                                                        bool& requested_flag) {
    if (requested_flag) {
        return;
    }
    if (!controller_params_client_ || !controller_params_client_->wait_for_service(std::chrono::seconds(0))) {
        return;
    }
    future = controller_params_client_->set_parameters(params);
    requested_flag = true;
}

bool SmartWaypointNavigator::pollSetControllerParams(SetParamsFuture& future, bool& requested_flag, const char* ctx) {
    if (!requested_flag) {
        return false;
    }
    if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return false;
    }

    try {
        const auto results = future.get();
        requested_flag = false;
        future = {};

        bool ok = true;
        for (const auto& r : results) {
            if (!r.successful) {
                ok = false;
            }
        }
        if (!ok) {
            RCLCPP_WARN(logger_, "SmartWaypointNavigator: %s: some parameters were rejected", ctx);
        }
        return true;
    } catch (const std::exception& ex) {
        RCLCPP_WARN(logger_, "SmartWaypointNavigator: %s: exception: %s", ctx, ex.what());
        requested_flag = false;
        future = {};
        return true;  // Return true to not block state machine
    }
}

std::vector<rclcpp::Parameter> SmartWaypointNavigator::buildControllerParamsForActive() const {
    if (!defaults_.loaded) {
        return {};
    }

    std::vector<rclcpp::Parameter> out;

    auto maybe_push = [&](const std::string& name, double value, double default_value) {
        if (std::abs(value - default_value) > 1e-9) {
            out.emplace_back(name, value);
        }
    };

    if (active_.tolerance.xy_tolerance >= 0.0) {
        maybe_push("general_goal_checker.xy_goal_tolerance", active_.tolerance.xy_tolerance, defaults_.xy_goal_tolerance);
    }
    if (active_.tolerance.yaw_tolerance >= 0.0) {
        maybe_push("general_goal_checker.yaw_goal_tolerance", active_.tolerance.yaw_tolerance, defaults_.yaw_goal_tolerance);
    }

    double v_lin = defaults_.v_linear_max;
    double v_ang = defaults_.v_angular_max;

    if (!active_.speed_profile.empty()) {
        double scale = 1.0;
        if (active_.speed_profile == "SLOW") {
            scale = 0.4;
        } else if (active_.speed_profile == "CREEP") {
            scale = 0.2;
        } else if (active_.speed_profile == "FAST") {
            scale = 1.0;
        }
        if (defaults_.v_linear_max > 0.0) {
            v_lin = defaults_.v_linear_max * scale;
        }
        if (defaults_.v_angular_max > 0.0) {
            v_ang = defaults_.v_angular_max * scale;
        }
    }

    const auto it_lin = active_.payload.find("v_linear_max");
    if (it_lin != active_.payload.end()) {
        v_lin = it_lin->second;
    }
    const auto it_ang = active_.payload.find("v_angular_max");
    if (it_ang != active_.payload.end()) {
        v_ang = it_ang->second;
    }

    if (defaults_.v_linear_max > 0.0) {
        maybe_push("FollowPath.v_linear_max", v_lin, defaults_.v_linear_max);
    }
    if (defaults_.v_angular_max > 0.0) {
        maybe_push("FollowPath.v_angular_max", v_ang, defaults_.v_angular_max);
    }

    return out;
}

std::vector<rclcpp::Parameter> SmartWaypointNavigator::buildControllerParamsRestoreDefaults() const {
    if (!defaults_.loaded) {
        return {};
    }
    return {
        rclcpp::Parameter("general_goal_checker.xy_goal_tolerance", defaults_.xy_goal_tolerance),
        rclcpp::Parameter("general_goal_checker.yaw_goal_tolerance", defaults_.yaw_goal_tolerance),
        rclcpp::Parameter("FollowPath.v_linear_max", defaults_.v_linear_max),
        rclcpp::Parameter("FollowPath.v_angular_max", defaults_.v_angular_max),
    };
}

bool SmartWaypointNavigator::sendMcuNavCommand() {
    if (!active_.mcu.enabled) {
        return true;
    }
    if (!cmd_serial_ || !cmd_serial_->isOpen()) {
        RCLCPP_WARN(logger_, "SmartWaypointNavigator: cmd_serial not available, skipping MCU nav command");
        return false;
    }
    const auto cmd = mcuModeToCommandId(active_.mcu.mode);
    const auto payload = floatToPayload(active_.mcu.speed_mps);
    return cmd_serial_->sendCommand(cmd, payload);
}

bool SmartWaypointNavigator::sendNav2Goal() {
    if (!nav2_client_) {
        return false;
    }
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = goal_frame_;
    goal.pose.header.stamp = node_.now();
    goal.pose.pose.position.x = active_.pose.x;
    goal.pose.pose.position.y = active_.pose.y;
    goal.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, active_.pose.yaw);
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

void SmartWaypointNavigator::abortWithFailure(const std::string& reason) {
    if (final_status_.has_value()) {
        return;
    }
    RCLCPP_ERROR(logger_, "SmartWaypointNavigator abort: %s", reason.c_str());
    final_status_ = Status::Failed;

    if (nav2_client_ && goal_handle_) {
        (void)nav2_client_->async_cancel_goal(goal_handle_);
    }

    if (cmd_serial_ && cmd_serial_->isOpen()) {
        (void)cmd_serial_->sendStop();
    }

    exec_state_ = ExecState::Cleanup;
    state_start_time_ = node_.now();
}

SmartWaypointNavigator::Status SmartWaypointNavigator::tick() {
    if (status_ != Status::Running) {
        return status_;
    }

    if (shouldAbort()) {
        abortWithFailure(nav_safety_stop_required_.load() ? "nav_safety.stop_required" : "nav_safety.timed_out");
    }

    if (has_deadline_ && node_.now() > deadline_) {
        abortWithFailure("timeout");
    }

    switch (exec_state_) {
    case ExecState::SetMode: {
        const auto target = active_.nav_safety_mode;
        const bool need_stop = (target == NavSafetyMode::MF_TRAVERSE || target == NavSafetyMode::MF_EXIT);
        if (need_stop && !robotStopped()) {
            if ((node_.now() - state_start_time_).seconds() > 3.0) {
                abortWithFailure("robot_not_stopped_before_TRAVERSE_or_EXIT");
            }
            return Status::Running;
        }

        if (!nav_mode_client_->wait_for_service(std::chrono::seconds(0))) {
            if ((node_.now() - state_start_time_).seconds() > 2.0) {
                abortWithFailure("nav_safety/set_mode service not available");
            }
            return Status::Running;
        }

        requestSetMode(static_cast<uint8_t>(target), active_.timeout_sec, active_.strategy_tag, set_mode_future_, set_mode_requested_);
        if (pollSetMode(set_mode_future_, set_mode_requested_, "pre-nav")) {
            (void)sendMcuNavCommand();
            exec_state_ = ExecState::ApplyParams;
            state_start_time_ = node_.now();
        }
        return Status::Running;
    }

    case ExecState::ApplyParams: {
        if (!controller_params_client_ || !controller_params_client_->wait_for_service(std::chrono::seconds(0))) {
            if ((node_.now() - state_start_time_).seconds() > 2.0) {
                exec_state_ = ExecState::SendGoal;
                state_start_time_ = node_.now();
            }
            return Status::Running;
        }

        requestControllerDefaults();
        if (!pollControllerDefaults()) {
            if ((node_.now() - state_start_time_).seconds() > 2.0) {
                exec_state_ = ExecState::SendGoal;
                state_start_time_ = node_.now();
            }
            return Status::Running;
        }

        const auto params = buildControllerParamsForActive();
        if (params.empty()) {
            exec_state_ = ExecState::SendGoal;
            state_start_time_ = node_.now();
            return Status::Running;
        }

        requestSetControllerParams(params, set_params_future_, set_params_requested_);
        if (pollSetControllerParams(set_params_future_, set_params_requested_, "apply")) {
            params_modified_ = true;
            exec_state_ = ExecState::SendGoal;
            state_start_time_ = node_.now();
        }
        return Status::Running;
    }

    case ExecState::SendGoal: {
        if (!nav2_client_->wait_for_action_server(std::chrono::seconds(0))) {
            if ((node_.now() - state_start_time_).seconds() > 5.0) {
                abortWithFailure("Nav2 action server not ready");
            }
            return Status::Running;
        }
        if (!sendNav2Goal()) {
            abortWithFailure("sendNav2Goal failed");
            return Status::Running;
        }
        exec_state_ = ExecState::WaitGoalHandle;
        return Status::Running;
    }

    case ExecState::WaitGoalHandle: {
        if (!goal_handle_future_.valid() ||
            goal_handle_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            return Status::Running;
        }
        try {
            goal_handle_ = goal_handle_future_.get();
        } catch (const std::exception& ex) {
            abortWithFailure(std::string("WaitGoalHandle exception: ") + ex.what());
            return Status::Running;
        }
        if (!goal_handle_) {
            abortWithFailure("Nav2 rejected goal");
            return Status::Running;
        }
        result_future_ = nav2_client_->async_get_result(goal_handle_);
        exec_state_ = ExecState::WaitResult;
        return Status::Running;
    }

    case ExecState::WaitResult: {
        if (!result_future_.valid() ||
            result_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            return Status::Running;
        }

        try {
            const auto wrapped = result_future_.get();
            switch (wrapped.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                final_status_ = Status::Succeeded;
                break;
            case rclcpp_action::ResultCode::CANCELED:
                final_status_ = Status::Canceled;
                break;
            default:
                final_status_ = Status::Failed;
                break;
            }
        } catch (const std::exception& ex) {
            RCLCPP_WARN(logger_, "WaitResult exception: %s", ex.what());
            final_status_ = Status::Failed;
        }

        exec_state_ = ExecState::Cleanup;
        state_start_time_ = node_.now();
        return Status::Running;
    }

    case ExecState::Cleanup: {
        if (params_modified_ && defaults_.loaded) {
            if (!restore_params_requested_) {
                const auto restore = buildControllerParamsRestoreDefaults();
                requestSetControllerParams(restore, restore_params_future_, restore_params_requested_);
            }
            (void)pollSetControllerParams(restore_params_future_, restore_params_requested_, "restore");
        }

        if (active_.nav_safety_mode == NavSafetyMode::MF_TRAVERSE || active_.nav_safety_mode == NavSafetyMode::MF_EXIT) {
            if (!nav_mode_client_->wait_for_service(std::chrono::seconds(0))) {
            } else {
                requestSetMode(static_cast<uint8_t>(NavSafetyMode::MF_SAFE), 0.0f, "post_nav", cleanup_set_mode_future_,
                               cleanup_set_mode_requested_);
                (void)pollSetMode(cleanup_set_mode_future_, cleanup_set_mode_requested_, "post-nav");
            }
        }

        const bool restore_done = !restore_params_requested_;
        const bool mode_done = !cleanup_set_mode_requested_;
        if (restore_done && mode_done) {
            status_ = final_status_.value_or(Status::Failed);
            exec_state_ = ExecState::Idle;
        }
        return Status::Running;
    }

    default:
        break;
    }

    return status_;
}

void SmartWaypointNavigator::cancelAndStop() {
    cancel_requested_ = true;
    final_status_ = Status::Canceled;

    // Cancel Nav2 goal if we have a valid handle
    if (nav2_client_ && goal_handle_) {
        (void)nav2_client_->async_cancel_goal(goal_handle_);
    }
    // Also try to cancel if goal_handle_future is pending (C2 fix)
    if (goal_handle_future_.valid() &&
        goal_handle_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        try {
            auto handle = goal_handle_future_.get();
            if (handle && nav2_client_) {
                (void)nav2_client_->async_cancel_goal(handle);
            }
        } catch (const std::exception& ex) {
            RCLCPP_WARN(logger_, "cancelAndStop goal_handle_future exception: %s", ex.what());
        }
    }

    goal_handle_.reset();
    goal_handle_future_ = {};
    result_future_ = {};

    if (cmd_serial_ && cmd_serial_->isOpen()) {
        (void)cmd_serial_->sendStop();
    }

    // Fire-and-forget parameter restore (M1 fix: no blocking wait)
    // Note: params might be in-flight (C3), so we set flag to trigger restore on next opportunity
    if (params_modified_ || set_params_requested_) {
        if (controller_params_client_ && controller_params_client_->wait_for_service(std::chrono::seconds(0))) {
            if (defaults_.loaded) {
                (void)controller_params_client_->set_parameters(buildControllerParamsRestoreDefaults());
            }
        }
        params_modified_ = false;
        set_params_requested_ = false;
    }

    // Fire-and-forget safety mode restore (M1 fix: no blocking wait)
    if ((active_.nav_safety_mode == NavSafetyMode::MF_TRAVERSE || active_.nav_safety_mode == NavSafetyMode::MF_EXIT) &&
        nav_mode_client_ && nav_mode_client_->wait_for_service(std::chrono::seconds(0))) {
        auto req = std::make_shared<SetNavMode::Request>();
        req->mode = static_cast<uint8_t>(NavSafetyMode::MF_SAFE);
        req->timeout = 0.0f;
        req->reason = "halted";
        (void)nav_mode_client_->async_send_request(req);
    }

    status_ = Status::Canceled;
    exec_state_ = ExecState::Idle;
}

}  // namespace rc26_decision
