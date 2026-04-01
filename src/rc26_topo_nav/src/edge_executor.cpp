#include "rc26_topo_nav/edge_executor.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <thread>
#include <utility>

namespace rc26_topo_nav {

namespace {

std::string toLowerCopy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

class ScopeExit {
public:
    explicit ScopeExit(std::function<void()> fn) : fn_(std::move(fn)) {}
    ~ScopeExit() {
        if (fn_) {
            fn_();
        }
    }

    ScopeExit(const ScopeExit&) = delete;
    ScopeExit& operator=(const ScopeExit&) = delete;

private:
    std::function<void()> fn_;
};

enum class WaitOutcome : uint8_t {
    READY,
    TIMEOUT,
    CANCELLED,
    SHUTDOWN
};

template <typename FutureT>
WaitOutcome waitForFuture(
    FutureT& future,
    const std::chrono::steady_clock::duration& timeout,
    const std::atomic<bool>* cancelled = nullptr) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    constexpr auto poll_interval = std::chrono::milliseconds(50);

    while (rclcpp::ok()) {
        if (cancelled != nullptr && cancelled->load()) {
            return WaitOutcome::CANCELLED;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready
                       ? WaitOutcome::READY
                       : WaitOutcome::TIMEOUT;
        }

        const auto remaining =
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
        const auto wait_slice = std::min(poll_interval, remaining);
        if (future.wait_for(wait_slice) == std::future_status::ready) {
            return WaitOutcome::READY;
        }
    }

    return WaitOutcome::SHUTDOWN;
}

}  // namespace

EdgeExecutor::EdgeExecutor(rclcpp::Node* node) : node_(node) {
    if (!node_->has_parameter("execution_backend")) {
        node_->declare_parameter("execution_backend", "nav2_follow_path");
    }
    const auto execution_backend = node_->get_parameter("execution_backend").as_string();
    use_xhu_backend_ = (toLowerCopy(execution_backend) == "xhu_direct");

    if (!node_->has_parameter("xhu.exec_timeout_sec")) {
        node_->declare_parameter("xhu.exec_timeout_sec", xhu_exec_timeout_sec_);
    }
    if (!node_->has_parameter("xhu.hold_replan_timeout_sec")) {
        node_->declare_parameter("xhu.hold_replan_timeout_sec", xhu_hold_replan_timeout_sec_);
    }
    if (!node_->has_parameter("xhu.corridor_accept_timeout_sec")) {
        node_->declare_parameter("xhu.corridor_accept_timeout_sec", xhu_corridor_accept_timeout_sec_);
    }
    if (!node_->has_parameter("xhu.corridor_republish_period_sec")) {
        node_->declare_parameter("xhu.corridor_republish_period_sec", xhu_corridor_republish_period_sec_);
    }
    if (!node_->has_parameter("xhu.tracking_state_timeout_sec")) {
        node_->declare_parameter("xhu.tracking_state_timeout_sec", xhu_tracking_state_timeout_sec_);
    }
    if (!node_->has_parameter("xhu.tracking_state_ttl_sec")) {
        node_->declare_parameter("xhu.tracking_state_ttl_sec", tracking_state_ttl_sec_);
    }
    xhu_exec_timeout_sec_ = std::max(1.0, node_->get_parameter("xhu.exec_timeout_sec").as_double());
    xhu_hold_replan_timeout_sec_ =
        std::max(0.1, node_->get_parameter("xhu.hold_replan_timeout_sec").as_double());
    xhu_corridor_accept_timeout_sec_ =
        std::max(0.2, node_->get_parameter("xhu.corridor_accept_timeout_sec").as_double());
    xhu_corridor_republish_period_sec_ =
        std::max(0.1, node_->get_parameter("xhu.corridor_republish_period_sec").as_double());
    xhu_tracking_state_timeout_sec_ =
        std::max(0.2, node_->get_parameter("xhu.tracking_state_timeout_sec").as_double());
    tracking_state_ttl_sec_ =
        std::max(xhu_tracking_state_timeout_sec_, node_->get_parameter("xhu.tracking_state_ttl_sec").as_double());

    if (!use_xhu_backend_) {
        follow_path_client_ = rclcpp_action::create_client<FollowPathAction>(node_, "follow_path");
    }
    nav_mode_client_ = node_->create_client<rc26_interfaces::srv::SetNavMode>("set_nav_mode");
    xhu_mode_client_ = node_->create_client<rc26_interfaces::srv::SetXhuMotionMode>("set_xhu_motion_mode");

    corridor_pub_ = node_->create_publisher<rc26_interfaces::msg::XhuSemanticCorridor>(
        "/xhu_nav/corridor_cmd", 10);
    tracking_sub_ = node_->create_subscription<rc26_interfaces::msg::XhuTrackingState>(
        "/xhu_nav/tracking_state", 30,
        std::bind(&EdgeExecutor::onTrackingState, this, std::placeholders::_1));

    RCLCPP_INFO(
        node_->get_logger(),
        "EdgeExecutor backend=%s, xhu_exec_timeout=%.1fs, xhu_hold_replan_timeout=%.1fs, "
        "accept_timeout=%.1fs, tracking_timeout=%.1fs",
        use_xhu_backend_ ? "xhu_direct" : "nav2_follow_path",
        xhu_exec_timeout_sec_, xhu_hold_replan_timeout_sec_,
        xhu_corridor_accept_timeout_sec_, xhu_tracking_state_timeout_sec_);
}

void EdgeExecutor::onTrackingState(const rc26_interfaces::msg::XhuTrackingState::SharedPtr msg) {
    if (msg->corridor_id.empty()) {
        return;
    }
    std::lock_guard<std::mutex> lock(tracking_mutex_);
    const auto stamp = node_->now();
    pruneTrackingStatesLocked(stamp);
    tracking_state_map_[msg->corridor_id] = TrackingEntry{*msg, stamp};
}

std::optional<rc26_interfaces::msg::XhuTrackingState> EdgeExecutor::latestTrackingState(
    const std::string& corridor_id) const {
    std::lock_guard<std::mutex> lock(tracking_mutex_);
    auto it = tracking_state_map_.find(corridor_id);
    if (it == tracking_state_map_.end()) {
        return std::nullopt;
    }
    if ((node_->now() - it->second.received_at).seconds() > tracking_state_ttl_sec_) {
        return std::nullopt;
    }
    return it->second.state;
}

void EdgeExecutor::clearTrackingState(const std::string& corridor_id) {
    std::lock_guard<std::mutex> lock(tracking_mutex_);
    tracking_state_map_.erase(corridor_id);
}

void EdgeExecutor::pruneTrackingStatesLocked(const rclcpp::Time& now) {
    for (auto it = tracking_state_map_.begin(); it != tracking_state_map_.end();) {
        if ((now - it->second.received_at).seconds() > tracking_state_ttl_sec_) {
            it = tracking_state_map_.erase(it);
        } else {
            ++it;
        }
    }
}

bool EdgeExecutor::requestMode(
    const std::string& profile,
    const std::string& reason,
    std::string& error) {
    if (profile.empty()) {
        return true;
    }

    if (use_xhu_backend_) {
        if (!xhu_mode_client_->wait_for_service(std::chrono::seconds(2))) {
            error = "set_xhu_motion_mode service not available";
            return false;
        }

        auto request = std::make_shared<rc26_interfaces::srv::SetXhuMotionMode::Request>();
        request->mode = profile;
        request->timeout = 0.0F;
        request->reason = reason;

        auto future = xhu_mode_client_->async_send_request(request);
        const auto wait_result = waitForFuture(future, std::chrono::seconds(3));
        if (wait_result != WaitOutcome::READY) {
            error = "set_xhu_motion_mode request timed out";
            if (wait_result == WaitOutcome::SHUTDOWN) {
                error = "set_xhu_motion_mode interrupted by shutdown";
            }
            return false;
        }

        const auto response = future.get();
        if (!response || !response->success) {
            error = response ? response->message : "set_xhu_motion_mode returned null response";
            return false;
        }
        return true;
    }

    if (!nav_mode_client_->wait_for_service(std::chrono::seconds(2))) {
        error = "set_nav_mode service not available";
        return false;
    }

    auto request = std::make_shared<rc26_interfaces::srv::SetNavMode::Request>();
    request->profile = profile;
    request->timeout = 0.0F;
    request->reason = reason;

    auto future = nav_mode_client_->async_send_request(request);
    const auto wait_result = waitForFuture(future, std::chrono::seconds(3));
    if (wait_result != WaitOutcome::READY) {
        error = "set_nav_mode request timed out";
        if (wait_result == WaitOutcome::SHUTDOWN) {
            error = "set_nav_mode interrupted by shutdown";
        }
        return false;
    }

    const auto response = future.get();
    if (!response || !response->success) {
        error = response ? response->message : "set_nav_mode returned null response";
        return false;
    }

    return true;
}

std::pair<float, float> EdgeExecutor::inferSpeedLimits(
    const std::string& required_mode,
    const std::string& motion_type) {
    const auto mode = toLowerCopy(required_mode);
    const auto motion = toLowerCopy(motion_type);

    if (mode == "hold") {
        return {0.0F, 0.0F};
    }
    if (mode == "ramp_up" || mode == "ramp_down" ||
        motion == "ramp_up" || motion == "ramp_down") {
        return {0.30F, 0.35F};
    }
    if (mode == "mf_traverse" || mode == "mf_exit") {
        return {0.50F, 0.50F};
    }
    return {0.80F, 1.00F};
}

nav_msgs::msg::Path EdgeExecutor::generateCorridor(
    const FieldGraph& graph,
    const GraphEdge& edge) const {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = node_->now();

    auto from_it = graph.nodes.find(edge.from);
    auto to_it = graph.nodes.find(edge.to);
    if (from_it == graph.nodes.end() || to_it == graph.nodes.end()) return path;

    const auto& fp = from_it->second.pose;
    const auto& tp = to_it->second.pose;

    std::vector<Pose3> waypoints;
    waypoints.push_back(fp);
    for (const auto& control_point : edge.control_points) {
        waypoints.push_back(control_point);
    }
    waypoints.push_back(tp);

    for (size_t segment_index = 1; segment_index < waypoints.size(); ++segment_index) {
        const auto& from = waypoints[segment_index - 1];
        const auto& to = waypoints[segment_index];
        const double dx = to.x - from.x;
        const double dy = to.y - from.y;
        const double dz = to.z - from.z;
        const double dist_2d = std::hypot(dx, dy);
        const int n_points = std::max(2, static_cast<int>(dist_2d / INTERPOLATION_SPACING) + 1);

        for (int i = 0; i <= n_points; ++i) {
            if (segment_index > 1 && i == 0) {
                continue;
            }
            const double t = static_cast<double>(i) / n_points;
            geometry_msgs::msg::PoseStamped ps;
            ps.header = path.header;
            ps.pose.position.x = from.x + t * dx;
            ps.pose.position.y = from.y + t * dy;
            ps.pose.position.z = from.z + t * dz;

            const double yaw = std::atan2(dy, dx);
            ps.pose.orientation.z = std::sin(yaw / 2.0);
            ps.pose.orientation.w = std::cos(yaw / 2.0);
            path.poses.push_back(ps);
        }
    }

    // ensure goal pose uses to-node yaw
    if (!path.poses.empty()) {
        auto& last = path.poses.back();
        last.pose.orientation.z = std::sin(tp.yaw / 2.0);
        last.pose.orientation.w = std::cos(tp.yaw / 2.0);
    }

    return path;
}

EdgeExecutor::ExecResult EdgeExecutor::executeEdge(
    const FieldGraph& graph,
    const GraphEdge& edge,
    const std::string& controller_id) {
    cancelled_.store(false);
    state_ = EdgeExecState::EXECUTING;

    if (use_xhu_backend_) {
        return executeEdgeViaXhu(graph, edge);
    }
    return executeEdgeViaNav2(graph, edge, controller_id);
}

EdgeExecutor::ExecResult EdgeExecutor::executeEdgeViaXhu(
    const FieldGraph& graph,
    const GraphEdge& edge) {
    ExecResult result;

    std::string mode_error;
    if (!requestMode(edge.required_mode, "topo_edge:" + edge.id, mode_error)) {
        result.failure_reason =
            "Failed to set xhu motion mode '" + edge.required_mode + "': " + mode_error;
        state_ = EdgeExecState::FAILED;
        return result;
    }

    const auto corridor_path = generateCorridor(graph, edge);
    if (corridor_path.poses.empty()) {
        result.failure_reason = "Empty corridor for edge '" + edge.id + "'";
        state_ = EdgeExecState::FAILED;
        return result;
    }

    const auto corridor_id = edge.id + "_" + std::to_string(corridor_seq_.fetch_add(1));
    clearTrackingState(corridor_id);
    ScopeExit clear_tracking([this, &corridor_id]() { clearTrackingState(corridor_id); });

    rc26_interfaces::msg::XhuSemanticCorridor corridor_msg;
    corridor_msg.header.stamp = node_->now();
    corridor_msg.header.frame_id = "map";
    corridor_msg.corridor_id = corridor_id;
    corridor_msg.edge_id = edge.id;
    corridor_msg.from_node_id = edge.from;
    corridor_msg.to_node_id = edge.to;
    corridor_msg.motion_type = edge.motion_type;
    corridor_msg.required_mode = edge.required_mode;
    corridor_msg.path = corridor_path;
    const auto [max_linear, max_angular] = inferSpeedLimits(edge.required_mode, edge.motion_type);
    corridor_msg.max_linear_speed = max_linear;
    corridor_msg.max_angular_speed = max_angular;
    corridor_msg.stop_at_end = true;
    corridor_msg.allow_reverse = false;
    corridor_pub_->publish(corridor_msg);

    const auto wait_begin = std::chrono::steady_clock::now();
    auto last_publish = wait_begin;
    auto last_tracking_update = wait_begin;
    bool received_tracking = false;
    std::optional<std::chrono::steady_clock::time_point> hold_begin;
    while (rclcpp::ok()) {
        if (cancelled_.load()) {
            state_ = EdgeExecState::FAILED;
            result.final_state = EdgeExecState::FAILED;
            result.failure_reason = "Cancelled";
            return result;
        }

        const auto now = std::chrono::steady_clock::now();
        const double waited_sec = std::chrono::duration<double>(now - wait_begin).count();
        if (waited_sec > xhu_exec_timeout_sec_) {
            state_ = EdgeExecState::REPLAN_REQUESTED;
            result.final_state = EdgeExecState::REPLAN_REQUESTED;
            result.failure_reason = "xhu corridor execution timed out";
            return result;
        }

        if (!received_tracking &&
            std::chrono::duration<double>(now - last_publish).count() >= xhu_corridor_republish_period_sec_) {
            corridor_msg.header.stamp = node_->now();
            corridor_msg.path.header.stamp = corridor_msg.header.stamp;
            corridor_pub_->publish(corridor_msg);
            last_publish = now;
        }

        if (!received_tracking && waited_sec > xhu_corridor_accept_timeout_sec_) {
            state_ = EdgeExecState::FAILED;
            result.final_state = EdgeExecState::FAILED;
            result.failure_reason = "No xhu tracking feedback after corridor publish";
            return result;
        }

        if (received_tracking &&
            std::chrono::duration<double>(now - last_tracking_update).count() > xhu_tracking_state_timeout_sec_) {
            state_ = EdgeExecState::FAILED;
            result.final_state = EdgeExecState::FAILED;
            result.failure_reason = "xhu tracking feedback timed out";
            return result;
        }

        const auto tracking = latestTrackingState(corridor_id);
        if (tracking.has_value()) {
            received_tracking = true;
            last_tracking_update = now;
            const auto status = toLowerCopy(tracking->status);

            if (status == "hold" && !tracking->terminal) {
                if (!hold_begin.has_value()) {
                    hold_begin = now;
                }
                const double hold_sec =
                    std::chrono::duration<double>(now - *hold_begin).count();
                if (hold_sec > xhu_hold_replan_timeout_sec_) {
                    state_ = EdgeExecState::REPLAN_REQUESTED;
                    result.final_state = EdgeExecState::REPLAN_REQUESTED;
                    result.failure_reason = "xhu hold exceeded replan timeout";
                    return result;
                }
            } else {
                hold_begin.reset();
            }

            if (!tracking->terminal && status == "replan") {
                state_ = EdgeExecState::REPLAN_REQUESTED;
                result.final_state = EdgeExecState::REPLAN_REQUESTED;
                result.failure_reason =
                    tracking->reason.empty() ? "xhu requested replan" : tracking->reason;
                return result;
            }

            if (!tracking->terminal && status == "abort") {
                state_ = EdgeExecState::FAILED;
                result.final_state = EdgeExecState::FAILED;
                result.failure_reason =
                    tracking->reason.empty() ? "xhu follower abort" : tracking->reason;
                return result;
            }

            if (tracking->terminal) {
                if (status == "pass") {
                    state_ = EdgeExecState::SUCCEEDED;
                    result.success = true;
                    result.final_state = EdgeExecState::SUCCEEDED;
                    return result;
                }

                if (status == "replan" || status == "hold") {
                    state_ = EdgeExecState::REPLAN_REQUESTED;
                    result.final_state = EdgeExecState::REPLAN_REQUESTED;
                    result.failure_reason =
                        tracking->reason.empty() ? "xhu requested replan" : tracking->reason;
                    return result;
                }

                state_ = EdgeExecState::FAILED;
                result.final_state = EdgeExecState::FAILED;
                result.failure_reason =
                    tracking->reason.empty() ? "xhu follower abort" : tracking->reason;
                return result;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    state_ = EdgeExecState::FAILED;
    result.failure_reason = "rclcpp shutdown";
    return result;
}

EdgeExecutor::ExecResult EdgeExecutor::executeEdgeViaNav2(
    const FieldGraph& graph,
    const GraphEdge& edge,
    const std::string& controller_id) {
    ExecResult result;

    std::string mode_error;
    if (!requestMode(edge.required_mode, "topo_edge:" + edge.id, mode_error)) {
        result.failure_reason =
            "Failed to set nav mode '" + edge.required_mode + "': " + mode_error;
        state_ = EdgeExecState::FAILED;
        return result;
    }

    if (!follow_path_client_) {
        follow_path_client_ = rclcpp_action::create_client<FollowPathAction>(node_, "follow_path");
    }
    if (!follow_path_client_->wait_for_action_server(std::chrono::seconds(5))) {
        result.failure_reason = "FollowPath action server not available";
        state_ = EdgeExecState::FAILED;
        return result;
    }

    auto corridor = generateCorridor(graph, edge);
    if (corridor.poses.empty()) {
        result.failure_reason = "Empty corridor for edge '" + edge.id + "'";
        state_ = EdgeExecState::FAILED;
        return result;
    }

    // retry loop: one local retry + one replan request
    for (int attempt = 0; attempt < 2; ++attempt) {
        if (cancelled_.load()) {
            state_ = EdgeExecState::FAILED;
            result.final_state = EdgeExecState::FAILED;
            result.failure_reason = "Cancelled";
            return result;
        }

        auto goal_msg = FollowPathAction::Goal();
        goal_msg.path = corridor;
        goal_msg.controller_id = controller_id;

        auto send_goal_options = rclcpp_action::Client<FollowPathAction>::SendGoalOptions();
        auto future = follow_path_client_->async_send_goal(goal_msg, send_goal_options);

        const auto send_result = waitForFuture(future, std::chrono::seconds(30), &cancelled_);
        if (send_result == WaitOutcome::CANCELLED) {
            state_ = EdgeExecState::FAILED;
            result.final_state = EdgeExecState::FAILED;
            result.failure_reason = "Cancelled";
            return result;
        }
        if (send_result != WaitOutcome::READY) {
            if (attempt == 0) {
                state_ = EdgeExecState::LOCAL_RETRY;
                continue;
            }
            result.failure_reason = send_result == WaitOutcome::SHUTDOWN
                                        ? "FollowPath goal send interrupted by shutdown"
                                        : "FollowPath goal send timed out";
            state_ = EdgeExecState::REPLAN_REQUESTED;
            result.final_state = EdgeExecState::REPLAN_REQUESTED;
            return result;
        }

        auto goal_handle = future.get();
        if (!goal_handle) {
            if (attempt == 0) {
                state_ = EdgeExecState::LOCAL_RETRY;
                continue;
            }
            result.failure_reason = "FollowPath goal rejected";
            state_ = EdgeExecState::REPLAN_REQUESTED;
            result.final_state = EdgeExecState::REPLAN_REQUESTED;
            return result;
        }

        auto result_future = follow_path_client_->async_get_result(goal_handle);
        const auto exec_wait_result =
            waitForFuture(result_future, std::chrono::seconds(60), &cancelled_);
        if (exec_wait_result == WaitOutcome::CANCELLED) {
            state_ = EdgeExecState::FAILED;
            result.final_state = EdgeExecState::FAILED;
            result.failure_reason = "Cancelled";
            return result;
        }
        if (exec_wait_result != WaitOutcome::READY) {
            if (attempt == 0) {
                state_ = EdgeExecState::LOCAL_RETRY;
                continue;
            }
            result.failure_reason = exec_wait_result == WaitOutcome::SHUTDOWN
                                        ? "FollowPath execution interrupted by shutdown"
                                        : "FollowPath execution timed out";
            state_ = EdgeExecState::REPLAN_REQUESTED;
            result.final_state = EdgeExecState::REPLAN_REQUESTED;
            return result;
        }

        auto wrapped = result_future.get();
        if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
            state_ = EdgeExecState::SUCCEEDED;
            result.success = true;
            result.final_state = EdgeExecState::SUCCEEDED;
            return result;
        }

        // first failure -> local retry; second -> replan
        if (attempt == 0) {
            state_ = EdgeExecState::LOCAL_RETRY;
            RCLCPP_WARN(node_->get_logger(),
                "Edge '%s' first attempt failed, retrying locally", edge.id.c_str());
        }
    }

    result.failure_reason = "Edge execution failed after retry";
    state_ = EdgeExecState::REPLAN_REQUESTED;
    result.final_state = EdgeExecState::REPLAN_REQUESTED;
    return result;
}

void EdgeExecutor::cancel() {
    cancelled_.store(true);
    if (use_xhu_backend_) {
        std::string error;
        (void)requestMode("hold", "topo_cancelled", error);
        return;
    }
    follow_path_client_->async_cancel_all_goals();
}

}  // namespace rc26_topo_nav
