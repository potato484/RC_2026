#include "rc26_topo_nav/topo_nav_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <exception>
#include <functional>
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

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion) {
    const double siny_cosp =
        2.0 * ((quaternion.w * quaternion.z) + (quaternion.x * quaternion.y));
    const double cosy_cosp =
        1.0 - 2.0 * ((quaternion.y * quaternion.y) + (quaternion.z * quaternion.z));
    return std::atan2(siny_cosp, cosy_cosp);
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

}  // namespace

TopoNavNode::TopoNavNode(const rclcpp::NodeOptions& options)
    : Node("topo_nav_node", options) {
    // Parameters
    this->declare_parameter("team", "blue");
    this->declare_parameter("graph_file", "");
    this->declare_parameter("surface_graph_file", "");
    this->declare_parameter("weights.time", 1.0);
    this->declare_parameter("weights.height_risk", 2.0);
    this->declare_parameter("weights.drop_risk", 3.0);
    this->declare_parameter("weights.localization_risk", 2.0);
    this->declare_parameter("weights.dynamic_block", 1000.0);
    this->declare_parameter("weights.confirm_required", 1.5);
    this->declare_parameter("weights.slow_only", 2.0);
    this->declare_parameter("xhu.exec_timeout_sec", 45.0);
    this->declare_parameter("xhu.hold_replan_timeout_sec", 2.0);
    this->declare_parameter("surface_start_match_xy_m", surface_start_match_xy_m_);
    this->declare_parameter("surface_start_match_z_m", surface_start_match_z_m_);
    this->declare_parameter("surface_start_match_yaw_deg", surface_start_match_yaw_deg_);
    this->declare_parameter("surface_anchor_radius_m", surface_anchor_radius_m_);

    team_ = this->get_parameter("team").as_string();
    std::string graph_file = this->get_parameter("graph_file").as_string();
    std::string surface_graph_file = this->get_parameter("surface_graph_file").as_string();
    if (graph_file.empty()) {
        const auto pkg_dir = ament_index_cpp::get_package_share_directory("rc26_topo_nav");
        const auto normalized_team = toLowerCopy(team_);
        graph_file = pkg_dir + "/config/" +
            (normalized_team == "red" ? std::string("r2_field_graph_red.yaml")
                                       : std::string("r2_field_graph_blue.yaml"));
        if (surface_graph_file.empty()) {
            surface_graph_file = pkg_dir + "/config/" +
                (normalized_team == "red" ? std::string("r2_surface_graph_red.yaml")
                                           : std::string("r2_surface_graph_blue.yaml"));
        }
    }

    weights_.time = this->get_parameter("weights.time").as_double();
    weights_.height_risk = this->get_parameter("weights.height_risk").as_double();
    weights_.drop_risk = this->get_parameter("weights.drop_risk").as_double();
    weights_.localization_risk = this->get_parameter("weights.localization_risk").as_double();
    weights_.dynamic_block = this->get_parameter("weights.dynamic_block").as_double();
    weights_.confirm_required = this->get_parameter("weights.confirm_required").as_double();
    weights_.slow_only = this->get_parameter("weights.slow_only").as_double();
    surface_start_match_xy_m_ = this->get_parameter("surface_start_match_xy_m").as_double();
    surface_start_match_z_m_ = this->get_parameter("surface_start_match_z_m").as_double();
    surface_start_match_yaw_deg_ = this->get_parameter("surface_start_match_yaw_deg").as_double();
    surface_anchor_radius_m_ = this->get_parameter("surface_anchor_radius_m").as_double();

    // Load graph
    if (!graph_file.empty()) {
        auto lr = loadFieldGraph(graph_file);
        if (!lr.success) {
            RCLCPP_FATAL(get_logger(), "Failed to load graph: %s", lr.error.c_str());
            throw std::runtime_error("Graph load failed: " + lr.error);
        }
        graph_ = std::move(lr.graph);

        auto vr = validateGraph(graph_);
        if (!vr.valid) {
            for (const auto& e : vr.errors) {
                RCLCPP_ERROR(get_logger(), "Graph validation: %s", e.c_str());
            }
            throw std::runtime_error("Graph validation failed");
        }
        if (!graph_.team.empty() && toLowerCopy(graph_.team) != toLowerCopy(team_)) {
            RCLCPP_WARN(
                get_logger(),
                "Graph team '%s' differs from node team '%s', using loaded graph as truth",
                graph_.team.c_str(), team_.c_str());
        }
        RCLCPP_INFO(get_logger(), "Loaded graph: %zu nodes, %zu edges",
                     graph_.nodes.size(), graph_.edges.size());
    }

    if (!surface_graph_file.empty()) {
        auto lr = loadFieldGraph(surface_graph_file);
        if (!lr.success) {
            RCLCPP_FATAL(get_logger(), "Failed to load surface graph: %s", lr.error.c_str());
            throw std::runtime_error("Surface graph load failed: " + lr.error);
        }
        surface_graph_ = std::move(lr.graph);

        auto vr = validateGraph(surface_graph_);
        if (!vr.valid) {
            for (const auto& e : vr.errors) {
                RCLCPP_ERROR(get_logger(), "Surface graph validation: %s", e.c_str());
            }
            throw std::runtime_error("Surface graph validation failed");
        }
        RCLCPP_INFO(
            get_logger(),
            "Loaded surface graph: %zu nodes, %zu edges",
            surface_graph_.nodes.size(),
            surface_graph_.edges.size());
    }

    // TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Components
    overlay_reducer_ = std::make_unique<OverlayReducer>(this);
    edge_executor_ = std::make_unique<EdgeExecutor>(this);
    diagnostics_ = std::make_unique<Diagnostics>(this);

    // Action server
    using namespace std::placeholders;
    action_server_ = rclcpp_action::create_server<NavigateTopoTarget>(
        this, "navigate_topo_target",
        std::bind(&TopoNavNode::handleGoal, this, _1, _2),
        std::bind(&TopoNavNode::handleCancel, this, _1),
        std::bind(&TopoNavNode::handleAccepted, this, _1));

    surface_action_server_ = rclcpp_action::create_server<NavigateSurfaceRoute>(
        this, "navigate_surface_route",
        std::bind(&TopoNavNode::handleSurfaceGoal, this, _1, _2),
        std::bind(&TopoNavNode::handleSurfaceCancel, this, _1),
        std::bind(&TopoNavNode::handleSurfaceAccepted, this, _1));

    diagnostics_->publishDiagnostic("OK", "TopoNavNode initialized");
}

TopoNavNode::~TopoNavNode() {
    if (edge_executor_) {
        edge_executor_->cancel();
    }

    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

rclcpp_action::GoalResponse TopoNavNode::handleGoal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const NavigateTopoTarget::Goal> goal) {
    if (!goal) {
        diagnostics_->publishDiagnostic("ERROR", "Rejected null topo goal");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->target_id.empty()) {
        diagnostics_->publishDiagnostic("WARN", "Rejected topo goal with empty target_id");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->target_type != NavigateTopoTarget::Goal::TARGET_NODE &&
        goal->target_type != NavigateTopoTarget::Goal::TARGET_TASK &&
        goal->target_type != NavigateTopoTarget::Goal::TARGET_ROUTE) {
        diagnostics_->publishDiagnostic("WARN", "Rejected topo goal with invalid target_type");
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal_active_.exchange(true)) {
        diagnostics_->publishDiagnostic("WARN", "Rejected concurrent topo goal");
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TopoNavNode::handleCancel(
    std::shared_ptr<GoalHandle> /*goal_handle*/) {
    edge_executor_->cancel();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TopoNavNode::handleAccepted(std::shared_ptr<GoalHandle> goal_handle) {
    try {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
        worker_thread_ = std::thread([this, goal_handle]() { execute(goal_handle); });
    } catch (const std::exception& e) {
        goal_active_.store(false);
        auto result = std::make_shared<NavigateTopoTarget::Result>();
        result->success = false;
        result->failure_code = "THREAD_START_FAILED";
        result->failure_reason = e.what();
        goal_handle->abort(result);
        diagnostics_->publishDiagnostic("ERROR", "Failed to start topo worker thread");
    }
}

rclcpp_action::GoalResponse TopoNavNode::handleSurfaceGoal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const NavigateSurfaceRoute::Goal> goal) {
    if (!goal) {
        diagnostics_->publishDiagnostic("ERROR", "Rejected null surface goal");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (surface_graph_.nodes.empty()) {
        diagnostics_->publishDiagnostic("ERROR", "Rejected surface goal without loaded surface graph");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal_active_.exchange(true)) {
        diagnostics_->publishDiagnostic("WARN", "Rejected concurrent surface goal");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TopoNavNode::handleSurfaceCancel(
    std::shared_ptr<SurfaceGoalHandle> /*goal_handle*/) {
    edge_executor_->cancel();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TopoNavNode::handleSurfaceAccepted(std::shared_ptr<SurfaceGoalHandle> goal_handle) {
    try {
        std::lock_guard<std::mutex> lock(worker_mutex_);
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
        worker_thread_ = std::thread([this, goal_handle]() { executeSurface(goal_handle); });
    } catch (const std::exception& e) {
        goal_active_.store(false);
        auto result = std::make_shared<NavigateSurfaceRoute::Result>();
        result->success = false;
        result->failure_code = "THREAD_START_FAILED";
        result->failure_reason = e.what();
        goal_handle->abort(result);
        diagnostics_->publishDiagnostic("ERROR", "Failed to start surface worker thread");
    }
}

std::string TopoNavNode::findNearestNode() const {
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException& e) {
        RCLCPP_WARN(get_logger(), "TF lookup failed: %s", e.what());
        return "";
    }

    double rx = tf.transform.translation.x;
    double ry = tf.transform.translation.y;

    std::string best;
    double best_dist = 1e9;
    for (const auto& [id, node] : graph_.nodes) {
        double d = std::hypot(node.pose.x - rx, node.pose.y - ry);
        if (d < best_dist) {
            best_dist = d;
            best = id;
        }
    }
    return best;
}

bool TopoNavNode::currentRobotPose(Pose3& pose_out) const {
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException& e) {
        RCLCPP_WARN(get_logger(), "TF lookup failed: %s", e.what());
        return false;
    }

    pose_out.x = tf.transform.translation.x;
    pose_out.y = tf.transform.translation.y;
    pose_out.z = tf.transform.translation.z;
    pose_out.yaw = yawFromQuaternion(tf.transform.rotation);
    return true;
}

void TopoNavNode::execute(std::shared_ptr<GoalHandle> goal_handle) {
    ScopeExit goal_guard([this]() { goal_active_.store(false); });
    std::lock_guard<std::mutex> execution_lock(execution_mutex_);

    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<NavigateTopoTarget::Feedback>();
    auto result = std::make_shared<NavigateTopoTarget::Result>();

    try {
        if (!goal->team.empty() && toLowerCopy(goal->team) != toLowerCopy(team_)) {
            result->success = false;
            result->failure_code = "TEAM_MISMATCH";
            result->failure_reason = "Goal team does not match topo_nav runtime team";
            goal_handle->abort(result);
            return;
        }

        std::string start_node = findNearestNode();
        if (start_node.empty()) {
            result->success = false;
            result->terminal_node_id.clear();
            result->failure_code = "NO_TF";
            result->failure_reason = "Cannot determine robot position";
            goal_handle->abort(result);
            return;
        }

        uint32_t replan_count = 0;

        for (int plan_attempt = 0; plan_attempt <= MAX_REPLAN_ATTEMPTS; ++plan_attempt) {
            if (goal_handle->is_canceling()) {
                edge_executor_->cancel();
                result->success = false;
                result->failure_code = "CANCELLED";
                goal_handle->canceled(result);
                return;
            }

            if (overlay_reducer_->shouldHold()) {
                std::string hold_error;
                (void)edge_executor_->requestMode("hold", "loc_red_pre_plan", hold_error);
                diagnostics_->publishDiagnostic("WARN", "Hold: localization RED");
                result->success = false;
                result->terminal_node_id = start_node;
                result->failure_code = "LOC_RED_HOLD";
                result->failure_reason = "Localization RED, action aborted";
                goal_handle->abort(result);
                return;
            }

            std::unordered_map<std::string, NodeOverlay> node_overlays;
            std::unordered_map<std::string, EdgeOverlay> edge_overlays;
            overlay_reducer_->applyOverlays(graph_, node_overlays, edge_overlays);

            PlanResult plan;
            if (goal->target_type == NavigateTopoTarget::Goal::TARGET_NODE) {
                plan = planRoute(graph_, start_node, goal->target_id,
                                 node_overlays, edge_overlays, weights_);
            } else if (goal->target_type == NavigateTopoTarget::Goal::TARGET_TASK) {
                plan = planToTask(graph_, start_node, goal->target_id,
                                  node_overlays, edge_overlays, weights_);
            } else if (goal->target_type == NavigateTopoTarget::Goal::TARGET_ROUTE) {
                plan = planRouteTag(graph_, start_node, goal->target_id,
                                    node_overlays, edge_overlays, weights_);
            } else {
                result->success = false;
                result->terminal_node_id = start_node;
                result->failure_code = "INVALID_TARGET_TYPE";
                result->failure_reason = "Unsupported topo target_type";
                goal_handle->abort(result);
                return;
            }

            if (!plan.success || plan.node_path.empty()) {
                result->success = false;
                result->terminal_node_id = start_node;
                result->failure_code = "NO_PATH";
                result->failure_reason = plan.failure_reason.empty() ? "Planner returned empty route"
                                                                    : plan.failure_reason;
                goal_handle->abort(result);
                return;
            }

            diagnostics_->publishRoute(plan, graph_);

            bool route_completed = true;
            for (size_t i = 0; i < plan.edge_indices.size(); ++i) {
                if (goal_handle->is_canceling()) {
                    edge_executor_->cancel();
                    result->success = false;
                    result->failure_code = "CANCELLED";
                    goal_handle->canceled(result);
                    return;
                }

                if (overlay_reducer_->shouldHold()) {
                    std::string hold_error;
                    (void)edge_executor_->requestMode("hold", "loc_red_during_exec", hold_error);
                    result->success = false;
                    result->terminal_node_id = start_node;
                    result->failure_code = "LOC_RED_HOLD";
                    result->failure_reason = "Localization RED during execution";
                    goal_handle->abort(result);
                    return;
                }

                const auto& edge = graph_.edges[plan.edge_indices[i]];
                diagnostics_->publishActiveEdge(edge, graph_, "PASS");

                feedback->active_node_id = edge.from;
                feedback->active_edge_id = edge.id;
                feedback->exec_state = "EXECUTING";
                feedback->replan_count = replan_count;
                goal_handle->publish_feedback(feedback);

                auto corridor = edge_executor_->generateCorridor(graph_, edge);
                diagnostics_->publishCorridor(corridor);

                auto exec_result = edge_executor_->executeEdge(graph_, edge);
                if (exec_result.success) {
                    start_node = edge.to;
                    continue;
                }

                if (exec_result.final_state == EdgeExecState::REPLAN_REQUESTED &&
                    goal->allow_replan && plan_attempt < MAX_REPLAN_ATTEMPTS) {
                    replan_count++;
                    start_node = findNearestNode();
                    if (start_node.empty()) {
                        start_node = edge.from;
                    }
                    route_completed = false;
                    diagnostics_->publishActiveEdge(edge, graph_, "REPLAN");
                    diagnostics_->publishDiagnostic("WARN", "Replanning from " + start_node);
                    break;
                }

                diagnostics_->publishActiveEdge(edge, graph_, "ABORT");
                result->success = false;
                result->terminal_node_id = edge.from;
                result->failure_code = "EDGE_EXEC_FAILED";
                result->failure_reason = exec_result.failure_reason;
                goal_handle->abort(result);
                diagnostics_->publishDiagnostic("ERROR", "Edge failed: " + edge.id);
                return;
            }

            if (route_completed) {
                result->success = true;
                result->terminal_node_id = plan.node_path.back();
                goal_handle->succeed(result);
                diagnostics_->publishDiagnostic("OK", "Navigation succeeded");
                return;
            }
        }

        result->success = false;
        result->failure_code = "MAX_REPLAN_EXCEEDED";
        result->failure_reason = "Exceeded maximum replan attempts";
        goal_handle->abort(result);
        diagnostics_->publishDiagnostic("ERROR", "Max replan exceeded");
    } catch (const std::exception& e) {
        edge_executor_->cancel();
        result->success = false;
        result->failure_code = "INTERNAL_ERROR";
        result->failure_reason = e.what();
        goal_handle->abort(result);
        diagnostics_->publishDiagnostic("ERROR", std::string("Unhandled exception: ") + e.what());
    } catch (...) {
        edge_executor_->cancel();
        result->success = false;
        result->failure_code = "INTERNAL_ERROR";
        result->failure_reason = "Unknown exception";
        goal_handle->abort(result);
        diagnostics_->publishDiagnostic("ERROR", "Unhandled unknown exception");
    }
}

void TopoNavNode::executeSurface(std::shared_ptr<SurfaceGoalHandle> goal_handle) {
    ScopeExit goal_guard([this]() { goal_active_.store(false); });
    std::lock_guard<std::mutex> execution_lock(execution_mutex_);

    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<NavigateSurfaceRoute::Feedback>();
    auto result = std::make_shared<NavigateSurfaceRoute::Result>();

    try {
        if (!goal->team.empty() && toLowerCopy(goal->team) != toLowerCopy(team_)) {
            result->success = false;
            result->failure_code = "TEAM_MISMATCH";
            result->failure_reason = "Goal team does not match topo_nav runtime team";
            goal_handle->abort(result);
            return;
        }

        const auto now = this->now();
        const auto start_pose = pose3FromPoseStamped(goal->start_pose);
        const auto goal_pose = pose3FromPoseStamped(goal->goal_pose);
        const auto surface_plan = planSurfaceRoute(
            surface_graph_,
            start_pose,
            goal_pose,
            weights_,
            surface_anchor_radius_m_,
            now,
            "map");

        result->projected_start_pose = poseStampedFromPose3(
            surface_plan.projected_start.pose, "map", now);
        result->projected_goal_pose = poseStampedFromPose3(
            surface_plan.projected_goal.pose, "map", now);
        result->planned_path = surface_plan.planned_path;

        if (!surface_plan.success) {
            result->success = false;
            result->failure_code = surface_plan.failure_code;
            result->failure_reason = surface_plan.failure_reason;
            goal_handle->abort(result);
            diagnostics_->publishDiagnostic("ERROR", "Surface route planning failed");
            return;
        }

        Pose3 robot_pose;
        if (!currentRobotPose(robot_pose)) {
            result->success = false;
            result->failure_code = "NO_TF";
            result->failure_reason = "Cannot determine robot position";
            goal_handle->abort(result);
            return;
        }

        if (!poseNear(
                robot_pose,
                surface_plan.projected_start.pose,
                surface_start_match_xy_m_,
                surface_start_match_z_m_,
                surface_start_match_yaw_deg_ * M_PI / 180.0)) {
            result->success = false;
            result->failure_code = "START_POSE_MISMATCH";
            result->failure_reason = "Robot is not close enough to the requested start pose";
            goal_handle->abort(result);
            diagnostics_->publishDiagnostic("WARN", "Surface route start pose mismatch");
            return;
        }

        diagnostics_->publishRoute(surface_plan.planned_path);
        for (const auto& segment : surface_plan.segments) {
            if (goal_handle->is_canceling()) {
                edge_executor_->cancel();
                result->success = false;
                result->failure_code = "CANCELLED";
                goal_handle->canceled(result);
                return;
            }

            diagnostics_->publishActiveLabel(segment.id, "PASS");
            diagnostics_->publishCorridor(segment.corridor);

            feedback->active_segment_id = segment.id;
            feedback->exec_state = "EXECUTING";
            feedback->replan_count = 0;
            goal_handle->publish_feedback(feedback);

            const auto exec_result = edge_executor_->executeCorridor(
                segment.id,
                segment.from_node_id,
                segment.to_node_id,
                segment.motion_type,
                segment.required_mode,
                segment.corridor);
            if (!exec_result.success) {
                result->success = false;
                result->terminal_segment_id = segment.id;
                result->failure_code =
                    exec_result.final_state == EdgeExecState::REPLAN_REQUESTED
                    ? "SEGMENT_REPLAN_REQUESTED"
                    : "SEGMENT_EXEC_FAILED";
                result->failure_reason = exec_result.failure_reason;
                diagnostics_->publishActiveLabel(segment.id, "ABORT");
                goal_handle->abort(result);
                diagnostics_->publishDiagnostic("ERROR", "Surface route execution failed");
                return;
            }
        }

        result->success = true;
        if (!surface_plan.segments.empty()) {
            result->terminal_segment_id = surface_plan.segments.back().id;
        }
        goal_handle->succeed(result);
        diagnostics_->publishDiagnostic("OK", "Surface navigation succeeded");
    } catch (const std::exception& e) {
        edge_executor_->cancel();
        result->success = false;
        result->failure_code = "INTERNAL_ERROR";
        result->failure_reason = e.what();
        goal_handle->abort(result);
        diagnostics_->publishDiagnostic("ERROR", std::string("Unhandled surface exception: ") + e.what());
    } catch (...) {
        edge_executor_->cancel();
        result->success = false;
        result->failure_code = "INTERNAL_ERROR";
        result->failure_reason = "Unknown exception";
        goal_handle->abort(result);
        diagnostics_->publishDiagnostic("ERROR", "Unhandled unknown surface exception");
    }
}

}  // namespace rc26_topo_nav
