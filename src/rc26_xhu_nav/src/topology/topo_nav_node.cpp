#include "rc26_xhu_nav/topology/topo_nav_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <sstream>
#include <thread>
#include <utility>

namespace rc26_xhu_nav::topology {

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

const char* topoTargetTypeName(const uint8_t target_type) {
    switch (target_type) {
        case rc26_interfaces::action::NavigateTopoTarget::Goal::TARGET_NODE:
            return "node";
        case rc26_interfaces::action::NavigateTopoTarget::Goal::TARGET_TASK:
            return "task";
        case rc26_interfaces::action::NavigateTopoTarget::Goal::TARGET_ROUTE:
            return "route";
        default:
            return "unknown";
    }
}

double elapsedMilliseconds(
    const std::chrono::steady_clock::time_point& begin,
    const std::chrono::steady_clock::time_point& end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

SurfacePlannerBackend parseSurfacePlannerBackend(const std::string& raw_value) {
    const auto normalized = toLowerCopy(raw_value);
    if (normalized == "body_planner") {
        return SurfacePlannerBackend::BODY_PLANNER;
    }
    return SurfacePlannerBackend::LEGACY;
}

std::string joinStrings(const std::vector<std::string>& values, const char* separator) {
    std::ostringstream ss;
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index > 0) {
            ss << separator;
        }
        ss << values[index];
    }
    return ss.str();
}

std::string dynamicOverlayReason(const OverlaySnapshot& snapshot) {
    if (snapshot.active_dynamic_sources.empty()) {
        return "";
    }
    return "source=" + joinStrings(snapshot.active_dynamic_sources, ",");
}

bool remainingSegmentsBlockedByDynamicOverlay(
    const OverlaySnapshot& snapshot,
    const FieldGraph& graph,
    const std::vector<SurfacePlanSegment>& segments,
    const std::size_t start_index) {
    for (std::size_t segment_index = start_index; segment_index < segments.size(); ++segment_index) {
        const auto& segment = segments[segment_index];
        if (snapshot.dynamic_blocked_nodes.find(segment.from_node_id) != snapshot.dynamic_blocked_nodes.end() ||
            snapshot.dynamic_blocked_nodes.find(segment.to_node_id) != snapshot.dynamic_blocked_nodes.end()) {
            return true;
        }
        for (const auto edge_index : segment.edge_indices) {
            if (edge_index >= graph.edges.size()) {
                continue;
            }
            if (snapshot.dynamic_blocked_edges.find(graph.edges[edge_index].id) !=
                snapshot.dynamic_blocked_edges.end()) {
                return true;
            }
        }
    }
    return false;
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

std::string execStateOrDefault(const std::string& exec_state) {
    return exec_state.empty() ? "EXECUTING" : exec_state;
}

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
    this->declare_parameter("surface_planner_backend", "body_planner");
    this->declare_parameter("robot_geometry_file", "");
    this->declare_parameter("robot_geometry_profile", "compact");
    this->declare_parameter("body_planning.enabled", surface_body_planning_.enabled);
    this->declare_parameter(
        "body_planning.require_annotated_surface_graph",
        surface_body_planning_.require_annotated_surface_graph);
    this->declare_parameter(
        "body_planning.clearance_margin_m",
        surface_body_planning_.clearance_margin_m);
    this->declare_parameter(
        "body_planning.max_surface_pitch_deg",
        surface_body_planning_.max_surface_pitch_deg);
    this->declare_parameter(
        "body_planning.max_edge_slope_deg",
        surface_body_planning_.max_edge_slope_deg);
    this->declare_parameter(
        "body_planning.max_step_height_m",
        surface_body_planning_.max_step_height_m);
    this->declare_parameter(
        "surface_body_planner.heading_bin_count",
        surface_planner_options_.body_planner.heading_bin_count);
    this->declare_parameter(
        "surface_body_planner.max_heading_change_deg",
        surface_planner_options_.body_planner.max_heading_change_deg);
    this->declare_parameter(
        "surface_body_planner.turn_cost_weight",
        surface_planner_options_.body_planner.turn_cost_weight);
    this->declare_parameter(
        "surface_body_planner.node_turn_clearance_gain",
        surface_planner_options_.body_planner.node_turn_clearance_gain);
    this->declare_parameter(
        "surface_body_planner.edge_turn_clearance_gain",
        surface_planner_options_.body_planner.edge_turn_clearance_gain);

    team_ = this->get_parameter("team").as_string();
    std::string graph_file = this->get_parameter("graph_file").as_string();
    std::string surface_graph_file = this->get_parameter("surface_graph_file").as_string();
    if (graph_file.empty()) {
        const auto pkg_dir = ament_index_cpp::get_package_share_directory("rc26_xhu_nav");
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
    surface_planner_options_.backend = parseSurfacePlannerBackend(
        this->get_parameter("surface_planner_backend").as_string());
    surface_body_planning_.enabled = this->get_parameter("body_planning.enabled").as_bool();
    surface_body_planning_.require_annotated_surface_graph =
        this->get_parameter("body_planning.require_annotated_surface_graph").as_bool();
    surface_body_planning_.clearance_margin_m =
        this->get_parameter("body_planning.clearance_margin_m").as_double();
    surface_body_planning_.max_surface_pitch_deg =
        this->get_parameter("body_planning.max_surface_pitch_deg").as_double();
    surface_body_planning_.max_edge_slope_deg =
        this->get_parameter("body_planning.max_edge_slope_deg").as_double();
    surface_body_planning_.max_step_height_m =
        this->get_parameter("body_planning.max_step_height_m").as_double();
    surface_planner_options_.body_planner.heading_bin_count =
        this->get_parameter("surface_body_planner.heading_bin_count").as_int();
    surface_planner_options_.body_planner.max_heading_change_deg =
        this->get_parameter("surface_body_planner.max_heading_change_deg").as_double();
    surface_planner_options_.body_planner.turn_cost_weight =
        this->get_parameter("surface_body_planner.turn_cost_weight").as_double();
    surface_planner_options_.body_planner.node_turn_clearance_gain =
        this->get_parameter("surface_body_planner.node_turn_clearance_gain").as_double();
    surface_planner_options_.body_planner.edge_turn_clearance_gain =
        this->get_parameter("surface_body_planner.edge_turn_clearance_gain").as_double();
    const std::string robot_geometry_file = this->get_parameter("robot_geometry_file").as_string();
    const std::string robot_geometry_profile =
        this->get_parameter("robot_geometry_profile").as_string();

    if (!robot_geometry_file.empty()) {
        std::string geometry_error;
        const auto geometry = loadRobotGeometryProfile(
            robot_geometry_file, robot_geometry_profile, geometry_error);
        if (!geometry) {
            RCLCPP_WARN(
                get_logger(),
                "Failed to load robot geometry file '%s': %s",
                robot_geometry_file.c_str(),
                geometry_error.c_str());
        } else {
            robot_geometry_ = geometry;
            surface_planner_options_.geometry = SurfacePlannerGeometry{
                geometry->half_length_m,
                geometry->half_width_m,
            };
            const double previous_anchor_radius = surface_anchor_radius_m_;
            surface_anchor_radius_m_ = std::max(
                surface_anchor_radius_m_,
                geometry->surface_projection_radius_m);
            RCLCPP_INFO(
                get_logger(),
                "Loaded robot geometry profile=%s half_length=%.3f half_width=%.3f height=%.3f "
                "surface_anchor_radius=%.3f (was %.3f)",
                geometry->name.c_str(),
                geometry->half_length_m,
                geometry->half_width_m,
                geometry->height_m,
                surface_anchor_radius_m_,
                previous_anchor_radius);
        }
    }
    RCLCPP_INFO(
        get_logger(),
        "Surface planner backend=%s heading_bin_count=%d max_heading_change_deg=%.1f "
        "turn_cost_weight=%.2f",
        surface_planner_options_.backend == SurfacePlannerBackend::BODY_PLANNER
            ? "body_planner"
            : "legacy",
        surface_planner_options_.body_planner.heading_bin_count,
        surface_planner_options_.body_planner.max_heading_change_deg,
        surface_planner_options_.body_planner.turn_cost_weight);

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
        std::string last_replan_failure_code;
        std::string last_replan_failure_reason;

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

            const auto plan_begin = std::chrono::steady_clock::now();
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
            const auto plan_end = std::chrono::steady_clock::now();
            const double plan_ms = elapsedMilliseconds(plan_begin, plan_end);

            if (!plan.success || plan.node_path.empty()) {
                const std::string failure_reason = plan.failure_reason.empty()
                    ? "Planner returned empty route"
                    : plan.failure_reason;
                RCLCPP_WARN(
                    get_logger(),
                    "Topo planning failed: attempt=%d target_type=%s target_id=%s start_node=%s elapsed_ms=%.2f "
                    "reason=%s",
                    plan_attempt + 1,
                    topoTargetTypeName(goal->target_type),
                    goal->target_id.c_str(),
                    start_node.c_str(),
                    plan_ms,
                    failure_reason.c_str());
                result->success = false;
                result->terminal_node_id = start_node;
                result->failure_code = "NO_PATH";
                result->failure_reason = failure_reason;
                goal_handle->abort(result);
                return;
            }

            RCLCPP_INFO(
                get_logger(),
                "Topo planning succeeded: attempt=%d target_type=%s target_id=%s start_node=%s terminal_node=%s "
                "elapsed_ms=%.2f node_count=%zu edge_count=%zu total_cost=%.3f",
                plan_attempt + 1,
                topoTargetTypeName(goal->target_type),
                goal->target_id.c_str(),
                start_node.c_str(),
                plan.node_path.back().c_str(),
                plan_ms,
                plan.node_path.size(),
                plan.edge_indices.size(),
                plan.total_cost);
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
                edge_executor_->setProgressCallback(
                    [goal_handle, feedback, &edge, &replan_count](
                        const EdgeExecutor::ExecProgress& progress) {
                        feedback->active_node_id = edge.from;
                        feedback->active_edge_id =
                            progress.edge_id.empty() ? edge.id : progress.edge_id;
                        feedback->exec_state = execStateOrDefault(progress.exec_state);
                        feedback->replan_count = replan_count;
                        goal_handle->publish_feedback(feedback);
                    });
                ScopeExit clear_progress([this]() { edge_executor_->setProgressCallback({}); });

                auto corridor = edge_executor_->generateCorridor(graph_, edge);
                diagnostics_->publishCorridor(corridor);

                auto exec_result = edge_executor_->executeEdge(graph_, edge);
                if (exec_result.success) {
                    start_node = edge.to;
                    continue;
                }

                if (!exec_result.failure_code.empty()) {
                    last_replan_failure_code = exec_result.failure_code;
                }
                if (!exec_result.failure_reason.empty()) {
                    last_replan_failure_reason = exec_result.failure_reason;
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
                result->failure_code =
                    exec_result.failure_code.empty() ? "EDGE_EXEC_FAILED" : exec_result.failure_code;
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
        result->failure_code = last_replan_failure_code.empty()
            ? "MAX_REPLAN_EXCEEDED"
            : last_replan_failure_code;
        result->failure_reason = last_replan_failure_reason.empty()
            ? "Exceeded maximum replan attempts"
            : last_replan_failure_reason;
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

        const auto start_pose = pose3FromPoseStamped(goal->start_pose);
        const auto goal_pose = pose3FromPoseStamped(goal->goal_pose);
        Pose3 requested_start_pose = start_pose;
        uint32_t replan_count = 0;
        std::string last_replan_failure_code;
        std::string last_replan_failure_reason;

        const bool geometry_required =
            surface_body_planning_.enabled ||
            surface_planner_options_.backend == SurfacePlannerBackend::BODY_PLANNER;
        if (geometry_required && !robot_geometry_) {
            result->success = false;
            result->failure_code = "ROBOT_GEOMETRY_UNAVAILABLE";
            result->failure_reason = "Surface planner requires robot geometry";
            goal_handle->abort(result);
            diagnostics_->publishDiagnostic("ERROR", "Surface planner missing robot geometry");
            return;
        }

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
                (void)edge_executor_->requestMode("hold", "loc_red_surface_pre_plan", hold_error);
                result->success = false;
                result->failure_code = "LOC_RED_HOLD";
                result->failure_reason = "Localization RED, surface route aborted";
                goal_handle->abort(result);
                diagnostics_->publishDiagnostic("WARN", "Hold: localization RED before surface planning");
                return;
            }

            Pose3 planning_start_pose = requested_start_pose;
            if (plan_attempt > 0 && !currentRobotPose(planning_start_pose)) {
                result->success = false;
                result->failure_code = "NO_TF";
                result->failure_reason = "Cannot determine robot position for surface replan";
                goal_handle->abort(result);
                return;
            }

            std::unordered_map<std::string, NodeOverlay> static_node_overlays;
            std::unordered_map<std::string, EdgeOverlay> static_edge_overlays;
            overlay_reducer_->applyOverlays(
                surface_graph_,
                static_node_overlays,
                static_edge_overlays,
                OverlayApplicationMode::STATIC_ONLY);

            std::unordered_map<std::string, NodeOverlay> runtime_node_overlays;
            std::unordered_map<std::string, EdgeOverlay> runtime_edge_overlays;
            overlay_reducer_->applyOverlays(
                surface_graph_,
                runtime_node_overlays,
                runtime_edge_overlays,
                OverlayApplicationMode::ALL);
            const auto runtime_overlay_snapshot = overlay_reducer_->snapshot();
            auto node_overlays = runtime_node_overlays;
            auto edge_overlays = runtime_edge_overlays;
            if (surface_body_planning_.enabled) {
                std::string body_planning_error;
                const auto stats = applySurfaceBodyPlanningOverlays(
                    surface_graph_,
                    *robot_geometry_,
                    surface_body_planning_,
                    node_overlays,
                    edge_overlays,
                    &body_planning_error);
                if (!body_planning_error.empty()) {
                    result->success = false;
                    result->failure_code = "SURFACE_GRAPH_NOT_BODY_AWARE";
                    result->failure_reason = body_planning_error;
                    goal_handle->abort(result);
                    diagnostics_->publishDiagnostic("ERROR", body_planning_error);
                    return;
                }
                RCLCPP_INFO(
                    get_logger(),
                    "Surface body planning overlays: penalized_nodes_clearance=%zu blocked_nodes_pitch=%zu "
                    "blocked_edges_clearance=%zu blocked_edges_slope=%zu blocked_edges_step=%zu",
                    stats.penalized_nodes_clearance,
                    stats.blocked_nodes_pitch,
                    stats.blocked_edges_clearance,
                    stats.blocked_edges_slope,
                    stats.blocked_edges_step);
            }

            const auto now = this->now();
            const auto plan_begin = std::chrono::steady_clock::now();
            const auto surface_plan = planSurfaceRoute(
                surface_graph_,
                planning_start_pose,
                goal_pose,
                node_overlays,
                edge_overlays,
                weights_,
                surface_anchor_radius_m_,
                now,
                surface_planner_options_,
                "map");
            const auto plan_end = std::chrono::steady_clock::now();
            const double plan_ms = elapsedMilliseconds(plan_begin, plan_end);

            result->projected_start_pose = poseStampedFromPose3(
                surface_plan.projected_start.pose, "map", now);
            result->projected_goal_pose = poseStampedFromPose3(
                surface_plan.projected_goal.pose, "map", now);
            result->planned_path = surface_plan.planned_path;

            if (!surface_plan.success) {
                const auto failure_analysis = classifySurfacePlanFailure(
                    surface_graph_,
                    planning_start_pose,
                    goal_pose,
                    static_node_overlays,
                    static_edge_overlays,
                    runtime_node_overlays,
                    runtime_edge_overlays,
                    node_overlays,
                    edge_overlays,
                    weights_,
                    surface_anchor_radius_m_,
                    surface_plan,
                    dynamicOverlayReason(runtime_overlay_snapshot));
                if (failure_analysis.best_projected_start.success) {
                    result->projected_start_pose = poseStampedFromPose3(
                        failure_analysis.best_projected_start.pose, "map", now);
                }
                if (failure_analysis.best_projected_goal.success) {
                    result->projected_goal_pose = poseStampedFromPose3(
                        failure_analysis.best_projected_goal.pose, "map", now);
                }
                RCLCPP_WARN(
                    get_logger(),
                    "Surface planning failed: attempt=%d start=(%.3f, %.3f, %.3f) "
                    "goal=(%.3f, %.3f, %.3f) projected_start=%s projected_goal=%s "
                    "elapsed_ms=%.2f code=%s reason=%s",
                    plan_attempt + 1,
                    planning_start_pose.x, planning_start_pose.y, planning_start_pose.z,
                    goal_pose.x, goal_pose.y, goal_pose.z,
                    surface_plan.projected_start.node_id.c_str(),
                    surface_plan.projected_goal.node_id.c_str(),
                    plan_ms,
                    failure_analysis.failure_code.empty()
                        ? surface_plan.failure_code.c_str()
                        : failure_analysis.failure_code.c_str(),
                    failure_analysis.failure_reason.empty()
                        ? surface_plan.failure_reason.c_str()
                        : failure_analysis.failure_reason.c_str());
                result->success = false;
                result->failure_code = failure_analysis.failure_code.empty()
                    ? surface_plan.failure_code
                    : failure_analysis.failure_code;
                result->failure_reason = failure_analysis.failure_reason.empty()
                    ? surface_plan.failure_reason
                    : failure_analysis.failure_reason;
                goal_handle->abort(result);
                diagnostics_->publishDiagnostic(
                    "ERROR",
                    "Surface route planning failed: " + result->failure_code);
                return;
            }

            RCLCPP_INFO(
                get_logger(),
                "Surface planning succeeded: attempt=%d start=(%.3f, %.3f, %.3f) "
                "goal=(%.3f, %.3f, %.3f) projected_start=%s projected_goal=%s "
                "backend=%s elapsed_ms=%.2f node_count=%zu edge_count=%zu path_points=%zu "
                "segments=%zu total_cost=%.3f",
                plan_attempt + 1,
                planning_start_pose.x, planning_start_pose.y, planning_start_pose.z,
                goal_pose.x, goal_pose.y, goal_pose.z,
                surface_plan.projected_start.node_id.c_str(),
                surface_plan.projected_goal.node_id.c_str(),
                surface_plan.planner_backend.c_str(),
                plan_ms,
                surface_plan.plan.node_path.size(),
                surface_plan.plan.edge_indices.size(),
                surface_plan.planned_path.poses.size(),
                surface_plan.segments.size(),
                surface_plan.plan.total_cost);

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
                RCLCPP_WARN(
                    get_logger(),
                    "Surface plan start pose mismatch after planning: attempt=%d projected_start=%s "
                    "elapsed_ms=%.2f robot=(%.3f, %.3f, %.3f, %.3f) "
                    "projected=(%.3f, %.3f, %.3f, %.3f)",
                    plan_attempt + 1,
                    surface_plan.projected_start.node_id.c_str(),
                    plan_ms,
                    robot_pose.x, robot_pose.y, robot_pose.z, robot_pose.yaw,
                    surface_plan.projected_start.pose.x,
                    surface_plan.projected_start.pose.y,
                    surface_plan.projected_start.pose.z,
                    surface_plan.projected_start.pose.yaw);
                result->success = false;
                result->failure_code = "START_POSE_MISMATCH";
                result->failure_reason = "Robot is not close enough to the requested start pose";
                goal_handle->abort(result);
                diagnostics_->publishDiagnostic("WARN", "Surface route start pose mismatch");
                return;
            }

            diagnostics_->publishRoute(surface_plan.planned_path);

            bool route_completed = true;
            const uint64_t planned_overlay_version = runtime_overlay_snapshot.version;
            for (std::size_t segment_index = 0; segment_index < surface_plan.segments.size(); ++segment_index) {
                const auto& segment = surface_plan.segments[segment_index];
                if (goal_handle->is_canceling()) {
                    edge_executor_->cancel();
                    result->success = false;
                    result->failure_code = "CANCELLED";
                    goal_handle->canceled(result);
                    return;
                }

                if (overlay_reducer_->shouldHold()) {
                    std::string hold_error;
                    (void)edge_executor_->requestMode("hold", "loc_red_surface_exec", hold_error);
                    result->success = false;
                    result->terminal_segment_id = segment.id;
                    result->failure_code = "LOC_RED_HOLD";
                    result->failure_reason = "Localization RED during surface execution";
                    goal_handle->abort(result);
                    diagnostics_->publishDiagnostic("WARN", "Hold: localization RED during surface execution");
                    return;
                }

                const auto current_overlay_snapshot = overlay_reducer_->snapshot();
                if (current_overlay_snapshot.version != planned_overlay_version) {
                    if (goal->allow_replan && plan_attempt < MAX_REPLAN_ATTEMPTS) {
                        Pose3 next_start_pose;
                        if (!currentRobotPose(next_start_pose)) {
                            const auto fallback_it = surface_graph_.nodes.find(segment.from_node_id);
                            next_start_pose = fallback_it != surface_graph_.nodes.end()
                                ? fallback_it->second.pose
                                : surface_plan.projected_start.pose;
                        }
                        requested_start_pose = next_start_pose;
                        replan_count++;
                        result->terminal_segment_id = segment.id;
                        route_completed = false;
                        diagnostics_->publishActiveLabel(segment.id, "REPLAN");
                        diagnostics_->publishDiagnostic(
                            "WARN",
                            current_overlay_snapshot.active_dynamic_sources.empty()
                                ? "Surface route replanning due to overlay update"
                                : "Surface route replanning due to dynamic overlay update");
                        break;
                    }

                    const bool dynamic_block = remainingSegmentsBlockedByDynamicOverlay(
                        current_overlay_snapshot,
                        surface_graph_,
                        surface_plan.segments,
                        segment_index);
                    result->success = false;
                    result->terminal_segment_id = segment.id;
                    result->failure_code = dynamic_block
                        ? "SURFACE_PATH_BLOCKED_BY_DYNAMIC_OVERLAY"
                        : "SURFACE_PATH_BLOCKED_BY_RUNTIME_OVERLAY";
                    result->failure_reason = dynamic_block
                        ? "Dynamic surface overlay updated before segment dispatch (" +
                            dynamicOverlayReason(current_overlay_snapshot) + ")"
                        : "Runtime overlay updated before segment dispatch";
                    diagnostics_->publishActiveLabel(segment.id, "ABORT");
                    goal_handle->abort(result);
                    diagnostics_->publishDiagnostic("ERROR", "Surface route blocked before segment dispatch");
                    return;
                }

                diagnostics_->publishActiveLabel(segment.id, "PASS");
                diagnostics_->publishCorridor(segment.corridor);
                edge_executor_->setProgressCallback(
                    [goal_handle, feedback, &segment, &replan_count](
                        const EdgeExecutor::ExecProgress& progress) {
                        feedback->active_segment_id =
                            progress.edge_id.empty() ? segment.id : progress.edge_id;
                        feedback->exec_state = execStateOrDefault(progress.exec_state);
                        feedback->replan_count = replan_count;
                        goal_handle->publish_feedback(feedback);
                    });
                ScopeExit clear_progress([this]() { edge_executor_->setProgressCallback({}); });

                const auto exec_result = edge_executor_->executeCorridor(
                    segment.id,
                    segment.from_node_id,
                    segment.to_node_id,
                    segment.motion_type,
                    segment.required_mode,
                    segment.corridor);
                if (exec_result.success) {
                    continue;
                }

                if (!exec_result.failure_code.empty()) {
                    last_replan_failure_code = exec_result.failure_code;
                }
                if (!exec_result.failure_reason.empty()) {
                    last_replan_failure_reason = exec_result.failure_reason;
                }

                if (exec_result.final_state == EdgeExecState::REPLAN_REQUESTED &&
                    goal->allow_replan && plan_attempt < MAX_REPLAN_ATTEMPTS) {
                    Pose3 next_start_pose;
                    if (!currentRobotPose(next_start_pose)) {
                        const auto fallback_it = surface_graph_.nodes.find(segment.from_node_id);
                        next_start_pose = fallback_it != surface_graph_.nodes.end()
                            ? fallback_it->second.pose
                            : surface_plan.projected_start.pose;
                    }
                    requested_start_pose = next_start_pose;
                    replan_count++;
                    result->terminal_segment_id = segment.id;
                    route_completed = false;
                    diagnostics_->publishActiveLabel(segment.id, "REPLAN");
                    diagnostics_->publishDiagnostic("WARN", "Surface route replanning requested");
                    break;
                }

                result->success = false;
                result->terminal_segment_id = segment.id;
                result->failure_code = exec_result.failure_code.empty()
                    ? (exec_result.final_state == EdgeExecState::REPLAN_REQUESTED
                           ? "SEGMENT_REPLAN_REQUESTED"
                           : "SEGMENT_EXEC_FAILED")
                    : exec_result.failure_code;
                result->failure_reason = exec_result.failure_reason;
                diagnostics_->publishActiveLabel(segment.id, "ABORT");
                goal_handle->abort(result);
                diagnostics_->publishDiagnostic("ERROR", "Surface route execution failed");
                return;
            }

            if (route_completed) {
                result->success = true;
                if (!surface_plan.segments.empty()) {
                    result->terminal_segment_id = surface_plan.segments.back().id;
                }
                goal_handle->succeed(result);
                diagnostics_->publishDiagnostic("OK", "Surface navigation succeeded");
                return;
            }
        }

        result->success = false;
        result->failure_code = last_replan_failure_code.empty()
            ? "MAX_REPLAN_EXCEEDED"
            : last_replan_failure_code;
        result->failure_reason = last_replan_failure_reason.empty()
            ? "Exceeded maximum surface replan attempts"
            : last_replan_failure_reason;
        goal_handle->abort(result);
        diagnostics_->publishDiagnostic("ERROR", "Max surface replan exceeded");
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

}  // namespace rc26_xhu_nav::topology
