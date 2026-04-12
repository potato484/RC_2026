#include "rc26_xhu_nav/topology/surface_route.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace rc26_xhu_nav::topology {

namespace {

struct SurfaceLayerAttempt {
    SurfaceProjectionResult projected_start;
    SurfaceProjectionResult projected_goal;
    PlanResult plan;
    bool success = false;
};

geometry_msgs::msg::PoseStamped makePoseStamped(
    const Pose3& pose,
    const std::string& frame_id,
    const rclcpp::Time& stamp) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    msg.pose.position.x = pose.x;
    msg.pose.position.y = pose.y;
    msg.pose.position.z = pose.z;
    msg.pose.orientation.z = std::sin(pose.yaw * 0.5);
    msg.pose.orientation.w = std::cos(pose.yaw * 0.5);
    return msg;
}

double normalizeAngleValue(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion) {
    const double siny_cosp =
        2.0 * ((quaternion.w * quaternion.z) + (quaternion.x * quaternion.y));
    const double cosy_cosp =
        1.0 - 2.0 * ((quaternion.y * quaternion.y) + (quaternion.z * quaternion.z));
    return std::atan2(siny_cosp, cosy_cosp);
}

void appendPathPose(
    nav_msgs::msg::Path& path,
    const Pose3& pose,
    const std::string& frame_id,
    const rclcpp::Time& stamp) {
    path.poses.push_back(makePoseStamped(pose, frame_id, stamp));
}

double elapsedMilliseconds(const std::chrono::steady_clock::time_point& begin) {
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - begin).count();
}

bool nodeAvailableForProjection(
    const std::string& node_id,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays) {
    const auto overlay_it = node_overlays.find(node_id);
    return overlay_it == node_overlays.end() || overlay_it->second.state != NodeState::BLOCKED;
}

std::string withPlannerDetail(const std::string& prefix, const std::string& detail) {
    if (detail.empty()) {
        return prefix;
    }
    return prefix + " (" + detail + ")";
}

SurfaceProjectionResult bestProjectionCandidate(
    const SurfaceProjectionResult& final_projection,
    const SurfaceProjectionResult& runtime_projection,
    const SurfaceProjectionResult& base_projection) {
    if (final_projection.success) {
        return final_projection;
    }
    if (runtime_projection.success) {
        return runtime_projection;
    }
    return base_projection;
}

SurfaceLayerAttempt planSurfaceLayer(
    const FieldGraph& graph,
    const Pose3& requested_start,
    const Pose3& requested_goal,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights,
    const double projection_radius_m) {
    SurfaceLayerAttempt attempt;
    attempt.projected_start = projectPoseToSurfaceGraph(
        graph, requested_start, projection_radius_m, node_overlays);
    if (!attempt.projected_start.success) {
        return attempt;
    }

    attempt.projected_goal = projectPoseToSurfaceGraph(
        graph, requested_goal, projection_radius_m, node_overlays);
    if (!attempt.projected_goal.success) {
        return attempt;
    }

    PlannerRunOptions run_options;
    run_options.heuristic_scale = estimateAdmissibleHeuristicScale(graph, weights);
    attempt.plan = planRoute(
        graph,
        attempt.projected_start.node_id,
        attempt.projected_goal.node_id,
        node_overlays,
        edge_overlays,
        weights,
        run_options);
    attempt.success = attempt.plan.success;
    return attempt;
}

const char* plannerBackendName(const SurfacePlannerBackend backend) {
    switch (backend) {
        case SurfacePlannerBackend::BODY_PLANNER:
            return "body_planner";
        case SurfacePlannerBackend::LEGACY:
        default:
            return "legacy";
    }
}

rc26_xhu_nav::body_planner::SurfaceGraph toBodyPlannerGraph(const FieldGraph& graph) {
    rc26_xhu_nav::body_planner::SurfaceGraph planner_graph;
    planner_graph.team = graph.team;
    planner_graph.schema_version = graph.schema_version;
    for (const auto& [node_id, node] : graph.nodes) {
        planner_graph.nodes[node_id] = rc26_xhu_nav::body_planner::SurfaceNode{
            node_id,
            {node.pose.x, node.pose.y, node.pose.z, node.pose.yaw},
            node.center_clearance_m,
            node.surface_pitch_deg,
        };
    }
    planner_graph.edges.reserve(graph.edges.size());
    for (const auto& edge : graph.edges) {
        planner_graph.edges.push_back(rc26_xhu_nav::body_planner::SurfaceEdge{
            edge.id,
            edge.from,
            edge.to,
            edge.motion_type,
            edge.required_mode,
            edge.base_cost,
            edge.height_change,
            edge.horizontal_length_m,
            edge.slope_deg,
            edge.center_clearance_m,
            edge.nominal_yaw,
            edge.same_surface,
        });
    }
    for (const auto& [node_id, adjacency] : graph.adjacency) {
        planner_graph.adjacency[node_id] = adjacency;
    }
    return planner_graph;
}

std::unordered_map<std::string, rc26_xhu_nav::body_planner::NodeOverlay> toBodyPlannerNodeOverlays(
    const std::unordered_map<std::string, NodeOverlay>& node_overlays) {
    std::unordered_map<std::string, rc26_xhu_nav::body_planner::NodeOverlay> planner_overlays;
    planner_overlays.reserve(node_overlays.size());
    for (const auto& [node_id, overlay] : node_overlays) {
        planner_overlays[node_id] = rc26_xhu_nav::body_planner::NodeOverlay{
            overlay.state == NodeState::BLOCKED,
            overlay.extra_cost,
        };
    }
    return planner_overlays;
}

std::unordered_map<std::string, rc26_xhu_nav::body_planner::EdgeOverlay> toBodyPlannerEdgeOverlays(
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays) {
    std::unordered_map<std::string, rc26_xhu_nav::body_planner::EdgeOverlay> planner_overlays;
    planner_overlays.reserve(edge_overlays.size());
    for (const auto& [edge_id, overlay] : edge_overlays) {
        planner_overlays[edge_id] = rc26_xhu_nav::body_planner::EdgeOverlay{
            overlay.state == EdgeState::BLOCKED,
            overlay.extra_cost,
        };
    }
    return planner_overlays;
}

rc26_xhu_nav::body_planner::PlannerWeights toBodyPlannerWeights(const PlannerWeights& weights) {
    rc26_xhu_nav::body_planner::PlannerWeights planner_weights;
    planner_weights.time = weights.time;
    planner_weights.height_risk = weights.height_risk;
    planner_weights.drop_risk = weights.drop_risk;
    planner_weights.localization_risk = weights.localization_risk;
    planner_weights.dynamic_block = weights.dynamic_block;
    planner_weights.confirm_required = weights.confirm_required;
    planner_weights.slow_only = weights.slow_only;
    return planner_weights;
}

PlanResult convertBodyPlannerResult(
    const FieldGraph& graph,
    const rc26_xhu_nav::body_planner::PlanResult& body_result,
    std::string& error) {
    PlanResult result;
    result.success = body_result.success;
    result.node_path = body_result.node_path;
    result.total_cost = body_result.total_cost;
    result.failure_reason = body_result.failure_reason;

    if (!body_result.success) {
        return result;
    }

    std::unordered_map<std::string, std::size_t> edge_index_by_id;
    edge_index_by_id.reserve(graph.edges.size());
    for (std::size_t index = 0; index < graph.edges.size(); ++index) {
        edge_index_by_id[graph.edges[index].id] = index;
    }

    result.edge_indices.reserve(body_result.edge_path.size());
    for (const auto& edge_id : body_result.edge_path) {
        const auto edge_it = edge_index_by_id.find(edge_id);
        if (edge_it == edge_index_by_id.end()) {
            error = "Body planner returned unknown edge id '" + edge_id + "'";
            result.success = false;
            result.edge_indices.clear();
            result.failure_reason = error;
            return result;
        }
        result.edge_indices.push_back(edge_it->second);
    }

    return result;
}

}  // namespace

Pose3 pose3FromPoseStamped(const geometry_msgs::msg::PoseStamped& msg) {
    Pose3 pose;
    pose.x = msg.pose.position.x;
    pose.y = msg.pose.position.y;
    pose.z = msg.pose.position.z;
    pose.yaw = yawFromQuaternion(msg.pose.orientation);
    return pose;
}

geometry_msgs::msg::PoseStamped poseStampedFromPose3(
    const Pose3& pose,
    const std::string& frame_id,
    const rclcpp::Time& stamp) {
    return makePoseStamped(pose, frame_id, stamp);
}

bool poseNear(
    const Pose3& actual,
    const Pose3& expected,
    const double max_xy_distance,
    const double max_z_distance,
    const double max_yaw_delta_rad) {
    const double dx = actual.x - expected.x;
    const double dy = actual.y - expected.y;
    const double xy_distance = std::hypot(dx, dy);
    const double z_distance = std::abs(actual.z - expected.z);
    const double yaw_delta = std::abs(normalizeAngleValue(actual.yaw - expected.yaw));
    return xy_distance <= max_xy_distance &&
           z_distance <= max_z_distance &&
           yaw_delta <= max_yaw_delta_rad;
}

SurfaceProjectionResult projectPoseToSurfaceGraph(
    const FieldGraph& graph,
    const Pose3& pose,
    const double max_xy_distance) {
    static const std::unordered_map<std::string, NodeOverlay> kEmptyNodeOverlays;
    return projectPoseToSurfaceGraph(graph, pose, max_xy_distance, kEmptyNodeOverlays);
}

SurfaceProjectionResult projectPoseToSurfaceGraph(
    const FieldGraph& graph,
    const Pose3& pose,
    const double max_xy_distance,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays) {
    SurfaceProjectionResult result;
    double best_distance_xy = std::numeric_limits<double>::infinity();
    double best_distance_3d = std::numeric_limits<double>::infinity();

    for (const auto& [node_id, node] : graph.nodes) {
        if (!nodeAvailableForProjection(node_id, node_overlays)) {
            continue;
        }
        const double dx = node.pose.x - pose.x;
        const double dy = node.pose.y - pose.y;
        const double dz = node.pose.z - pose.z;
        const double distance_xy = std::hypot(dx, dy);
        if (max_xy_distance > 0.0 && distance_xy > max_xy_distance) {
            continue;
        }
        const double distance_3d = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (distance_3d < best_distance_3d ||
            (distance_3d == best_distance_3d && distance_xy < best_distance_xy)) {
            result.success = true;
            result.node_id = node_id;
            result.pose = node.pose;
            result.distance_xy = distance_xy;
            result.distance_3d = distance_3d;
            best_distance_xy = distance_xy;
            best_distance_3d = distance_3d;
        }
    }

    if (!result.success) {
        result.failure_reason = "No traversable surface node near the requested point";
    }
    return result;
}

nav_msgs::msg::Path buildExpandedPath(
    const FieldGraph& graph,
    const std::vector<size_t>& edge_indices,
    const std::vector<std::string>& node_path,
    const rclcpp::Time& stamp,
    const std::string& frame_id) {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_id;
    path.header.stamp = stamp;

    if (!edge_indices.empty()) {
        bool first_point = true;
        for (const size_t edge_index : edge_indices) {
            if (edge_index >= graph.edges.size()) {
                continue;
            }
            const auto& edge = graph.edges[edge_index];
            const auto from_it = graph.nodes.find(edge.from);
            const auto to_it = graph.nodes.find(edge.to);
            if (from_it == graph.nodes.end() || to_it == graph.nodes.end()) {
                continue;
            }
            if (first_point) {
                appendPathPose(path, from_it->second.pose, frame_id, stamp);
                first_point = false;
            }
            for (const auto& control_point : edge.control_points) {
                appendPathPose(path, control_point, frame_id, stamp);
            }
            appendPathPose(path, to_it->second.pose, frame_id, stamp);
        }
        if (!path.poses.empty()) {
            return path;
        }
    }

    for (const auto& node_id : node_path) {
        const auto it = graph.nodes.find(node_id);
        if (it == graph.nodes.end()) {
            continue;
        }
        appendPathPose(path, it->second.pose, frame_id, stamp);
    }
    return path;
}

std::vector<SurfacePlanSegment> buildSurfacePlanSegments(
    const FieldGraph& graph,
    const PlanResult& plan,
    const rclcpp::Time& stamp,
    const std::string& frame_id) {
    std::vector<SurfacePlanSegment> segments;
    if (!plan.success || plan.edge_indices.empty()) {
        return segments;
    }

    size_t start_index = 0;
    while (start_index < plan.edge_indices.size()) {
        const auto& first_edge = graph.edges[plan.edge_indices[start_index]];
        size_t end_index = start_index + 1;
        while (end_index < plan.edge_indices.size()) {
            const auto& candidate = graph.edges[plan.edge_indices[end_index]];
            if (candidate.required_mode != first_edge.required_mode ||
                candidate.motion_type != first_edge.motion_type) {
                break;
            }
            ++end_index;
        }

        SurfacePlanSegment segment;
        segment.id = "surface_segment_" + std::to_string(segments.size() + 1);
        segment.from_node_id = first_edge.from;
        segment.to_node_id = graph.edges[plan.edge_indices[end_index - 1]].to;
        segment.motion_type = first_edge.motion_type;
        segment.required_mode = first_edge.required_mode;
        segment.edge_indices.assign(
            plan.edge_indices.begin() + static_cast<std::ptrdiff_t>(start_index),
            plan.edge_indices.begin() + static_cast<std::ptrdiff_t>(end_index));
        segment.corridor = buildExpandedPath(graph, segment.edge_indices, {}, stamp, frame_id);
        segments.push_back(std::move(segment));
        start_index = end_index;
    }

    return segments;
}

SurfacePlanResult planSurfaceRoute(
    const FieldGraph& graph,
    const Pose3& requested_start,
    const Pose3& requested_goal,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights,
    const double projection_radius_m,
    const rclcpp::Time& stamp,
    const SurfacePlannerOptions& options,
    const std::string& frame_id) {
    SurfacePlanResult result;
    result.planner_backend = plannerBackendName(options.backend);
    const auto complete_begin = std::chrono::steady_clock::now();
    const auto projection_begin = std::chrono::steady_clock::now();
    result.projected_start = projectPoseToSurfaceGraph(
        graph, requested_start, projection_radius_m, node_overlays);
    if (!result.projected_start.success) {
        result.projection_ms = elapsedMilliseconds(projection_begin);
        result.complete_planning_ms = elapsedMilliseconds(complete_begin);
        result.failure_code = "POINT_NOT_TRAVERSABLE";
        result.failure_reason = "Start point is not on a traversable surface";
        return result;
    }

    result.projected_goal = projectPoseToSurfaceGraph(
        graph, requested_goal, projection_radius_m, node_overlays);
    result.projection_ms = elapsedMilliseconds(projection_begin);
    if (!result.projected_goal.success) {
        result.complete_planning_ms = elapsedMilliseconds(complete_begin);
        result.failure_code = "POINT_NOT_TRAVERSABLE";
        result.failure_reason = "Goal point is not on a traversable surface";
        return result;
    }

    const auto planning_begin = std::chrono::steady_clock::now();
    if (options.backend == SurfacePlannerBackend::BODY_PLANNER) {
        if (!options.geometry.has_value()) {
            result.complete_planning_ms = elapsedMilliseconds(complete_begin);
            result.failure_code = "NO_SURFACE_PATH";
            result.failure_reason = "Body planner backend requires robot geometry";
            return result;
        }

        rc26_xhu_nav::body_planner::PlanRequest request;
        request.start_node_id = result.projected_start.node_id;
        request.goal_node_id = result.projected_goal.node_id;
        request.start_yaw = requested_start.yaw;
        request.goal_yaw = requested_goal.yaw;

        rc26_xhu_nav::body_planner::RobotGeometry geometry;
        geometry.half_length_m = options.geometry->half_length_m;
        geometry.half_width_m = options.geometry->half_width_m;

        const auto body_result = rc26_xhu_nav::body_planner::planRoute(
            toBodyPlannerGraph(graph),
            request,
            toBodyPlannerNodeOverlays(node_overlays),
            toBodyPlannerEdgeOverlays(edge_overlays),
            toBodyPlannerWeights(weights),
            geometry,
            options.body_planner);
        result.route_planning_ms = elapsedMilliseconds(planning_begin);
        result.heading_path = body_result.heading_path;

        std::string conversion_error;
        result.plan = convertBodyPlannerResult(graph, body_result, conversion_error);
        if (!conversion_error.empty()) {
            result.plan.success = false;
            result.plan.failure_reason = conversion_error;
        }
    } else {
        PlannerRunOptions run_options;
        run_options.heuristic_scale = estimateAdmissibleHeuristicScale(graph, weights);
        result.plan = planRoute(
            graph,
            result.projected_start.node_id,
            result.projected_goal.node_id,
            node_overlays,
            edge_overlays,
            weights,
            run_options);
        result.route_planning_ms = elapsedMilliseconds(planning_begin);
    }

    if (!result.plan.success) {
        result.complete_planning_ms = elapsedMilliseconds(complete_begin);
        result.failure_code = "NO_SURFACE_PATH";
        result.failure_reason = result.plan.failure_reason.empty()
            ? "Planner could not find a traversable surface path"
            : result.plan.failure_reason;
        return result;
    }

    const auto path_expand_begin = std::chrono::steady_clock::now();
    result.planned_path = buildExpandedPath(
        graph,
        result.plan.edge_indices,
        result.plan.node_path,
        stamp,
        frame_id);
    result.path_expand_ms = elapsedMilliseconds(path_expand_begin);
    const auto segment_build_begin = std::chrono::steady_clock::now();
    result.segments = buildSurfacePlanSegments(graph, result.plan, stamp, frame_id);
    result.segment_build_ms = elapsedMilliseconds(segment_build_begin);
    result.complete_planning_ms = elapsedMilliseconds(complete_begin);
    result.success = true;
    return result;
}

SurfacePlanResult planSurfaceRoute(
    const FieldGraph& graph,
    const Pose3& requested_start,
    const Pose3& requested_goal,
    const PlannerWeights& weights,
    const double projection_radius_m,
    const rclcpp::Time& stamp,
    const SurfacePlannerOptions& options,
    const std::string& frame_id) {
    static const std::unordered_map<std::string, NodeOverlay> kEmptyNodeOverlays;
    static const std::unordered_map<std::string, EdgeOverlay> kEmptyEdgeOverlays;
    return planSurfaceRoute(
        graph,
        requested_start,
        requested_goal,
        kEmptyNodeOverlays,
        kEmptyEdgeOverlays,
        weights,
        projection_radius_m,
        stamp,
        options,
        frame_id);
}

SurfacePlanResult planSurfaceRoute(
    const FieldGraph& graph,
    const Pose3& requested_start,
    const Pose3& requested_goal,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights,
    const double projection_radius_m,
    const rclcpp::Time& stamp,
    const std::string& frame_id) {
    SurfacePlannerOptions options;
    options.backend = SurfacePlannerBackend::LEGACY;
    return planSurfaceRoute(
        graph,
        requested_start,
        requested_goal,
        node_overlays,
        edge_overlays,
        weights,
        projection_radius_m,
        stamp,
        options,
        frame_id);
}

SurfacePlanResult planSurfaceRoute(
    const FieldGraph& graph,
    const Pose3& requested_start,
    const Pose3& requested_goal,
    const PlannerWeights& weights,
    const double projection_radius_m,
    const rclcpp::Time& stamp,
    const std::string& frame_id) {
    SurfacePlannerOptions options;
    options.backend = SurfacePlannerBackend::LEGACY;
    return planSurfaceRoute(
        graph,
        requested_start,
        requested_goal,
        weights,
        projection_radius_m,
        stamp,
        options,
        frame_id);
}

SurfaceFailureAnalysis classifySurfacePlanFailure(
    const FieldGraph& graph,
    const Pose3& requested_start,
    const Pose3& requested_goal,
    const std::unordered_map<std::string, NodeOverlay>& static_node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& static_edge_overlays,
    const std::unordered_map<std::string, NodeOverlay>& runtime_node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& runtime_edge_overlays,
    const std::unordered_map<std::string, NodeOverlay>& final_node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& final_edge_overlays,
    const PlannerWeights& weights,
    const double projection_radius_m,
    const SurfacePlanResult& final_plan,
    const std::string& dynamic_overlay_reason) {
    static const std::unordered_map<std::string, NodeOverlay> kEmptyNodeOverlays;
    static const std::unordered_map<std::string, EdgeOverlay> kEmptyEdgeOverlays;

    SurfaceFailureAnalysis analysis;
    const auto base_attempt = planSurfaceLayer(
        graph,
        requested_start,
        requested_goal,
        kEmptyNodeOverlays,
        kEmptyEdgeOverlays,
        weights,
        projection_radius_m);
    const auto static_attempt = planSurfaceLayer(
        graph,
        requested_start,
        requested_goal,
        static_node_overlays,
        static_edge_overlays,
        weights,
        projection_radius_m);
    const auto runtime_attempt = planSurfaceLayer(
        graph,
        requested_start,
        requested_goal,
        runtime_node_overlays,
        runtime_edge_overlays,
        weights,
        projection_radius_m);
    const auto final_attempt = planSurfaceLayer(
        graph,
        requested_start,
        requested_goal,
        final_node_overlays,
        final_edge_overlays,
        weights,
        projection_radius_m);

    analysis.best_projected_start = bestProjectionCandidate(
        final_attempt.projected_start,
        runtime_attempt.projected_start,
        base_attempt.projected_start);
    analysis.best_projected_goal = bestProjectionCandidate(
        final_attempt.projected_goal,
        runtime_attempt.projected_goal,
        base_attempt.projected_goal);

    if (!base_attempt.projected_start.success) {
        analysis.failure_code = "START_POINT_NOT_PROJECTABLE";
        analysis.failure_reason = "Start point is outside the sampled traversable surface graph";
        return analysis;
    }
    if (!static_attempt.projected_start.success) {
        analysis.failure_code = "START_POINT_BLOCKED_BY_OVERLAY";
        analysis.failure_reason = "Start point only projects to nodes blocked by runtime overlays";
        return analysis;
    }
    if (!runtime_attempt.projected_start.success) {
        analysis.failure_code = "START_POINT_BLOCKED_BY_OVERLAY";
        analysis.failure_reason = withPlannerDetail(
            "Start point only projects to nodes blocked by runtime overlays",
            dynamic_overlay_reason);
        return analysis;
    }
    if (!final_attempt.projected_start.success) {
        analysis.failure_code = "START_POINT_BLOCKED_BY_BODY_CONSTRAINT";
        analysis.failure_reason = "Start point only projects to nodes rejected by body-aware constraints";
        return analysis;
    }

    if (!base_attempt.projected_goal.success) {
        analysis.failure_code = "GOAL_POINT_NOT_PROJECTABLE";
        analysis.failure_reason = "Goal point is outside the sampled traversable surface graph";
        return analysis;
    }
    if (!static_attempt.projected_goal.success) {
        analysis.failure_code = "GOAL_POINT_BLOCKED_BY_OVERLAY";
        analysis.failure_reason = "Goal point only projects to nodes blocked by runtime overlays";
        return analysis;
    }
    if (!runtime_attempt.projected_goal.success) {
        analysis.failure_code = "GOAL_POINT_BLOCKED_BY_OVERLAY";
        analysis.failure_reason = withPlannerDetail(
            "Goal point only projects to nodes blocked by runtime overlays",
            dynamic_overlay_reason);
        return analysis;
    }
    if (!final_attempt.projected_goal.success) {
        analysis.failure_code = "GOAL_POINT_BLOCKED_BY_BODY_CONSTRAINT";
        analysis.failure_reason = "Goal point only projects to nodes rejected by body-aware constraints";
        return analysis;
    }

    if (!base_attempt.plan.success) {
        analysis.failure_code = "SURFACE_GRAPH_DISCONNECTED";
        analysis.failure_reason = withPlannerDetail(
            "The surface graph has no connected route between the projected start and goal",
            base_attempt.plan.failure_reason);
        return analysis;
    }

    if (!static_attempt.plan.success) {
        analysis.failure_code = "SURFACE_PATH_BLOCKED_BY_RUNTIME_OVERLAY";
        analysis.failure_reason = withPlannerDetail(
            "Runtime overlays removed every traversable route between the projected start and goal",
            static_attempt.plan.failure_reason);
        return analysis;
    }

    if (!runtime_attempt.plan.success) {
        analysis.failure_code = "SURFACE_PATH_BLOCKED_BY_DYNAMIC_OVERLAY";
        analysis.failure_reason = withPlannerDetail(
            "Dynamic surface overlays removed every traversable route between the projected start and goal",
            dynamic_overlay_reason.empty() ? runtime_attempt.plan.failure_reason : dynamic_overlay_reason);
        return analysis;
    }

    analysis.failure_code = "BODY_CONSTRAINT_UNSATISFIED";
    analysis.failure_reason = withPlannerDetail(
        "Body-aware constraints removed every traversable route between the projected start and goal",
        final_plan.plan.failure_reason.empty() ? final_attempt.plan.failure_reason : final_plan.plan.failure_reason);
    return analysis;
}

}  // namespace rc26_xhu_nav::topology
