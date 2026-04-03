#include "rc26_topo_nav/surface_route.hpp"

#include <cmath>
#include <limits>

namespace rc26_topo_nav {

namespace {

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

double normalizeAngle(double angle) {
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
    const double yaw_delta = std::abs(normalizeAngle(actual.yaw - expected.yaw));
    return xy_distance <= max_xy_distance &&
           z_distance <= max_z_distance &&
           yaw_delta <= max_yaw_delta_rad;
}

SurfaceProjectionResult projectPoseToSurfaceGraph(
    const FieldGraph& graph,
    const Pose3& pose,
    const double max_xy_distance) {
    SurfaceProjectionResult result;
    double best_distance_xy = std::numeric_limits<double>::infinity();
    double best_distance_3d = std::numeric_limits<double>::infinity();

    for (const auto& [node_id, node] : graph.nodes) {
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
    const PlannerWeights& weights,
    const double projection_radius_m,
    const rclcpp::Time& stamp,
    const std::string& frame_id) {
    SurfacePlanResult result;
    result.projected_start = projectPoseToSurfaceGraph(graph, requested_start, projection_radius_m);
    if (!result.projected_start.success) {
        result.failure_code = "POINT_NOT_TRAVERSABLE";
        result.failure_reason = "Start point is not on a traversable surface";
        return result;
    }

    result.projected_goal = projectPoseToSurfaceGraph(graph, requested_goal, projection_radius_m);
    if (!result.projected_goal.success) {
        result.failure_code = "POINT_NOT_TRAVERSABLE";
        result.failure_reason = "Goal point is not on a traversable surface";
        return result;
    }

    const std::unordered_map<std::string, NodeOverlay> node_overlays;
    const std::unordered_map<std::string, EdgeOverlay> edge_overlays;
    result.plan = planRoute(
        graph,
        result.projected_start.node_id,
        result.projected_goal.node_id,
        node_overlays,
        edge_overlays,
        weights);

    if (!result.plan.success) {
        result.failure_code = "NO_SURFACE_PATH";
        result.failure_reason = result.plan.failure_reason.empty()
            ? "Planner could not find a traversable surface path"
            : result.plan.failure_reason;
        return result;
    }

    result.planned_path = buildExpandedPath(
        graph,
        result.plan.edge_indices,
        result.plan.node_path,
        stamp,
        frame_id);
    result.segments = buildSurfacePlanSegments(graph, result.plan, stamp, frame_id);
    result.success = true;
    return result;
}

}  // namespace rc26_topo_nav
