#pragma once

#include "rc26_xhu_nav/topology/planner.hpp"
#include "rc26_xhu_nav/body_planner/planner.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace rc26_xhu_nav::topology {

struct SurfaceProjectionResult {
    bool success = false;
    std::string node_id;
    Pose3 pose;
    double distance_xy = 0.0;
    double distance_3d = 0.0;
    std::string failure_reason;
};

struct SurfacePlanSegment {
    std::string id;
    std::string from_node_id;
    std::string to_node_id;
    std::string motion_type;
    std::string required_mode;
    std::vector<size_t> edge_indices;
    nav_msgs::msg::Path corridor;
};

struct SurfacePlanResult {
    bool success = false;
    SurfaceProjectionResult projected_start;
    SurfaceProjectionResult projected_goal;
    PlanResult plan;
    std::vector<double> heading_path;
    nav_msgs::msg::Path planned_path;
    std::vector<SurfacePlanSegment> segments;
    std::string planner_backend;
    std::string failure_code;
    std::string failure_reason;
    double projection_ms = 0.0;
    double route_planning_ms = 0.0;
    double path_expand_ms = 0.0;
    double segment_build_ms = 0.0;
    double complete_planning_ms = 0.0;
};

struct SurfaceFailureAnalysis {
    std::string failure_code;
    std::string failure_reason;
    SurfaceProjectionResult best_projected_start;
    SurfaceProjectionResult best_projected_goal;
};

enum class SurfacePlannerBackend : uint8_t {
    LEGACY = 0,
    BODY_PLANNER = 1,
};

struct SurfacePlannerGeometry {
    double half_length_m = 0.0;
    double half_width_m = 0.0;
};

struct SurfacePlannerOptions {
    SurfacePlannerBackend backend = SurfacePlannerBackend::LEGACY;
    rc26_xhu_nav::body_planner::PlannerConfig body_planner;
    std::optional<SurfacePlannerGeometry> geometry;
};

Pose3 pose3FromPoseStamped(const geometry_msgs::msg::PoseStamped& msg);
geometry_msgs::msg::PoseStamped poseStampedFromPose3(
    const Pose3& pose,
    const std::string& frame_id,
    const rclcpp::Time& stamp);

bool poseNear(
    const Pose3& actual,
    const Pose3& expected,
    double max_xy_distance,
    double max_z_distance,
    double max_yaw_delta_rad);

SurfaceProjectionResult projectPoseToSurfaceGraph(
    const FieldGraph& graph,
    const Pose3& pose,
    double max_xy_distance);

SurfaceProjectionResult projectPoseToSurfaceGraph(
    const FieldGraph& graph,
    const Pose3& pose,
    double max_xy_distance,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays);

nav_msgs::msg::Path buildExpandedPath(
    const FieldGraph& graph,
    const std::vector<size_t>& edge_indices,
    const std::vector<std::string>& node_path,
    const rclcpp::Time& stamp,
    const std::string& frame_id = "map");

std::vector<SurfacePlanSegment> buildSurfacePlanSegments(
    const FieldGraph& graph,
    const PlanResult& plan,
    const rclcpp::Time& stamp,
    const std::string& frame_id = "map");

SurfacePlanResult planSurfaceRoute(
    const FieldGraph& graph,
    const Pose3& requested_start,
    const Pose3& requested_goal,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights,
    double projection_radius_m,
    const rclcpp::Time& stamp,
    const SurfacePlannerOptions& options,
    const std::string& frame_id = "map");

SurfacePlanResult planSurfaceRoute(
    const FieldGraph& graph,
    const Pose3& requested_start,
    const Pose3& requested_goal,
    const PlannerWeights& weights,
    double projection_radius_m,
    const rclcpp::Time& stamp,
    const SurfacePlannerOptions& options,
    const std::string& frame_id = "map");

SurfacePlanResult planSurfaceRoute(
    const FieldGraph& graph,
    const Pose3& requested_start,
    const Pose3& requested_goal,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights,
    double projection_radius_m,
    const rclcpp::Time& stamp,
    const std::string& frame_id = "map");

SurfacePlanResult planSurfaceRoute(
    const FieldGraph& graph,
    const Pose3& requested_start,
    const Pose3& requested_goal,
    const PlannerWeights& weights,
    double projection_radius_m,
    const rclcpp::Time& stamp,
    const std::string& frame_id = "map");

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
    double projection_radius_m,
    const SurfacePlanResult& final_plan,
    const std::string& dynamic_overlay_reason = "");

}  // namespace rc26_xhu_nav::topology
