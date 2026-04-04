#pragma once

#include "rc26_surface_body_planner/types.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace rc26_surface_body_planner {

struct PlannerWeights {
    double time = 1.0;
    double height_risk = 2.0;
    double drop_risk = 3.0;
    double localization_risk = 2.0;
    double dynamic_block = 1000.0;
    double confirm_required = 1.5;
    double slow_only = 2.0;
};

struct RobotGeometry {
    double half_length_m = 0.0;
    double half_width_m = 0.0;
};

struct PlannerConfig {
    int heading_bin_count = 16;
    double max_heading_change_deg = 85.0;
    double turn_cost_weight = 0.75;
    double node_turn_clearance_gain = 1.0;
    double edge_turn_clearance_gain = 0.75;
};

struct PlanRequest {
    std::string start_node_id;
    std::string goal_node_id;
    double start_yaw = 0.0;
    bool require_goal_heading = false;
    double goal_yaw = 0.0;
};

struct PlanResult {
    bool success = false;
    std::vector<std::string> node_path;
    std::vector<std::string> edge_path;
    std::vector<double> heading_path;
    double total_cost = 0.0;
    std::string failure_reason;
    std::string blocked_transition_from_node;
    std::string blocked_transition_edge_id;
    double blocked_heading_change_deg = 0.0;
};

double normalizeAngle(double angle);

PlanResult planRoute(
    const SurfaceGraph& graph,
    const PlanRequest& request,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights,
    const RobotGeometry& geometry,
    const PlannerConfig& config);

}  // namespace rc26_surface_body_planner
