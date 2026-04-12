#pragma once

#include "rc26_xhu_nav/topology/types.hpp"

#include <optional>
#include <string>
#include <unordered_map>

namespace rc26_xhu_nav::topology {

struct RobotGeometryProfile {
    std::string name;
    double half_length_m = 0.0;
    double half_width_m = 0.0;
    double height_m = 0.0;
    double stop_envelope_half_width_m = 0.0;
    double surface_projection_radius_m = 0.0;
};

struct SurfaceBodyPlanningConfig {
    bool enabled = true;
    bool require_annotated_surface_graph = true;
    double clearance_margin_m = 0.02;
    double max_surface_pitch_deg = 35.0;
    double max_edge_slope_deg = 35.0;
    double max_step_height_m = 0.18;
};

struct SurfaceBodyPlanningStats {
    bool annotations_available = false;
    size_t penalized_nodes_clearance = 0;
    size_t blocked_nodes_pitch = 0;
    size_t blocked_edges_clearance = 0;
    size_t blocked_edges_slope = 0;
    size_t blocked_edges_step = 0;
};

std::optional<RobotGeometryProfile> loadRobotGeometryProfile(
    const std::string& geometry_file,
    const std::string& requested_profile,
    std::string& error);

bool graphHasSurfaceBodyAnnotations(const FieldGraph& graph);

SurfaceBodyPlanningStats applySurfaceBodyPlanningOverlays(
    const FieldGraph& graph,
    const RobotGeometryProfile& geometry,
    const SurfaceBodyPlanningConfig& config,
    std::unordered_map<std::string, NodeOverlay>& node_overlays,
    std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    std::string* error = nullptr);

}  // namespace rc26_xhu_nav::topology
