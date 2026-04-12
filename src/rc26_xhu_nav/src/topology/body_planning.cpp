#include "rc26_xhu_nav/topology/body_planning.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>

namespace rc26_xhu_nav::topology {

namespace {

double yamlDouble(const YAML::Node& node, const char* key, const double fallback) {
    if (!node || !node[key]) {
        return fallback;
    }
    return node[key].as<double>();
}

std::string yamlString(const YAML::Node& node, const char* key, const std::string& fallback) {
    if (!node || !node[key]) {
        return fallback;
    }
    return node[key].as<std::string>();
}

void setNodeBlocked(std::unordered_map<std::string, NodeOverlay>& node_overlays, const std::string& node_id) {
    auto& overlay = node_overlays[node_id];
    overlay.state = NodeState::BLOCKED;
    overlay.extra_cost = std::max(overlay.extra_cost, 1000.0);
}

void setNodePenalty(
    std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::string& node_id,
    const double extra_cost) {
    if (extra_cost <= 0.0) {
        return;
    }
    auto& overlay = node_overlays[node_id];
    overlay.extra_cost = std::max(overlay.extra_cost, extra_cost);
}

void setEdgeBlocked(std::unordered_map<std::string, EdgeOverlay>& edge_overlays, const std::string& edge_id) {
    auto& overlay = edge_overlays[edge_id];
    overlay.state = EdgeState::BLOCKED;
    overlay.extra_cost = std::max(overlay.extra_cost, 1000.0);
}

bool isSurfaceEdge(const FieldGraph& graph, const GraphEdge& edge) {
    const auto from_it = graph.nodes.find(edge.from);
    const auto to_it = graph.nodes.find(edge.to);
    return from_it != graph.nodes.end() &&
           to_it != graph.nodes.end() &&
           from_it->second.type == "surface_point" &&
           to_it->second.type == "surface_point";
}

}  // namespace

std::optional<RobotGeometryProfile> loadRobotGeometryProfile(
    const std::string& geometry_file,
    const std::string& requested_profile,
    std::string& error) {
    try {
        const YAML::Node root = YAML::LoadFile(geometry_file);
        const YAML::Node geometry_root = root["robot_geometry"] ? root["robot_geometry"] : root;
        const YAML::Node defaults = geometry_root["defaults"];
        const YAML::Node profiles = geometry_root["profiles"];

        std::string profile_name = requested_profile;
        if (profile_name.empty()) {
            profile_name = yamlString(defaults, "active_profile", "");
        }
        if (profile_name.empty()) {
            error = "robot geometry profile is empty";
            return std::nullopt;
        }
        if (!profiles || !profiles[profile_name]) {
            error = "robot geometry profile not found: " + profile_name;
            return std::nullopt;
        }

        const YAML::Node profile_node = profiles[profile_name];
        const YAML::Node body = profile_node["body"];
        const YAML::Node safety = profile_node["safety"];
        const YAML::Node planning = profile_node["planning"];

        RobotGeometryProfile profile;
        profile.name = profile_name;
        profile.half_length_m = std::max(0.0, yamlDouble(body, "half_length_m", 0.0));
        profile.half_width_m = std::max(0.0, yamlDouble(body, "half_width_m", 0.0));
        profile.height_m = std::max(0.0, yamlDouble(body, "height_m", 0.0));
        profile.stop_envelope_half_width_m =
            std::max(profile.half_width_m, yamlDouble(safety, "stop_envelope_half_width_m", 0.0));
        profile.surface_projection_radius_m =
            std::max(0.0, yamlDouble(planning, "surface_projection_radius_m", 0.0));
        return profile;
    } catch (const std::exception& e) {
        error = e.what();
        return std::nullopt;
    }
}

bool graphHasSurfaceBodyAnnotations(const FieldGraph& graph) {
    bool saw_surface_node = false;
    for (const auto& [_, node] : graph.nodes) {
        if (node.type != "surface_point") {
            continue;
        }
        saw_surface_node = true;
        if (node.center_clearance_m < 0.0 || node.surface_pitch_deg < 0.0) {
            return false;
        }
    }

    if (!saw_surface_node) {
        return false;
    }

    for (const auto& edge : graph.edges) {
        if (!isSurfaceEdge(graph, edge)) {
            continue;
        }
        if (edge.center_clearance_m < 0.0 || edge.slope_deg < 0.0 || edge.horizontal_length_m < 0.0) {
            return false;
        }
    }

    return true;
}

SurfaceBodyPlanningStats applySurfaceBodyPlanningOverlays(
    const FieldGraph& graph,
    const RobotGeometryProfile& geometry,
    const SurfaceBodyPlanningConfig& config,
    std::unordered_map<std::string, NodeOverlay>& node_overlays,
    std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    std::string* error) {
    SurfaceBodyPlanningStats stats;
    stats.annotations_available = graphHasSurfaceBodyAnnotations(graph);
    if (!config.enabled) {
        return stats;
    }
    if (config.require_annotated_surface_graph && !stats.annotations_available) {
        if (error != nullptr) {
            *error = "Surface graph missing body-aware annotations";
        }
        return stats;
    }

    const double required_node_clearance =
        geometry.half_width_m + config.clearance_margin_m;
    const double required_edge_clearance =
        geometry.half_width_m + config.clearance_margin_m;
    const double node_penalty_scale = 10.0;

    for (const auto& [node_id, node] : graph.nodes) {
        if (node.type != "surface_point") {
            continue;
        }
        if (node.center_clearance_m >= 0.0 && node.center_clearance_m < required_node_clearance) {
            const double clearance_shortage = required_node_clearance - node.center_clearance_m;
            setNodePenalty(node_overlays, node_id, clearance_shortage * node_penalty_scale);
            ++stats.penalized_nodes_clearance;
        }
        if (node.surface_pitch_deg >= 0.0 && node.surface_pitch_deg > config.max_surface_pitch_deg) {
            setNodeBlocked(node_overlays, node_id);
            ++stats.blocked_nodes_pitch;
        }
    }

    for (const auto& edge : graph.edges) {
        if (!isSurfaceEdge(graph, edge)) {
            continue;
        }
        if (edge.center_clearance_m >= 0.0 && edge.center_clearance_m < required_edge_clearance) {
            setEdgeBlocked(edge_overlays, edge.id);
            ++stats.blocked_edges_clearance;
            continue;
        }
        if (edge.slope_deg >= 0.0 && edge.slope_deg > config.max_edge_slope_deg) {
            setEdgeBlocked(edge_overlays, edge.id);
            ++stats.blocked_edges_slope;
            continue;
        }
        const bool abrupt_transition =
            edge.horizontal_length_m <= std::max(0.12, geometry.half_length_m);
        if (abrupt_transition && std::abs(edge.height_change) > config.max_step_height_m) {
            setEdgeBlocked(edge_overlays, edge.id);
            ++stats.blocked_edges_step;
        }
    }

    return stats;
}

}  // namespace rc26_xhu_nav::topology
