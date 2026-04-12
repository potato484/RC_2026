#include "rc26_xhu_nav/body_planner/graph_loader.hpp"

#include <yaml-cpp/yaml.h>

namespace rc26_xhu_nav::body_planner {

namespace {

Pose3 parsePose(const YAML::Node& node) {
    Pose3 pose;
    pose.x = node["x"].as<double>(0.0);
    pose.y = node["y"].as<double>(0.0);
    pose.z = node["z"].as<double>(0.0);
    pose.yaw = node["yaw"].as<double>(0.0);
    return pose;
}

}  // namespace

LoadResult loadSurfaceGraph(const std::string& yaml_path) {
    LoadResult result;
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& e) {
        result.error = e.what();
        return result;
    }

    if (const auto meta = root["meta"]) {
        result.graph.team = meta["team"].as<std::string>("");
        result.graph.schema_version = meta["schema_version"].as<std::string>("");
    }

    if (const auto nodes = root["nodes"]) {
        for (const auto& node_yaml : nodes) {
            const std::string type = node_yaml["type"].as<std::string>("");
            if (type != "surface_point") {
                continue;
            }

            SurfaceNode node;
            node.id = node_yaml["id"].as<std::string>();
            node.pose = parsePose(node_yaml["pose"]);
            node.center_clearance_m = node_yaml["center_clearance_m"].as<double>(-1.0);
            node.surface_pitch_deg = node_yaml["surface_pitch_deg"].as<double>(-1.0);
            result.graph.nodes[node.id] = node;
        }
    }

    if (const auto edges = root["edges"]) {
        for (const auto& edge_yaml : edges) {
            SurfaceEdge edge;
            edge.id = edge_yaml["id"].as<std::string>();
            edge.from = edge_yaml["from"].as<std::string>();
            edge.to = edge_yaml["to"].as<std::string>();
            if (result.graph.nodes.find(edge.from) == result.graph.nodes.end() ||
                result.graph.nodes.find(edge.to) == result.graph.nodes.end()) {
                continue;
            }
            edge.motion_type = edge_yaml["motion_type"].as<std::string>("");
            edge.required_mode = edge_yaml["required_mode"].as<std::string>("");
            edge.base_cost = edge_yaml["base_cost"].as<double>(0.0);
            edge.height_change = edge_yaml["height_change"].as<double>(0.0);
            edge.horizontal_length_m = edge_yaml["horizontal_length_m"].as<double>(0.0);
            edge.slope_deg = edge_yaml["slope_deg"].as<double>(-1.0);
            edge.center_clearance_m = edge_yaml["center_clearance_m"].as<double>(-1.0);
            edge.nominal_yaw = edge_yaml["nominal_yaw"].as<double>(0.0);
            edge.same_surface = edge_yaml["same_surface"].as<bool>(false);
            result.graph.adjacency[edge.from].push_back(result.graph.edges.size());
            result.graph.edges.push_back(edge);
        }
    }

    result.success = true;
    return result;
}

ValidationResult validateSurfaceGraph(const SurfaceGraph& graph) {
    ValidationResult result;
    if (graph.nodes.empty()) {
        result.errors.push_back("surface graph has no surface nodes");
    }

    for (const auto& [node_id, node] : graph.nodes) {
        if (node.center_clearance_m < 0.0) {
            result.errors.push_back("surface node '" + node_id + "' missing center_clearance_m");
        }
        if (node.surface_pitch_deg < 0.0) {
            result.errors.push_back("surface node '" + node_id + "' missing surface_pitch_deg");
        }
    }

    for (const auto& edge : graph.edges) {
        if (graph.nodes.find(edge.from) == graph.nodes.end()) {
            result.errors.push_back("surface edge '" + edge.id + "' missing from node");
        }
        if (graph.nodes.find(edge.to) == graph.nodes.end()) {
            result.errors.push_back("surface edge '" + edge.id + "' missing to node");
        }
        if (edge.center_clearance_m < 0.0) {
            result.errors.push_back("surface edge '" + edge.id + "' missing center_clearance_m");
        }
        if (edge.slope_deg < 0.0) {
            result.errors.push_back("surface edge '" + edge.id + "' missing slope_deg");
        }
        if (edge.horizontal_length_m < 0.0) {
            result.errors.push_back("surface edge '" + edge.id + "' has invalid horizontal_length_m");
        }
    }

    result.valid = result.errors.empty();
    return result;
}

}  // namespace rc26_xhu_nav::body_planner
