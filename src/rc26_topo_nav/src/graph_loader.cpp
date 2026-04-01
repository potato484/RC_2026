#include "rc26_topo_nav/graph_loader.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <set>
#include <unordered_set>

namespace rc26_topo_nav {

static Pose3 parsePose(const YAML::Node& n) {
    Pose3 p;
    p.x = n["x"].as<double>();
    p.y = n["y"].as<double>();
    p.z = n["z"] ? n["z"].as<double>() : 0.0;
    p.yaw = n["yaw"] ? n["yaw"].as<double>() : 0.0;
    return p;
}

static std::vector<Pose3> parseControlPoints(const YAML::Node& n) {
    std::vector<Pose3> points;
    if (!n || !n.IsSequence()) {
        return points;
    }
    points.reserve(n.size());
    for (const auto& item : n) {
        points.push_back(parsePose(item));
    }
    return points;
}

LoadResult loadFieldGraph(const std::string& yaml_path) {
    LoadResult result;
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& e) {
        result.error = "YAML parse error: " + std::string(e.what());
        return result;
    }

    auto& g = result.graph;

    // meta
    if (auto meta = root["meta"]) {
        g.team = meta["team"].as<std::string>("");
        g.schema_version = meta["schema_version"].as<std::string>("");
        g.grid_spacing_m = meta["grid_spacing_m"].as<double>(1.2);
    }

    // nodes
    if (auto nodes = root["nodes"]) {
        for (const auto& n : nodes) {
            GraphNode node;
            node.id = n["id"].as<std::string>();
            node.type = n["type"].as<std::string>();
            node.pose = parsePose(n["pose"]);
            node.level = n["level"].as<int>(0);
            node.phase_mask = static_cast<uint8_t>(n["phase_mask"].as<int>(0xFF));
            node.block_id = n["block_id"].as<int>(0);
            node.base_cost = n["base_cost"].as<double>(0);
            node.operation_tag = n["operation_tag"].as<std::string>("");
            g.nodes[node.id] = node;
        }
    }

    // edges
    if (auto edges = root["edges"]) {
        for (const auto& e : edges) {
            GraphEdge edge;
            edge.id = e["id"].as<std::string>();
            edge.from = e["from"].as<std::string>();
            edge.to = e["to"].as<std::string>();
            edge.motion_type = e["motion_type"].as<std::string>();
            edge.height_change = e["height_change"].as<double>(0);
            edge.required_mode = e["required_mode"].as<std::string>();
            edge.requires_confirmation = e["requires_confirmation"].as<bool>(false);
            edge.can_block = e["can_block"].as<bool>(false);
            edge.phase_mask = static_cast<uint8_t>(e["phase_mask"].as<int>(0xFF));
            edge.base_cost = e["base_cost"].as<double>(0);
            edge.control_points = parseControlPoints(e["control_points"]);
            g.edges.push_back(edge);
        }
    }

    // build adjacency
    for (size_t i = 0; i < g.edges.size(); ++i) {
        g.adjacency[g.edges[i].from].push_back(i);
    }

    // tasks
    if (auto tasks = root["tasks"]) {
        for (const auto& t : tasks) {
            TaskDef task;
            task.task_tag = t["task_tag"].as<std::string>();
            for (const auto& cn : t["candidate_nodes"]) {
                task.candidate_nodes.push_back(cn.as<std::string>());
            }
            task.selection_policy = t["selection_policy"].as<std::string>("min_total_cost");
            g.tasks.push_back(task);
        }
    }

    if (auto routes = root["routes"]) {
        for (const auto& r : routes) {
            RouteDef route;
            route.route_tag = r["route_tag"].as<std::string>();
            if (auto nodes = r["nodes"]) {
                for (const auto& node_id : nodes) {
                    route.nodes.push_back(node_id.as<std::string>());
                }
            }
            g.routes[route.route_tag] = route;
        }
    }

    result.success = true;
    return result;
}

ValidationResult validateGraph(const FieldGraph& graph) {
    ValidationResult vr;

    // check edge endpoints exist
    for (const auto& edge : graph.edges) {
        if (graph.nodes.find(edge.from) == graph.nodes.end()) {
            vr.errors.push_back("Edge '" + edge.id + "' references missing from-node '" + edge.from + "'");
        }
        if (graph.nodes.find(edge.to) == graph.nodes.end()) {
            vr.errors.push_back("Edge '" + edge.id + "' references missing to-node '" + edge.to + "'");
        }
    }

    // check MF edges: only manhattan-adjacent allowed (no diagonal)
    for (const auto& edge : graph.edges) {
        auto from_it = graph.nodes.find(edge.from);
        auto to_it = graph.nodes.find(edge.to);
        if (from_it == graph.nodes.end() || to_it == graph.nodes.end()) continue;

        const auto& fn = from_it->second;
        const auto& tn = to_it->second;

        // only check MF block nodes (block_id > 0)
        if (fn.block_id > 0 && tn.block_id > 0) {
            double dx = std::abs(fn.pose.x - tn.pose.x);
            double dy = std::abs(fn.pose.y - tn.pose.y);
            double spacing = graph.grid_spacing_m;
            double tol = spacing * 0.1;

            bool is_x_adjacent = std::abs(dx - spacing) < tol && dy < tol;
            bool is_y_adjacent = std::abs(dy - spacing) < tol && dx < tol;

            if (!is_x_adjacent && !is_y_adjacent) {
                vr.errors.push_back(
                    "Edge '" + edge.id + "' between MF blocks is diagonal or non-adjacent");
            }
        }
    }

    // check task candidate nodes exist
    for (const auto& task : graph.tasks) {
        if (task.candidate_nodes.empty()) {
            vr.errors.push_back("Task '" + task.task_tag + "' has no candidate nodes");
        }
        for (const auto& cn : task.candidate_nodes) {
            if (graph.nodes.find(cn) == graph.nodes.end()) {
                vr.errors.push_back(
                    "Task '" + task.task_tag + "' references missing node '" + cn + "'");
            }
        }
    }

    for (const auto& [route_tag, route] : graph.routes) {
        if (route.nodes.empty()) {
            vr.errors.push_back("Route '" + route_tag + "' has no nodes");
            continue;
        }

        for (const auto& node_id : route.nodes) {
            if (graph.nodes.find(node_id) == graph.nodes.end()) {
                vr.errors.push_back(
                    "Route '" + route_tag + "' references missing node '" + node_id + "'");
            }
        }

        for (size_t i = 1; i < route.nodes.size(); ++i) {
            const auto& from = route.nodes[i - 1];
            const auto& to = route.nodes[i];
            bool found_direct_edge = false;
            auto adj_it = graph.adjacency.find(from);
            if (adj_it != graph.adjacency.end()) {
                for (size_t edge_index : adj_it->second) {
                    if (graph.edges[edge_index].to == to) {
                        found_direct_edge = true;
                        break;
                    }
                }
            }
            if (!found_direct_edge) {
                vr.errors.push_back(
                    "Route '" + route_tag + "' has no direct edge from '" + from + "' to '" + to + "'");
            }
        }
    }

    vr.valid = vr.errors.empty();
    return vr;
}

ValidationResult validateSymmetry(const FieldGraph& blue, const FieldGraph& red) {
    ValidationResult vr;

    if (blue.nodes.size() != red.nodes.size()) {
        vr.errors.push_back("Node count mismatch: blue=" +
            std::to_string(blue.nodes.size()) + " red=" + std::to_string(red.nodes.size()));
    }
    if (blue.edges.size() != red.edges.size()) {
        vr.errors.push_back("Edge count mismatch: blue=" +
            std::to_string(blue.edges.size()) + " red=" + std::to_string(red.edges.size()));
    }

    // check same node IDs
    std::set<std::string> blue_ids, red_ids;
    for (const auto& [id, _] : blue.nodes) blue_ids.insert(id);
    for (const auto& [id, _] : red.nodes) red_ids.insert(id);

    if (blue_ids != red_ids) {
        vr.errors.push_back("Node ID sets differ between blue and red graphs");
    }

    std::set<std::string> blue_edge_ids, red_edge_ids;
    for (const auto& edge : blue.edges) blue_edge_ids.insert(edge.id);
    for (const auto& edge : red.edges) red_edge_ids.insert(edge.id);
    if (blue_edge_ids != red_edge_ids) {
        vr.errors.push_back("Edge ID sets differ between blue and red graphs");
    }

    std::set<std::string> blue_task_tags, red_task_tags;
    for (const auto& task : blue.tasks) blue_task_tags.insert(task.task_tag);
    for (const auto& task : red.tasks) red_task_tags.insert(task.task_tag);
    if (blue_task_tags != red_task_tags) {
        vr.errors.push_back("Task tag sets differ between blue and red graphs");
    }

    std::set<std::string> blue_route_tags, red_route_tags;
    for (const auto& [route_tag, _] : blue.routes) blue_route_tags.insert(route_tag);
    for (const auto& [route_tag, _] : red.routes) red_route_tags.insert(route_tag);
    if (blue_route_tags != red_route_tags) {
        vr.errors.push_back("Route tag sets differ between blue and red graphs");
    }

    vr.valid = vr.errors.empty();
    return vr;
}

}  // namespace rc26_topo_nav
