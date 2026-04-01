#pragma once

#include "rc26_topo_nav/types.hpp"
#include <string>
#include <unordered_map>
#include <vector>

namespace rc26_topo_nav {

struct PlannerWeights {
    double time = 1.0;
    double height_risk = 2.0;
    double drop_risk = 3.0;
    double localization_risk = 2.0;
    double dynamic_block = 1000.0;
    double confirm_required = 1.5;
    double slow_only = 2.0;
};

struct PlanResult {
    bool success = false;
    std::vector<std::string> node_path;    // ordered node IDs
    std::vector<size_t> edge_indices;      // ordered edge indices into graph.edges
    double total_cost = 0;
    std::string failure_reason;
};

PlanResult planRoute(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& goal_node,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights);

PlanResult planToTask(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& task_tag,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights);

PlanResult planRouteTag(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& route_tag,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights);

}  // namespace rc26_topo_nav
