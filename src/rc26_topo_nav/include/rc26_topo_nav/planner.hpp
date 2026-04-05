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

enum class TraceEventType : uint8_t {
    INIT,
    POP,
    SKIP_STALE,
    BLOCKED_NODE,
    EDGE_BLOCKED,
    RELAX,
    KEEP_BEST,
    GOAL,
    FAILED,
    ROUTE_TAG,
    CANDIDATE_BLOCKED,
    CANDIDATE_SELECTED,
};

struct PlannerRunOptions {
    double heuristic_scale = 0.0;
};

struct PlannerTraceOptions {
    double heuristic_scale = 0.0;
    bool capture_frontier = true;
    bool capture_paths = true;
};

struct TraceFrontierEntry {
    std::string node_id;
    double g_cost = 0;
    double f_cost = 0;
};

struct PlanTraceFrame {
    TraceEventType event = TraceEventType::INIT;
    size_t step_index = 0;
    std::string node_id;
    std::string from_node;
    std::string edge_id;
    double g_cost = 0;
    double f_cost = 0;
    double step_cost = 0;
    std::vector<TraceFrontierEntry> frontier;
    std::vector<std::string> best_path;
    std::vector<std::string> expanded_nodes;
    std::string message;
};

struct TaskCandidateResult {
    std::string candidate_node;
    bool success = false;
    double total_cost = 0;
    std::string failure_reason;
};

struct PlanTraceResult {
    PlanResult result;
    std::vector<PlanTraceFrame> frames;
    std::vector<TaskCandidateResult> candidate_results;
    std::string selected_candidate;
    double planning_ms = 0.0;
};

const char* traceEventTypeName(TraceEventType event);

double estimateAdmissibleHeuristicScale(
    const FieldGraph& graph,
    const PlannerWeights& weights,
    double safety_margin = 0.99);

PlanTraceResult planRouteTrace(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& goal_node,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights,
    const PlannerTraceOptions& options = {});

PlanTraceResult planToTaskTrace(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& task_tag,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights,
    const PlannerTraceOptions& options = {});

PlanTraceResult planRouteTagTrace(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& route_tag,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights,
    const PlannerTraceOptions& options = {});

PlanResult planRoute(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& goal_node,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights,
    const PlannerRunOptions& options = {});

PlanResult planToTask(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& task_tag,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights,
    const PlannerRunOptions& options = {});

PlanResult planRouteTag(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& route_tag,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights,
    const PlannerRunOptions& options = {});

}  // namespace rc26_topo_nav
