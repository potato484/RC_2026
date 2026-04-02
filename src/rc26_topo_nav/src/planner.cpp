#include "rc26_topo_nav/planner.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_set>

namespace rc26_topo_nav {

static double edgeCost(const GraphEdge& edge, const EdgeOverlay& ov, const PlannerWeights& w) {
    if (ov.state == EdgeState::BLOCKED) return std::numeric_limits<double>::infinity();

    double cost = edge.base_cost * w.time;
    cost += std::abs(edge.height_change) * w.height_risk;

    if (ov.state == EdgeState::SLOW_ONLY) cost += w.slow_only;
    if (ov.state == EdgeState::CONFIRM_REQUIRED || edge.requires_confirmation) {
        cost += w.confirm_required;
    }

    cost += ov.extra_cost;
    return cost;
}

static const GraphEdge* findDirectEdge(
    const FieldGraph& graph,
    const std::string& from,
    const std::string& to,
    size_t* edge_index_out = nullptr) {
    auto adj_it = graph.adjacency.find(from);
    if (adj_it == graph.adjacency.end()) {
        return nullptr;
    }
    for (size_t edge_index : adj_it->second) {
        if (graph.edges[edge_index].to == to) {
            if (edge_index_out != nullptr) {
                *edge_index_out = edge_index;
            }
            return &graph.edges[edge_index];
        }
    }
    return nullptr;
}

namespace {

struct QueueEntry {
    double priority = 0.0;
    double g_cost = 0.0;
    std::string node_id;

    bool operator>(const QueueEntry& other) const {
        if (priority == other.priority) {
            return g_cost > other.g_cost;
        }
        return priority > other.priority;
    }
};

double heuristicCost(
    const FieldGraph& graph,
    const std::string& node_id,
    const std::string& goal_node,
    const PlannerTraceOptions& options) {
    if (options.heuristic_scale <= 0.0) {
        return 0.0;
    }
    const auto node_it = graph.nodes.find(node_id);
    const auto goal_it = graph.nodes.find(goal_node);
    if (node_it == graph.nodes.end() || goal_it == graph.nodes.end()) {
        return 0.0;
    }
    const double dx = node_it->second.pose.x - goal_it->second.pose.x;
    const double dy = node_it->second.pose.y - goal_it->second.pose.y;
    const double dz = node_it->second.pose.z - goal_it->second.pose.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz) * options.heuristic_scale;
}

std::vector<std::string> reconstructNodePath(
    const std::string& start_node,
    const std::string& goal_node,
    const std::unordered_map<std::string, std::string>& prev_node) {
    std::vector<std::string> path;
    std::string cursor = goal_node;
    while (cursor != start_node) {
        path.push_back(cursor);
        auto prev_it = prev_node.find(cursor);
        if (prev_it == prev_node.end()) {
            return {};
        }
        cursor = prev_it->second;
    }
    path.push_back(start_node);
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<TraceFrontierEntry> buildFrontierSnapshot(
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue_copy,
    const std::unordered_map<std::string, double>& dist,
    const FieldGraph& graph,
    const std::string& goal_node,
    const PlannerTraceOptions& options) {
    std::vector<TraceFrontierEntry> frontier;
    std::unordered_set<std::string> seen;
    while (!queue_copy.empty()) {
        const auto entry = queue_copy.top();
        queue_copy.pop();
        auto dist_it = dist.find(entry.node_id);
        if (dist_it == dist.end() || entry.g_cost > dist_it->second) {
            continue;
        }
        if (!seen.insert(entry.node_id).second) {
            continue;
        }
        frontier.push_back({
            entry.node_id,
            dist_it->second,
            dist_it->second + heuristicCost(graph, entry.node_id, goal_node, options),
        });
    }
    std::sort(frontier.begin(), frontier.end(), [](const TraceFrontierEntry& left, const TraceFrontierEntry& right) {
        if (left.f_cost == right.f_cost) {
            return left.node_id < right.node_id;
        }
        return left.f_cost < right.f_cost;
    });
    return frontier;
}

void appendFrame(
    PlanTraceResult& trace,
    TraceEventType event,
    const PlannerTraceOptions& options,
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue_copy,
    const std::unordered_map<std::string, double>& dist,
    const FieldGraph& graph,
    const std::string& goal_node,
    const std::vector<std::string>& expanded_nodes,
    const std::vector<std::string>& best_path,
    const std::string& node_id,
    const std::string& from_node,
    const std::string& edge_id,
    const double g_cost,
    const double f_cost,
    const double step_cost,
    const std::string& message) {
    PlanTraceFrame frame;
    frame.event = event;
    frame.step_index = trace.frames.size();
    frame.node_id = node_id;
    frame.from_node = from_node;
    frame.edge_id = edge_id;
    frame.g_cost = g_cost;
    frame.f_cost = f_cost;
    frame.step_cost = step_cost;
    frame.message = message;
    if (options.capture_frontier) {
        frame.frontier = buildFrontierSnapshot(std::move(queue_copy), dist, graph, goal_node, options);
    }
    if (options.capture_paths) {
        frame.best_path = best_path;
    }
    frame.expanded_nodes = expanded_nodes;
    trace.frames.push_back(std::move(frame));
}

}  // namespace

const char* traceEventTypeName(TraceEventType event) {
    switch (event) {
        case TraceEventType::INIT:
            return "init";
        case TraceEventType::POP:
            return "pop";
        case TraceEventType::SKIP_STALE:
            return "skip_stale";
        case TraceEventType::BLOCKED_NODE:
            return "blocked_node";
        case TraceEventType::EDGE_BLOCKED:
            return "edge_blocked";
        case TraceEventType::RELAX:
            return "relax";
        case TraceEventType::KEEP_BEST:
            return "keep_best";
        case TraceEventType::GOAL:
            return "goal";
        case TraceEventType::FAILED:
            return "failed";
        case TraceEventType::ROUTE_TAG:
            return "route_tag";
        case TraceEventType::CANDIDATE_BLOCKED:
            return "candidate_blocked";
        case TraceEventType::CANDIDATE_SELECTED:
            return "candidate_selected";
    }
    return "unknown";
}

PlanTraceResult planRouteTrace(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& goal_node,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights,
    const PlannerTraceOptions& options) {
    PlanTraceResult trace;
    auto& result = trace.result;

    if (graph.nodes.find(start_node) == graph.nodes.end()) {
        result.failure_reason = "Start node '" + start_node + "' not found";
        return trace;
    }
    if (graph.nodes.find(goal_node) == graph.nodes.end()) {
        result.failure_reason = "Goal node '" + goal_node + "' not found";
        return trace;
    }

    auto goal_ov = node_overlays.find(goal_node);
    if (goal_ov != node_overlays.end() && goal_ov->second.state == NodeState::BLOCKED) {
        result.failure_reason = "Goal node '" + goal_node + "' is blocked";
        return trace;
    }

    std::unordered_map<std::string, double> dist;
    std::unordered_map<std::string, std::string> prev_node;
    std::unordered_map<std::string, size_t> prev_edge;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> pq;
    std::vector<std::string> expanded_nodes;

    for (const auto& [id, _] : graph.nodes) {
        dist[id] = std::numeric_limits<double>::infinity();
    }
    dist[start_node] = 0.0;
    pq.push({heuristicCost(graph, start_node, goal_node, options), 0.0, start_node});
    appendFrame(
        trace,
        TraceEventType::INIT,
        options,
        pq,
        dist,
        graph,
        goal_node,
        expanded_nodes,
        {start_node},
        start_node,
        "",
        "",
        0.0,
        heuristicCost(graph, start_node, goal_node, options),
        0.0,
        "planner initialized");

    while (!pq.empty()) {
        const auto entry = pq.top();
        pq.pop();

        auto current_dist_it = dist.find(entry.node_id);
        if (current_dist_it == dist.end()) {
            continue;
        }
        if (entry.g_cost > current_dist_it->second) {
            appendFrame(
                trace,
                TraceEventType::SKIP_STALE,
                options,
                pq,
                dist,
                graph,
                goal_node,
                expanded_nodes,
                {},
                entry.node_id,
                "",
                "",
                entry.g_cost,
                entry.priority,
                0.0,
                "stale queue entry skipped");
            continue;
        }

        expanded_nodes.push_back(entry.node_id);
        const auto current_path =
            entry.node_id == start_node ? std::vector<std::string>{start_node}
                                        : reconstructNodePath(start_node, entry.node_id, prev_node);
        appendFrame(
            trace,
            TraceEventType::POP,
            options,
            pq,
            dist,
            graph,
            goal_node,
            expanded_nodes,
            current_path,
            entry.node_id,
            "",
            "",
            entry.g_cost,
            entry.priority,
            0.0,
            "expanded current best node");

        if (entry.node_id == goal_node) {
            result.node_path = current_path;
            result.total_cost = current_dist_it->second;
            result.success = true;
            appendFrame(
                trace,
                TraceEventType::GOAL,
                options,
                pq,
                dist,
                graph,
                goal_node,
                expanded_nodes,
                result.node_path,
                goal_node,
                "",
                "",
                result.total_cost,
                result.total_cost,
                0.0,
                "goal reached");
            break;
        }

        auto n_ov = node_overlays.find(entry.node_id);
        if (n_ov != node_overlays.end() && n_ov->second.state == NodeState::BLOCKED && entry.node_id != start_node) {
            appendFrame(
                trace,
                TraceEventType::BLOCKED_NODE,
                options,
                pq,
                dist,
                graph,
                goal_node,
                expanded_nodes,
                current_path,
                entry.node_id,
                "",
                "",
                entry.g_cost,
                entry.priority,
                0.0,
                "node blocked by overlay");
            continue;
        }

        auto adj_it = graph.adjacency.find(entry.node_id);
        if (adj_it == graph.adjacency.end()) {
            continue;
        }

        for (size_t ei : adj_it->second) {
            const auto& edge = graph.edges[ei];
            EdgeOverlay edge_overlay;
            auto edge_overlay_it = edge_overlays.find(edge.id);
            if (edge_overlay_it != edge_overlays.end()) {
                edge_overlay = edge_overlay_it->second;
            }

            const double step_cost = edgeCost(edge, edge_overlay, weights);
            if (std::isinf(step_cost)) {
                appendFrame(
                    trace,
                    TraceEventType::EDGE_BLOCKED,
                    options,
                    pq,
                    dist,
                    graph,
                    goal_node,
                    expanded_nodes,
                    current_path,
                    edge.to,
                    entry.node_id,
                    edge.id,
                    current_dist_it->second,
                    current_dist_it->second + heuristicCost(graph, entry.node_id, goal_node, options),
                    step_cost,
                    "edge blocked by overlay");
                continue;
            }

            const double new_dist = current_dist_it->second + step_cost;
            const double new_priority = new_dist + heuristicCost(graph, edge.to, goal_node, options);
            if (new_dist < dist[edge.to]) {
                dist[edge.to] = new_dist;
                prev_node[edge.to] = entry.node_id;
                prev_edge[edge.to] = ei;
                pq.push({new_priority, new_dist, edge.to});

                const auto best_path = reconstructNodePath(start_node, edge.to, prev_node);
                appendFrame(
                    trace,
                    TraceEventType::RELAX,
                    options,
                    pq,
                    dist,
                    graph,
                    goal_node,
                    expanded_nodes,
                    best_path,
                    edge.to,
                    entry.node_id,
                    edge.id,
                    new_dist,
                    new_priority,
                    step_cost,
                    "better path discovered");
            } else {
                appendFrame(
                    trace,
                    TraceEventType::KEEP_BEST,
                    options,
                    pq,
                    dist,
                    graph,
                    goal_node,
                    expanded_nodes,
                    current_path,
                    edge.to,
                    entry.node_id,
                    edge.id,
                    dist[edge.to],
                    dist[edge.to] + heuristicCost(graph, edge.to, goal_node, options),
                    step_cost,
                    "existing path kept");
            }
        }
    }

    if (!result.success) {
        if (std::isinf(dist[goal_node])) {
            result.failure_reason = "No path from '" + start_node + "' to '" + goal_node + "'";
            appendFrame(
                trace,
                TraceEventType::FAILED,
                options,
                pq,
                dist,
                graph,
                goal_node,
                expanded_nodes,
                {},
                goal_node,
                "",
                "",
                0.0,
                0.0,
                0.0,
                result.failure_reason);
        }
        return trace;
    }

    std::string cur = goal_node;
    while (cur != start_node) {
        result.edge_indices.push_back(prev_edge[cur]);
        cur = prev_node[cur];
    }
    std::reverse(result.edge_indices.begin(), result.edge_indices.end());
    return trace;
}

PlanResult planRoute(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& goal_node,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights) {
    return planRouteTrace(
               graph,
               start_node,
               goal_node,
               node_overlays,
               edge_overlays,
               weights,
               PlannerTraceOptions{})
        .result;
}

PlanTraceResult planToTaskTrace(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& task_tag,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights,
    const PlannerTraceOptions& options) {
    PlanTraceResult trace;
    auto& result = trace.result;

    const TaskDef* task = nullptr;
    for (const auto& t : graph.tasks) {
        if (t.task_tag == task_tag) {
            task = &t;
            break;
        }
    }

    if (!task) {
        result.failure_reason = "Task '" + task_tag + "' not found";
        return trace;
    }

    PlanResult best;
    best.total_cost = std::numeric_limits<double>::infinity();
    PlanTraceResult best_trace;

    for (const auto& cn : task->candidate_nodes) {
        auto n_ov = node_overlays.find(cn);
        if (n_ov != node_overlays.end() && n_ov->second.state == NodeState::BLOCKED) {
            trace.candidate_results.push_back({cn, false, 0.0, "candidate blocked"});
            trace.frames.push_back({
                TraceEventType::CANDIDATE_BLOCKED,
                trace.frames.size(),
                cn,
                "",
                "",
                0.0,
                0.0,
                0.0,
                {},
                {},
                {},
                "candidate blocked by overlay",
            });
            continue;
        }

        auto candidate_trace = planRouteTrace(
            graph,
            start_node,
            cn,
            node_overlays,
            edge_overlays,
            weights,
            options);
        trace.candidate_results.push_back(
            {cn, candidate_trace.result.success, candidate_trace.result.total_cost, candidate_trace.result.failure_reason});
        if (candidate_trace.result.success && candidate_trace.result.total_cost < best.total_cost) {
            best = candidate_trace.result;
            best_trace = std::move(candidate_trace);
            trace.selected_candidate = cn;
        }
    }

    if (!best.success) {
        result.failure_reason = "No reachable candidate for task '" + task_tag + "'";
        return trace;
    }

    trace.frames = best_trace.frames;
    trace.frames.push_back({
        TraceEventType::CANDIDATE_SELECTED,
        trace.frames.size(),
        trace.selected_candidate,
        "",
        "",
        best.total_cost,
        best.total_cost,
        0.0,
        {},
        best.node_path,
        {},
        "selected best reachable candidate",
    });
    result = best;
    return trace;
}

PlanResult planToTask(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& task_tag,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights) {
    return planToTaskTrace(
               graph,
               start_node,
               task_tag,
               node_overlays,
               edge_overlays,
               weights,
               PlannerTraceOptions{})
        .result;
}

PlanTraceResult planRouteTagTrace(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& route_tag,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights,
    const PlannerTraceOptions& options) {
    PlanTraceResult trace;
    auto& combined = trace.result;
    auto route_it = graph.routes.find(route_tag);
    if (route_it == graph.routes.end()) {
        combined.failure_reason = "Route '" + route_tag + "' not found";
        return trace;
    }

    const auto& route = route_it->second;
    if (route.nodes.empty()) {
        combined.failure_reason = "Route '" + route_tag + "' is empty";
        return trace;
    }
    combined.success = true;

    std::string cursor = start_node;
    if (cursor != route.nodes.front()) {
        auto prefix = planRouteTrace(
            graph,
            cursor,
            route.nodes.front(),
            node_overlays,
            edge_overlays,
            weights,
            options);
        if (!prefix.result.success) {
            trace.result = prefix.result;
            trace.result.failure_reason = "Route '" + route_tag + "' prefix failed: " + prefix.result.failure_reason;
            trace.frames = std::move(prefix.frames);
            return trace;
        }
        combined = prefix.result;
        trace.frames = std::move(prefix.frames);
        cursor = route.nodes.front();
    } else {
        combined.node_path.push_back(cursor);
    }

    for (size_t i = 1; i < route.nodes.size(); ++i) {
        const auto& next = route.nodes[i];
        size_t edge_index = 0;
        const auto* edge = findDirectEdge(graph, cursor, next, &edge_index);
        if (edge == nullptr) {
            combined = {};
            combined.failure_reason =
                "Route '" + route_tag + "' has no direct edge from '" + cursor + "' to '" + next + "'";
            return trace;
        }

        auto node_it = node_overlays.find(next);
        if (node_it != node_overlays.end() && node_it->second.state == NodeState::BLOCKED) {
            combined = {};
            combined.failure_reason =
                "Route '" + route_tag + "' target node '" + next + "' is blocked";
            return trace;
        }

        EdgeOverlay edge_overlay;
        auto edge_overlay_it = edge_overlays.find(edge->id);
        if (edge_overlay_it != edge_overlays.end()) {
            edge_overlay = edge_overlay_it->second;
        }

        const double cost = edgeCost(*edge, edge_overlay, weights);
        if (std::isinf(cost)) {
            combined = {};
            combined.failure_reason =
                "Route '" + route_tag + "' edge '" + edge->id + "' is blocked";
            return trace;
        }

        if (combined.node_path.empty() || combined.node_path.back() != cursor) {
            combined.node_path.push_back(cursor);
        }
        combined.node_path.push_back(next);
        combined.edge_indices.push_back(edge_index);
        combined.total_cost += cost;
        trace.frames.push_back({
            TraceEventType::ROUTE_TAG,
            trace.frames.size(),
            next,
            cursor,
            edge->id,
            combined.total_cost,
            combined.total_cost,
            cost,
            {},
            combined.node_path,
            {},
            "route tag advanced by declared edge",
        });
        cursor = next;
    }

    return trace;
}

PlanResult planRouteTag(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& route_tag,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights) {
    return planRouteTagTrace(
               graph,
               start_node,
               route_tag,
               node_overlays,
               edge_overlays,
               weights,
               PlannerTraceOptions{})
        .result;
}

}  // namespace rc26_topo_nav
