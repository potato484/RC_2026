#include "rc26_topo_nav/planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_set>
#include <utility>

namespace rc26_topo_nav {

static double edgeCost(const GraphEdge& edge, const EdgeOverlay& ov, const PlannerWeights& w) {
    if (ov.state == EdgeState::BLOCKED) {
        return std::numeric_limits<double>::infinity();
    }

    double cost = edge.base_cost * w.time;
    cost += std::abs(edge.height_change) * w.height_risk;

    if (ov.state == EdgeState::SLOW_ONLY) {
        cost += w.slow_only;
    }
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
    const auto adj_it = graph.adjacency.find(from);
    if (adj_it == graph.adjacency.end()) {
        return nullptr;
    }
    for (const size_t edge_index : adj_it->second) {
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

double elapsedMilliseconds(
    const std::chrono::steady_clock::time_point& begin,
    const std::chrono::steady_clock::time_point& end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

struct ScopedTraceTiming {
    explicit ScopedTraceTiming(double* target_in)
    : begin(std::chrono::steady_clock::now()), target(target_in) {}

    ~ScopedTraceTiming() {
        if (target != nullptr) {
            *target = elapsedMilliseconds(begin, std::chrono::steady_clock::now());
        }
    }

    std::chrono::steady_clock::time_point begin;
    double* target = nullptr;
};

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

struct SearchOptions {
    double heuristic_scale = 0.0;
    bool capture_trace = false;
    bool capture_frontier = false;
    bool capture_paths = false;
};

SearchOptions makeSearchOptions(const PlannerRunOptions& options) {
    SearchOptions search_options;
    search_options.heuristic_scale = options.heuristic_scale;
    return search_options;
}

SearchOptions makeSearchOptions(const PlannerTraceOptions& options) {
    SearchOptions search_options;
    search_options.heuristic_scale = options.heuristic_scale;
    search_options.capture_trace = true;
    search_options.capture_frontier = options.capture_frontier;
    search_options.capture_paths = options.capture_paths;
    return search_options;
}

double heuristicCost(
    const FieldGraph& graph,
    const std::string& node_id,
    const std::string& goal_node,
    const double heuristic_scale) {
    if (heuristic_scale <= 0.0) {
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
    return std::sqrt(dx * dx + dy * dy + dz * dz) * heuristic_scale;
}

double pointDistance(const Pose3& from, const Pose3& to) {
    const double dx = from.x - to.x;
    const double dy = from.y - to.y;
    const double dz = from.z - to.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double edgeTravelDistance(const FieldGraph& graph, const GraphEdge& edge) {
    const auto from_it = graph.nodes.find(edge.from);
    const auto to_it = graph.nodes.find(edge.to);
    if (from_it == graph.nodes.end() || to_it == graph.nodes.end()) {
        return 0.0;
    }

    double distance = 0.0;
    Pose3 last_pose = from_it->second.pose;
    for (const auto& control_point : edge.control_points) {
        distance += pointDistance(last_pose, control_point);
        last_pose = control_point;
    }
    distance += pointDistance(last_pose, to_it->second.pose);
    return distance;
}

std::vector<std::string> reconstructNodePath(
    const std::string& start_node,
    const std::string& goal_node,
    const std::unordered_map<std::string, std::string>& prev_node) {
    std::vector<std::string> path;
    std::string cursor = goal_node;
    while (cursor != start_node) {
        path.push_back(cursor);
        const auto prev_it = prev_node.find(cursor);
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
    const SearchOptions& options) {
    std::vector<TraceFrontierEntry> frontier;
    std::unordered_set<std::string> seen;
    while (!queue_copy.empty()) {
        const auto entry = queue_copy.top();
        queue_copy.pop();
        const auto dist_it = dist.find(entry.node_id);
        if (dist_it == dist.end() || entry.g_cost > dist_it->second) {
            continue;
        }
        if (!seen.insert(entry.node_id).second) {
            continue;
        }
        frontier.push_back({
            entry.node_id,
            dist_it->second,
            dist_it->second + heuristicCost(graph, entry.node_id, goal_node, options.heuristic_scale),
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
    PlanTraceResult* trace,
    const SearchOptions& options,
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue_copy,
    const std::unordered_map<std::string, double>& dist,
    const FieldGraph& graph,
    const std::string& goal_node,
    const std::vector<std::string>& expanded_nodes,
    const std::vector<std::string>& best_path,
    const TraceEventType event,
    const std::string& node_id,
    const std::string& from_node,
    const std::string& edge_id,
    const double g_cost,
    const double f_cost,
    const double step_cost,
    const std::string& message) {
    if (trace == nullptr || !options.capture_trace) {
        return;
    }

    PlanTraceFrame frame;
    frame.event = event;
    frame.step_index = trace->frames.size();
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
    trace->frames.push_back(std::move(frame));
}

void appendSimpleFrame(
    PlanTraceResult* trace,
    const SearchOptions& options,
    const TraceEventType event,
    const std::string& node_id,
    const std::string& from_node,
    const std::string& edge_id,
    const double g_cost,
    const double f_cost,
    const double step_cost,
    const std::vector<std::string>& best_path,
    const std::string& message) {
    if (trace == nullptr || !options.capture_trace) {
        return;
    }

    trace->frames.push_back({
        event,
        trace->frames.size(),
        node_id,
        from_node,
        edge_id,
        g_cost,
        f_cost,
        step_cost,
        {},
        best_path,
        {},
        message,
    });
}

PlanResult planRouteCore(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& goal_node,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights,
    const SearchOptions& options,
    PlanTraceResult* trace = nullptr) {
    PlanResult result;

    if (graph.nodes.find(start_node) == graph.nodes.end()) {
        result.failure_reason = "Start node '" + start_node + "' not found";
        return result;
    }
    if (graph.nodes.find(goal_node) == graph.nodes.end()) {
        result.failure_reason = "Goal node '" + goal_node + "' not found";
        return result;
    }

    const auto goal_overlay_it = node_overlays.find(goal_node);
    if (goal_overlay_it != node_overlays.end() && goal_overlay_it->second.state == NodeState::BLOCKED) {
        result.failure_reason = "Goal node '" + goal_node + "' is blocked";
        return result;
    }

    std::unordered_map<std::string, double> dist;
    std::unordered_map<std::string, std::string> prev_node;
    std::unordered_map<std::string, size_t> prev_edge;
    dist.reserve(graph.nodes.size());
    prev_node.reserve(graph.nodes.size());
    prev_edge.reserve(graph.nodes.size());

    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> pq;
    std::vector<std::string> expanded_nodes;
    if (options.capture_trace) {
        expanded_nodes.reserve(graph.nodes.size());
    }

    for (const auto& [node_id, _] : graph.nodes) {
        dist[node_id] = std::numeric_limits<double>::infinity();
    }

    const double start_heuristic = heuristicCost(graph, start_node, goal_node, options.heuristic_scale);
    dist[start_node] = 0.0;
    pq.push({start_heuristic, 0.0, start_node});
    appendFrame(
        trace,
        options,
        pq,
        dist,
        graph,
        goal_node,
        expanded_nodes,
        {start_node},
        TraceEventType::INIT,
        start_node,
        "",
        "",
        0.0,
        start_heuristic,
        0.0,
        "planner initialized");

    while (!pq.empty()) {
        const auto entry = pq.top();
        pq.pop();

        const auto current_dist_it = dist.find(entry.node_id);
        if (current_dist_it == dist.end()) {
            continue;
        }
        if (entry.g_cost > current_dist_it->second) {
            appendFrame(
                trace,
                options,
                pq,
                dist,
                graph,
                goal_node,
                expanded_nodes,
                {},
                TraceEventType::SKIP_STALE,
                entry.node_id,
                "",
                "",
                entry.g_cost,
                entry.priority,
                0.0,
                "stale queue entry skipped");
            continue;
        }

        if (options.capture_trace) {
            expanded_nodes.push_back(entry.node_id);
        }

        std::vector<std::string> current_path;
        if (options.capture_trace) {
            current_path =
                entry.node_id == start_node
                    ? std::vector<std::string>{start_node}
                    : reconstructNodePath(start_node, entry.node_id, prev_node);
        }

        appendFrame(
            trace,
            options,
            pq,
            dist,
            graph,
            goal_node,
            expanded_nodes,
            current_path,
            TraceEventType::POP,
            entry.node_id,
            "",
            "",
            entry.g_cost,
            entry.priority,
            0.0,
            "expanded current best node");

        if (entry.node_id == goal_node) {
            result.node_path = current_path.empty()
                ? reconstructNodePath(start_node, entry.node_id, prev_node)
                : current_path;
            result.total_cost = current_dist_it->second;
            result.success = true;
            appendFrame(
                trace,
                options,
                pq,
                dist,
                graph,
                goal_node,
                expanded_nodes,
                result.node_path,
                TraceEventType::GOAL,
                goal_node,
                "",
                "",
                result.total_cost,
                result.total_cost,
                0.0,
                "goal reached");
            break;
        }

        const auto node_overlay_it = node_overlays.find(entry.node_id);
        if (node_overlay_it != node_overlays.end() &&
            node_overlay_it->second.state == NodeState::BLOCKED &&
            entry.node_id != start_node) {
            appendFrame(
                trace,
                options,
                pq,
                dist,
                graph,
                goal_node,
                expanded_nodes,
                current_path,
                TraceEventType::BLOCKED_NODE,
                entry.node_id,
                "",
                "",
                entry.g_cost,
                entry.priority,
                0.0,
                "node blocked by overlay");
            continue;
        }

        const auto adj_it = graph.adjacency.find(entry.node_id);
        if (adj_it == graph.adjacency.end()) {
            continue;
        }

        for (const size_t edge_index : adj_it->second) {
            const auto& edge = graph.edges[edge_index];
            EdgeOverlay edge_overlay;
            const auto edge_overlay_it = edge_overlays.find(edge.id);
            if (edge_overlay_it != edge_overlays.end()) {
                edge_overlay = edge_overlay_it->second;
            }

            const double step_cost = edgeCost(edge, edge_overlay, weights);
            if (std::isinf(step_cost)) {
                appendFrame(
                    trace,
                    options,
                    pq,
                    dist,
                    graph,
                    goal_node,
                    expanded_nodes,
                    current_path,
                    TraceEventType::EDGE_BLOCKED,
                    edge.to,
                    entry.node_id,
                    edge.id,
                    current_dist_it->second,
                    current_dist_it->second + heuristicCost(graph, entry.node_id, goal_node, options.heuristic_scale),
                    step_cost,
                    "edge blocked by overlay");
                continue;
            }

            const double new_dist = current_dist_it->second + step_cost;
            const double new_priority = new_dist + heuristicCost(graph, edge.to, goal_node, options.heuristic_scale);
            if (new_dist < dist[edge.to]) {
                dist[edge.to] = new_dist;
                prev_node[edge.to] = entry.node_id;
                prev_edge[edge.to] = edge_index;
                pq.push({new_priority, new_dist, edge.to});

                std::vector<std::string> best_path;
                if (options.capture_paths) {
                    best_path = reconstructNodePath(start_node, edge.to, prev_node);
                }

                appendFrame(
                    trace,
                    options,
                    pq,
                    dist,
                    graph,
                    goal_node,
                    expanded_nodes,
                    best_path,
                    TraceEventType::RELAX,
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
                    options,
                    pq,
                    dist,
                    graph,
                    goal_node,
                    expanded_nodes,
                    current_path,
                    TraceEventType::KEEP_BEST,
                    edge.to,
                    entry.node_id,
                    edge.id,
                    dist[edge.to],
                    dist[edge.to] + heuristicCost(graph, edge.to, goal_node, options.heuristic_scale),
                    step_cost,
                    "existing path kept");
            }
        }
    }

    if (!result.success) {
        const auto goal_dist_it = dist.find(goal_node);
        if (goal_dist_it == dist.end() || std::isinf(goal_dist_it->second)) {
            result.failure_reason = "No path from '" + start_node + "' to '" + goal_node + "'";
            appendFrame(
                trace,
                options,
                pq,
                dist,
                graph,
                goal_node,
                expanded_nodes,
                {},
                TraceEventType::FAILED,
                goal_node,
                "",
                "",
                0.0,
                0.0,
                0.0,
                result.failure_reason);
        }
        return result;
    }

    std::string cursor = goal_node;
    while (cursor != start_node) {
        result.edge_indices.push_back(prev_edge[cursor]);
        cursor = prev_node[cursor];
    }
    std::reverse(result.edge_indices.begin(), result.edge_indices.end());
    return result;
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

double estimateAdmissibleHeuristicScale(
    const FieldGraph& graph,
    const PlannerWeights& weights,
    const double safety_margin) {
    double min_cost_per_meter = std::numeric_limits<double>::infinity();
    const double clamped_margin =
        safety_margin < 0.0 ? 0.0 : (safety_margin > 1.0 ? 1.0 : safety_margin);
    const EdgeOverlay no_overlay;

    for (const auto& edge : graph.edges) {
        const double distance_m = edgeTravelDistance(graph, edge);
        if (distance_m <= 1e-9) {
            continue;
        }

        const double cost = edgeCost(edge, no_overlay, weights);
        if (!std::isfinite(cost) || cost <= 0.0) {
            continue;
        }

        min_cost_per_meter = std::min(min_cost_per_meter, cost / distance_m);
    }

    if (!std::isfinite(min_cost_per_meter)) {
        return 0.0;
    }
    return min_cost_per_meter * clamped_margin;
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
    const ScopedTraceTiming scoped_timing(&trace.planning_ms);
    trace.result = planRouteCore(
        graph,
        start_node,
        goal_node,
        node_overlays,
        edge_overlays,
        weights,
        makeSearchOptions(options),
        &trace);
    return trace;
}

PlanResult planRoute(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& goal_node,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights,
    const PlannerRunOptions& options) {
    return planRouteCore(
        graph,
        start_node,
        goal_node,
        node_overlays,
        edge_overlays,
        weights,
        makeSearchOptions(options));
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
    const ScopedTraceTiming scoped_timing(&trace.planning_ms);
    auto& result = trace.result;

    const TaskDef* task = nullptr;
    for (const auto& candidate_task : graph.tasks) {
        if (candidate_task.task_tag == task_tag) {
            task = &candidate_task;
            break;
        }
    }

    if (task == nullptr) {
        result.failure_reason = "Task '" + task_tag + "' not found";
        return trace;
    }

    PlanResult best;
    best.total_cost = std::numeric_limits<double>::infinity();
    PlanTraceResult best_trace;

    for (const auto& candidate_node : task->candidate_nodes) {
        const auto node_overlay_it = node_overlays.find(candidate_node);
        if (node_overlay_it != node_overlays.end() && node_overlay_it->second.state == NodeState::BLOCKED) {
            trace.candidate_results.push_back({candidate_node, false, 0.0, "candidate blocked"});
            appendSimpleFrame(
                &trace,
                makeSearchOptions(options),
                TraceEventType::CANDIDATE_BLOCKED,
                candidate_node,
                "",
                "",
                0.0,
                0.0,
                0.0,
                {},
                "candidate blocked by overlay");
            continue;
        }

        auto candidate_trace = planRouteTrace(
            graph,
            start_node,
            candidate_node,
            node_overlays,
            edge_overlays,
            weights,
            options);
        trace.candidate_results.push_back(
            {
                candidate_node,
                candidate_trace.result.success,
                candidate_trace.result.total_cost,
                candidate_trace.result.failure_reason,
            });
        if (candidate_trace.result.success && candidate_trace.result.total_cost < best.total_cost) {
            best = candidate_trace.result;
            best_trace = std::move(candidate_trace);
            trace.selected_candidate = candidate_node;
        }
    }

    if (!best.success) {
        result.failure_reason = "No reachable candidate for task '" + task_tag + "'";
        return trace;
    }

    trace.frames = std::move(best_trace.frames);
    appendSimpleFrame(
        &trace,
        makeSearchOptions(options),
        TraceEventType::CANDIDATE_SELECTED,
        trace.selected_candidate,
        "",
        "",
        best.total_cost,
        best.total_cost,
        0.0,
        best.node_path,
        "selected best reachable candidate");
    result = best;
    return trace;
}

PlanResult planToTask(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& task_tag,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights,
    const PlannerRunOptions& options) {
    const TaskDef* task = nullptr;
    for (const auto& candidate_task : graph.tasks) {
        if (candidate_task.task_tag == task_tag) {
            task = &candidate_task;
            break;
        }
    }

    if (task == nullptr) {
        return {false, {}, {}, 0.0, "Task '" + task_tag + "' not found"};
    }

    PlanResult best;
    best.total_cost = std::numeric_limits<double>::infinity();
    for (const auto& candidate_node : task->candidate_nodes) {
        const auto node_overlay_it = node_overlays.find(candidate_node);
        if (node_overlay_it != node_overlays.end() && node_overlay_it->second.state == NodeState::BLOCKED) {
            continue;
        }

        auto candidate = planRoute(
            graph,
            start_node,
            candidate_node,
            node_overlays,
            edge_overlays,
            weights,
            options);
        if (candidate.success && candidate.total_cost < best.total_cost) {
            best = std::move(candidate);
        }
    }

    if (!best.success) {
        best.failure_reason = "No reachable candidate for task '" + task_tag + "'";
    }
    return best;
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
    const ScopedTraceTiming scoped_timing(&trace.planning_ms);
    auto& combined = trace.result;
    const auto route_it = graph.routes.find(route_tag);
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

    const SearchOptions search_options = makeSearchOptions(options);
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

    for (size_t index = 1; index < route.nodes.size(); ++index) {
        const auto& next_node = route.nodes[index];
        size_t edge_index = 0;
        const auto* edge = findDirectEdge(graph, cursor, next_node, &edge_index);
        if (edge == nullptr) {
            combined = {};
            combined.failure_reason =
                "Route '" + route_tag + "' has no direct edge from '" + cursor + "' to '" + next_node + "'";
            return trace;
        }

        const auto node_overlay_it = node_overlays.find(next_node);
        if (node_overlay_it != node_overlays.end() && node_overlay_it->second.state == NodeState::BLOCKED) {
            combined = {};
            combined.failure_reason =
                "Route '" + route_tag + "' target node '" + next_node + "' is blocked";
            return trace;
        }

        EdgeOverlay edge_overlay;
        const auto edge_overlay_it = edge_overlays.find(edge->id);
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
        combined.node_path.push_back(next_node);
        combined.edge_indices.push_back(edge_index);
        combined.total_cost += cost;
        appendSimpleFrame(
            &trace,
            search_options,
            TraceEventType::ROUTE_TAG,
            next_node,
            cursor,
            edge->id,
            combined.total_cost,
            combined.total_cost,
            cost,
            combined.node_path,
            "route tag advanced by declared edge");
        cursor = next_node;
    }

    return trace;
}

PlanResult planRouteTag(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& route_tag,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights,
    const PlannerRunOptions& options) {
    const auto route_it = graph.routes.find(route_tag);
    if (route_it == graph.routes.end()) {
        return {false, {}, {}, 0.0, "Route '" + route_tag + "' not found"};
    }

    PlanResult combined;
    const auto& route = route_it->second;
    if (route.nodes.empty()) {
        combined.failure_reason = "Route '" + route_tag + "' is empty";
        return combined;
    }
    combined.success = true;

    std::string cursor = start_node;
    if (cursor != route.nodes.front()) {
        auto prefix = planRoute(
            graph,
            cursor,
            route.nodes.front(),
            node_overlays,
            edge_overlays,
            weights,
            options);
        if (!prefix.success) {
            prefix.failure_reason = "Route '" + route_tag + "' prefix failed: " + prefix.failure_reason;
            return prefix;
        }
        combined = std::move(prefix);
        cursor = route.nodes.front();
    } else {
        combined.node_path.push_back(cursor);
    }

    for (size_t index = 1; index < route.nodes.size(); ++index) {
        const auto& next_node = route.nodes[index];
        size_t edge_index = 0;
        const auto* edge = findDirectEdge(graph, cursor, next_node, &edge_index);
        if (edge == nullptr) {
            return {false, {}, {}, 0.0,
                "Route '" + route_tag + "' has no direct edge from '" + cursor + "' to '" + next_node + "'"};
        }

        const auto node_overlay_it = node_overlays.find(next_node);
        if (node_overlay_it != node_overlays.end() && node_overlay_it->second.state == NodeState::BLOCKED) {
            return {false, {}, {}, 0.0,
                "Route '" + route_tag + "' target node '" + next_node + "' is blocked"};
        }

        EdgeOverlay edge_overlay;
        const auto edge_overlay_it = edge_overlays.find(edge->id);
        if (edge_overlay_it != edge_overlays.end()) {
            edge_overlay = edge_overlay_it->second;
        }
        const double cost = edgeCost(*edge, edge_overlay, weights);
        if (std::isinf(cost)) {
            return {false, {}, {}, 0.0, "Route '" + route_tag + "' edge '" + edge->id + "' is blocked"};
        }

        if (combined.node_path.empty() || combined.node_path.back() != cursor) {
            combined.node_path.push_back(cursor);
        }
        combined.node_path.push_back(next_node);
        combined.edge_indices.push_back(edge_index);
        combined.total_cost += cost;
        cursor = next_node;
    }

    return combined;
}

}  // namespace rc26_topo_nav
