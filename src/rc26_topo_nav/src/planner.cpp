#include "rc26_topo_nav/planner.hpp"
#include <cmath>
#include <queue>
#include <limits>
#include <algorithm>

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

PlanResult planRoute(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& goal_node,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights) {
    PlanResult result;

    if (graph.nodes.find(start_node) == graph.nodes.end()) {
        result.failure_reason = "Start node '" + start_node + "' not found";
        return result;
    }
    if (graph.nodes.find(goal_node) == graph.nodes.end()) {
        result.failure_reason = "Goal node '" + goal_node + "' not found";
        return result;
    }

    // Check goal not blocked
    auto goal_ov = node_overlays.find(goal_node);
    if (goal_ov != node_overlays.end() && goal_ov->second.state == NodeState::BLOCKED) {
        result.failure_reason = "Goal node '" + goal_node + "' is blocked";
        return result;
    }

    // A* with Dijkstra (h=0 since graph is small)
    struct Entry {
        double cost;
        std::string node_id;
        bool operator>(const Entry& o) const { return cost > o.cost; }
    };

    std::unordered_map<std::string, double> dist;
    std::unordered_map<std::string, std::string> prev_node;
    std::unordered_map<std::string, size_t> prev_edge;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pq;

    for (const auto& [id, _] : graph.nodes) {
        dist[id] = std::numeric_limits<double>::infinity();
    }
    dist[start_node] = 0;
    pq.push({0, start_node});

    while (!pq.empty()) {
        auto [d, u] = pq.top();
        pq.pop();

        if (d > dist[u]) continue;
        if (u == goal_node) break;

        // check node not blocked
        auto n_ov = node_overlays.find(u);
        if (n_ov != node_overlays.end() && n_ov->second.state == NodeState::BLOCKED && u != start_node) {
            continue;
        }

        auto adj_it = graph.adjacency.find(u);
        if (adj_it == graph.adjacency.end()) continue;

        for (size_t ei : adj_it->second) {
            const auto& edge = graph.edges[ei];
            auto e_ov_it = edge_overlays.find(edge.id);
            EdgeOverlay e_ov;
            if (e_ov_it != edge_overlays.end()) e_ov = e_ov_it->second;

            double ec = edgeCost(edge, e_ov, weights);
            if (std::isinf(ec)) continue;

            double new_dist = dist[u] + ec;
            if (new_dist < dist[edge.to]) {
                dist[edge.to] = new_dist;
                prev_node[edge.to] = u;
                prev_edge[edge.to] = ei;
                pq.push({new_dist, edge.to});
            }
        }
    }

    if (std::isinf(dist[goal_node])) {
        result.failure_reason = "No path from '" + start_node + "' to '" + goal_node + "'";
        return result;
    }

    // reconstruct path
    std::string cur = goal_node;
    while (cur != start_node) {
        result.node_path.push_back(cur);
        result.edge_indices.push_back(prev_edge[cur]);
        cur = prev_node[cur];
    }
    result.node_path.push_back(start_node);
    std::reverse(result.node_path.begin(), result.node_path.end());
    std::reverse(result.edge_indices.begin(), result.edge_indices.end());
    result.total_cost = dist[goal_node];
    result.success = true;
    return result;
}

PlanResult planToTask(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& task_tag,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights) {

    // find task definition
    const TaskDef* task = nullptr;
    for (const auto& t : graph.tasks) {
        if (t.task_tag == task_tag) { task = &t; break; }
    }

    if (!task) {
        PlanResult r;
        r.failure_reason = "Task '" + task_tag + "' not found";
        return r;
    }

    // plan to each candidate, pick min cost
    PlanResult best;
    best.total_cost = std::numeric_limits<double>::infinity();

    for (const auto& cn : task->candidate_nodes) {
        // skip blocked candidates
        auto n_ov = node_overlays.find(cn);
        if (n_ov != node_overlays.end() && n_ov->second.state == NodeState::BLOCKED) continue;

        auto pr = planRoute(graph, start_node, cn, node_overlays, edge_overlays, weights);
        if (pr.success && pr.total_cost < best.total_cost) {
            best = pr;
        }
    }

    if (!best.success) {
        best.failure_reason = "No reachable candidate for task '" + task_tag + "'";
    }
    return best;
}

PlanResult planRouteTag(
    const FieldGraph& graph,
    const std::string& start_node,
    const std::string& route_tag,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights) {
    auto route_it = graph.routes.find(route_tag);
    if (route_it == graph.routes.end()) {
        PlanResult result;
        result.failure_reason = "Route '" + route_tag + "' not found";
        return result;
    }

    const auto& route = route_it->second;
    if (route.nodes.empty()) {
        PlanResult result;
        result.failure_reason = "Route '" + route_tag + "' is empty";
        return result;
    }

    PlanResult combined;
    combined.success = true;

    std::string cursor = start_node;
    if (cursor != route.nodes.front()) {
        auto prefix = planRoute(graph, cursor, route.nodes.front(), node_overlays, edge_overlays, weights);
        if (!prefix.success) {
            prefix.failure_reason = "Route '" + route_tag + "' prefix failed: " + prefix.failure_reason;
            return prefix;
        }
        combined = prefix;
        cursor = route.nodes.front();
    } else {
        combined.node_path.push_back(cursor);
    }

    for (size_t i = 1; i < route.nodes.size(); ++i) {
        const auto& next = route.nodes[i];
        size_t edge_index = 0;
        const auto* edge = findDirectEdge(graph, cursor, next, &edge_index);
        if (edge == nullptr) {
            PlanResult result;
            result.failure_reason =
                "Route '" + route_tag + "' has no direct edge from '" + cursor + "' to '" + next + "'";
            return result;
        }

        auto node_it = node_overlays.find(next);
        if (node_it != node_overlays.end() && node_it->second.state == NodeState::BLOCKED) {
            PlanResult result;
            result.failure_reason =
                "Route '" + route_tag + "' target node '" + next + "' is blocked";
            return result;
        }

        EdgeOverlay edge_overlay;
        auto edge_overlay_it = edge_overlays.find(edge->id);
        if (edge_overlay_it != edge_overlays.end()) {
            edge_overlay = edge_overlay_it->second;
        }

        const double cost = edgeCost(*edge, edge_overlay, weights);
        if (std::isinf(cost)) {
            PlanResult result;
            result.failure_reason =
                "Route '" + route_tag + "' edge '" + edge->id + "' is blocked";
            return result;
        }

        if (combined.node_path.empty() || combined.node_path.back() != cursor) {
            combined.node_path.push_back(cursor);
        }
        combined.node_path.push_back(next);
        combined.edge_indices.push_back(edge_index);
        combined.total_cost += cost;
        cursor = next;
    }

    return combined;
}

}  // namespace rc26_topo_nav
