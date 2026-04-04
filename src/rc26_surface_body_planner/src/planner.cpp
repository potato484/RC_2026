#include "rc26_surface_body_planner/planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <sstream>
#include <unordered_set>
#include <unordered_map>

namespace rc26_surface_body_planner {

namespace {

struct QueueEntry {
    double priority = 0.0;
    double g_cost = 0.0;
    std::string node_id;
    int heading_bin = 0;

    bool operator>(const QueueEntry& other) const {
        if (priority == other.priority) {
            return g_cost > other.g_cost;
        }
        return priority > other.priority;
    }
};

std::string stateKey(const std::string& node_id, const int heading_bin) {
    return node_id + "#" + std::to_string(heading_bin);
}

int wrapHeadingBin(const int bin, const int heading_bin_count) {
    if (heading_bin_count <= 0) {
        return 0;
    }
    int wrapped = bin % heading_bin_count;
    if (wrapped < 0) {
        wrapped += heading_bin_count;
    }
    return wrapped;
}

double headingBinToYaw(const int heading_bin, const int heading_bin_count) {
    if (heading_bin_count <= 0) {
        return 0.0;
    }
    const double step = (2.0 * M_PI) / static_cast<double>(heading_bin_count);
    return normalizeAngle(-M_PI + step * static_cast<double>(heading_bin));
}

int yawToHeadingBin(const double yaw, const int heading_bin_count) {
    if (heading_bin_count <= 0) {
        return 0;
    }
    const double step = (2.0 * M_PI) / static_cast<double>(heading_bin_count);
    const double normalized = normalizeAngle(yaw) + M_PI;
    return wrapHeadingBin(static_cast<int>(std::llround(normalized / step)), heading_bin_count);
}

double pointDistance(const Pose3& from, const Pose3& to) {
    const double dx = from.x - to.x;
    const double dy = from.y - to.y;
    const double dz = from.z - to.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double heuristicScale(const SurfaceGraph& graph, const PlannerWeights& weights) {
    double best_density = std::numeric_limits<double>::infinity();
    for (const auto& edge : graph.edges) {
        const auto from_it = graph.nodes.find(edge.from);
        const auto to_it = graph.nodes.find(edge.to);
        if (from_it == graph.nodes.end() || to_it == graph.nodes.end()) {
            continue;
        }
        const double distance = std::max(1e-6, pointDistance(from_it->second.pose, to_it->second.pose));
        const double base_edge_cost =
            edge.base_cost * weights.time + std::abs(edge.height_change) * weights.height_risk;
        best_density = std::min(best_density, base_edge_cost / distance);
    }
    if (!std::isfinite(best_density)) {
        return 0.0;
    }
    return best_density * 0.99;
}

double heuristicCost(
    const SurfaceGraph& graph,
    const std::string& node_id,
    const std::string& goal_node_id,
    const double scale) {
    if (scale <= 0.0) {
        return 0.0;
    }
    const auto node_it = graph.nodes.find(node_id);
    const auto goal_it = graph.nodes.find(goal_node_id);
    if (node_it == graph.nodes.end() || goal_it == graph.nodes.end()) {
        return 0.0;
    }
    return pointDistance(node_it->second.pose, goal_it->second.pose) * scale;
}

double edgeCost(const SurfaceEdge& edge, const EdgeOverlay& overlay, const PlannerWeights& weights) {
    if (overlay.blocked) {
        return std::numeric_limits<double>::infinity();
    }
    double cost = edge.base_cost * weights.time;
    cost += std::abs(edge.height_change) * weights.height_risk;
    cost += overlay.extra_cost;
    return cost;
}

struct TransitionCheck {
    bool allowed = true;
    double heading_delta_rad = 0.0;
    std::string reason;
};

TransitionCheck checkTransition(
    const SurfaceGraph& graph,
    const std::string& node_id,
    const SurfaceEdge& edge,
    const double current_heading,
    const RobotGeometry& geometry,
    const PlannerConfig& config) {
    TransitionCheck result;
    const auto node_it = graph.nodes.find(node_id);
    if (node_it == graph.nodes.end()) {
        result.allowed = false;
        result.reason = "current node missing";
        return result;
    }

    const double next_heading = normalizeAngle(edge.nominal_yaw);
    const double heading_delta = std::abs(normalizeAngle(next_heading - current_heading));
    result.heading_delta_rad = heading_delta;

    if (config.max_heading_change_deg > 0.0 &&
        heading_delta > config.max_heading_change_deg * M_PI / 180.0) {
        result.allowed = false;
        std::ostringstream ss;
        ss << "heading change " << heading_delta * 180.0 / M_PI
           << " deg exceeds max_heading_change_deg";
        result.reason = ss.str();
        return result;
    }

    const double turn_sweep = geometry.half_length_m * std::sin(heading_delta * 0.5);
    const double required_node_clearance = turn_sweep * config.node_turn_clearance_gain;
    if (required_node_clearance > 0.0 &&
        node_it->second.center_clearance_m >= 0.0 &&
        node_it->second.center_clearance_m < required_node_clearance) {
        result.allowed = false;
        std::ostringstream ss;
        ss << "node clearance " << node_it->second.center_clearance_m
           << " m is below required turn clearance " << required_node_clearance << " m";
        result.reason = ss.str();
        return result;
    }

    const double required_edge_clearance = turn_sweep * config.edge_turn_clearance_gain;
    if (required_edge_clearance > 0.0 &&
        edge.center_clearance_m >= 0.0 &&
        edge.center_clearance_m < required_edge_clearance) {
        result.allowed = false;
        std::ostringstream ss;
        ss << "edge clearance " << edge.center_clearance_m
           << " m is below required turn clearance " << required_edge_clearance << " m";
        result.reason = ss.str();
        return result;
    }

    return result;
}

}  // namespace

double normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

PlanResult planRoute(
    const SurfaceGraph& graph,
    const PlanRequest& request,
    const std::unordered_map<std::string, NodeOverlay>& node_overlays,
    const std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    const PlannerWeights& weights,
    const RobotGeometry& geometry,
    const PlannerConfig& config) {
    PlanResult result;

    if (graph.nodes.find(request.start_node_id) == graph.nodes.end()) {
        result.failure_reason = "start node not found";
        return result;
    }
    if (graph.nodes.find(request.goal_node_id) == graph.nodes.end()) {
        result.failure_reason = "goal node not found";
        return result;
    }

    const auto goal_overlay_it = node_overlays.find(request.goal_node_id);
    if (goal_overlay_it != node_overlays.end() && goal_overlay_it->second.blocked) {
        result.failure_reason = "goal node is blocked";
        return result;
    }

    const int heading_bin_count = std::max(4, config.heading_bin_count);
    const int goal_heading_bin = yawToHeadingBin(request.goal_yaw, heading_bin_count);
    const double search_heuristic_scale = heuristicScale(graph, weights);

    std::unordered_map<std::string, double> best_cost;
    std::unordered_map<std::string, std::string> previous_state;
    std::unordered_map<std::string, std::string> previous_edge;
    std::unordered_set<std::string> start_states;
    best_cost.reserve(graph.nodes.size() * static_cast<std::size_t>(heading_bin_count));
    previous_state.reserve(graph.nodes.size() * static_cast<std::size_t>(heading_bin_count));
    previous_edge.reserve(graph.nodes.size() * static_cast<std::size_t>(heading_bin_count));
    start_states.reserve(static_cast<std::size_t>(heading_bin_count));

    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
    const double start_heuristic = heuristicCost(
        graph, request.start_node_id, request.goal_node_id, search_heuristic_scale);
    for (int heading_bin = 0; heading_bin < heading_bin_count; ++heading_bin) {
        const std::string start_key = stateKey(request.start_node_id, heading_bin);
        best_cost[start_key] = 0.0;
        start_states.insert(start_key);
        queue.push({
            start_heuristic,
            0.0,
            request.start_node_id,
            heading_bin,
        });
    }

    bool blocked_transition_captured = false;
    std::string goal_state;

    while (!queue.empty()) {
        const auto current = queue.top();
        queue.pop();

        const std::string current_key = stateKey(current.node_id, current.heading_bin);
        const auto cost_it = best_cost.find(current_key);
        if (cost_it == best_cost.end() || current.g_cost > cost_it->second) {
            continue;
        }

        if (current.node_id == request.goal_node_id &&
            (!request.require_goal_heading || current.heading_bin == goal_heading_bin)) {
            goal_state = current_key;
            break;
        }

        const auto adjacency_it = graph.adjacency.find(current.node_id);
        if (adjacency_it == graph.adjacency.end()) {
            continue;
        }

        const auto node_overlay_it = node_overlays.find(current.node_id);
        if (node_overlay_it != node_overlays.end() && node_overlay_it->second.blocked) {
            continue;
        }

        for (const std::size_t edge_index : adjacency_it->second) {
            if (edge_index >= graph.edges.size()) {
                continue;
            }
            const auto& edge = graph.edges[edge_index];
            const auto edge_overlay_it = edge_overlays.find(edge.id);
            const EdgeOverlay edge_overlay =
                edge_overlay_it != edge_overlays.end() ? edge_overlay_it->second : EdgeOverlay{};
            if (edge_overlay.blocked) {
                continue;
            }

            const auto next_node_overlay_it = node_overlays.find(edge.to);
            if (next_node_overlay_it != node_overlays.end() && next_node_overlay_it->second.blocked) {
                continue;
            }

            const double current_heading = headingBinToYaw(current.heading_bin, heading_bin_count);
            const auto transition = checkTransition(
                graph, current.node_id, edge, current_heading, geometry, config);
            if (!transition.allowed) {
                if (!blocked_transition_captured) {
                    blocked_transition_captured = true;
                    result.blocked_transition_from_node = current.node_id;
                    result.blocked_transition_edge_id = edge.id;
                    result.blocked_heading_change_deg =
                        transition.heading_delta_rad * 180.0 / M_PI;
                    result.failure_reason = transition.reason;
                }
                continue;
            }

            const int next_heading_bin = yawToHeadingBin(edge.nominal_yaw, heading_bin_count);
            const std::string next_key = stateKey(edge.to, next_heading_bin);
            double next_cost = current.g_cost + edgeCost(edge, edge_overlay, weights);
            if (next_node_overlay_it != node_overlays.end()) {
                next_cost += next_node_overlay_it->second.extra_cost;
            }
            next_cost += transition.heading_delta_rad * config.turn_cost_weight;

            const auto best_next_it = best_cost.find(next_key);
            if (best_next_it != best_cost.end() && next_cost >= best_next_it->second) {
                continue;
            }

            best_cost[next_key] = next_cost;
            previous_state[next_key] = current_key;
            previous_edge[next_key] = edge.id;
            queue.push({
                next_cost + heuristicCost(graph, edge.to, request.goal_node_id, search_heuristic_scale),
                next_cost,
                edge.to,
                next_heading_bin,
            });
        }
    }

    if (goal_state.empty()) {
        if (result.failure_reason.empty()) {
            result.failure_reason = "no heading-aware body-feasible route found";
        }
        return result;
    }

    std::vector<std::string> reversed_states;
    std::vector<std::string> reversed_edges;
    std::string cursor = goal_state;
    while (start_states.find(cursor) == start_states.end()) {
        reversed_states.push_back(cursor);
        const auto edge_it = previous_edge.find(cursor);
        const auto state_it = previous_state.find(cursor);
        if (edge_it == previous_edge.end() || state_it == previous_state.end()) {
            result.failure_reason = "failed to reconstruct heading-aware path";
            result.node_path.clear();
            result.edge_path.clear();
            result.heading_path.clear();
            return result;
        }
        reversed_edges.push_back(edge_it->second);
        cursor = state_it->second;
    }
    reversed_states.push_back(cursor);

    std::reverse(reversed_states.begin(), reversed_states.end());
    std::reverse(reversed_edges.begin(), reversed_edges.end());

    result.node_path.reserve(reversed_states.size());
    result.heading_path.reserve(reversed_states.size());
    for (const auto& state : reversed_states) {
        const auto separator = state.rfind('#');
        result.node_path.push_back(state.substr(0, separator));
        const int heading_bin = std::stoi(state.substr(separator + 1));
        result.heading_path.push_back(headingBinToYaw(heading_bin, heading_bin_count));
    }
    result.edge_path = std::move(reversed_edges);
    result.total_cost = best_cost[goal_state];
    result.success = true;
    result.failure_reason.clear();
    return result;
}

}  // namespace rc26_surface_body_planner
