#include "rc26_topo_nav/graph_loader.hpp"
#include "rc26_topo_nav/planner.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <cmath>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace rc26_topo_nav {
namespace {

std::string escapeJson(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size() + 8);
    for (const unsigned char ch : value) {
        switch (ch) {
            case '\\':
                escaped += "\\\\";
                break;
            case '"':
                escaped += "\\\"";
                break;
            case '\b':
                escaped += "\\b";
                break;
            case '\f':
                escaped += "\\f";
                break;
            case '\n':
                escaped += "\\n";
                break;
            case '\r':
                escaped += "\\r";
                break;
            case '\t':
                escaped += "\\t";
                break;
            default:
                if (ch < 0x20) {
                    char buffer[7];
                    std::snprintf(buffer, sizeof(buffer), "\\u%04x", ch);
                    escaped += buffer;
                } else {
                    escaped.push_back(static_cast<char>(ch));
                }
        }
    }
    return escaped;
}

std::string quoted(const std::string& value) {
    return "\"" + escapeJson(value) + "\"";
}

bool parseTaggedValue(
    const std::string& raw,
    std::string& key_out,
    double& value_out) {
    const auto split = raw.find(':');
    if (split == std::string::npos || split == 0 || split == raw.size() - 1) {
        return false;
    }
    key_out = raw.substr(0, split);
    char* end = nullptr;
    value_out = std::strtod(raw.c_str() + static_cast<long>(split) + 1L, &end);
    return end != nullptr && *end == '\0';
}

struct ParsedArgs {
    std::string graph;
    std::string start;
    std::string goal_node;
    std::string goal_task;
    std::string goal_route;
    size_t max_frames = 0;
    PlannerTraceOptions options;
    std::unordered_map<std::string, NodeOverlay> node_overlays;
    std::unordered_map<std::string, EdgeOverlay> edge_overlays;
};

bool parseArgs(const int argc, char** argv, ParsedArgs& args, std::string& error) {
    for (int index = 1; index < argc; ++index) {
        const std::string flag = argv[index];
        auto requireValue = [&](const std::string& label) -> const char* {
            if (index + 1 >= argc) {
                error = "Missing value for " + label;
                return nullptr;
            }
            ++index;
            return argv[index];
        };

        if (flag == "--graph") {
            const char* value = requireValue(flag);
            if (value == nullptr) return false;
            args.graph = value;
        } else if (flag == "--start") {
            const char* value = requireValue(flag);
            if (value == nullptr) return false;
            args.start = value;
        } else if (flag == "--goal-node") {
            const char* value = requireValue(flag);
            if (value == nullptr) return false;
            args.goal_node = value;
        } else if (flag == "--goal-task") {
            const char* value = requireValue(flag);
            if (value == nullptr) return false;
            args.goal_task = value;
        } else if (flag == "--goal-route") {
            const char* value = requireValue(flag);
            if (value == nullptr) return false;
            args.goal_route = value;
        } else if (flag == "--blocked-node") {
            const char* value = requireValue(flag);
            if (value == nullptr) return false;
            args.node_overlays[value] = {NodeState::BLOCKED, 1000.0};
        } else if (flag == "--blocked-edge") {
            const char* value = requireValue(flag);
            if (value == nullptr) return false;
            args.edge_overlays[value] = {EdgeState::BLOCKED, 0.0};
        } else if (flag == "--slow-edge") {
            const char* value = requireValue(flag);
            if (value == nullptr) return false;
            args.edge_overlays[value] = {EdgeState::SLOW_ONLY, 0.0};
        } else if (flag == "--confirm-edge") {
            const char* value = requireValue(flag);
            if (value == nullptr) return false;
            args.edge_overlays[value] = {EdgeState::CONFIRM_REQUIRED, 0.0};
        } else if (flag == "--node-extra-cost") {
            const char* value = requireValue(flag);
            if (value == nullptr) return false;
            std::string key;
            double extra_cost = 0.0;
            if (!parseTaggedValue(value, key, extra_cost)) {
                error = "Invalid node cost override: " + std::string(value);
                return false;
            }
            auto& overlay = args.node_overlays[key];
            overlay.extra_cost = extra_cost;
        } else if (flag == "--edge-extra-cost") {
            const char* value = requireValue(flag);
            if (value == nullptr) return false;
            std::string key;
            double extra_cost = 0.0;
            if (!parseTaggedValue(value, key, extra_cost)) {
                error = "Invalid edge cost override: " + std::string(value);
                return false;
            }
            auto& overlay = args.edge_overlays[key];
            overlay.extra_cost = extra_cost;
        } else if (flag == "--heuristic-scale") {
            const char* value = requireValue(flag);
            if (value == nullptr) return false;
            char* end = nullptr;
            args.options.heuristic_scale = std::strtod(value, &end);
            if (end == nullptr || *end != '\0') {
                error = "Invalid heuristic scale: " + std::string(value);
                return false;
            }
        } else if (flag == "--max-frames") {
            const char* value = requireValue(flag);
            if (value == nullptr) return false;
            char* end = nullptr;
            const auto parsed = std::strtoul(value, &end, 10);
            if (end == nullptr || *end != '\0') {
                error = "Invalid max frame count: " + std::string(value);
                return false;
            }
            args.max_frames = static_cast<size_t>(parsed);
        } else {
            error = "Unknown flag: " + flag;
            return false;
        }
    }

    if (args.graph.empty()) {
        error = "Missing required flag --graph";
        return false;
    }
    if (args.start.empty()) {
        error = "Missing required flag --start";
        return false;
    }

    int goal_count = 0;
    goal_count += !args.goal_node.empty() ? 1 : 0;
    goal_count += !args.goal_task.empty() ? 1 : 0;
    goal_count += !args.goal_route.empty() ? 1 : 0;
    if (goal_count != 1) {
        error = "Exactly one of --goal-node, --goal-task, or --goal-route must be provided";
        return false;
    }
    return true;
}

void printStringArray(std::ostream& out, const std::vector<std::string>& values) {
    out << "[";
    for (size_t index = 0; index < values.size(); ++index) {
        if (index > 0) {
            out << ",";
        }
        out << quoted(values[index]);
    }
    out << "]";
}

void printFrontier(std::ostream& out, const std::vector<TraceFrontierEntry>& entries) {
    out << "[";
    for (size_t index = 0; index < entries.size(); ++index) {
        if (index > 0) {
            out << ",";
        }
        out << "{"
            << "\"node_id\":" << quoted(entries[index].node_id) << ","
            << "\"g_cost\":" << entries[index].g_cost << ","
            << "\"f_cost\":" << entries[index].f_cost
            << "}";
    }
    out << "]";
}

void printPose(std::ostream& out, const Pose3& pose) {
    out << "{"
        << "\"x\":" << pose.x << ","
        << "\"y\":" << pose.y << ","
        << "\"z\":" << pose.z << ","
        << "\"yaw\":" << pose.yaw
        << "}";
}

std::vector<size_t> buildFrameIndices(const size_t total_frames, const size_t max_frames) {
    std::vector<size_t> indices;
    if (total_frames == 0) {
        return indices;
    }
    if (max_frames == 0 || total_frames <= max_frames) {
        indices.reserve(total_frames);
        for (size_t index = 0; index < total_frames; ++index) {
            indices.push_back(index);
        }
        return indices;
    }

    indices.reserve(max_frames);
    const double step = static_cast<double>(total_frames - 1) / static_cast<double>(max_frames - 1);
    for (size_t sample_index = 0; sample_index < max_frames; ++sample_index) {
        size_t frame_index = static_cast<size_t>(std::llround(step * static_cast<double>(sample_index)));
        if (!indices.empty() && frame_index <= indices.back()) {
            frame_index = std::min(total_frames - 1, indices.back() + 1);
        }
        indices.push_back(frame_index);
    }
    indices.back() = total_frames - 1;
    return indices;
}

std::vector<std::string> collectReferencedNodeIds(
    const PlanTraceResult& trace,
    const std::vector<size_t>& frame_indices) {
    std::vector<std::string> node_ids;
    std::unordered_set<std::string> seen;
    auto append = [&](const std::string& node_id) {
        if (node_id.empty()) {
            return;
        }
        if (seen.insert(node_id).second) {
            node_ids.push_back(node_id);
        }
    };

    for (const auto& node_id : trace.result.node_path) {
        append(node_id);
    }
    for (const size_t frame_index : frame_indices) {
        const auto& frame = trace.frames[frame_index];
        append(frame.node_id);
        append(frame.from_node);
        for (const auto& frontier_entry : frame.frontier) {
            append(frontier_entry.node_id);
        }
        for (const auto& node_id : frame.expanded_nodes) {
            append(node_id);
        }
        for (const auto& node_id : frame.best_path) {
            append(node_id);
        }
    }
    std::sort(node_ids.begin(), node_ids.end());
    return node_ids;
}

void printNodePoses(
    std::ostream& out,
    const FieldGraph& graph,
    const std::vector<std::string>& node_ids) {
    out << "{";
    bool first = true;
    for (const auto& node_id : node_ids) {
        const auto node_it = graph.nodes.find(node_id);
        if (node_it == graph.nodes.end()) {
            continue;
        }
        if (!first) {
            out << ",";
        }
        first = false;
        out << quoted(node_id) << ":";
        printPose(out, node_it->second.pose);
    }
    out << "}";
}

void printFrames(std::ostream& out, const PlanTraceResult& trace, const std::vector<size_t>& frame_indices) {
    out << "[";
    for (size_t emitted_index = 0; emitted_index < frame_indices.size(); ++emitted_index) {
        const auto& frame = trace.frames[frame_indices[emitted_index]];
        if (emitted_index > 0) {
            out << ",";
        }
        out << "{"
            << "\"event\":" << quoted(traceEventTypeName(frame.event)) << ","
            << "\"step_index\":" << frame.step_index << ","
            << "\"node_id\":" << quoted(frame.node_id) << ","
            << "\"from_node\":" << quoted(frame.from_node) << ","
            << "\"edge_id\":" << quoted(frame.edge_id) << ","
            << "\"g_cost\":" << frame.g_cost << ","
            << "\"f_cost\":" << frame.f_cost << ","
            << "\"step_cost\":" << frame.step_cost << ","
            << "\"message\":" << quoted(frame.message) << ","
            << "\"frontier\":";
        printFrontier(out, frame.frontier);
        out << ",\"best_path\":";
        printStringArray(out, frame.best_path);
        out << ",\"expanded_nodes\":";
        printStringArray(out, frame.expanded_nodes);
        out << "}";
    }
    out << "]";
}

void printCandidateResults(std::ostream& out, const std::vector<TaskCandidateResult>& results) {
    out << "[";
    for (size_t index = 0; index < results.size(); ++index) {
        const auto& result = results[index];
        if (index > 0) {
            out << ",";
        }
        out << "{"
            << "\"candidate_node\":" << quoted(result.candidate_node) << ","
            << "\"success\":" << (result.success ? "true" : "false") << ","
            << "\"total_cost\":" << (result.success ? std::to_string(result.total_cost) : "null") << ","
            << "\"failure_reason\":" << quoted(result.failure_reason)
            << "}";
    }
    out << "]";
}

void printTraceJson(
    std::ostream& out,
    const FieldGraph& graph,
    const PlanTraceResult& trace,
    const std::string& goal_kind,
    const std::string& goal_value,
    const size_t max_frames) {
    const auto frame_indices = buildFrameIndices(trace.frames.size(), max_frames);
    const auto referenced_node_ids = collectReferencedNodeIds(trace, frame_indices);
    const bool frames_sampled = frame_indices.size() != trace.frames.size();
    out << "{"
        << "\"success\":" << (trace.result.success ? "true" : "false") << ","
        << "\"goal_kind\":" << quoted(goal_kind) << ","
        << "\"goal_value\":" << quoted(goal_value) << ","
        << "\"team\":" << quoted(graph.team) << ","
        << "\"schema_version\":" << quoted(graph.schema_version) << ","
        << "\"total_cost\":" << trace.result.total_cost << ","
        << "\"failure_reason\":" << quoted(trace.result.failure_reason) << ","
        << "\"selected_candidate\":" << quoted(trace.selected_candidate) << ","
        << "\"node_path\":";
    printStringArray(out, trace.result.node_path);
    out << ",\"edge_path\":[";
    for (size_t index = 0; index < trace.result.edge_indices.size(); ++index) {
        if (index > 0) {
            out << ",";
        }
        out << quoted(graph.edges[trace.result.edge_indices[index]].id);
    }
    out << "],\"candidate_results\":";
    printCandidateResults(out, trace.candidate_results);
    out << ",\"node_poses\":";
    printNodePoses(out, graph, referenced_node_ids);
    out << ",\"timing_ms\":{";
    out << "\"planning\":" << trace.planning_ms;
    out << "}";
    out << ",\"frame_count_total\":" << trace.frames.size();
    out << ",\"frame_count_emitted\":" << frame_indices.size();
    out << ",\"frames_sampled\":" << (frames_sampled ? "true" : "false");
    out << ",\"frames\":";
    printFrames(out, trace, frame_indices);
    out << "}";
}

}  // namespace
}  // namespace rc26_topo_nav

int main(int argc, char** argv) {
    using namespace rc26_topo_nav;

    ParsedArgs args;
    std::string error;
    if (!parseArgs(argc, argv, args, error)) {
        std::cerr << "[ERROR] " << error << "\n";
        return 1;
    }

    const auto load_result = loadFieldGraph(args.graph);
    if (!load_result.success) {
        std::cerr << "[ERROR] Failed to load graph: " << load_result.error << "\n";
        return 1;
    }

    const auto validation = validateGraph(load_result.graph);
    if (!validation.valid) {
        std::cerr << "[ERROR] Graph validation failed\n";
        for (const auto& item : validation.errors) {
            std::cerr << "  - " << item << "\n";
        }
        return 1;
    }

    PlannerWeights weights;
    PlanTraceResult trace;
    std::string goal_kind;
    std::string goal_value;
    if (!args.goal_node.empty()) {
        goal_kind = "node";
        goal_value = args.goal_node;
        trace = planRouteTrace(
            load_result.graph,
            args.start,
            args.goal_node,
            args.node_overlays,
            args.edge_overlays,
            weights,
            args.options);
    } else if (!args.goal_task.empty()) {
        goal_kind = "task";
        goal_value = args.goal_task;
        trace = planToTaskTrace(
            load_result.graph,
            args.start,
            args.goal_task,
            args.node_overlays,
            args.edge_overlays,
            weights,
            args.options);
    } else {
        goal_kind = "route";
        goal_value = args.goal_route;
        trace = planRouteTagTrace(
            load_result.graph,
            args.start,
            args.goal_route,
            args.node_overlays,
            args.edge_overlays,
            weights,
            args.options);
    }

    printTraceJson(std::cout, load_result.graph, trace, goal_kind, goal_value, args.max_frames);
    std::cout << "\n";
    return trace.result.success ? 0 : 2;
}
