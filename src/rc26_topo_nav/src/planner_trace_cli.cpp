#include "rc26_topo_nav/graph_loader.hpp"
#include "rc26_topo_nav/planner.hpp"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
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

void printFrames(std::ostream& out, const PlanTraceResult& trace) {
    out << "[";
    for (size_t index = 0; index < trace.frames.size(); ++index) {
        const auto& frame = trace.frames[index];
        if (index > 0) {
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
    const std::string& goal_value) {
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
    out << ",\"frames\":";
    printFrames(out, trace);
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

    printTraceJson(std::cout, load_result.graph, trace, goal_kind, goal_value);
    std::cout << "\n";
    return trace.result.success ? 0 : 2;
}
