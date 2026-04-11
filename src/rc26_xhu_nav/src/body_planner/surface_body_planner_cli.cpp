#include "rc26_xhu_nav/body_planner/graph_loader.hpp"
#include "rc26_xhu_nav/body_planner/planner.hpp"

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

namespace rc26_xhu_nav::body_planner {
namespace {

struct ParsedArgs {
    std::string graph_file;
    std::string start_node_id;
    std::string goal_node_id;
    double start_yaw = 0.0;
    double goal_yaw = 0.0;
    bool require_goal_heading = false;
    RobotGeometry geometry;
    PlannerConfig config;
};

bool parseDouble(const char* raw, double& value) {
    char* end = nullptr;
    value = std::strtod(raw, &end);
    return end != nullptr && *end == '\0';
}

bool parseInt(const char* raw, int& value) {
    char* end = nullptr;
    value = static_cast<int>(std::strtol(raw, &end, 10));
    return end != nullptr && *end == '\0';
}

bool parseArgs(int argc, char** argv, ParsedArgs& args, std::string& error) {
    for (int index = 1; index < argc; ++index) {
        const std::string flag = argv[index];
        auto requireValue = [&](const std::string& label) -> const char* {
            if (index + 1 >= argc) {
                error = "missing value for " + label;
                return nullptr;
            }
            ++index;
            return argv[index];
        };

        if (flag == "--graph") {
            const char* value = requireValue(flag);
            if (value == nullptr) {
                return false;
            }
            args.graph_file = value;
        } else if (flag == "--start-node") {
            const char* value = requireValue(flag);
            if (value == nullptr) {
                return false;
            }
            args.start_node_id = value;
        } else if (flag == "--goal-node") {
            const char* value = requireValue(flag);
            if (value == nullptr) {
                return false;
            }
            args.goal_node_id = value;
        } else if (flag == "--start-yaw") {
            const char* value = requireValue(flag);
            if (value == nullptr || !parseDouble(value, args.start_yaw)) {
                error = "invalid --start-yaw";
                return false;
            }
        } else if (flag == "--goal-yaw") {
            const char* value = requireValue(flag);
            if (value == nullptr || !parseDouble(value, args.goal_yaw)) {
                error = "invalid --goal-yaw";
                return false;
            }
        } else if (flag == "--require-goal-heading") {
            args.require_goal_heading = true;
        } else if (flag == "--half-length") {
            const char* value = requireValue(flag);
            if (value == nullptr || !parseDouble(value, args.geometry.half_length_m)) {
                error = "invalid --half-length";
                return false;
            }
        } else if (flag == "--half-width") {
            const char* value = requireValue(flag);
            if (value == nullptr || !parseDouble(value, args.geometry.half_width_m)) {
                error = "invalid --half-width";
                return false;
            }
        } else if (flag == "--heading-bin-count") {
            const char* value = requireValue(flag);
            if (value == nullptr || !parseInt(value, args.config.heading_bin_count)) {
                error = "invalid --heading-bin-count";
                return false;
            }
        } else if (flag == "--max-heading-change-deg") {
            const char* value = requireValue(flag);
            if (value == nullptr || !parseDouble(value, args.config.max_heading_change_deg)) {
                error = "invalid --max-heading-change-deg";
                return false;
            }
        } else if (flag == "--turn-cost-weight") {
            const char* value = requireValue(flag);
            if (value == nullptr || !parseDouble(value, args.config.turn_cost_weight)) {
                error = "invalid --turn-cost-weight";
                return false;
            }
        } else if (flag == "--node-turn-clearance-gain") {
            const char* value = requireValue(flag);
            if (value == nullptr || !parseDouble(value, args.config.node_turn_clearance_gain)) {
                error = "invalid --node-turn-clearance-gain";
                return false;
            }
        } else if (flag == "--edge-turn-clearance-gain") {
            const char* value = requireValue(flag);
            if (value == nullptr || !parseDouble(value, args.config.edge_turn_clearance_gain)) {
                error = "invalid --edge-turn-clearance-gain";
                return false;
            }
        } else {
            error = "unknown flag: " + flag;
            return false;
        }
    }

    if (args.graph_file.empty() || args.start_node_id.empty() || args.goal_node_id.empty()) {
        error = "required: --graph --start-node --goal-node";
        return false;
    }
    return true;
}

void printStringArray(const std::vector<std::string>& values) {
    std::cout << "[";
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index > 0) {
            std::cout << ",";
        }
        std::cout << "\"" << values[index] << "\"";
    }
    std::cout << "]";
}

void printDoubleArray(const std::vector<double>& values) {
    std::cout << "[";
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (index > 0) {
            std::cout << ",";
        }
        std::cout << values[index];
    }
    std::cout << "]";
}

}  // namespace
}  // namespace rc26_xhu_nav::body_planner

int main(int argc, char** argv) {
    using namespace rc26_xhu_nav::body_planner;

    ParsedArgs args;
    std::string error;
    if (!parseArgs(argc, argv, args, error)) {
        std::cerr << error << std::endl;
        return 1;
    }

    const auto load_result = loadSurfaceGraph(args.graph_file);
    if (!load_result.success) {
        std::cerr << load_result.error << std::endl;
        return 2;
    }

    const auto validation = validateSurfaceGraph(load_result.graph);
    if (!validation.valid) {
        std::cerr << validation.errors.front() << std::endl;
        return 3;
    }

    PlanRequest request;
    request.start_node_id = args.start_node_id;
    request.goal_node_id = args.goal_node_id;
    request.start_yaw = args.start_yaw;
    request.goal_yaw = args.goal_yaw;
    request.require_goal_heading = args.require_goal_heading;

    const PlannerWeights weights;
    const std::unordered_map<std::string, NodeOverlay> node_overlays;
    const std::unordered_map<std::string, EdgeOverlay> edge_overlays;
    const auto plan = planRoute(
        load_result.graph,
        request,
        node_overlays,
        edge_overlays,
        weights,
        args.geometry,
        args.config);

    std::cout << "{"
              << "\"success\":" << (plan.success ? "true" : "false") << ","
              << "\"failure_reason\":\"" << plan.failure_reason << "\","
              << "\"node_path\":";
    printStringArray(plan.node_path);
    std::cout << ",\"edge_path\":";
    printStringArray(plan.edge_path);
    std::cout << ",\"heading_path\":";
    printDoubleArray(plan.heading_path);
    std::cout << ",\"total_cost\":" << plan.total_cost
              << "}" << std::endl;
    return plan.success ? 0 : 4;
}
