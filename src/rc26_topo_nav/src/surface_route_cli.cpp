#include "rc26_topo_nav/body_planning.hpp"
#include "rc26_topo_nav/graph_loader.hpp"
#include "rc26_topo_nav/surface_route.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

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

bool parsePose4(const std::string& raw, Pose3& pose) {
    std::stringstream ss(raw);
    std::string item;
    double values[4] = {0.0, 0.0, 0.0, 0.0};
    size_t index = 0;
    while (std::getline(ss, item, ',')) {
        if (index >= 4) {
            return false;
        }
        char* end = nullptr;
        values[index] = std::strtod(item.c_str(), &end);
        if (end == nullptr || *end != '\0') {
            return false;
        }
        ++index;
    }
    if (index < 3) {
        return false;
    }
    pose.x = values[0];
    pose.y = values[1];
    pose.z = values[2];
    pose.yaw = values[3];
    return true;
}

struct ParsedArgs {
    std::string graph;
    std::string robot_geometry_file;
    std::string robot_geometry_profile = "compact";
    Pose3 start;
    Pose3 goal;
    double projection_radius_m = 0.30;
    SurfaceBodyPlanningConfig body_planning;
    SurfacePlannerOptions planner_options{SurfacePlannerBackend::BODY_PLANNER, {}, std::nullopt};
    std::vector<std::string> blocked_nodes;
    std::vector<std::string> blocked_edges;
};

bool parseArgs(int argc, char** argv, ParsedArgs& args, std::string& error) {
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
            if (value == nullptr) {
                return false;
            }
            args.graph = value;
        } else if (flag == "--robot-geometry-file") {
            const char* value = requireValue(flag);
            if (value == nullptr) {
                return false;
            }
            args.robot_geometry_file = value;
        } else if (flag == "--robot-geometry-profile") {
            const char* value = requireValue(flag);
            if (value == nullptr) {
                return false;
            }
            args.robot_geometry_profile = value;
        } else if (flag == "--start-pose") {
            const char* value = requireValue(flag);
            if (value == nullptr || !parsePose4(value, args.start)) {
                error = "Invalid --start-pose, expected x,y,z,yaw";
                return false;
            }
        } else if (flag == "--goal-pose") {
            const char* value = requireValue(flag);
            if (value == nullptr || !parsePose4(value, args.goal)) {
                error = "Invalid --goal-pose, expected x,y,z,yaw";
                return false;
            }
        } else if (flag == "--projection-radius") {
            const char* value = requireValue(flag);
            if (value == nullptr) {
                return false;
            }
            char* end = nullptr;
            args.projection_radius_m = std::strtod(value, &end);
            if (end == nullptr || *end != '\0') {
                error = "Invalid --projection-radius";
                return false;
            }
        } else if (flag == "--disable-body-planning") {
            args.body_planning.enabled = false;
        } else if (flag == "--planner-backend") {
            const char* value = requireValue(flag);
            if (value == nullptr) {
                return false;
            }
            const std::string backend = value;
            if (backend == "legacy") {
                args.planner_options.backend = SurfacePlannerBackend::LEGACY;
            } else if (backend == "body_planner") {
                args.planner_options.backend = SurfacePlannerBackend::BODY_PLANNER;
            } else {
                error = "Invalid --planner-backend, expected legacy|body_planner";
                return false;
            }
        } else if (flag == "--body-clearance-margin") {
            const char* value = requireValue(flag);
            if (value == nullptr) {
                return false;
            }
            char* end = nullptr;
            args.body_planning.clearance_margin_m = std::strtod(value, &end);
            if (end == nullptr || *end != '\0') {
                error = "Invalid --body-clearance-margin";
                return false;
            }
        } else if (flag == "--max-surface-pitch-deg") {
            const char* value = requireValue(flag);
            if (value == nullptr) {
                return false;
            }
            char* end = nullptr;
            args.body_planning.max_surface_pitch_deg = std::strtod(value, &end);
            if (end == nullptr || *end != '\0') {
                error = "Invalid --max-surface-pitch-deg";
                return false;
            }
        } else if (flag == "--max-edge-slope-deg") {
            const char* value = requireValue(flag);
            if (value == nullptr) {
                return false;
            }
            char* end = nullptr;
            args.body_planning.max_edge_slope_deg = std::strtod(value, &end);
            if (end == nullptr || *end != '\0') {
                error = "Invalid --max-edge-slope-deg";
                return false;
            }
        } else if (flag == "--max-step-height-m") {
            const char* value = requireValue(flag);
            if (value == nullptr) {
                return false;
            }
            char* end = nullptr;
            args.body_planning.max_step_height_m = std::strtod(value, &end);
            if (end == nullptr || *end != '\0') {
                error = "Invalid --max-step-height-m";
                return false;
            }
        } else if (flag == "--heading-bin-count") {
            const char* value = requireValue(flag);
            if (value == nullptr) {
                return false;
            }
            char* end = nullptr;
            args.planner_options.body_planner.heading_bin_count =
                static_cast<int>(std::strtol(value, &end, 10));
            if (end == nullptr || *end != '\0') {
                error = "Invalid --heading-bin-count";
                return false;
            }
        } else if (flag == "--max-heading-change-deg") {
            const char* value = requireValue(flag);
            if (value == nullptr) {
                return false;
            }
            char* end = nullptr;
            args.planner_options.body_planner.max_heading_change_deg = std::strtod(value, &end);
            if (end == nullptr || *end != '\0') {
                error = "Invalid --max-heading-change-deg";
                return false;
            }
        } else if (flag == "--turn-cost-weight") {
            const char* value = requireValue(flag);
            if (value == nullptr) {
                return false;
            }
            char* end = nullptr;
            args.planner_options.body_planner.turn_cost_weight = std::strtod(value, &end);
            if (end == nullptr || *end != '\0') {
                error = "Invalid --turn-cost-weight";
                return false;
            }
        } else if (flag == "--node-turn-clearance-gain") {
            const char* value = requireValue(flag);
            if (value == nullptr) {
                return false;
            }
            char* end = nullptr;
            args.planner_options.body_planner.node_turn_clearance_gain = std::strtod(value, &end);
            if (end == nullptr || *end != '\0') {
                error = "Invalid --node-turn-clearance-gain";
                return false;
            }
        } else if (flag == "--edge-turn-clearance-gain") {
            const char* value = requireValue(flag);
            if (value == nullptr) {
                return false;
            }
            char* end = nullptr;
            args.planner_options.body_planner.edge_turn_clearance_gain = std::strtod(value, &end);
            if (end == nullptr || *end != '\0') {
                error = "Invalid --edge-turn-clearance-gain";
                return false;
            }
        } else if (flag == "--blocked-node") {
            const char* value = requireValue(flag);
            if (value == nullptr || std::string(value).empty()) {
                error = "Invalid --blocked-node";
                return false;
            }
            args.blocked_nodes.emplace_back(value);
        } else if (flag == "--blocked-edge") {
            const char* value = requireValue(flag);
            if (value == nullptr || std::string(value).empty()) {
                error = "Invalid --blocked-edge";
                return false;
            }
            args.blocked_edges.emplace_back(value);
        } else {
            error = "Unknown flag: " + flag;
            return false;
        }
    }

    if (args.graph.empty()) {
        error = "Missing required flag --graph";
        return false;
    }
    const bool geometry_required =
        args.body_planning.enabled ||
        args.planner_options.backend == SurfacePlannerBackend::BODY_PLANNER;
    if (args.robot_geometry_file.empty() && geometry_required) {
        try {
            const auto pkg_dir = ament_index_cpp::get_package_share_directory("rc26_robot_geometry");
            args.robot_geometry_file = pkg_dir + "/config/r2_body_geometry.yaml";
        } catch (...) {
            args.body_planning.enabled = false;
            args.planner_options.backend = SurfacePlannerBackend::LEGACY;
        }
    }
    return true;
}

void printPose(std::ostream& out, const Pose3& pose) {
    out << "{"
        << "\"x\":" << pose.x << ","
        << "\"y\":" << pose.y << ","
        << "\"z\":" << pose.z << ","
        << "\"yaw\":" << pose.yaw
        << "}";
}

void printPath(std::ostream& out, const nav_msgs::msg::Path& path) {
    out << "[";
    for (size_t index = 0; index < path.poses.size(); ++index) {
        if (index > 0) {
            out << ",";
        }
        const auto& pose = path.poses[index].pose.position;
        out << "{"
            << "\"x\":" << pose.x << ","
            << "\"y\":" << pose.y << ","
            << "\"z\":" << pose.z << ","
            << "\"yaw\":0.0"
            << "}";
    }
    out << "]";
}

void printSegments(std::ostream& out, const std::vector<SurfacePlanSegment>& segments) {
    out << "[";
    for (size_t index = 0; index < segments.size(); ++index) {
        if (index > 0) {
            out << ",";
        }
        const auto& segment = segments[index];
        out << "{"
            << "\"segment_id\":" << quoted(segment.id) << ","
            << "\"from_node_id\":" << quoted(segment.from_node_id) << ","
            << "\"to_node_id\":" << quoted(segment.to_node_id) << ","
            << "\"motion_type\":" << quoted(segment.motion_type) << ","
            << "\"required_mode\":" << quoted(segment.required_mode) << ","
            << "\"point_count\":" << segment.corridor.poses.size()
            << "}";
    }
    out << "]";
}

bool applySyntheticRuntimeOverlays(
    const FieldGraph& graph,
    const ParsedArgs& args,
    std::unordered_map<std::string, NodeOverlay>& node_overlays,
    std::unordered_map<std::string, EdgeOverlay>& edge_overlays,
    std::string& error) {
    for (const auto& node_id : args.blocked_nodes) {
        if (graph.nodes.find(node_id) == graph.nodes.end()) {
            error = "Unknown --blocked-node id: " + node_id;
            return false;
        }
        auto& overlay = node_overlays[node_id];
        overlay.state = NodeState::BLOCKED;
        overlay.extra_cost = std::max(overlay.extra_cost, 1000.0);
    }

    for (const auto& edge_id : args.blocked_edges) {
        const auto edge_it = std::find_if(
            graph.edges.begin(),
            graph.edges.end(),
            [&](const GraphEdge& edge) { return edge.id == edge_id; });
        if (edge_it == graph.edges.end()) {
            error = "Unknown --blocked-edge id: " + edge_id;
            return false;
        }
        auto& overlay = edge_overlays[edge_id];
        overlay.state = EdgeState::BLOCKED;
        overlay.extra_cost = std::max(overlay.extra_cost, 1000.0);
    }

    return true;
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
    std::unordered_map<std::string, NodeOverlay> runtime_node_overlays;
    std::unordered_map<std::string, EdgeOverlay> runtime_edge_overlays;
    if (!applySyntheticRuntimeOverlays(
            load_result.graph, args, runtime_node_overlays, runtime_edge_overlays, error)) {
        std::cerr << "[ERROR] " << error << "\n";
        return 1;
    }

    auto node_overlays = runtime_node_overlays;
    auto edge_overlays = runtime_edge_overlays;
    SurfaceBodyPlanningStats body_planning_stats;
    const bool geometry_required =
        args.body_planning.enabled ||
        args.planner_options.backend == SurfacePlannerBackend::BODY_PLANNER;
    if (geometry_required) {
        std::string geometry_error;
        const auto geometry = loadRobotGeometryProfile(
            args.robot_geometry_file, args.robot_geometry_profile, geometry_error);
        if (!geometry) {
            std::cerr << "[ERROR] Failed to load robot geometry: " << geometry_error << "\n";
            return 1;
        }
        args.planner_options.geometry = SurfacePlannerGeometry{
            geometry->half_length_m,
            geometry->half_width_m,
        };
        if (args.body_planning.enabled) {
            std::string body_planning_error;
            body_planning_stats = applySurfaceBodyPlanningOverlays(
                load_result.graph,
                *geometry,
                args.body_planning,
                node_overlays,
                edge_overlays,
                &body_planning_error);
            if (!body_planning_error.empty()) {
                std::cerr << "[ERROR] " << body_planning_error << "\n";
                return 1;
            }
        }
    }
    auto plan = planSurfaceRoute(
        load_result.graph,
        args.start,
        args.goal,
        node_overlays,
        edge_overlays,
        weights,
        args.projection_radius_m,
        rclcpp::Time(0),
        args.planner_options,
        "map");
    if (!plan.success) {
        const auto failure_analysis = classifySurfacePlanFailure(
            load_result.graph,
            args.start,
            args.goal,
            runtime_node_overlays,
            runtime_edge_overlays,
            runtime_node_overlays,
            runtime_edge_overlays,
            node_overlays,
            edge_overlays,
            weights,
            args.projection_radius_m,
            plan,
            "");
        if (!failure_analysis.failure_code.empty()) {
            plan.failure_code = failure_analysis.failure_code;
        }
        if (!failure_analysis.failure_reason.empty()) {
            plan.failure_reason = failure_analysis.failure_reason;
        }
        if (!plan.projected_start.success && failure_analysis.best_projected_start.success) {
            plan.projected_start = failure_analysis.best_projected_start;
        }
        if (!plan.projected_goal.success && failure_analysis.best_projected_goal.success) {
            plan.projected_goal = failure_analysis.best_projected_goal;
        }
    }

    std::cout << "{"
              << "\"success\":" << (plan.success ? "true" : "false") << ","
              << "\"failure_code\":" << quoted(plan.failure_code) << ","
              << "\"failure_reason\":" << quoted(plan.failure_reason) << ","
              << "\"projected_start_node_id\":" << quoted(plan.projected_start.node_id) << ","
              << "\"projected_goal_node_id\":" << quoted(plan.projected_goal.node_id) << ","
              << "\"projected_start\":";
    printPose(std::cout, plan.projected_start.pose);
    std::cout << ",\"projected_goal\":";
    printPose(std::cout, plan.projected_goal.pose);
    std::cout << ",\"node_path\":[";
    for (size_t index = 0; index < plan.plan.node_path.size(); ++index) {
        if (index > 0) {
            std::cout << ",";
        }
        std::cout << quoted(plan.plan.node_path[index]);
    }
    std::cout << "],\"path_points\":";
    printPath(std::cout, plan.planned_path);
    std::cout << ",\"segments\":";
    printSegments(std::cout, plan.segments);
    std::cout << ",\"planner_backend\":" << quoted(plan.planner_backend);
    std::cout << ",\"body_planning\":{"
              << "\"enabled\":" << (args.body_planning.enabled ? "true" : "false") << ","
              << "\"annotations_available\":" << (body_planning_stats.annotations_available ? "true" : "false") << ","
              << "\"penalized_nodes_clearance\":" << body_planning_stats.penalized_nodes_clearance << ","
              << "\"blocked_nodes_pitch\":" << body_planning_stats.blocked_nodes_pitch << ","
              << "\"blocked_edges_clearance\":" << body_planning_stats.blocked_edges_clearance << ","
              << "\"blocked_edges_slope\":" << body_planning_stats.blocked_edges_slope << ","
              << "\"blocked_edges_step\":" << body_planning_stats.blocked_edges_step
              << "}";
    std::cout << ",\"runtime_overlay\":{"
              << "\"blocked_node_count\":" << args.blocked_nodes.size() << ","
              << "\"blocked_edge_count\":" << args.blocked_edges.size()
              << "}";
    std::cout << ",\"timing_ms\":{"
              << "\"projection\":" << plan.projection_ms << ","
              << "\"routePlanning\":" << plan.route_planning_ms << ","
              << "\"pathExpand\":" << plan.path_expand_ms << ","
              << "\"segmentBuild\":" << plan.segment_build_ms << ","
              << "\"completePlanning\":" << plan.complete_planning_ms
              << "}";
    std::cout << "}\n";
    return plan.success ? 0 : 2;
}
