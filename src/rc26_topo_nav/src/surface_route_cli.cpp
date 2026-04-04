#include "rc26_topo_nav/body_planning.hpp"
#include "rc26_topo_nav/graph_loader.hpp"
#include "rc26_topo_nav/surface_route.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
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
        } else {
            error = "Unknown flag: " + flag;
            return false;
        }
    }

    if (args.graph.empty()) {
        error = "Missing required flag --graph";
        return false;
    }
    if (args.robot_geometry_file.empty() && args.body_planning.enabled) {
        try {
            const auto pkg_dir = ament_index_cpp::get_package_share_directory("rc26_robot_geometry");
            args.robot_geometry_file = pkg_dir + "/config/r2_body_geometry.yaml";
        } catch (...) {
            args.body_planning.enabled = false;
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
    std::unordered_map<std::string, NodeOverlay> node_overlays;
    std::unordered_map<std::string, EdgeOverlay> edge_overlays;
    SurfaceBodyPlanningStats body_planning_stats;
    if (args.body_planning.enabled) {
        std::string geometry_error;
        const auto geometry = loadRobotGeometryProfile(
            args.robot_geometry_file, args.robot_geometry_profile, geometry_error);
        if (!geometry) {
            std::cerr << "[ERROR] Failed to load robot geometry: " << geometry_error << "\n";
            return 1;
        }
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
    const auto plan = planSurfaceRoute(
        load_result.graph,
        args.start,
        args.goal,
        node_overlays,
        edge_overlays,
        weights,
        args.projection_radius_m,
        rclcpp::Time(0),
        "map");

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
    std::cout << ",\"body_planning\":{"
              << "\"enabled\":" << (args.body_planning.enabled ? "true" : "false") << ","
              << "\"annotations_available\":" << (body_planning_stats.annotations_available ? "true" : "false") << ","
              << "\"penalized_nodes_clearance\":" << body_planning_stats.penalized_nodes_clearance << ","
              << "\"blocked_nodes_pitch\":" << body_planning_stats.blocked_nodes_pitch << ","
              << "\"blocked_edges_clearance\":" << body_planning_stats.blocked_edges_clearance << ","
              << "\"blocked_edges_slope\":" << body_planning_stats.blocked_edges_slope << ","
              << "\"blocked_edges_step\":" << body_planning_stats.blocked_edges_step
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
