#include "rc26_xhu_nav/local_planner/planner_core.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace rc26_xhu_nav::local_planner {
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

double yamlDouble(const YAML::Node& node, const char* key, const double fallback) {
    if (!node || !node[key]) {
        return fallback;
    }
    return node[key].as<double>();
}

bool yamlBool(const YAML::Node& node, const char* key, const bool fallback) {
    if (!node || !node[key]) {
        return fallback;
    }
    return node[key].as<bool>();
}

std::string yamlString(const YAML::Node& node, const char* key, const std::string& fallback) {
    if (!node || !node[key]) {
        return fallback;
    }
    return node[key].as<std::string>();
}

geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw) {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
}

bool parsePose2D(
    const YAML::Node& node, const char* label, double& x, double& y, double& yaw,
    std::string& error) {
    if (!node) {
        error = std::string("Missing required pose node: ") + label;
        return false;
    }
    if (!node["x"] || !node["y"]) {
        error = std::string("Pose node missing x/y: ") + label;
        return false;
    }
    x = node["x"].as<double>();
    y = node["y"].as<double>();
    yaw = yamlDouble(node, "yaw", 0.0);
    return true;
}

bool parseCorridor(
    const YAML::Node& node, rc26_interfaces::msg::XhuSemanticCorridor& corridor,
    std::string& error) {
    if (!node || !node["poses"] || !node["poses"].IsSequence()) {
        error = "corridor.poses must be a sequence";
        return false;
    }
    corridor.header.frame_id = yamlString(node, "frame_id", "map");
    corridor.corridor_id = yamlString(node, "corridor_id", "trace_corridor");
    corridor.edge_id = yamlString(node, "edge_id", "trace_edge");
    corridor.from_node_id = yamlString(node, "from_node_id", "trace_from");
    corridor.to_node_id = yamlString(node, "to_node_id", "trace_to");
    corridor.motion_type = yamlString(node, "motion_type", "plane_move");
    corridor.required_mode = yamlString(node, "required_mode", "normal");
    corridor.preferred_linear_speed =
        static_cast<float>(yamlDouble(node, "preferred_linear_speed", 0.30));
    corridor.max_linear_speed =
        static_cast<float>(yamlDouble(node, "max_linear_speed", 0.50));
    corridor.max_angular_speed =
        static_cast<float>(yamlDouble(node, "max_angular_speed", 1.00));
    corridor.stop_at_end = yamlBool(node, "stop_at_end", true);
    corridor.allow_reverse = yamlBool(node, "allow_reverse", false);
    corridor.allow_in_place_rotate = yamlBool(node, "allow_in_place_rotate", true);
    corridor.speed_limit_reason = yamlString(node, "speed_limit_reason", "trace_profile");
    if (node["active_risk_sources"] && node["active_risk_sources"].IsSequence()) {
        for (const auto& item : node["active_risk_sources"]) {
            corridor.active_risk_sources.push_back(item.as<std::string>());
        }
    }

    corridor.path.header.frame_id = corridor.header.frame_id;
    for (const auto& pose_node : node["poses"]) {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        if (!parsePose2D(pose_node, "corridor pose", x, y, yaw, error)) {
            return false;
        }
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = corridor.header.frame_id;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = yamlDouble(pose_node, "z", 0.0);
        pose.pose.orientation = quaternionFromYaw(yaw);
        corridor.path.poses.push_back(pose);
    }
    return !corridor.path.poses.empty();
}

template <typename T>
void fillGridVector(
    const YAML::Node& node, const char* array_key, const char* default_key,
    const char* hotspot_key, const std::size_t total_cells, std::vector<T>& values) {
    values.assign(total_cells, static_cast<T>(yamlDouble(node, default_key, 0.0)));
    if (node[array_key] && node[array_key].IsSequence()) {
        values.clear();
        values.reserve(node[array_key].size());
        for (const auto& item : node[array_key]) {
            values.push_back(static_cast<T>(item.as<double>()));
        }
        values.resize(total_cells, T{});
        return;
    }
    if (!node[hotspot_key] || !node[hotspot_key].IsSequence()) {
        return;
    }
    const auto width = static_cast<std::size_t>(node["width"].as<uint32_t>());
    for (const auto& hotspot : node[hotspot_key]) {
        const auto gx = hotspot["gx"].as<std::size_t>();
        const auto gy = hotspot["gy"].as<std::size_t>();
        const auto flat_index = gy * width + gx;
        if (flat_index < values.size()) {
            values[flat_index] = static_cast<T>(hotspot["value"].as<double>());
        }
    }
}

void fillGridMask(
    const YAML::Node& node, const char* array_key, const std::size_t total_cells,
    std::vector<uint8_t>& values) {
    values.assign(total_cells, 0U);
    if (!node[array_key] || !node[array_key].IsSequence()) {
        return;
    }
    values.clear();
    values.reserve(node[array_key].size());
    for (const auto& item : node[array_key]) {
        values.push_back(static_cast<uint8_t>(item.as<int>()));
    }
    values.resize(total_cells, 0U);
}

bool parseTerrainGrid(
    const YAML::Node& node, rc26_interfaces::msg::TerrainFeatureGrid& grid,
    std::string& error) {
    if (!node) {
        return true;
    }
    if (!node["resolution_m"] || !node["width"] || !node["height"] || !node["origin"]) {
        error = "terrain_grid requires resolution_m, width, height, and origin";
        return false;
    }
    grid.resolution_m = static_cast<float>(node["resolution_m"].as<double>());
    grid.width = node["width"].as<uint32_t>();
    grid.height = node["height"].as<uint32_t>();
    grid.origin.position.x = node["origin"]["x"].as<double>();
    grid.origin.position.y = node["origin"]["y"].as<double>();
    const std::size_t total_cells =
        static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height);
    fillGridMask(node, "in_radius", total_cells, grid.in_radius);
    fillGridMask(node, "fresh", total_cells, grid.fresh);
    fillGridVector<uint16_t>(node, "density", "default_density", "density_hotspots", total_cells, grid.density);
    fillGridVector<float>(node, "h_ground", "default_h_ground", "h_ground_hotspots", total_cells, grid.h_ground);
    fillGridVector<float>(node, "sigma_h", "default_sigma_h", "sigma_h_hotspots", total_cells, grid.sigma_h);
    fillGridVector<float>(node, "h_top", "default_h_top", "h_top_hotspots", total_cells, grid.h_top);
    fillGridVector<float>(node, "slope_x", "default_slope_x", "slope_x_hotspots", total_cells, grid.slope_x);
    fillGridVector<float>(node, "slope_y", "default_slope_y", "slope_y_hotspots", total_cells, grid.slope_y);
    fillGridVector<float>(node, "roughness", "default_roughness", "roughness_hotspots", total_cells, grid.roughness);
    fillGridVector<float>(node, "p_obstacle", "default_obstacle", "obstacle_hotspots", total_cells, grid.p_obstacle);
    fillGridVector<float>(node, "p_drop", "default_drop", "drop_hotspots", total_cells, grid.p_drop);
    fillGridVector<float>(node, "step_up", "default_step_up", "step_up_hotspots", total_cells, grid.step_up);
    fillGridVector<float>(node, "p_climbable", "default_climbable", "climbable_hotspots", total_cells, grid.p_climbable);
    return true;
}

bool loadSnapshot(
    const std::string& path, PlannerInput& input, PlannerConfig& config, std::string& label,
    std::string& error) {
    const YAML::Node root = YAML::LoadFile(path);
    label = yamlString(root, "label", "");
    const YAML::Node robot_pose = root["robot_pose"];
    if (!parsePose2D(robot_pose, "robot_pose", input.robot_x, input.robot_y, input.robot_yaw, error)) {
        return false;
    }
    input.has_pose = true;
    input.current_vx = yamlDouble(root["current_velocity"], "vx", 0.0);
    input.current_vy = yamlDouble(root["current_velocity"], "vy", 0.0);
    input.current_wz = yamlDouble(root["current_velocity"], "wz", 0.0);
    if (!parseCorridor(root["corridor"], input.corridor, error)) {
        if (error.empty()) {
            error = "corridor parsing failed";
        }
        return false;
    }

    if (root["mode_state"]) {
        input.has_mode_state = true;
        input.mode_state.active_mode = yamlString(root["mode_state"], "active_mode", "normal");
        input.mode_state.reason = yamlString(root["mode_state"], "reason", "snapshot");
        input.mode_state.stop_required = yamlBool(root["mode_state"], "stop_required", false);
        input.mode_state.timed_out = yamlBool(root["mode_state"], "timed_out", false);
        input.mode_state.max_linear_speed =
            static_cast<float>(yamlDouble(root["mode_state"], "max_linear_speed", 0.5));
        input.mode_state.max_angular_speed =
            static_cast<float>(yamlDouble(root["mode_state"], "max_angular_speed", 1.0));
        input.mode_state.max_linear_accel =
            static_cast<float>(yamlDouble(root["mode_state"], "max_linear_accel", 0.6));
        input.mode_state.max_angular_accel =
            static_cast<float>(yamlDouble(root["mode_state"], "max_angular_accel", 0.8));
    }

    if (root["semantic_summary"]) {
        input.has_semantic_summary = true;
        input.semantic_summary.revision =
            root["semantic_summary"]["revision"] ? root["semantic_summary"]["revision"].as<uint64_t>() : 1U;
        input.semantic_summary.terrain_available =
            yamlBool(root["semantic_summary"], "terrain_available", true);
        input.semantic_summary.keepout_available =
            yamlBool(root["semantic_summary"], "keepout_available", true);
        input.semantic_summary.obstacle_cells =
            root["semantic_summary"]["obstacle_cells"] ? root["semantic_summary"]["obstacle_cells"].as<uint32_t>() : 0U;
        input.semantic_summary.drop_cells =
            root["semantic_summary"]["drop_cells"] ? root["semantic_summary"]["drop_cells"].as<uint32_t>() : 0U;
        input.semantic_summary.blocked_cells =
            root["semantic_summary"]["blocked_cells"] ? root["semantic_summary"]["blocked_cells"].as<uint32_t>() : 0U;
        input.semantic_summary.slow_cells =
            root["semantic_summary"]["slow_cells"] ? root["semantic_summary"]["slow_cells"].as<uint32_t>() : 0U;
        input.semantic_summary.max_obstacle_probability =
            static_cast<float>(yamlDouble(root["semantic_summary"], "max_obstacle_probability", 0.0));
        input.semantic_summary.max_drop_probability =
            static_cast<float>(yamlDouble(root["semantic_summary"], "max_drop_probability", 0.0));
        if (root["semantic_summary"]["active_sources"]) {
            for (const auto& item : root["semantic_summary"]["active_sources"]) {
                input.semantic_summary.active_sources.push_back(item.as<std::string>());
            }
        }
        if (root["semantic_summary"]["active_reasons"]) {
            for (const auto& item : root["semantic_summary"]["active_reasons"]) {
                input.semantic_summary.active_reasons.push_back(item.as<std::string>());
            }
        }
    }

    if (root["terrain_grid"]) {
        input.has_terrain_grid = true;
        if (!parseTerrainGrid(root["terrain_grid"], input.terrain_grid, error)) {
            return false;
        }
    }

    if (root["planner_config"]) {
        const YAML::Node planner_node = root["planner_config"];
        config.lookahead_distance_m = yamlDouble(planner_node, "lookahead_distance_m", config.lookahead_distance_m);
        config.horizon_sec = yamlDouble(planner_node, "horizon_sec", config.horizon_sec);
        config.integration_step_sec = yamlDouble(planner_node, "integration_step_sec", config.integration_step_sec);
        config.terrain_obstacle_threshold =
            yamlDouble(planner_node, "terrain_obstacle_threshold", config.terrain_obstacle_threshold);
        config.terrain_drop_threshold =
            yamlDouble(planner_node, "terrain_drop_threshold", config.terrain_drop_threshold);
        config.recovery_heading_threshold_rad =
            yamlDouble(planner_node, "recovery_heading_threshold_rad", config.recovery_heading_threshold_rad);
        config.recovery_angular_speed =
            yamlDouble(planner_node, "recovery_angular_speed", config.recovery_angular_speed);
        config.slow_zone_speed_scale =
            yamlDouble(planner_node, "slow_zone_speed_scale", config.slow_zone_speed_scale);
        config.stop_envelope_half_width_m =
            yamlDouble(planner_node, "stop_envelope_half_width_m", config.stop_envelope_half_width_m);
    }

    return true;
}

void printPose(std::ostream& out, const TracePose& pose) {
    out << "{"
        << "\"x\":" << pose.x << ","
        << "\"y\":" << pose.y << ","
        << "\"z\":" << pose.z << ","
        << "\"yaw\":" << pose.yaw
        << "}";
}

void printTracePoseArray(std::ostream& out, const std::vector<TracePose>& poses) {
    out << "[";
    for (std::size_t index = 0; index < poses.size(); ++index) {
        if (index > 0) {
            out << ",";
        }
        printPose(out, poses[index]);
    }
    out << "]";
}

void printCandidateTrajectories(
    std::ostream& out, const std::vector<CandidateTrajectoryTrace>& candidates,
    const std::size_t visible_count) {
    out << "[";
    const auto safe_count = std::min(visible_count, candidates.size());
    for (std::size_t index = 0; index < safe_count; ++index) {
        if (index > 0) {
            out << ",";
        }
        const auto& candidate = candidates[index];
        out << "{"
            << "\"velocity\":{\"vx\":" << candidate.sampled_vx
            << ",\"vy\":" << candidate.sampled_vy << ",\"wz\":" << candidate.sampled_wz << "},"
            << "\"points\":";
        printTracePoseArray(out, candidate.points);
        out << ",\"score\":" << candidate.score
            << ",\"collision\":" << (candidate.collision ? "true" : "false")
            << ",\"selected\":" << (candidate.selected ? "true" : "false")
            << ",\"clearance\":" << candidate.clearance_margin_m
            << "}";
    }
    out << "]";
}

void printFrame(
    std::ostream& out, const std::size_t index, const PlannerTrace& trace,
    const TracePose& robot_pose) {
    const auto& candidate = trace.candidates[index];
    out << "{"
        << "\"stepIndex\":" << index << ","
        << "\"algorithm\":\"local_planner\","
        << "\"phase\":\"candidate\","
        << "\"label\":" << quoted("evaluate candidate " + std::to_string(index + 1)) << ","
        << "\"robotPose\":";
    printPose(out, robot_pose);
    out << ",\"openSet\":[],\"expandedNodes\":[],\"bestPath\":{\"nodeIds\":[],\"points\":[]},"
        << "\"treeSegments\":[],\"candidateTrajectories\":";
    printCandidateTrajectories(out, trace.candidates, index + 1U);
    out << ",\"selectedTrajectory\":[],"
        << "\"metrics\":{"
        << "\"traceMode\":\"local_planner\","
        << "\"sampledVx\":" << candidate.sampled_vx << ","
        << "\"sampledVy\":" << candidate.sampled_vy << ","
        << "\"sampledWz\":" << candidate.sampled_wz << ","
        << "\"score\":" << candidate.score << ","
        << "\"collision\":" << (candidate.collision ? "true" : "false") << ","
        << "\"rejectReason\":" << quoted(candidate.reject_reason)
        << "}"
        << "}";
}

void printResultFrame(
    std::ostream& out, const std::size_t index, const PlannerTrace& trace,
    const PlannerResult& result, const TracePose& robot_pose) {
    const auto selected_it = std::find_if(
        trace.candidates.begin(), trace.candidates.end(),
        [](const CandidateTrajectoryTrace& candidate) { return candidate.selected; });
    const std::vector<TracePose> selected_points =
        selected_it == trace.candidates.end() ? std::vector<TracePose>{} : selected_it->points;
    out << "{"
        << "\"stepIndex\":" << index << ","
        << "\"algorithm\":\"local_planner\","
        << "\"phase\":\"result\","
        << "\"label\":" << quoted(trace.final_reason.empty() ? "planner result" : trace.final_reason) << ","
        << "\"robotPose\":";
    printPose(out, robot_pose);
    out << ",\"openSet\":[],\"expandedNodes\":[],\"bestPath\":{\"nodeIds\":[],\"points\":";
    printTracePoseArray(out, selected_points);
    out << "},\"treeSegments\":[],\"candidateTrajectories\":";
    printCandidateTrajectories(out, trace.candidates, trace.candidates.size());
    out << ",\"selectedTrajectory\":";
    printTracePoseArray(out, selected_points);
    out << ",\"metrics\":{"
        << "\"traceMode\":\"local_planner\","
        << "\"finalStatus\":" << quoted(trace.final_status) << ","
        << "\"hasSolution\":" << (result.has_solution ? "true" : "false") << ","
        << "\"blockedByKeepout\":" << (result.blocked_by_keepout ? "true" : "false") << ","
        << "\"blockedByTerrain\":" << (result.blocked_by_terrain ? "true" : "false") << ","
        << "\"shouldRotateRecovery\":" << (result.should_rotate_recovery ? "true" : "false")
        << "}"
        << "}";
}

void printTraceJson(
    std::ostream& out, const std::string& label, const PlannerInput& input,
    const PlannerResult& result, const PlannerTrace& trace) {
    const TracePose robot_pose{input.robot_x, input.robot_y, 0.0, input.robot_yaw};
    out << "{"
        << "\"success\":true,"
        << "\"snapshotLabel\":" << quoted(label) << ","
        << "\"traceMode\":\"local_planner\","
        << "\"result\":{"
        << "\"status\":" << quoted(result.status) << ","
        << "\"reason\":" << quoted(result.reason) << ","
        << "\"hasSolution\":" << (result.has_solution ? "true" : "false") << ","
        << "\"blockedByKeepout\":" << (result.blocked_by_keepout ? "true" : "false") << ","
        << "\"blockedByTerrain\":" << (result.blocked_by_terrain ? "true" : "false") << ","
        << "\"shouldRotateRecovery\":" << (result.should_rotate_recovery ? "true" : "false") << ","
        << "\"cmd\":{\"vx\":" << result.cmd_vx << ",\"vy\":" << result.cmd_vy << ",\"wz\":" << result.cmd_wz << "},"
        << "\"bestScore\":" << result.best_score << ","
        << "\"clearanceMarginM\":" << result.clearance_margin_m
        << "},"
        << "\"summary\":{"
        << "\"candidateCount\":" << trace.candidates.size() << ","
        << "\"linearLimit\":" << trace.linear_limit << ","
        << "\"angularLimit\":" << trace.angular_limit << ","
        << "\"preferredLinearSpeed\":" << trace.preferred_linear_speed << ","
        << "\"currentPathDistance\":" << trace.current_path_distance << ","
        << "\"goalHeadingError\":" << trace.goal_heading_error << ","
        << "\"semanticRevision\":" << trace.semantic_revision << ","
        << "\"finalStatus\":" << quoted(trace.final_status) << ","
        << "\"finalReason\":" << quoted(trace.final_reason)
        << "},"
        << "\"frames\":[";
    for (std::size_t index = 0; index < trace.candidates.size(); ++index) {
        if (index > 0) {
            out << ",";
        }
        printFrame(out, index, trace, robot_pose);
    }
    if (!trace.candidates.empty()) {
        out << ",";
    }
    printResultFrame(out, trace.candidates.size(), trace, result, robot_pose);
    out << "]"
        << "}";
}

}  // namespace

bool loadSnapshotFile(
    const std::string& path, PlannerInput& input, PlannerConfig& config, std::string& label,
    std::string& error) {
    return loadSnapshot(path, input, config, label, error);
}

void printTraceJsonDocument(
    std::ostream& out, const std::string& label, const PlannerInput& input,
    const PlannerResult& result, const PlannerTrace& trace) {
    printTraceJson(out, label, input, result, trace);
}
}  // namespace rc26_xhu_nav::local_planner

int main(int argc, char** argv) {
    std::string snapshot_file;
    for (int index = 1; index < argc; ++index) {
        const std::string flag = argv[index];
        if (flag == "--snapshot" && index + 1 < argc) {
            snapshot_file = argv[++index];
            continue;
        }
        std::cerr << "Usage: local_planner_trace_cli --snapshot <file>\n";
        return 1;
    }
    if (snapshot_file.empty()) {
        std::cerr << "Missing required --snapshot\n";
        return 1;
    }

    try {
        rc26_xhu_nav::local_planner::PlannerInput input;
        rc26_xhu_nav::local_planner::PlannerConfig config;
        std::string label;
        std::string error;
        if (!rc26_xhu_nav::local_planner::loadSnapshotFile(
                snapshot_file, input, config, label, error)) {
            std::cerr << error << "\n";
            return 1;
        }

        rc26_xhu_nav::local_planner::PlannerCore planner(config);
        rc26_xhu_nav::local_planner::PlannerTrace trace;
        const auto result = planner.plan(input, &trace);
        rc26_xhu_nav::local_planner::printTraceJsonDocument(
            std::cout, label, input, result, trace);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
}
