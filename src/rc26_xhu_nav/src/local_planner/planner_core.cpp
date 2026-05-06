#include "rc26_xhu_nav/local_planner/planner_core.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace rc26_xhu_nav::local_planner {

namespace {

double yamlDouble(const YAML::Node& node, const char* key, const double fallback) {
    if (!node || !node[key]) {
        return fallback;
    }
    return node[key].as<double>();
}

std::string yamlString(const YAML::Node& node, const char* key, const std::string& fallback) {
    if (!node || !node[key]) {
        return fallback;
    }
    return node[key].as<std::string>();
}

TracePose makeTracePose(const double x, const double y, const double yaw) {
    return TracePose{x, y, 0.0, yaw};
}

}  // namespace

PlannerCore::PlannerCore(PlannerConfig config) : config_(std::move(config)) {}

void PlannerCore::setConfig(const PlannerConfig& config) {
    config_ = config;
}

double PlannerCore::normalizeAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double PlannerCore::clamp(const double value, const double lower, const double upper) {
    return std::min(std::max(value, lower), upper);
}

double PlannerCore::yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

double PlannerCore::terrainProbabilityAt(
    const rc26_interfaces::msg::TerrainFeatureGrid& grid, const std::vector<float>& values,
    const double x, const double y) {
    if (grid.resolution_m <= 0.0F || grid.width == 0 || grid.height == 0) {
        return 0.0;
    }

    const int gx = static_cast<int>(std::floor((x - grid.origin.position.x) / grid.resolution_m));
    const int gy = static_cast<int>(std::floor((y - grid.origin.position.y) / grid.resolution_m));
    if (gx < 0 || gy < 0 || gx >= static_cast<int>(grid.width) ||
        gy >= static_cast<int>(grid.height)) {
        return 0.0;
    }

    const auto flat_index =
        static_cast<std::size_t>(gy) * grid.width + static_cast<std::size_t>(gx);
    if (flat_index >= values.size()) {
        return 0.0;
    }
    return values[flat_index];
}

double PlannerCore::pathDistanceToNearest(
    const nav_msgs::msg::Path& path, const double x, const double y,
    const std::size_t hint_index, std::size_t* nearest_index_out) {
    if (path.poses.empty()) {
        if (nearest_index_out != nullptr) {
            *nearest_index_out = 0;
        }
        return std::numeric_limits<double>::infinity();
    }

    std::size_t best_index = std::min(hint_index, path.poses.size() - 1U);
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = best_index; index < path.poses.size(); ++index) {
        const auto& pose = path.poses[index].pose.position;
        const double distance = std::hypot(pose.x - x, pose.y - y);
        if (distance < best_distance) {
            best_distance = distance;
            best_index = index;
        }
    }
    if (nearest_index_out != nullptr) {
        *nearest_index_out = best_index;
    }
    return best_distance;
}

std::size_t PlannerCore::findLookaheadIndex(
    const nav_msgs::msg::Path& path, const std::size_t start_index,
    const double lookahead_distance) {
    if (path.poses.empty()) {
        return 0;
    }

    const auto safe_start = std::min(start_index, path.poses.size() - 1U);
    double accumulated = 0.0;
    for (std::size_t index = safe_start; index + 1U < path.poses.size(); ++index) {
        const auto& from = path.poses[index].pose.position;
        const auto& to = path.poses[index + 1U].pose.position;
        accumulated += std::hypot(to.x - from.x, to.y - from.y);
        if (accumulated >= lookahead_distance) {
            return index + 1U;
        }
    }
    return path.poses.size() - 1U;
}

nav_msgs::msg::Path PlannerCore::makePreviewPath(
    const std::vector<SimState>& states, const rclcpp::Time& stamp,
    const std::string& frame_id) {
    nav_msgs::msg::Path preview;
    preview.header.stamp = stamp;
    preview.header.frame_id = frame_id;
    preview.poses.reserve(states.size());
    for (const auto& state : states) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = preview.header;
        pose.pose.position.x = state.x;
        pose.pose.position.y = state.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.z = std::sin(state.yaw * 0.5);
        pose.pose.orientation.w = std::cos(state.yaw * 0.5);
        preview.poses.push_back(pose);
    }
    return preview;
}

std::optional<RobotGeometryProfile> PlannerCore::loadRobotGeometryProfile(
    const std::string& geometry_file, const std::string& requested_profile,
    std::string& error) {
    try {
        const YAML::Node root = YAML::LoadFile(geometry_file);
        const YAML::Node geometry_root = root["robot_geometry"] ? root["robot_geometry"] : root;
        const YAML::Node defaults = geometry_root["defaults"];
        const YAML::Node profiles = geometry_root["profiles"];

        std::string profile_name = requested_profile;
        if (profile_name.empty()) {
            profile_name = yamlString(defaults, "active_profile", "");
        }
        if (profile_name.empty()) {
            error = "robot geometry profile is empty";
            return std::nullopt;
        }
        if (!profiles || !profiles[profile_name]) {
            error = "robot geometry profile not found: " + profile_name;
            return std::nullopt;
        }

        const YAML::Node profile_node = profiles[profile_name];
        const YAML::Node body = profile_node["body"];
        const YAML::Node safety = profile_node["safety"];

        RobotGeometryProfile profile;
        profile.name = profile_name;
        profile.half_length_m = std::max(0.0, yamlDouble(body, "half_length_m", 0.0));
        profile.half_width_m = std::max(0.0, yamlDouble(body, "half_width_m", 0.0));
        profile.height_m = std::max(0.0, yamlDouble(body, "height_m", 0.0));
        profile.stop_envelope_half_width_m =
            std::max(profile.half_width_m, yamlDouble(safety, "stop_envelope_half_width_m", 0.0));
        return profile;
    } catch (const std::exception& e) {
        error = e.what();
        return std::nullopt;
    }
}

PlannerResult PlannerCore::plan(const PlannerInput& input, PlannerTrace* trace) const {
    PlannerResult result;
    result.status = "HOLD";
    result.reason = "planner input unavailable";
    if (trace != nullptr) {
        *trace = PlannerTrace{};
        trace->semantic_revision = input.has_semantic_summary ? input.semantic_summary.revision : 0U;
    }

    if (!input.has_pose) {
        result.reason = "robot pose unavailable";
        if (trace != nullptr) {
            trace->final_status = result.status;
            trace->final_reason = result.reason;
        }
        return result;
    }
    if (input.corridor.path.poses.empty()) {
        result.reason = "corridor path empty";
        if (trace != nullptr) {
            trace->final_status = result.status;
            trace->final_reason = result.reason;
        }
        return result;
    }

    const auto& path = input.corridor.path;
    std::size_t nearest_index = 0U;
    const double current_path_distance = pathDistanceToNearest(
        path, input.robot_x, input.robot_y, 0U, &nearest_index);
    const auto lookahead_index = findLookaheadIndex(path, nearest_index, config_.lookahead_distance_m);
    const auto& goal_pose = path.poses.back().pose;
    const auto& lookahead_pose = path.poses[lookahead_index].pose;
    const double lookahead_heading =
        yawFromQuaternion(lookahead_pose.orientation);
    const double goal_heading = yawFromQuaternion(goal_pose.orientation);
    const double goal_heading_error = normalizeAngle(goal_heading - input.robot_yaw);
    double desired_world_x = lookahead_pose.position.x - input.robot_x;
    double desired_world_y = lookahead_pose.position.y - input.robot_y;
    double desired_world_norm = std::hypot(desired_world_x, desired_world_y);
    if (desired_world_norm <= 1e-6) {
        desired_world_x = std::cos(lookahead_heading);
        desired_world_y = std::sin(lookahead_heading);
        desired_world_norm = 1.0;
    }
    desired_world_x /= desired_world_norm;
    desired_world_y /= desired_world_norm;
    const double cos_yaw = std::cos(input.robot_yaw);
    const double sin_yaw = std::sin(input.robot_yaw);

    double linear_limit = input.mode_state.max_linear_speed > 0.0F
                              ? input.mode_state.max_linear_speed
                              : std::numeric_limits<double>::infinity();
    if (input.corridor.max_linear_speed > 0.0F) {
        linear_limit = std::min(linear_limit, static_cast<double>(input.corridor.max_linear_speed));
    }
    if (std::isinf(linear_limit)) {
        linear_limit = 0.8;
    }

    if (input.has_semantic_summary && input.semantic_summary.slow_cells > 0U) {
        linear_limit *= config_.slow_zone_speed_scale;
    }

    double angular_limit = input.mode_state.max_angular_speed > 0.0F
                               ? input.mode_state.max_angular_speed
                               : std::numeric_limits<double>::infinity();
    if (input.corridor.max_angular_speed > 0.0F) {
        angular_limit =
            std::min(angular_limit, static_cast<double>(input.corridor.max_angular_speed));
    }
    if (std::isinf(angular_limit)) {
        angular_limit = 1.0;
    }

    const double preferred_linear_speed = input.corridor.preferred_linear_speed > 0.0F
                                              ? input.corridor.preferred_linear_speed
                                              : std::min(0.35, linear_limit);
    if (trace != nullptr) {
        trace->linear_limit = linear_limit;
        trace->angular_limit = angular_limit;
        trace->preferred_linear_speed = preferred_linear_speed;
        trace->current_path_distance = current_path_distance;
        trace->goal_heading_error = goal_heading_error;
    }

    double best_score = std::numeric_limits<double>::infinity();
    double best_clearance = 0.0;
    std::vector<SimState> best_states;
    double best_cmd_vx = 0.0;
    double best_cmd_vy = 0.0;
    double best_cmd_wz = 0.0;
    std::size_t best_candidate_index = std::numeric_limits<std::size_t>::max();

    for (const double sampled_speed : config_.sample_linear_speeds) {
        if (sampled_speed < 0.0 && !input.corridor.allow_reverse) {
            continue;
        }
        if (std::abs(sampled_speed) > linear_limit + 1e-6) {
            continue;
        }
        const double sampled_vx = sampled_speed * (desired_world_x * cos_yaw + desired_world_y * sin_yaw);
        const double sampled_vy = sampled_speed * (-desired_world_x * sin_yaw + desired_world_y * cos_yaw);
        for (const double sampled_wz : config_.sample_angular_speeds) {
            if (std::abs(sampled_wz) > angular_limit + 1e-6) {
                continue;
            }

            CandidateTrajectoryTrace candidate;
            candidate.sampled_vx = sampled_vx;
            candidate.sampled_vy = sampled_vy;
            candidate.sampled_wz = sampled_wz;
            SimState state{input.robot_x, input.robot_y, input.robot_yaw};
            std::vector<SimState> states;
            states.push_back(state);
            bool collision = false;
            double min_clearance = 1.0;
            std::size_t hint_index = nearest_index;

            for (double elapsed = 0.0; elapsed < config_.horizon_sec;
                 elapsed += config_.integration_step_sec) {
                state.x += (sampled_vx * std::cos(state.yaw) - sampled_vy * std::sin(state.yaw)) *
                           config_.integration_step_sec;
                state.y += (sampled_vx * std::sin(state.yaw) + sampled_vy * std::cos(state.yaw)) *
                           config_.integration_step_sec;
                state.yaw = normalizeAngle(state.yaw + sampled_wz * config_.integration_step_sec);
                states.push_back(state);

                if (input.has_terrain_grid) {
                    const double normal_x = -std::sin(state.yaw);
                    const double normal_y = std::cos(state.yaw);
                    const double center_obstacle = terrainProbabilityAt(
                        input.terrain_grid, input.terrain_grid.p_obstacle, state.x, state.y);
                    const double center_drop = terrainProbabilityAt(
                        input.terrain_grid, input.terrain_grid.p_drop, state.x, state.y);
                    const double left_obstacle = terrainProbabilityAt(
                        input.terrain_grid, input.terrain_grid.p_obstacle,
                        state.x + normal_x * config_.stop_envelope_half_width_m,
                        state.y + normal_y * config_.stop_envelope_half_width_m);
                    const double right_obstacle = terrainProbabilityAt(
                        input.terrain_grid, input.terrain_grid.p_obstacle,
                        state.x - normal_x * config_.stop_envelope_half_width_m,
                        state.y - normal_y * config_.stop_envelope_half_width_m);
                    const double left_drop = terrainProbabilityAt(
                        input.terrain_grid, input.terrain_grid.p_drop,
                        state.x + normal_x * config_.stop_envelope_half_width_m,
                        state.y + normal_y * config_.stop_envelope_half_width_m);
                    const double right_drop = terrainProbabilityAt(
                        input.terrain_grid, input.terrain_grid.p_drop,
                        state.x - normal_x * config_.stop_envelope_half_width_m,
                        state.y - normal_y * config_.stop_envelope_half_width_m);

                    const double max_obstacle =
                        std::max({center_obstacle, left_obstacle, right_obstacle});
                    const double max_drop = std::max({center_drop, left_drop, right_drop});
                    min_clearance = std::min(min_clearance, 1.0 - std::max(max_obstacle, max_drop));
                    if (max_obstacle >= config_.terrain_obstacle_threshold ||
                        max_drop >= config_.terrain_drop_threshold) {
                        collision = true;
                        break;
                    }
                }

                pathDistanceToNearest(path, state.x, state.y, hint_index, &hint_index);
            }

            if (collision) {
                result.blocked_by_terrain = true;
                candidate.collision = true;
                candidate.reject_reason = "terrain_collision";
                candidate.clearance_margin_m =
                    min_clearance * config_.stop_envelope_half_width_m;
                for (const auto& sampled_state : states) {
                    candidate.points.push_back(
                        makeTracePose(sampled_state.x, sampled_state.y, sampled_state.yaw));
                }
                if (trace != nullptr) {
                    trace->candidates.push_back(std::move(candidate));
                }
                continue;
            }

            const auto& terminal_state = states.back();
            const double path_distance = pathDistanceToNearest(
                path, terminal_state.x, terminal_state.y, nearest_index, nullptr);
            const double sampled_heading_error =
                std::abs(normalizeAngle(lookahead_heading - terminal_state.yaw));
            const double score =
                config_.path_alignment_weight * path_distance +
                config_.heading_alignment_weight * sampled_heading_error +
                config_.speed_preference_weight * std::abs(preferred_linear_speed - std::abs(sampled_speed)) +
                config_.angular_effort_weight * std::abs(sampled_wz) +
                config_.clearance_weight * (1.0 - min_clearance);
            candidate.score = score;
            candidate.path_distance = path_distance;
            candidate.heading_error = sampled_heading_error;
            candidate.speed_error = std::abs(preferred_linear_speed - std::abs(sampled_speed));
            candidate.angular_effort = std::abs(sampled_wz);
            candidate.clearance_margin_m =
                min_clearance * config_.stop_envelope_half_width_m;
            candidate.reject_reason = "candidate_scored";
            for (const auto& sampled_state : states) {
                candidate.points.push_back(
                    makeTracePose(sampled_state.x, sampled_state.y, sampled_state.yaw));
            }
            if (score < best_score) {
                best_score = score;
                best_clearance = min_clearance;
                best_states = std::move(states);
                best_cmd_vx = sampled_vx;
                best_cmd_vy = sampled_vy;
                best_cmd_wz = sampled_wz;
                candidate.selected = true;
                best_candidate_index =
                    trace != nullptr ? trace->candidates.size() : best_candidate_index;
            }
            if (trace != nullptr) {
                trace->candidates.push_back(std::move(candidate));
            }
        }
    }

    if (trace != nullptr && best_candidate_index < trace->candidates.size()) {
        for (std::size_t index = 0; index < trace->candidates.size(); ++index) {
            trace->candidates[index].selected = index == best_candidate_index;
            if (index != best_candidate_index &&
                trace->candidates[index].reject_reason == "candidate_scored") {
                trace->candidates[index].reject_reason = "higher_score";
            }
        }
    }

    if (!best_states.empty()) {
        result.has_solution = true;
        result.cmd_vx = best_cmd_vx;
        result.cmd_vy = best_cmd_vy;
        result.cmd_wz = best_cmd_wz;
        result.best_score = best_score;
        result.clearance_margin_m = best_clearance * config_.stop_envelope_half_width_m;
        result.status = "PASS";
        result.reason =
            current_path_distance > config_.stop_envelope_half_width_m ? "planner detour" : "tracking";
        result.preview_path = makePreviewPath(
            best_states, input.corridor.header.stamp, input.corridor.header.frame_id.empty()
                                                    ? "map"
                                                    : input.corridor.header.frame_id);
        if (trace != nullptr) {
            trace->final_status = result.status;
            trace->final_reason = result.reason;
        }
        return result;
    }

    if (input.corridor.allow_in_place_rotate &&
        std::abs(goal_heading_error) >= config_.recovery_heading_threshold_rad) {
        result.should_rotate_recovery = true;
        result.cmd_vx = 0.0;
        result.cmd_vy = 0.0;
        result.cmd_wz = clamp(goal_heading_error, -config_.recovery_angular_speed,
                              config_.recovery_angular_speed);
        result.status = "RECOVERY_RUNNING";
        result.reason = "rotate in place";
        result.recovery_state.corridor_id = input.corridor.corridor_id;
        result.recovery_state.edge_id = input.corridor.edge_id;
        result.recovery_state.recovery_name = "rotate_in_place";
        result.recovery_state.status = "SUGGESTED";
        result.recovery_state.reason = result.reason;
        if (trace != nullptr) {
            trace->final_status = result.status;
            trace->final_reason = result.reason;
        }
        return result;
    }

    if (input.has_semantic_summary && input.semantic_summary.blocked_cells > 0U) {
        result.blocked_by_keepout = true;
        result.status = "WAITING_ON_BLOCK";
        result.reason = "keepout blocked";
        if (trace != nullptr) {
            trace->final_status = result.status;
            trace->final_reason = result.reason;
        }
        return result;
    }

    result.status = "LOCAL_COLLISION_BLOCKED";
    result.reason = "terrain collision";
    if (trace != nullptr) {
        trace->final_status = result.status;
        trace->final_reason = result.reason;
    }
    return result;
}

}  // namespace rc26_xhu_nav::local_planner
