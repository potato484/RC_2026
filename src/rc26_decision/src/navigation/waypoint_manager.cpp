#include "rc26_decision/navigation/waypoint_manager.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace rc26_decision {

namespace {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

Vec2 operator+(const Vec2& a, const Vec2& b) { return Vec2{a.x + b.x, a.y + b.y}; }
Vec2 operator-(const Vec2& a, const Vec2& b) { return Vec2{a.x - b.x, a.y - b.y}; }
Vec2 operator*(double s, const Vec2& v) { return Vec2{s * v.x, s * v.y}; }

double dot(const Vec2& a, const Vec2& b) { return a.x * b.x + a.y * b.y; }
double norm(const Vec2& v) { return std::sqrt(dot(v, v)); }

Vec2 normalize(const Vec2& v, double* out_norm = nullptr) {
    const double n = norm(v);
    if (out_norm) {
        *out_norm = n;
    }
    if (n < 1e-6) {
        return Vec2{0.0, 0.0};
    }
    return (1.0 / n) * v;
}

bool isValidMerlinAnchors(const MerlinAnchors& a) {
    const Vec2 p1{a.block_1.x, a.block_1.y};
    const Vec2 p2{a.block_2.x, a.block_2.y};
    const Vec2 p4{a.block_4.x, a.block_4.y};
    return norm(p2 - p1) > 0.1 && norm(p4 - p1) > 0.1;
}

double getScalarDouble(const YAML::Node& node, const char* key) {
    if (!node || !node[key]) {
        throw std::runtime_error(std::string("missing key: ") + key);
    }
    return node[key].as<double>();
}

std::string getScalarStringOrEmpty(const YAML::Node& node, const char* key) {
    if (!node || !node[key]) {
        return "";
    }
    return node[key].as<std::string>();
}

std::string parseNavProfile(const YAML::Node& profile_node) {
    if (profile_node && profile_node.IsScalar()) {
        std::string s = profile_node.as<std::string>();
        if (!s.empty()) {
            return s;
        }
    }
    return "normal";
}

Pose2D parsePose2D(const YAML::Node& point_node) {
    const YAML::Node pose_node = point_node["pose"] ? point_node["pose"] : point_node;

    Pose2D pose;
    pose.x = getScalarDouble(pose_node, "x");
    pose.y = getScalarDouble(pose_node, "y");
    if (pose_node["theta"]) {
        pose.yaw = pose_node["theta"].as<double>();
    } else if (pose_node["yaw"]) {
        pose.yaw = pose_node["yaw"].as<double>();
    } else {
        pose.yaw = 0.0;
    }
    return pose;
}

NavTolerance parseTolerance(const YAML::Node& point_node) {
    NavTolerance tol;
    const YAML::Node t = point_node["tolerance"];
    if (!t) {
        return tol;
    }
    if (t["xy"]) {
        tol.xy_tolerance = t["xy"].as<double>();
    } else if (t["xy_tolerance"]) {
        tol.xy_tolerance = t["xy_tolerance"].as<double>();
    }

    if (t["yaw"]) {
        tol.yaw_tolerance = t["yaw"].as<double>();
    } else if (t["yaw_tolerance"]) {
        tol.yaw_tolerance = t["yaw_tolerance"].as<double>();
    }
    return tol;
}

std::unordered_map<std::string, double> parsePayload(const YAML::Node& point_node) {
    std::unordered_map<std::string, double> payload;
    const YAML::Node p = point_node["payload"];
    if (!p || !p.IsMap()) {
        return payload;
    }

    for (auto it = p.begin(); it != p.end(); ++it) {
        const auto k = it->first.as<std::string>();
        const auto v = it->second.as<double>();
        payload.emplace(k, v);
    }
    return payload;
}

SmartWaypointSpec parseSmartWaypoint(const YAML::Node& point_node) {
    SmartWaypointSpec spec;
    spec.pose = parsePose2D(point_node);
    spec.strategy_tag = getScalarStringOrEmpty(point_node, "strategy_tag");
    spec.tolerance = parseTolerance(point_node);
    spec.nav_profile = parseNavProfile(point_node["nav_profile"]);
    spec.speed_profile = getScalarStringOrEmpty(point_node, "speed_profile");
    if (point_node["timeout_sec"]) {
        spec.timeout_sec = point_node["timeout_sec"].as<float>();
    }

    spec.payload = parsePayload(point_node);
    return spec;
}

MerlinAnchors parseMerlinAnchors(const YAML::Node& anchors_node) {
    MerlinAnchors a;
    const YAML::Node b1 = anchors_node["block_1"];
    const YAML::Node b2 = anchors_node["block_2"];
    const YAML::Node b4 = anchors_node["block_4"];
    if (!b1 || !b2 || !b4) {
        throw std::runtime_error("merlin_config.anchors requires block_1, block_2, block_4");
    }

    a.block_1.x = getScalarDouble(b1, "x");
    a.block_1.y = getScalarDouble(b1, "y");
    a.block_2.x = getScalarDouble(b2, "x");
    a.block_2.y = getScalarDouble(b2, "y");
    a.block_4.x = getScalarDouble(b4, "x");
    a.block_4.y = getScalarDouble(b4, "y");
    return a;
}

MerlinParams parseMerlinParams(const YAML::Node& params_node) {
    MerlinParams p;
    if (!params_node) {
        return p;
    }
    if (params_node["safe_offset"]) {
        p.safe_offset = params_node["safe_offset"].as<double>();
    }
    if (params_node["jump_margin"]) {
        p.jump_margin = params_node["jump_margin"].as<double>();
    }
    return p;
}

MerlinGraph buildMerlinAdjacencyGraph() {
    MerlinGraph g;
    for (int r = 1; r <= 4; ++r) {
        for (int c = 1; c <= 3; ++c) {
            const int id = (r - 1) * 3 + c;
            std::vector<int> nb;
            if (r > 1) {
                nb.push_back(id - 3);
            }
            if (r < 4) {
                nb.push_back(id + 3);
            }
            if (c > 1) {
                nb.push_back(id - 1);
            }
            if (c < 3) {
                nb.push_back(id + 1);
            }
            g.adjacency.emplace(id, std::move(nb));
        }
    }
    return g;
}

bool generateMerlinPoints(const MerlinAnchors& anchors, const MerlinParams& params,
                          std::unordered_map<std::string, SmartWaypointSpec>& out_points,
                          rclcpp::Logger& logger) {
    if (!isValidMerlinAnchors(anchors)) {
        RCLCPP_WARN(logger, "Merlin anchors are degenerate (zero distance), skipping topology generation");
        return false;
    }

    const Vec2 p1{anchors.block_1.x, anchors.block_1.y};
    const Vec2 p2{anchors.block_2.x, anchors.block_2.y};
    const Vec2 p4{anchors.block_4.x, anchors.block_4.y};

    const Vec2 vx_raw = p2 - p1;
    double dx = 0.0;
    const Vec2 vx = normalize(vx_raw, &dx);
    if (dx < 1e-6) {
        return false;
    }

    const Vec2 vy_raw = p4 - p1;
    const Vec2 vy_proj = vy_raw - dot(vy_raw, vx) * vx;
    double dy_proj = 0.0;
    Vec2 vy = normalize(vy_proj, &dy_proj);
    if (dy_proj < 1e-6) {
        return false;
    }
    double dy = dot(vy_raw, vy);
    if (dy < 0.0) {
        dy = -dy;
        vy = Vec2{-vy.x, -vy.y};
    }

    const double yaw_align = std::atan2(vx.y, vx.x);

    auto make_spec = [&](const Vec2& p) {
        SmartWaypointSpec s;
        s.pose.x = p.x;
        s.pose.y = p.y;
        s.pose.yaw = yaw_align;
        s.nav_profile = "safe";
        s.speed_profile = "SLOW";
        s.timeout_sec = 10.0f;
        s.tolerance.xy_tolerance = 0.10;
        s.tolerance.yaw_tolerance = 0.25;
        return s;
    };

    for (int r = 1; r <= 4; ++r) {
        for (int c = 1; c <= 3; ++c) {
            const int id = (r - 1) * 3 + c;
            const Vec2 center = p1 + (static_cast<double>(c - 1) * dx) * vx + (static_cast<double>(r - 1) * dy) * vy;
            const Vec2 stand = center - (params.safe_offset * vy);

            const std::string center_name = "mf_block_" + std::to_string(id) + "_center";
            const std::string stand_name = "mf_block_" + std::to_string(id) + "_stand";

            out_points.emplace(center_name, make_spec(center));
            out_points.emplace(stand_name, make_spec(stand));
        }
    }

    return true;
}

}  // namespace

bool WaypointManager::loadFromYamlFile(const std::string& yaml_path) {
    try {
        YAML::Node root = YAML::LoadFile(yaml_path);

        // Parse into temporaries first (M2 fix: don't clear until success)
        std::unordered_map<std::string, SmartWaypointSpec> new_points;
        std::optional<MerlinGraph> new_merlin_graph;

        const YAML::Node static_points = root["static_points"];
        if (static_points && static_points.IsMap()) {
            for (auto it = static_points.begin(); it != static_points.end(); ++it) {
                const auto name = it->first.as<std::string>();
                const YAML::Node& point_node = it->second;
                new_points.emplace(name, parseSmartWaypoint(point_node));
            }
        }

        const YAML::Node merlin = root["merlin_config"];
        if (merlin && merlin.IsMap()) {
            const YAML::Node anchors_node = merlin["anchors"];
            const YAML::Node params_node = merlin["params"];

            if (anchors_node && anchors_node.IsMap() && anchors_node["block_1"] && anchors_node["block_2"] &&
                anchors_node["block_4"]) {
                const MerlinAnchors anchors = parseMerlinAnchors(anchors_node);
                const MerlinParams params = parseMerlinParams(params_node);
                if (generateMerlinPoints(anchors, params, new_points, logger_)) {
                    new_merlin_graph = buildMerlinAdjacencyGraph();
                }
            }
        }

        // Swap only on complete success
        points_ = std::move(new_points);
        merlin_graph_ = std::move(new_merlin_graph);

        RCLCPP_INFO(logger_, "WaypointManager: loaded %zu points from %s", points_.size(), yaml_path.c_str());
        return true;

    } catch (const std::exception& ex) {
        RCLCPP_ERROR(logger_, "WaypointManager: failed to load YAML [%s]: %s", yaml_path.c_str(), ex.what());
        return false;
    }
}

const SmartWaypointSpec* WaypointManager::find(const std::string& name) const {
    const auto it = points_.find(name);
    if (it == points_.end()) {
        return nullptr;
    }
    return &it->second;
}

std::vector<std::string> WaypointManager::listNames() const {
    std::vector<std::string> out;
    out.reserve(points_.size());
    for (const auto& kv : points_) {
        out.push_back(kv.first);
    }
    std::sort(out.begin(), out.end());
    return out;
}

const MerlinGraph& WaypointManager::merlinGraph() const {
    if (!merlin_graph_) {
        throw std::runtime_error("WaypointManager: merlin graph not available");
    }
    return *merlin_graph_;
}

void WaypointManager::injectPointsToBlackboard(const BT::Blackboard::Ptr& blackboard, const std::string& key_prefix) const {
    if (!blackboard) {
        return;
    }
    for (const auto& kv : points_) {
        blackboard->set(key_prefix + kv.first, kv.second);
    }
}

}  // namespace rc26_decision
