#pragma once

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace rc26_xhu_nav::body_planner {

struct Pose3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
};

struct SurfaceNode {
    std::string id;
    Pose3 pose;
    double center_clearance_m = -1.0;
    double surface_pitch_deg = -1.0;
};

struct SurfaceEdge {
    std::string id;
    std::string from;
    std::string to;
    std::string motion_type;
    std::string required_mode;
    double base_cost = 0.0;
    double height_change = 0.0;
    double horizontal_length_m = 0.0;
    double slope_deg = 0.0;
    double center_clearance_m = -1.0;
    double nominal_yaw = 0.0;
    bool same_surface = false;
};

struct NodeOverlay {
    bool blocked = false;
    double extra_cost = 0.0;
};

struct EdgeOverlay {
    bool blocked = false;
    double extra_cost = 0.0;
};

struct SurfaceGraph {
    std::string team;
    std::string schema_version;
    std::unordered_map<std::string, SurfaceNode> nodes;
    std::vector<SurfaceEdge> edges;
    std::unordered_map<std::string, std::vector<std::size_t>> adjacency;
};

}  // namespace rc26_xhu_nav::body_planner
