#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace rc26_xhu_nav::topology {

struct Pose3 {
    double x = 0, y = 0, z = 0, yaw = 0;
};

struct GraphNode {
    std::string id;
    std::string type;
    Pose3 pose;
    int level = 0;
    uint8_t phase_mask = 0xFF;
    int block_id = 0;
    double base_cost = 0;
    std::string operation_tag;
    std::string surface_id;
    std::string surface_name;
    std::string render_class;
    double center_clearance_m = -1.0;
    double surface_pitch_deg = -1.0;
};

struct GraphEdge {
    std::string id;
    std::string from;
    std::string to;
    std::string motion_type;
    double height_change = 0;
    std::string required_mode;
    bool requires_confirmation = false;
    bool can_block = false;
    uint8_t phase_mask = 0xFF;
    double base_cost = 0;
    std::vector<Pose3> control_points;
    double horizontal_length_m = 0.0;
    double slope_deg = 0.0;
    double center_clearance_m = -1.0;
    double nominal_yaw = 0.0;
    bool same_surface = false;
};

struct TaskDef {
    std::string task_tag;
    std::vector<std::string> candidate_nodes;
    std::string selection_policy;
};

struct RouteDef {
    std::string route_tag;
    std::vector<std::string> nodes;
};

enum class NodeState : uint8_t { FREE, BLOCKED, UNKNOWN };
enum class EdgeState : uint8_t { ENABLED, BLOCKED, SLOW_ONLY, CONFIRM_REQUIRED };

struct NodeOverlay {
    NodeState state = NodeState::FREE;
    double extra_cost = 0;
};

struct EdgeOverlay {
    EdgeState state = EdgeState::ENABLED;
    double extra_cost = 0;
};

struct FieldGraph {
    std::string team;
    std::string schema_version;
    double grid_spacing_m = 1.2;
    std::unordered_map<std::string, GraphNode> nodes;
    std::vector<GraphEdge> edges;
    std::vector<TaskDef> tasks;
    std::unordered_map<std::string, RouteDef> routes;
    // adjacency: node_id -> list of edge indices
    std::unordered_map<std::string, std::vector<size_t>> adjacency;
};

}  // namespace rc26_xhu_nav::topology
