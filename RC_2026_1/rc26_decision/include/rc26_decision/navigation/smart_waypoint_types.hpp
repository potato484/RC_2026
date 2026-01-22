#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace rc26_decision {

struct Point2D {
    double x = 0.0;
    double y = 0.0;
};

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

struct NavTolerance {
    double xy_tolerance = 0.25;
    double yaw_tolerance = 0.25;
};

enum class NavSafetyMode : uint8_t {
    NORMAL = 0,
    MF_SAFE = 1,
    MF_TRAVERSE = 2,
    MF_EXIT = 3,
};

enum class McuNavMode : uint8_t {
    Normal = 0,
    StairUp = 1,
    StairDown = 2,
};

struct McuNavConfig {
    bool enabled = false;
    McuNavMode mode = McuNavMode::Normal;
    float speed_mps = 0.0f;
};

struct SmartWaypointSpec {
    Pose2D pose;
    std::string strategy_tag;
    NavTolerance tolerance;
    NavSafetyMode nav_safety_mode = NavSafetyMode::NORMAL;
    std::string speed_profile;
    float timeout_sec = 0.0f;
    McuNavConfig mcu;
    std::unordered_map<std::string, double> payload;
};

struct MerlinAnchors {
    Point2D block_1;
    Point2D block_2;
    Point2D block_4;
};

struct MerlinParams {
    double safe_offset = 0.20;
    double jump_margin = 0.20;
};

struct MerlinGraph {
    std::unordered_map<int, std::vector<int>> adjacency;
};

}  // namespace rc26_decision
