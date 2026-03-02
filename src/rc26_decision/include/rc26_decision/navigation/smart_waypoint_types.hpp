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

struct SmartWaypointSpec {
    Pose2D pose;
    std::string strategy_tag;
    NavTolerance tolerance;
    std::string nav_profile = "normal";
    std::string speed_profile;
    float timeout_sec = 0.0f;
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
