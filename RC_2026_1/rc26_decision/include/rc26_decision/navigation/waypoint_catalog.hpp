#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace rc26_decision {

enum class NavMode : uint8_t {
    Normal = 0,
    StairUp = 1,
    StairDown = 2,
};

struct Waypoint {
    uint8_t id = 0;
    NavMode mode = NavMode::Normal;
    float speed_mps = 0.0f;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    std::string name;
};

const std::vector<Waypoint>& waypointCatalog();
const Waypoint* findWaypoint(uint8_t id);

}  // namespace rc26_decision
