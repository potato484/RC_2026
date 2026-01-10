#include "rc26_decision/navigation/waypoint_catalog.hpp"

namespace rc26_decision
{

namespace
{
std::vector<Waypoint> buildCatalog()
{
    std::vector<Waypoint> waypoints;
    waypoints.reserve(10);

    // NAV_NORMAL (ID 1-4): 正常行驶点，速度 1.5 m/s
    for (uint8_t id = 1; id <= 4; ++id)
    {
        Waypoint wp;
        wp.id = id;
        wp.mode = NavMode::Normal;
        wp.speed_mps = 1.5f;
        wp.x = static_cast<double>(id) * 1.0;
        wp.y = 0.0;
        wp.yaw = 0.0;
        wp.name = "normal_" + std::to_string(static_cast<int>(id));
        waypoints.push_back(std::move(wp));
    }

    // NAV_STAIR_UP (ID 5-7): 攀爬阶梯点，速度 0.5 m/s
    for (uint8_t id = 5; id <= 7; ++id)
    {
        Waypoint wp;
        wp.id = id;
        wp.mode = NavMode::StairUp;
        wp.speed_mps = 0.5f;
        wp.x = static_cast<double>(id) * 1.0;
        wp.y = 0.0;
        wp.yaw = 0.0;
        wp.name = "stair_up_" + std::to_string(static_cast<int>(id - 4));
        waypoints.push_back(std::move(wp));
    }

    // NAV_STAIR_DOWN (ID 8-10): 下阶梯点，速度 0.3 m/s
    for (uint8_t id = 8; id <= 10; ++id)
    {
        Waypoint wp;
        wp.id = id;
        wp.mode = NavMode::StairDown;
        wp.speed_mps = 0.3f;
        wp.x = static_cast<double>(id) * 1.0;
        wp.y = 0.0;
        wp.yaw = 0.0;
        wp.name = "stair_down_" + std::to_string(static_cast<int>(id - 7));
        waypoints.push_back(std::move(wp));
    }

    return waypoints;
}
}  // namespace

const std::vector<Waypoint>& waypointCatalog()
{
    static const std::vector<Waypoint> kCatalog = buildCatalog();
    return kCatalog;
}

const Waypoint* findWaypoint(uint8_t id)
{
    const auto& catalog = waypointCatalog();
    for (const auto& wp : catalog)
    {
        if (wp.id == id)
        {
            return &wp;
        }
    }
    return nullptr;
}

}  // namespace rc26_decision
