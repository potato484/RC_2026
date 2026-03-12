#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rc26_terrain_nav2/terrain_speed_limit_bridge.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rc26_terrain_nav2::TerrainSpeedLimitBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
