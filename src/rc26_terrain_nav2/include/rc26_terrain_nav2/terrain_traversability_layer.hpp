#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rc26_terrain_nav2 {

unsigned char mapTraversabilityToCost(float traversability,
                                      double lethal_threshold,
                                      double inscribed_threshold);

class TerrainTraversabilityLayer : public nav2_costmap_2d::Layer {
public:
    TerrainTraversabilityLayer();
    ~TerrainTraversabilityLayer() override = default;

    void onInitialize() override;
    void updateBounds(double robot_x, double robot_y, double robot_yaw,
                      double* min_x, double* min_y, double* max_x, double* max_y) override;
    void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                     int min_i, int min_j, int max_i, int max_j) override;
    void reset() override;
    bool isClearable() override;
    void matchSize() override;

private:
    void terrainGridCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
    bool readLayerValue(const grid_map::GridMap& map,
                        const std::string& layer,
                        const grid_map::Position& pos,
                        float& value) const;

    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr terrain_sub_;
    mutable std::mutex terrain_mutex_;
    std::shared_ptr<grid_map::GridMap> terrain_map_;
    rclcpp::Time last_terrain_stamp_{0, 0, RCL_ROS_TIME};

    std::string terrain_grid_topic_{"/terrain_grid_map_local"};
    std::string traversability_layer_{"traversability"};
    std::string fresh_layer_{"fresh"};
    std::string drop_layer_{"drop_prob"};
    std::string climbable_layer_{"climbable_prob"};
    double lethal_threshold_{0.25};
    double inscribed_threshold_{0.45};
    double drop_lethal_threshold_{0.8};
    double climbable_soft_cost_max_{80.0};
    double stale_timeout_sec_{0.0};
    std::string unknown_policy_{"keep"};
};

}  // namespace rc26_terrain_nav2
