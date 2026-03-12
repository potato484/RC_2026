#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace rc26_decision {

struct MerlinBlockSummary {
    int block_id{0};
    int sampled_cells{0};
    int occupied_cells{0};
    int unknown_occupied_cells{0};
    int keepout_cells{0};
    float min_rule_legality{1.0F};
    float min_traversability{1.0F};

    bool hasSamples() const { return sampled_cells > 0; }
    bool hasUnknownOccupancy() const { return unknown_occupied_cells > 0; }
    bool isOccupied() const { return occupied_cells > 0; }
    bool hasKeepout() const { return keepout_cells > 0; }
};

struct TransitionVerdict {
    bool allowed{false};
    std::string reason{"unknown"};
    MerlinBlockSummary target_summary{};
};

class MerlinRuleWorldModel {
public:
    explicit MerlinRuleWorldModel(rclcpp::Node& node);

    bool isReady() const;
    bool resolveCurrentBlock(int& block_id, std::string* reason = nullptr) const;
    MerlinBlockSummary summarizeBlock(int block_id) const;
    TransitionVerdict canMove(int from_block, int to_block) const;
    std::vector<int> legalGrabTargetsFromEntry(const std::vector<int>& candidates) const;

private:
    void terrainGridCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
    bool copyTerrainMap(std::shared_ptr<grid_map::GridMap>& out) const;
    bool hasTraversableEdge(const grid_map::GridMap& map, int from_block, int to_block) const;
    static bool areGridAdjacent(int a, int b);

    rclcpp::Node& node_;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr terrain_sub_;
    mutable std::mutex map_mutex_;
    std::shared_ptr<grid_map::GridMap> terrain_map_;
    rclcpp::Time terrain_stamp_{0, 0, RCL_ROS_TIME};

    std::string terrain_grid_topic_{"/terrain_grid_map"};
    std::string base_frame_{"base_link"};
    std::string block_id_layer_{"block_id"};
    std::string block_occupied_layer_{"block_occupied"};
    std::string keepout_layer_{"kfs_keepout"};
    std::string traversable_edge_layer_{"traversable_edge_mask"};
    std::string rule_legality_layer_{"rule_legality"};
    std::string traversability_layer_{"traversability"};
    std::string fresh_layer_{"fresh"};
    double tf_timeout_sec_{0.1};
    double map_stale_timeout_sec_{1.0};

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace rc26_decision
