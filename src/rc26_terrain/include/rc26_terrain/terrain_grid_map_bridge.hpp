#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rc26_interfaces/msg/terrain_feature_grid.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rc26_terrain {

class TerrainGridMapBridge : public rclcpp::Node {
public:
    explicit TerrainGridMapBridge(const rclcpp::NodeOptions& options);

private:
    struct MfCell {
        double x{0.0};
        double y{0.0};
        bool valid{false};
    };

    struct DiagnosticsState {
        bool feature_valid{true};
        bool tf_ok{true};
        bool keepout_available{true};
        bool keepout_stale{false};
        bool mf_layout_ready{true};
        bool team_valid{true};
        std::string detail;
    };

    void featureCallback(const rc26_interfaces::msg::TerrainFeatureGrid::ConstSharedPtr& msg);
    void keepoutCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr& msg);

    bool validateFeatureMessage(const rc26_interfaces::msg::TerrainFeatureGrid& msg,
                                std::string& reason) const;
    bool loadMfGridLayout(const std::string& path);
    std::optional<double> expectedHeightForGridId(int grid_id) const;
    int resolveBlockId(double x_map, double y_map) const;
    std::optional<float> sampleKeepoutValue(const nav_msgs::msg::OccupancyGrid& grid,
                                            double x_map, double y_map) const;
    void publishDiagnostics(const rclcpp::Time& stamp, const DiagnosticsState& state) const;
    static std::string toLower(std::string value);

    // Parameters.
    std::string terrain_features_topic_{"terrain_features"};
    std::string kfs_mask_topic_{"/kfs_filter_mask"};
    std::string output_topic_{"/terrain_grid_map"};
    std::string output_topic_local_{"/terrain_grid_map_local"};
    std::string output_topic_raw_{"/terrain_grid_map_raw"};
    std::string output_topic_local_raw_{"/terrain_grid_map_local_raw"};
    std::string output_marker_topic_{"/terrain_grid_map_markers"};
    std::string output_marker_topic_local_{"/terrain_grid_map_local_markers"};
    std::string diagnostics_topic_{"diagnostics"};
    std::string map_frame_{"map"};
    std::string local_frame_{"odom"};
    std::string base_frame_{"base_link"};
    double tf_timeout_sec_{0.1};
    double keepout_stale_timeout_sec_{2.0};
    bool publish_local_map_{true};
    bool publish_marker_array_{true};
    bool fusion_enable_{true};
    bool fusion_publish_raw_{true};
    double fusion_time_constant_sec_{0.7};
    double fusion_unknown_decay_sec_{1.2};
    double marker_height_min_m_{0.03};
    double marker_height_scale_m_{0.20};
    double marker_alpha_{0.85};
    bool enable_mf_semantics_{true};
    std::string mf_grid_layout_file_{""};
    double step_edge_height_thresh_m_{0.10};
    double slope_norm_limit_{0.35};
    double roughness_norm_limit_{0.08};
    double height_error_limit_m_{0.25};
    double traversability_height_error_weight_{0.20};
    double traversability_step_edge_weight_{0.20};

    // MF layout state.
    std::array<MfCell, 13> mf_cells_{};
    bool mf_layout_ready_{false};
    std::string mf_layout_team_;
    double mf_grid_spacing_m_{1.2};
    double mf_block_half_extent_m_{0.6};
    std::string mf_layout_status_;

    // ROS interfaces.
    rclcpp::Subscription<rc26_interfaces::msg::TerrainFeatureGrid>::SharedPtr sub_features_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_keepout_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_grid_map_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_grid_map_local_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_grid_map_raw_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_grid_map_local_raw_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_array_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_array_local_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::optional<grid_map::GridMap> fused_map_;
    rclcpp::Time last_fusion_stamp_{0, 0, RCL_ROS_TIME};

    // Cached keepout grid.
    mutable std::mutex keepout_mutex_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr keepout_mask_;
    rclcpp::Time keepout_receive_time_{0, 0, RCL_ROS_TIME};
    bool keepout_received_{false};
};

}  // namespace rc26_terrain
