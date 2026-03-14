#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "filters/filter_chain.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rc26_interfaces/msg/mf_kfs_state.hpp"
#include "rc26_interfaces/msg/terrain_feature_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rc26_terrain {

class TerrainGridMapBridge : public rclcpp::Node {
public:
  explicit TerrainGridMapBridge(const rclcpp::NodeOptions &options);

private:
  enum class BlockOccupiedState : int8_t {
    kUnknown = -1,
    kFree = 0,
    kOccupied = 1,
  };

  struct MfCell {
    double x{0.0};
    double y{0.0};
    double expected_height_m{0.0};
    bool has_expected_height{false};
    bool valid{false};
  };

  struct AxisAlignedZone {
    double x_min{0.0};
    double x_max{0.0};
    double y_min{0.0};
    double y_max{0.0};

    bool contains(double x, double y) const {
      return x >= x_min && x <= x_max && y >= y_min && y <= y_max;
    }
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

  void featureCallback(
      const rc26_interfaces::msg::TerrainFeatureGrid::ConstSharedPtr &msg);
  void keepoutCallback(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &msg);
  void mfKfsStateCallback(
      const rc26_interfaces::msg::MfKfsState::ConstSharedPtr &msg);

  bool
  validateFeatureMessage(const rc26_interfaces::msg::TerrainFeatureGrid &msg,
                         std::string &reason) const;
  bool loadMfGridLayout(const std::string &path);
  bool resolveLayoutPath(const std::string &raw_path,
                         std::string &resolved_path) const;
  std::optional<double>
  expectedHeightForGridId(int grid_id, const std::string &runtime_team) const;
  int resolveBlockId(double x_map, double y_map) const;
  std::optional<float>
  sampleKeepoutValue(const nav_msgs::msg::OccupancyGrid &grid, double x_map,
                     double y_map) const;
  void publishDiagnostics(const rclcpp::Time &stamp,
                          const DiagnosticsState &state) const;
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
  std::string mf_world_layout_file_{""};
  std::string mf_grid_layout_file_{""};
  std::string mf_kfs_state_topic_{"/mf_kfs_state"};
  double mf_kfs_min_confidence_{0.6};
  bool rule_legality_enforce_ramp_corridor_{false};
  bool enable_filter_chain_{false};
  std::string filter_chain_parameter_name_{"terrain_filter_chain"};
  double step_edge_height_thresh_m_{0.10};
  double slope_norm_limit_{0.35};
  double roughness_norm_limit_{0.08};
  double height_error_limit_m_{0.25};
  double traversability_height_error_weight_{0.20};
  double traversability_step_edge_weight_{0.20};
  double shared_edge_band_width_m_{0.08};
  double traversable_edge_height_delta_limit_m_{0.25};

  // MF layout state.
  std::array<MfCell, 13> mf_cells_{};
  mutable std::mutex mf_state_mutex_;
  std::array<BlockOccupiedState, 13> block_occupied_states_{};
  std::array<float, 13> block_occupied_confidences_{};
  bool mf_layout_ready_{false};
  std::string mf_layout_team_;
  std::string runtime_team_;
  double mf_grid_spacing_m_{1.2};
  double mf_block_half_extent_m_{0.6};
  std::vector<AxisAlignedZone> ramp_corridor_zones_;
  std::vector<AxisAlignedZone> battle_approach_zones_;
  std::string mf_layout_status_;

  // ROS interfaces.
  rclcpp::Subscription<rc26_interfaces::msg::TerrainFeatureGrid>::SharedPtr
      sub_features_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_keepout_;
  rclcpp::Subscription<rc26_interfaces::msg::MfKfsState>::SharedPtr
      sub_mf_kfs_state_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_grid_map_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_grid_map_local_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_grid_map_raw_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr
      pub_grid_map_local_raw_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      pub_marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      pub_marker_array_local_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      pub_diagnostics_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  filters::FilterChain<grid_map::GridMap> filter_chain_{"grid_map::GridMap"};
  bool filter_chain_ready_{false};
  std::optional<grid_map::GridMap> global_raw_map_;
  std::optional<grid_map::GridMap> local_raw_map_;
  std::optional<grid_map::GridMap> fused_map_;
  rclcpp::Time last_fusion_stamp_{0, 0, RCL_ROS_TIME};

  // Cached keepout grid.
  mutable std::mutex keepout_mutex_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr keepout_mask_;
  rclcpp::Time keepout_receive_time_{0, 0, RCL_ROS_TIME};
  bool keepout_received_{false};
};

} // namespace rc26_terrain
