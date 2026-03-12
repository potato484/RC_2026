#include "rc26_terrain/terrain_grid_map_bridge.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_core/iterators/SubmapIterator.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "yaml-cpp/yaml.h"

namespace {

constexpr float kNaNf = std::numeric_limits<float>::quiet_NaN();
constexpr const char *kLayerElevationAbs = "elevation_abs";
constexpr const char *kLayerElevationTopAbs = "elevation_top_abs";
constexpr const char *kLayerSigmaH = "sigma_h";
constexpr const char *kLayerFresh = "fresh";
constexpr const char *kLayerDensity = "density";
constexpr const char *kLayerSlope = "slope";
constexpr const char *kLayerSlopeX = "slope_x";
constexpr const char *kLayerSlopeY = "slope_y";
constexpr const char *kLayerRoughness = "roughness";
constexpr const char *kLayerObstacleProb = "obstacle_prob";
constexpr const char *kLayerDropProb = "drop_prob";
constexpr const char *kLayerStepUp = "step_up";
constexpr const char *kLayerClimbableProb = "climbable_prob";
constexpr const char *kLayerKfsKeepout = "kfs_keepout";

constexpr const char *kLayerBlockId = "block_id";
constexpr const char *kLayerExpectedHeight = "expected_height";
constexpr const char *kLayerHeightError = "height_error";
constexpr const char *kLayerBlockOccupied = "block_occupied";
constexpr const char *kLayerTraversableEdgeMask = "traversable_edge_mask";
constexpr const char *kLayerRampCorridorMask = "ramp_corridor_mask";
constexpr const char *kLayerBattleApproachMask = "battle_approach_mask";
constexpr const char *kLayerRuleLegality = "rule_legality";
constexpr const char *kLayerTraversabilityContinuous =
    "traversability_continuous";
constexpr const char *kLayerEdgeStrength = "edge_strength";
constexpr const char *kLayerStepEdgeMask = "step_edge_mask";
constexpr const char *kLayerTraversability = "traversability";
constexpr const char *kLayerAgeSec = "age_sec";
constexpr const char *kLayerHitCount = "hit_count";
constexpr const char *kLayerElevationFused = "elevation_fused";
constexpr const char *kLayerTraversabilityFused = "traversability_fused";
constexpr const char *kLayerStepEdgeConfidence = "step_edge_confidence";

inline rclcpp::QoS makeFeatureQos() {
  return rclcpp::QoS(rclcpp::KeepLast(5))
      .best_effort()
      .durability(rclcpp::DurabilityPolicy::Volatile);
}

inline rclcpp::QoS makeKeepoutQos() {
  return rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable()
      .durability(rclcpp::DurabilityPolicy::TransientLocal);
}

inline rclcpp::QoS makeOutputQos() {
  return rclcpp::QoS(rclcpp::KeepLast(1))
      .reliable()
      .durability(rclcpp::DurabilityPolicy::TransientLocal);
}

inline rclcpp::QoS makeDiagnosticsQos() {
  return rclcpp::QoS(rclcpp::KeepLast(10))
      .reliable()
      .durability(rclcpp::DurabilityPolicy::Volatile);
}

inline float clamp01(float value) {
  if (!std::isfinite(static_cast<double>(value))) {
    return 0.0f;
  }
  return std::clamp(value, 0.0f, 1.0f);
}

inline bool isFinite(float value) {
  return std::isfinite(static_cast<double>(value));
}

int mirrorGridIdAcrossColumns(int grid_id) {
  if (grid_id < 1 || grid_id > 12) {
    return grid_id;
  }
  const int row = (grid_id - 1) / 3;
  const int col = (grid_id - 1) % 3;
  return row * 3 + (2 - col) + 1;
}

std_msgs::msg::ColorRGBA makeTerrainColor(float traversability,
                                          float obstacle_prob, float drop_prob,
                                          float alpha) {
  std_msgs::msg::ColorRGBA color;
  const float trav = clamp01(traversability);
  const float obstacle = clamp01(obstacle_prob);
  const float drop = clamp01(drop_prob);

  if (drop > obstacle && drop > 0.45f) {
    color.r = 0.20f + 0.30f * (1.0f - drop);
    color.g = 0.35f + 0.25f * (1.0f - drop);
    color.b = 0.85f;
  } else if (obstacle > 0.45f) {
    color.r = 0.95f;
    color.g = 0.35f + 0.25f * (1.0f - obstacle);
    color.b = 0.18f;
  } else {
    color.r = 0.95f * (1.0f - trav);
    color.g = 0.25f + 0.70f * trav;
    color.b = 0.18f + 0.30f * trav;
  }
  color.a = clamp01(alpha);
  return color;
}

visualization_msgs::msg::MarkerArray
makeTerrainMarkerArray(const grid_map::GridMap &map, const rclcpp::Time &stamp,
                       const std::string &marker_ns, double marker_height_min_m,
                       double marker_height_scale, double marker_alpha) {
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  visualization_msgs::msg::Marker cubes;
  cubes.header.frame_id = map.getFrameId();
  cubes.header.stamp = stamp;
  cubes.ns = marker_ns;
  cubes.id = 0;
  cubes.type = visualization_msgs::msg::Marker::CUBE_LIST;
  cubes.action = visualization_msgs::msg::Marker::ADD;
  cubes.pose.orientation.w = 1.0;
  cubes.frame_locked = true;
  cubes.scale.x = std::max(1e-3, map.getResolution() * 0.96);
  cubes.scale.y = std::max(1e-3, map.getResolution() * 0.96);
  // Keep cubes thin so the height is dominated by Z placement, not by extrusion
  // thickness.
  cubes.scale.z =
      std::max(marker_height_min_m, map.getResolution() * marker_height_scale);

  const bool has_traversability = map.exists(kLayerTraversability);
  const bool has_obstacle = map.exists(kLayerObstacleProb);
  const bool has_drop = map.exists(kLayerDropProb);

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index index(*it);
    if (!map.isValid(index, kLayerElevationAbs)) {
      continue;
    }

    grid_map::Position position;
    if (!map.getPosition(index, position)) {
      continue;
    }

    geometry_msgs::msg::Point point;
    point.x = position.x();
    point.y = position.y();
    point.z = static_cast<double>(map.at(kLayerElevationAbs, index));
    cubes.points.push_back(point);

    const float traversability =
        has_traversability && map.isValid(index, kLayerTraversability)
            ? map.at(kLayerTraversability, index)
            : 0.5f;
    const float obstacle_prob =
        has_obstacle && map.isValid(index, kLayerObstacleProb)
            ? map.at(kLayerObstacleProb, index)
            : 0.0f;
    const float drop_prob = has_drop && map.isValid(index, kLayerDropProb)
                                ? map.at(kLayerDropProb, index)
                                : 0.0f;
    cubes.colors.push_back(makeTerrainColor(traversability, obstacle_prob,
                                            drop_prob,
                                            static_cast<float>(marker_alpha)));
  }

  markers.markers.push_back(std::move(cubes));
  return markers;
}

} // namespace

namespace rc26_terrain {

TerrainGridMapBridge::TerrainGridMapBridge(const rclcpp::NodeOptions &options)
    : Node("terrain_grid_map_bridge", options) {
  this->declare_parameter<std::string>("terrain_features_topic",
                                       terrain_features_topic_);
  this->declare_parameter<std::string>("kfs_mask_topic", kfs_mask_topic_);
  this->declare_parameter<std::string>("output_topic", output_topic_);
  this->declare_parameter<bool>("publish_local_map", publish_local_map_);
  this->declare_parameter<std::string>("output_topic_local",
                                       output_topic_local_);
  this->declare_parameter<bool>("publish_marker_array", publish_marker_array_);
  this->declare_parameter<std::string>("output_marker_topic",
                                       output_marker_topic_);
  this->declare_parameter<std::string>("output_marker_topic_local",
                                       output_marker_topic_local_);
  this->declare_parameter<bool>("fusion_enable", fusion_enable_);
  this->declare_parameter<bool>("fusion_publish_raw", fusion_publish_raw_);
  this->declare_parameter<std::string>("output_topic_raw", output_topic_raw_);
  this->declare_parameter<std::string>("output_topic_local_raw",
                                       output_topic_local_raw_);
  this->declare_parameter<double>("fusion_time_constant_sec",
                                  fusion_time_constant_sec_);
  this->declare_parameter<double>("fusion_unknown_decay_sec",
                                  fusion_unknown_decay_sec_);
  this->declare_parameter<double>("marker_height_min_m", marker_height_min_m_);
  this->declare_parameter<double>("marker_height_scale_m",
                                  marker_height_scale_m_);
  this->declare_parameter<double>("marker_alpha", marker_alpha_);
  this->declare_parameter<std::string>("map_frame", map_frame_);
  this->declare_parameter<std::string>("local_frame", local_frame_);
  this->declare_parameter<std::string>("base_frame", base_frame_);
  this->declare_parameter<double>("tf_timeout_sec", tf_timeout_sec_);
  this->declare_parameter<double>("keepout_stale_timeout_sec",
                                  keepout_stale_timeout_sec_);
  this->declare_parameter<bool>("enable_mf_semantics", enable_mf_semantics_);
  this->declare_parameter<std::string>("mf_world_layout_file",
                                       mf_world_layout_file_);
  this->declare_parameter<std::string>("mf_grid_layout_file",
                                       mf_grid_layout_file_);
  this->declare_parameter<std::string>("mf_kfs_state_topic",
                                       mf_kfs_state_topic_);
  this->declare_parameter<double>("mf_kfs_min_confidence",
                                  mf_kfs_min_confidence_);
  this->declare_parameter<bool>("rule_legality_enforce_ramp_corridor",
                                rule_legality_enforce_ramp_corridor_);
  this->declare_parameter<bool>("enable_filter_chain", enable_filter_chain_);
  this->declare_parameter<std::string>("filter_chain_parameter_name",
                                       filter_chain_parameter_name_);
  this->declare_parameter<double>("step_edge_height_thresh_m",
                                  step_edge_height_thresh_m_);
  this->declare_parameter<double>("traversable_edge_height_delta_limit_m",
                                  traversable_edge_height_delta_limit_m_);
  this->declare_parameter<double>("slope_norm_limit", slope_norm_limit_);
  this->declare_parameter<double>("roughness_norm_limit",
                                  roughness_norm_limit_);
  this->declare_parameter<double>("height_error_limit_m",
                                  height_error_limit_m_);
  this->declare_parameter<double>("traversability_height_error_weight",
                                  traversability_height_error_weight_);
  this->declare_parameter<double>("traversability_step_edge_weight",
                                  traversability_step_edge_weight_);
  this->declare_parameter<std::string>("diagnostics_topic", diagnostics_topic_);

  this->get_parameter("terrain_features_topic", terrain_features_topic_);
  this->get_parameter("kfs_mask_topic", kfs_mask_topic_);
  this->get_parameter("output_topic", output_topic_);
  this->get_parameter("publish_local_map", publish_local_map_);
  this->get_parameter("output_topic_local", output_topic_local_);
  this->get_parameter("publish_marker_array", publish_marker_array_);
  this->get_parameter("output_marker_topic", output_marker_topic_);
  this->get_parameter("output_marker_topic_local", output_marker_topic_local_);
  this->get_parameter("fusion_enable", fusion_enable_);
  this->get_parameter("fusion_publish_raw", fusion_publish_raw_);
  this->get_parameter("output_topic_raw", output_topic_raw_);
  this->get_parameter("output_topic_local_raw", output_topic_local_raw_);
  this->get_parameter("fusion_time_constant_sec", fusion_time_constant_sec_);
  this->get_parameter("fusion_unknown_decay_sec", fusion_unknown_decay_sec_);
  this->get_parameter("marker_height_min_m", marker_height_min_m_);
  this->get_parameter("marker_height_scale_m", marker_height_scale_m_);
  this->get_parameter("marker_alpha", marker_alpha_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("local_frame", local_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("tf_timeout_sec", tf_timeout_sec_);
  this->get_parameter("keepout_stale_timeout_sec", keepout_stale_timeout_sec_);
  this->get_parameter("enable_mf_semantics", enable_mf_semantics_);
  this->get_parameter("mf_world_layout_file", mf_world_layout_file_);
  this->get_parameter("mf_grid_layout_file", mf_grid_layout_file_);
  this->get_parameter("mf_kfs_state_topic", mf_kfs_state_topic_);
  this->get_parameter("mf_kfs_min_confidence", mf_kfs_min_confidence_);
  this->get_parameter("rule_legality_enforce_ramp_corridor",
                      rule_legality_enforce_ramp_corridor_);
  this->get_parameter("enable_filter_chain", enable_filter_chain_);
  this->get_parameter("filter_chain_parameter_name",
                      filter_chain_parameter_name_);
  this->get_parameter("step_edge_height_thresh_m", step_edge_height_thresh_m_);
  this->get_parameter("traversable_edge_height_delta_limit_m",
                      traversable_edge_height_delta_limit_m_);
  this->get_parameter("slope_norm_limit", slope_norm_limit_);
  this->get_parameter("roughness_norm_limit", roughness_norm_limit_);
  this->get_parameter("height_error_limit_m", height_error_limit_m_);
  this->get_parameter("traversability_height_error_weight",
                      traversability_height_error_weight_);
  this->get_parameter("traversability_step_edge_weight",
                      traversability_step_edge_weight_);
  this->get_parameter("diagnostics_topic", diagnostics_topic_);

  tf_timeout_sec_ = std::max(0.01, tf_timeout_sec_);
  keepout_stale_timeout_sec_ = std::max(0.01, keepout_stale_timeout_sec_);
  fusion_time_constant_sec_ = std::max(1e-3, fusion_time_constant_sec_);
  fusion_unknown_decay_sec_ = std::max(0.0, fusion_unknown_decay_sec_);
  marker_height_min_m_ = std::max(1e-3, marker_height_min_m_);
  marker_height_scale_m_ = std::max(0.0, marker_height_scale_m_);
  marker_alpha_ = std::clamp(marker_alpha_, 0.0, 1.0);
  mf_kfs_min_confidence_ = std::clamp(mf_kfs_min_confidence_, 0.0, 1.0);
  step_edge_height_thresh_m_ = std::max(0.0, step_edge_height_thresh_m_);
  traversable_edge_height_delta_limit_m_ =
      std::max(1e-6, traversable_edge_height_delta_limit_m_);
  slope_norm_limit_ = std::max(1e-6, slope_norm_limit_);
  roughness_norm_limit_ = std::max(1e-6, roughness_norm_limit_);
  height_error_limit_m_ = std::max(1e-6, height_error_limit_m_);
  traversability_height_error_weight_ =
      std::clamp(traversability_height_error_weight_, 0.0, 1.0);
  traversability_step_edge_weight_ =
      std::clamp(traversability_step_edge_weight_, 0.0, 1.0);
  block_occupied_states_.fill(BlockOccupiedState::kUnknown);
  block_occupied_confidences_.fill(0.0f);

  if (enable_filter_chain_) {
    filter_chain_ready_ = filter_chain_.configure(
        filter_chain_parameter_name_, this->get_node_logging_interface(),
        this->get_node_parameters_interface());
    if (!filter_chain_ready_) {
      RCLCPP_WARN(this->get_logger(),
                  "terrain filter chain configure failed (parameter=%s), "
                  "fallback to C++ rules",
                  filter_chain_parameter_name_.c_str());
      enable_filter_chain_ = false;
    }
  }

  if (enable_mf_semantics_) {
    std::vector<std::string> layout_candidates;
    if (!mf_world_layout_file_.empty()) {
      layout_candidates.push_back(mf_world_layout_file_);
    }
    if (!mf_grid_layout_file_.empty() &&
        mf_grid_layout_file_ != mf_world_layout_file_) {
      layout_candidates.push_back(mf_grid_layout_file_);
    }
    if (layout_candidates.empty()) {
      try {
        const auto keepout_share =
            ament_index_cpp::get_package_share_directory("rc26_kfs_keepout");
        layout_candidates.push_back(keepout_share + "/config/r2_mf_world.yaml");
        layout_candidates.push_back(keepout_share +
                                    "/config/mf_grid_layout.yaml");
      } catch (const std::exception &ex) {
        mf_layout_status_ =
            std::string("cannot resolve default mf layout: ") + ex.what();
        mf_layout_ready_ = false;
      }
    }
    for (const auto &candidate : layout_candidates) {
      if (candidate.empty()) {
        continue;
      }
      if (loadMfGridLayout(candidate)) {
        mf_layout_ready_ = true;
        mf_world_layout_file_ = candidate;
        break;
      }
    }
    if (!mf_layout_ready_) {
      RCLCPP_WARN(
          this->get_logger(),
          "MF semantic layout unavailable, semantic layers will remain NaN: %s",
          mf_layout_status_.empty() ? "unknown reason"
                                    : mf_layout_status_.c_str());
    }
  } else {
    mf_layout_ready_ = false;
    mf_layout_status_ = "MF semantics disabled by parameter";
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  pub_grid_map_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
      output_topic_, makeOutputQos());
  if (publish_local_map_) {
    pub_grid_map_local_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        output_topic_local_, makeOutputQos());
  }
  if (publish_marker_array_) {
    pub_marker_array_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            output_marker_topic_, makeOutputQos());
    if (publish_local_map_) {
      pub_marker_array_local_ =
          this->create_publisher<visualization_msgs::msg::MarkerArray>(
              output_marker_topic_local_, makeOutputQos());
    }
  }
  if (fusion_publish_raw_) {
    pub_grid_map_raw_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        output_topic_raw_, makeOutputQos());
    if (publish_local_map_) {
      pub_grid_map_local_raw_ =
          this->create_publisher<grid_map_msgs::msg::GridMap>(
              output_topic_local_raw_, makeOutputQos());
    }
  }
  pub_diagnostics_ =
      this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          diagnostics_topic_, makeDiagnosticsQos());

  sub_features_ =
      this->create_subscription<rc26_interfaces::msg::TerrainFeatureGrid>(
          terrain_features_topic_, makeFeatureQos(),
          std::bind(&TerrainGridMapBridge::featureCallback, this,
                    std::placeholders::_1));

  sub_keepout_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      kfs_mask_topic_, makeKeepoutQos(),
      std::bind(&TerrainGridMapBridge::keepoutCallback, this,
                std::placeholders::_1));
  sub_mf_kfs_state_ =
      this->create_subscription<rc26_interfaces::msg::MfKfsState>(
          mf_kfs_state_topic_, rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
          std::bind(&TerrainGridMapBridge::mfKfsStateCallback, this,
                    std::placeholders::_1));

  RCLCPP_INFO(
      this->get_logger(),
      "terrain_grid_map_bridge started: feature_topic=%s keepout_topic=%s "
      "output_topic=%s map_frame=%s "
      "publish_local_map=%s output_topic_local=%s local_frame=%s "
      "fusion_enable=%s "
      "fusion_publish_raw=%s output_topic_raw=%s output_topic_local_raw=%s "
      "publish_marker_array=%s output_marker_topic=%s "
      "output_marker_topic_local=%s "
      "mf_kfs_state_topic=%s mf_layout_file=%s fusion_tau=%.2f "
      "fusion_decay=%.2f "
      "filter_chain=%s filter_chain_param=%s",
      terrain_features_topic_.c_str(), kfs_mask_topic_.c_str(),
      output_topic_.c_str(), map_frame_.c_str(),
      publish_local_map_ ? "true" : "false", output_topic_local_.c_str(),
      local_frame_.c_str(), fusion_enable_ ? "true" : "false",
      fusion_publish_raw_ ? "true" : "false", output_topic_raw_.c_str(),
      output_topic_local_raw_.c_str(), publish_marker_array_ ? "true" : "false",
      output_marker_topic_.c_str(), output_marker_topic_local_.c_str(),
      mf_kfs_state_topic_.c_str(), mf_world_layout_file_.c_str(),
      fusion_time_constant_sec_, fusion_unknown_decay_sec_,
      (enable_filter_chain_ && filter_chain_ready_) ? "enabled" : "disabled",
      filter_chain_parameter_name_.c_str());
}

void TerrainGridMapBridge::keepoutCallback(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr &msg) {
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(keepout_mutex_);
  keepout_mask_ = msg;
  keepout_receive_time_ = this->get_clock()->now();
  keepout_received_ = true;
}

void TerrainGridMapBridge::mfKfsStateCallback(
    const rc26_interfaces::msg::MfKfsState::ConstSharedPtr &msg) {
  if (!msg) {
    return;
  }

  std::array<BlockOccupiedState, 13> next_states;
  std::array<float, 13> next_confidences;
  next_states.fill(BlockOccupiedState::kUnknown);
  next_confidences.fill(0.0f);

  const std::string msg_team = toLower(msg->team);
  if (!mf_layout_team_.empty() && !msg_team.empty() &&
      msg_team != mf_layout_team_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "mf_kfs_state team=%s differs from layout team=%s, "
                         "applying mirrored layout compatibility",
                         msg_team.c_str(), mf_layout_team_.c_str());
  }

  for (const auto &cell : msg->cells) {
    const int id = static_cast<int>(cell.grid_id);
    if (id < 1 || id > 12) {
      continue;
    }
    const float confidence = std::clamp(cell.confidence, 0.0f, 1.0f);
    if (confidence < static_cast<float>(mf_kfs_min_confidence_)) {
      continue;
    }
    const size_t idx = static_cast<size_t>(id);
    if (confidence + 1e-5f < next_confidences[idx]) {
      continue;
    }

    BlockOccupiedState state = BlockOccupiedState::kUnknown;
    switch (cell.kfs_type) {
    case 0U: // NONE
      state = BlockOccupiedState::kFree;
      break;
    case 1U: // R1
    case 2U: // R2
    case 3U: // FAKE
      state = BlockOccupiedState::kOccupied;
      break;
    default:
      state = BlockOccupiedState::kUnknown;
      break;
    }
    next_states[idx] = state;
    next_confidences[idx] = confidence;
  }

  {
    std::lock_guard<std::mutex> lock(mf_state_mutex_);
    runtime_team_ = msg_team;
    block_occupied_states_ = next_states;
    block_occupied_confidences_ = next_confidences;
  }
}

void TerrainGridMapBridge::featureCallback(
    const rc26_interfaces::msg::TerrainFeatureGrid::ConstSharedPtr &msg) {
  if (!msg) {
    return;
  }

  const bool stamp_is_zero =
      msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0U;
  const rclcpp::Time feature_stamp = stamp_is_zero
                                         ? this->get_clock()->now()
                                         : rclcpp::Time(msg->header.stamp);

  DiagnosticsState diagnostics;
  std::string invalid_reason;
  if (!validateFeatureMessage(*msg, invalid_reason)) {
    diagnostics.feature_valid = false;
    diagnostics.detail = invalid_reason;
    publishDiagnostics(feature_stamp, diagnostics);
    return;
  }
  if (msg->header.frame_id != base_frame_) {
    diagnostics.feature_valid = false;
    diagnostics.detail = "feature frame_id mismatch, expected " + base_frame_ +
                         ", got " + msg->header.frame_id;
    publishDiagnostics(feature_stamp, diagnostics);
    return;
  }

  geometry_msgs::msg::TransformStamped tf_map_base;
  try {
    tf_map_base = tf_buffer_->lookupTransform(
        map_frame_, base_frame_, feature_stamp,
        rclcpp::Duration::from_seconds(tf_timeout_sec_));
  } catch (const tf2::TransformException &ex) {
    diagnostics.tf_ok = false;
    diagnostics.detail = std::string("lookup ") + map_frame_ + "<-" +
                         base_frame_ + " failed: " + ex.what();
    publishDiagnostics(feature_stamp, diagnostics);
    return;
  }

  const auto toPose2D = [](const geometry_msgs::msg::TransformStamped &tf_msg,
                           double &out_x, double &out_y, double &out_z,
                           double &out_cos_yaw, double &out_sin_yaw) {
    out_x = tf_msg.transform.translation.x;
    out_y = tf_msg.transform.translation.y;
    out_z = tf_msg.transform.translation.z;

    tf2::Quaternion q;
    tf2::fromMsg(tf_msg.transform.rotation, q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    (void)roll;
    (void)pitch;
    out_cos_yaw = std::cos(yaw);
    out_sin_yaw = std::sin(yaw);
  };

  double base_x_map = 0.0;
  double base_y_map = 0.0;
  double base_z_map = 0.0;
  double cos_yaw_map = 1.0;
  double sin_yaw_map = 0.0;
  toPose2D(tf_map_base, base_x_map, base_y_map, base_z_map, cos_yaw_map,
           sin_yaw_map);

  double base_x_local = base_x_map;
  double base_y_local = base_y_map;
  double base_z_local = base_z_map;
  double cos_yaw_local = cos_yaw_map;
  double sin_yaw_local = sin_yaw_map;

  if (publish_local_map_) {
    if (local_frame_ == map_frame_) {
      base_x_local = base_x_map;
      base_y_local = base_y_map;
      base_z_local = base_z_map;
      cos_yaw_local = cos_yaw_map;
      sin_yaw_local = sin_yaw_map;
    } else {
      geometry_msgs::msg::TransformStamped tf_local_base;
      try {
        tf_local_base = tf_buffer_->lookupTransform(
            local_frame_, base_frame_, feature_stamp,
            rclcpp::Duration::from_seconds(tf_timeout_sec_));
      } catch (const tf2::TransformException &ex) {
        diagnostics.tf_ok = false;
        diagnostics.detail = std::string("lookup ") + local_frame_ + "<-" +
                             base_frame_ + " failed: " + ex.what();
        publishDiagnostics(feature_stamp, diagnostics);
        return;
      }
      toPose2D(tf_local_base, base_x_local, base_y_local, base_z_local,
               cos_yaw_local, sin_yaw_local);
    }
  }

  const int width = static_cast<int>(msg->width);
  const int height = static_cast<int>(msg->height);
  const int half_width = width / 2;
  const double resolution = static_cast<double>(msg->resolution_m);

  std::vector<std::string> layers = {
      kLayerElevationAbs,  kLayerElevationTopAbs, kLayerSigmaH,
      kLayerFresh,         kLayerDensity,         kLayerSlope,
      kLayerSlopeX,        kLayerSlopeY,          kLayerRoughness,
      kLayerObstacleProb,  kLayerDropProb,        kLayerStepUp,
      kLayerClimbableProb, kLayerKfsKeepout,      kLayerTraversability,
  };
  if (fusion_enable_) {
    layers.push_back(kLayerAgeSec);
    layers.push_back(kLayerHitCount);
    layers.push_back(kLayerElevationFused);
    layers.push_back(kLayerTraversabilityFused);
    layers.push_back(kLayerStepEdgeConfidence);
  }
  if (enable_mf_semantics_) {
    layers.push_back(kLayerBlockId);
    layers.push_back(kLayerExpectedHeight);
    layers.push_back(kLayerHeightError);
    layers.push_back(kLayerBlockOccupied);
    layers.push_back(kLayerTraversableEdgeMask);
    layers.push_back(kLayerRampCorridorMask);
    layers.push_back(kLayerBattleApproachMask);
    layers.push_back(kLayerRuleLegality);
    layers.push_back(kLayerTraversabilityContinuous);
    layers.push_back(kLayerEdgeStrength);
    layers.push_back(kLayerStepEdgeMask);
  }

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr keepout_mask;
  double keepout_age_sec = std::numeric_limits<double>::infinity();
  bool keepout_received = false;
  {
    std::lock_guard<std::mutex> lock(keepout_mutex_);
    keepout_mask = keepout_mask_;
    keepout_received = keepout_received_;
    if (keepout_received) {
      keepout_age_sec =
          (this->get_clock()->now() - keepout_receive_time_).seconds();
    }
  }

  const bool keepout_available =
      keepout_received && keepout_mask &&
      (keepout_age_sec <= keepout_stale_timeout_sec_);
  if (!keepout_received || !keepout_mask) {
    diagnostics.keepout_available = false;
  } else if (!keepout_available) {
    diagnostics.keepout_available = false;
    diagnostics.keepout_stale = true;
  }

  const bool semantics_active = enable_mf_semantics_ && mf_layout_ready_;
  if (enable_mf_semantics_) {
    if (!mf_layout_ready_) {
      diagnostics.mf_layout_ready = false;
    } else {
      const bool team_valid = !mf_layout_team_.empty();
      if (!team_valid) {
        diagnostics.team_valid = false;
      }
    }
  }
  std::array<BlockOccupiedState, 13> block_occupied_states;
  std::string runtime_team;
  {
    std::lock_guard<std::mutex> lock(mf_state_mutex_);
    block_occupied_states = block_occupied_states_;
    runtime_team = runtime_team_;
  }

  const auto srcIndex = [width](int ix, int iy) -> size_t {
    return static_cast<size_t>(ix * width + iy);
  };

  const auto clearBufferRegions =
      [&](grid_map::GridMap &map,
          const std::vector<grid_map::BufferRegion> &regions) {
        if (regions.empty()) {
          return;
        }
        for (const auto &region : regions) {
          for (grid_map::SubmapIterator it(map, region); !it.isPastEnd();
               ++it) {
            const grid_map::Index index(*it);
            for (const auto &layer : layers) {
              if (map.exists(layer)) {
                map.at(layer, index) = kNaNf;
              }
            }
            if (map.exists(kLayerFresh)) {
              map.at(kLayerFresh, index) = 0.0f;
            }
          }
        }
      };

  const auto prepareRollingMap =
      [&](std::optional<grid_map::GridMap> &map_slot,
          const std::string &frame_id, double center_x, double center_y,
          bool reset_fresh_layer) -> grid_map::GridMap & {
    const grid_map::Length map_length(width * resolution, height * resolution);
    const grid_map::Position map_center(center_x, center_y);
    const auto need_reinit = [&]() {
      if (!map_slot.has_value()) {
        return true;
      }
      const auto &map = *map_slot;
      if (map.getFrameId() != frame_id) {
        return true;
      }
      const auto size = map.getSize();
      if (size(0) != width || size(1) != height) {
        return true;
      }
      if (std::abs(map.getResolution() - resolution) > 1e-6) {
        return true;
      }
      for (const auto &layer : layers) {
        if (!map.exists(layer)) {
          return true;
        }
      }
      return false;
    };

    if (need_reinit()) {
      map_slot.emplace(layers);
      map_slot->setFrameId(frame_id);
      map_slot->setGeometry(map_length, resolution, map_center);
      map_slot->setBasicLayers({kLayerElevationAbs});
      for (const auto &layer : layers) {
        (*map_slot)[layer].setConstant(kNaNf);
      }
      if (map_slot->exists(kLayerFresh)) {
        (*map_slot)[kLayerFresh].setZero();
      }
    } else {
      std::vector<grid_map::BufferRegion> new_regions;
      map_slot->move(map_center, new_regions);
      clearBufferRegions(*map_slot, new_regions);
    }

    map_slot->setFrameId(frame_id);
    map_slot->setTimestamp(static_cast<grid_map::Time>(
        std::max<int64_t>(0, feature_stamp.nanoseconds())));
    if (reset_fresh_layer && map_slot->exists(kLayerFresh)) {
      (*map_slot)[kLayerFresh].setZero();
    }
    return *map_slot;
  };

  const auto buildOutputMap = [&](std::optional<grid_map::GridMap> &map_slot,
                                  const std::string &frame_id, double base_x,
                                  double base_y, double base_z, double cos_yaw,
                                  double sin_yaw,
                                  bool sample_keepout) -> grid_map::GridMap & {
    grid_map::GridMap &output_map =
        prepareRollingMap(map_slot, frame_id, base_x, base_y, true);

    if (output_map.exists(kLayerAgeSec)) {
      output_map[kLayerAgeSec].setConstant(kNaNf);
    }
    if (output_map.exists(kLayerHitCount)) {
      output_map[kLayerHitCount].setConstant(kNaNf);
    }
    if (output_map.exists(kLayerElevationFused)) {
      output_map[kLayerElevationFused].setConstant(kNaNf);
    }
    if (output_map.exists(kLayerTraversabilityFused)) {
      output_map[kLayerTraversabilityFused].setConstant(kNaNf);
    }
    if (output_map.exists(kLayerStepEdgeConfidence)) {
      output_map[kLayerStepEdgeConfidence].setConstant(kNaNf);
    }
    if (sample_keepout && !keepout_available &&
        output_map.exists(kLayerKfsKeepout)) {
      output_map[kLayerKfsKeepout].setConstant(kNaNf);
    }

    for (grid_map::GridMapIterator it(output_map); !it.isPastEnd(); ++it) {
      const grid_map::Index index(*it);
      grid_map::Position position;
      if (!output_map.getPosition(index, position)) {
        continue;
      }

      const double dx = position.x() - base_x;
      const double dy = position.y() - base_y;
      const double x_bl = cos_yaw * dx + sin_yaw * dy;
      const double y_bl = -sin_yaw * dx + cos_yaw * dy;

      const int ix =
          static_cast<int>(std::llround(x_bl / resolution)) + half_width;
      const int iy =
          static_cast<int>(std::llround(y_bl / resolution)) + half_width;
      if (ix < 0 || ix >= width || iy < 0 || iy >= height) {
        continue;
      }

      const size_t source_idx = srcIndex(ix, iy);
      if (source_idx >= msg->in_radius.size() ||
          msg->in_radius[source_idx] == 0U) {
        continue;
      }

      const bool fresh = msg->fresh[source_idx] != 0U;
      output_map.at(kLayerFresh, index) = fresh ? 1.0f : 0.0f;
      output_map.at(kLayerObstacleProb, index) = msg->p_obstacle[source_idx];
      output_map.at(kLayerDropProb, index) = msg->p_drop[source_idx];
      output_map.at(kLayerStepUp, index) = msg->step_up[source_idx];
      output_map.at(kLayerClimbableProb, index) = msg->p_climbable[source_idx];

      if (sample_keepout && keepout_available) {
        const auto keepout_value =
            sampleKeepoutValue(*keepout_mask, position.x(), position.y());
        output_map.at(kLayerKfsKeepout, index) =
            keepout_value.has_value() ? *keepout_value : kNaNf;
      }

      if (!fresh) {
        continue;
      }

      output_map.at(kLayerElevationAbs, index) =
          static_cast<float>(base_z) + msg->h_ground[source_idx];
      output_map.at(kLayerElevationTopAbs, index) =
          static_cast<float>(base_z) + msg->h_top[source_idx];
      output_map.at(kLayerSigmaH, index) = msg->sigma_h[source_idx];
      output_map.at(kLayerDensity, index) =
          static_cast<float>(msg->density[source_idx]);
      output_map.at(kLayerSlopeX, index) = msg->slope_x[source_idx];
      output_map.at(kLayerSlopeY, index) = msg->slope_y[source_idx];
      output_map.at(kLayerSlope, index) =
          std::max(std::abs(msg->slope_x[source_idx]),
                   std::abs(msg->slope_y[source_idx]));
      output_map.at(kLayerRoughness, index) = msg->roughness[source_idx];
    }

    if (semantics_active) {
      const grid_map::Size map_size = output_map.getSize();
      const std::array<grid_map::Index, 4> kNeighbors = {
          grid_map::Index(1, 0),
          grid_map::Index(-1, 0),
          grid_map::Index(0, 1),
          grid_map::Index(0, -1),
      };

      for (grid_map::GridMapIterator it(output_map); !it.isPastEnd(); ++it) {
        const grid_map::Index index(*it);
        grid_map::Position position;
        if (!output_map.getPosition(index, position)) {
          continue;
        }

        output_map.at(kLayerBlockId, index) = kNaNf;
        output_map.at(kLayerExpectedHeight, index) = kNaNf;
        output_map.at(kLayerHeightError, index) = kNaNf;
        output_map.at(kLayerBlockOccupied, index) = kNaNf;
        output_map.at(kLayerTraversableEdgeMask, index) = 0.0f;
        output_map.at(kLayerEdgeStrength, index) = kNaNf;
        output_map.at(kLayerStepEdgeMask, index) = kNaNf;

        bool in_ramp = false;
        for (const auto &zone : ramp_corridor_zones_) {
          if (zone.contains(position.x(), position.y())) {
            in_ramp = true;
            break;
          }
        }
        output_map.at(kLayerRampCorridorMask, index) = in_ramp ? 1.0f : 0.0f;

        bool in_battle = false;
        for (const auto &zone : battle_approach_zones_) {
          if (zone.contains(position.x(), position.y())) {
            in_battle = true;
            break;
          }
        }
        output_map.at(kLayerBattleApproachMask, index) =
            in_battle ? 1.0f : 0.0f;

        const int block_id = resolveBlockId(position.x(), position.y());
        if (block_id <= 0) {
          continue;
        }

        output_map.at(kLayerBlockId, index) = static_cast<float>(block_id);
        const auto expected_height =
            expectedHeightForGridId(block_id, runtime_team);
        if (expected_height.has_value()) {
          output_map.at(kLayerExpectedHeight, index) =
              static_cast<float>(*expected_height);
          if (output_map.isValid(index, kLayerElevationAbs)) {
            output_map.at(kLayerHeightError, index) =
                output_map.at(kLayerElevationAbs, index) -
                static_cast<float>(*expected_height);
          }
        }

        const auto occupied_state =
            block_occupied_states[static_cast<size_t>(block_id)];
        if (occupied_state == BlockOccupiedState::kFree) {
          output_map.at(kLayerBlockOccupied, index) = 0.0f;
        } else if (occupied_state == BlockOccupiedState::kOccupied) {
          output_map.at(kLayerBlockOccupied, index) = 1.0f;
        }
      }

      for (grid_map::GridMapIterator it(output_map); !it.isPastEnd(); ++it) {
        const grid_map::Index index(*it);
        if (!output_map.isValid(index, kLayerBlockId)) {
          continue;
        }

        const int center_block =
            static_cast<int>(std::lround(output_map.at(kLayerBlockId, index)));
        if (center_block < 1 || center_block > 12) {
          continue;
        }

        if (output_map.isValid(index, kLayerElevationAbs)) {
          const float center_elevation =
              output_map.at(kLayerElevationAbs, index);
          float edge_strength = 0.0f;
          bool has_neighbor = false;
          for (const auto &offset : kNeighbors) {
            const grid_map::Index neighbor = index + offset;
            if (neighbor(0) < 0 || neighbor(0) >= map_size(0) ||
                neighbor(1) < 0 || neighbor(1) >= map_size(1)) {
              continue;
            }
            if (!output_map.isValid(neighbor, kLayerElevationAbs)) {
              continue;
            }
            const float neighbor_elevation =
                output_map.at(kLayerElevationAbs, neighbor);
            edge_strength = std::max(
                edge_strength, std::abs(neighbor_elevation - center_elevation));
            has_neighbor = true;
          }
          if (has_neighbor) {
            output_map.at(kLayerEdgeStrength, index) = edge_strength;
            output_map.at(kLayerStepEdgeMask, index) =
                edge_strength >= static_cast<float>(step_edge_height_thresh_m_)
                    ? 1.0f
                    : 0.0f;
          }
        }

        grid_map::Position position;
        if (!output_map.getPosition(index, position)) {
          continue;
        }
        const auto &center_cell = mf_cells_[static_cast<size_t>(center_block)];
        if (!center_cell.valid) {
          continue;
        }
        const double dx = std::abs(position.x() - center_cell.x);
        const double dy = std::abs(position.y() - center_cell.y);
        const bool near_vertical_edge =
            std::abs(dx - mf_block_half_extent_m_) <=
                shared_edge_band_width_m_ &&
            dy <= mf_block_half_extent_m_ + shared_edge_band_width_m_;
        const bool near_horizontal_edge =
            std::abs(dy - mf_block_half_extent_m_) <=
                shared_edge_band_width_m_ &&
            dx <= mf_block_half_extent_m_ + shared_edge_band_width_m_;
        if (!near_vertical_edge && !near_horizontal_edge) {
          continue;
        }

        bool edge_legal = false;
        const auto center_expected =
            expectedHeightForGridId(center_block, runtime_team);
        for (const auto &offset : kNeighbors) {
          const grid_map::Index neighbor = index + offset;
          if (neighbor(0) < 0 || neighbor(0) >= map_size(0) ||
              neighbor(1) < 0 || neighbor(1) >= map_size(1)) {
            continue;
          }
          if (!output_map.isValid(neighbor, kLayerBlockId)) {
            continue;
          }
          const int nb_block = static_cast<int>(
              std::lround(output_map.at(kLayerBlockId, neighbor)));
          if (nb_block <= 0 || nb_block == center_block) {
            continue;
          }
          const auto nb_expected =
              expectedHeightForGridId(nb_block, runtime_team);
          if (!center_expected.has_value() || !nb_expected.has_value()) {
            continue;
          }
          const double delta = std::abs(*center_expected - *nb_expected);
          if (delta <= traversable_edge_height_delta_limit_m_) {
            edge_legal = true;
            break;
          }
        }
        output_map.at(kLayerTraversableEdgeMask, index) =
            edge_legal ? 1.0f : 0.0f;
      }
    }

    for (grid_map::GridMapIterator it(output_map); !it.isPastEnd(); ++it) {
      const grid_map::Index index(*it);
      if (output_map.at(kLayerFresh, index) <= 0.5f) {
        continue;
      }

      const float slope = output_map.at(kLayerSlope, index);
      const float roughness = output_map.at(kLayerRoughness, index);
      const float obstacle_prob = output_map.at(kLayerObstacleProb, index);
      const float drop_prob = output_map.at(kLayerDropProb, index);
      if (!isFinite(slope) || !isFinite(roughness) ||
          !isFinite(obstacle_prob) || !isFinite(drop_prob)) {
        continue;
      }

      const float slope_norm =
          clamp01(slope / static_cast<float>(slope_norm_limit_));
      const float roughness_norm =
          clamp01(roughness / static_cast<float>(roughness_norm_limit_));
      float max_term = 0.0f;
      max_term = std::max(max_term, 0.35f * slope_norm);
      max_term = std::max(max_term, 0.20f * roughness_norm);
      max_term = std::max(max_term, 0.25f * clamp01(obstacle_prob));
      max_term = std::max(max_term, 0.35f * clamp01(drop_prob));
      if (semantics_active) {
        const float height_error = output_map.at(kLayerHeightError, index);
        if (isFinite(height_error)) {
          const float err_norm =
              clamp01(std::abs(height_error) /
                      static_cast<float>(height_error_limit_m_));
          max_term =
              std::max(max_term,
                       static_cast<float>(traversability_height_error_weight_) *
                           err_norm);
        }
        const float step_edge = output_map.at(kLayerStepEdgeMask, index);
        if (isFinite(step_edge)) {
          max_term = std::max(
              max_term, static_cast<float>(traversability_step_edge_weight_) *
                            clamp01(step_edge));
        }
      }

      const float traversability_continuous = clamp01(1.0f - max_term);
      if (semantics_active) {
        output_map.at(kLayerTraversabilityContinuous, index) =
            traversability_continuous;

        bool legal = true;

        if (output_map.isValid(index, kLayerKfsKeepout) &&
            output_map.at(kLayerKfsKeepout, index) >= 0.5f) {
          legal = false;
        }

        if (output_map.isValid(index, kLayerBlockId) &&
            output_map.isValid(index, kLayerBlockOccupied) &&
            output_map.at(kLayerBlockOccupied, index) >= 0.5f) {
          legal = false;
        }

        if (rule_legality_enforce_ramp_corridor_ &&
            output_map.isValid(index, kLayerRampCorridorMask) &&
            output_map.at(kLayerRampCorridorMask, index) < 0.5f) {
          legal = false;
        }

        output_map.at(kLayerRuleLegality, index) = legal ? 1.0f : 0.0f;
        output_map.at(kLayerTraversability, index) =
            legal ? traversability_continuous : 0.0f;
      } else {
        output_map.at(kLayerTraversability, index) = traversability_continuous;
      }
    }

    if (enable_filter_chain_ && filter_chain_ready_) {
      grid_map::GridMap filtered_map;
      if (filter_chain_.update(output_map, filtered_map)) {
        output_map = std::move(filtered_map);
      } else {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "terrain filter chain update failed, keep C++ output");
      }
    }

    return output_map;
  };

  auto &output_map_raw_ref =
      buildOutputMap(global_raw_map_, map_frame_, base_x_map, base_y_map,
                     base_z_map, cos_yaw_map, sin_yaw_map, true);
  grid_map::GridMap output_map_raw = output_map_raw_ref;
  grid_map::GridMap output_map = output_map_raw;

  if (fusion_enable_) {
    auto &fused_map = prepareRollingMap(fused_map_, map_frame_, base_x_map,
                                        base_y_map, false);
    grid_map::GridMap previous_map = fused_map;
    if ((previous_map.getStartIndex() != output_map_raw.getStartIndex())
            .any()) {
      for (const auto &layer : layers) {
        if (previous_map.exists(layer)) {
          previous_map[layer].setConstant(kNaNf);
        }
      }
    }

    const double dt_sec =
        (last_fusion_stamp_.nanoseconds() > 0 &&
         feature_stamp.nanoseconds() > last_fusion_stamp_.nanoseconds())
            ? (feature_stamp - last_fusion_stamp_).seconds()
            : fusion_time_constant_sec_;
    const float alpha = static_cast<float>(std::clamp(
        dt_sec / std::max(1e-3, fusion_time_constant_sec_), 0.0, 1.0));

    const std::array<const char *, 15> kBlendLayers = {
        kLayerElevationAbs,   kLayerElevationTopAbs,
        kLayerSigmaH,         kLayerDensity,
        kLayerSlope,          kLayerSlopeX,
        kLayerSlopeY,         kLayerRoughness,
        kLayerObstacleProb,   kLayerDropProb,
        kLayerStepUp,         kLayerClimbableProb,
        kLayerTraversability, kLayerTraversabilityContinuous,
        kLayerEdgeStrength,
    };
    const std::array<const char *, 9> kCarryOnlyLayers = {
        kLayerBlockId,          kLayerExpectedHeight,
        kLayerHeightError,      kLayerStepEdgeMask,
        kLayerBlockOccupied,    kLayerTraversableEdgeMask,
        kLayerRampCorridorMask, kLayerBattleApproachMask,
        kLayerRuleLegality,
    };

    const auto readPrevValue = [&](const char *layer,
                                   const grid_map::Index &index,
                                   float &value) -> bool {
      if (!previous_map.exists(layer) || !previous_map.isValid(index, layer)) {
        return false;
      }
      value = previous_map.at(layer, index);
      return isFinite(value);
    };

    output_map = output_map_raw;

    for (grid_map::GridMapIterator it(output_map); !it.isPastEnd(); ++it) {
      const grid_map::Index index(*it);
      const bool raw_fresh = output_map_raw.isValid(index, kLayerFresh) &&
                             output_map_raw.at(kLayerFresh, index) > 0.5f;

      float prev_age = 0.0f;
      float prev_hit_count = 0.0f;
      const bool has_prev_age = readPrevValue(kLayerAgeSec, index, prev_age);
      const bool has_prev_hit =
          readPrevValue(kLayerHitCount, index, prev_hit_count);
      const float updated_hit = has_prev_hit ? prev_hit_count + 1.0f : 1.0f;

      const auto blendLayerIfPossible = [&](const char *layer) {
        if (!output_map.exists(layer) || !output_map_raw.exists(layer)) {
          return;
        }
        if (!output_map_raw.isValid(index, layer)) {
          return;
        }
        float prev_value = 0.0f;
        if (!readPrevValue(layer, index, prev_value)) {
          return;
        }
        const float raw_value = output_map_raw.at(layer, index);
        output_map.at(layer, index) =
            (1.0f - alpha) * prev_value + alpha * raw_value;
      };

      const auto copyPreviousIfExists = [&](const char *layer) {
        if (!output_map.exists(layer)) {
          return;
        }
        float prev_value = 0.0f;
        if (readPrevValue(layer, index, prev_value)) {
          output_map.at(layer, index) = prev_value;
        } else {
          output_map.at(layer, index) = kNaNf;
        }
      };

      if (raw_fresh) {
        for (const auto *layer : kBlendLayers) {
          blendLayerIfPossible(layer);
        }
        for (const auto *layer : kCarryOnlyLayers) {
          if (!output_map_raw.exists(layer) ||
              !output_map_raw.isValid(index, layer)) {
            copyPreviousIfExists(layer);
          }
        }

        float prev_conf = 0.0f;
        const bool has_prev_conf =
            readPrevValue(kLayerStepEdgeConfidence, index, prev_conf);
        float edge_mask = 0.0f;
        if (output_map.exists(kLayerStepEdgeMask) &&
            output_map.isValid(index, kLayerStepEdgeMask)) {
          edge_mask = clamp01(output_map.at(kLayerStepEdgeMask, index));
        }
        output_map.at(kLayerStepEdgeConfidence, index) =
            has_prev_conf ? ((1.0f - alpha) * prev_conf + alpha * edge_mask)
                          : edge_mask;
        output_map.at(kLayerAgeSec, index) = 0.0f;
        output_map.at(kLayerHitCount, index) = updated_hit;
      } else {
        const float age_next = has_prev_age
                                   ? (prev_age + static_cast<float>(dt_sec))
                                   : std::numeric_limits<float>::infinity();
        const bool keep_memory =
            has_prev_age && std::isfinite(age_next) &&
            age_next <= static_cast<float>(fusion_unknown_decay_sec_);
        if (keep_memory) {
          for (const auto *layer : kBlendLayers) {
            copyPreviousIfExists(layer);
          }
          for (const auto *layer : kCarryOnlyLayers) {
            copyPreviousIfExists(layer);
          }
          float prev_conf = 0.0f;
          if (readPrevValue(kLayerStepEdgeConfidence, index, prev_conf)) {
            const float decay = static_cast<float>(
                std::exp(-dt_sec / fusion_time_constant_sec_));
            output_map.at(kLayerStepEdgeConfidence, index) = prev_conf * decay;
          } else {
            output_map.at(kLayerStepEdgeConfidence, index) = kNaNf;
          }
          output_map.at(kLayerFresh, index) = 1.0f;
          output_map.at(kLayerAgeSec, index) = age_next;
          output_map.at(kLayerHitCount, index) =
              has_prev_hit ? prev_hit_count : 0.0f;
        } else {
          for (const auto *layer : kBlendLayers) {
            if (output_map.exists(layer)) {
              output_map.at(layer, index) = kNaNf;
            }
          }
          for (const auto *layer : kCarryOnlyLayers) {
            if (output_map.exists(layer)) {
              output_map.at(layer, index) = kNaNf;
            }
          }
          if (output_map.exists(kLayerStepEdgeConfidence)) {
            output_map.at(kLayerStepEdgeConfidence, index) = kNaNf;
          }
          output_map.at(kLayerFresh, index) = 0.0f;
          output_map.at(kLayerAgeSec, index) = age_next;
          output_map.at(kLayerHitCount, index) =
              has_prev_hit ? prev_hit_count : 0.0f;
        }
      }

      if (output_map.exists(kLayerElevationFused)) {
        if (output_map.isValid(index, kLayerElevationAbs)) {
          output_map.at(kLayerElevationFused, index) =
              output_map.at(kLayerElevationAbs, index);
        } else {
          output_map.at(kLayerElevationFused, index) = kNaNf;
        }
      }
      if (output_map.exists(kLayerTraversabilityFused)) {
        if (output_map.isValid(index, kLayerTraversability)) {
          output_map.at(kLayerTraversabilityFused, index) =
              output_map.at(kLayerTraversability, index);
        } else {
          output_map.at(kLayerTraversabilityFused, index) = kNaNf;
        }
      }
    }

    fused_map_ = output_map;
    last_fusion_stamp_ = feature_stamp;
  } else {
    fused_map_.reset();
    last_fusion_stamp_ = feature_stamp;
  }

  if (fusion_publish_raw_ && pub_grid_map_raw_) {
    auto output_msg_raw =
        grid_map::GridMapRosConverter::toMessage(output_map_raw);
    if (output_msg_raw) {
      pub_grid_map_raw_->publish(std::move(*output_msg_raw));
    }
  }

  auto output_msg = grid_map::GridMapRosConverter::toMessage(output_map);
  if (output_msg) {
    pub_grid_map_->publish(std::move(*output_msg));
  }
  if (publish_marker_array_ && pub_marker_array_) {
    pub_marker_array_->publish(makeTerrainMarkerArray(
        output_map, feature_stamp, "terrain_grid_map", marker_height_min_m_,
        marker_height_scale_m_, marker_alpha_));
  }

  if (publish_local_map_ && pub_grid_map_local_) {
    const bool sample_local_keepout =
        keepout_available && (local_frame_ == map_frame_);
    auto &output_map_local_raw = buildOutputMap(
        local_raw_map_, local_frame_, base_x_local, base_y_local, base_z_local,
        cos_yaw_local, sin_yaw_local, sample_local_keepout);
    if (fusion_publish_raw_ && pub_grid_map_local_raw_) {
      auto output_msg_local_raw =
          grid_map::GridMapRosConverter::toMessage(output_map_local_raw);
      if (output_msg_local_raw) {
        pub_grid_map_local_raw_->publish(std::move(*output_msg_local_raw));
      }
    }
    auto output_msg_local =
        grid_map::GridMapRosConverter::toMessage(output_map_local_raw);
    if (output_msg_local) {
      pub_grid_map_local_->publish(std::move(*output_msg_local));
    }
    if (publish_marker_array_ && pub_marker_array_local_) {
      pub_marker_array_local_->publish(makeTerrainMarkerArray(
          output_map_local_raw, feature_stamp, "terrain_grid_map_local",
          marker_height_min_m_, marker_height_scale_m_, marker_alpha_));
    }
  }

  publishDiagnostics(feature_stamp, diagnostics);
}

bool TerrainGridMapBridge::validateFeatureMessage(
    const rc26_interfaces::msg::TerrainFeatureGrid &msg,
    std::string &reason) const {
  if (msg.width == 0U || msg.height == 0U) {
    reason = "feature grid width/height must be > 0";
    return false;
  }
  if (msg.width != msg.height) {
    reason = "feature grid must be square";
    return false;
  }
  if ((msg.width % 2U) == 0U) {
    reason = "feature grid width must be odd";
    return false;
  }
  if (!std::isfinite(static_cast<double>(msg.resolution_m)) ||
      msg.resolution_m <= 0.0f) {
    reason = "feature grid resolution_m must be finite and > 0";
    return false;
  }
  if (msg.header.frame_id.empty()) {
    reason = "feature header.frame_id is empty";
    return false;
  }

  const size_t expected_size =
      static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);
  const auto check_size = [&reason, expected_size](
                              size_t actual, const std::string &field) -> bool {
    if (actual == expected_size) {
      return true;
    }
    std::ostringstream oss;
    oss << "feature field '" << field << "' size mismatch, expected "
        << expected_size << " got " << actual;
    reason = oss.str();
    return false;
  };

  if (!check_size(msg.in_radius.size(), "in_radius"))
    return false;
  if (!check_size(msg.fresh.size(), "fresh"))
    return false;
  if (!check_size(msg.density.size(), "density"))
    return false;
  if (!check_size(msg.h_ground.size(), "h_ground"))
    return false;
  if (!check_size(msg.sigma_h.size(), "sigma_h"))
    return false;
  if (!check_size(msg.h_top.size(), "h_top"))
    return false;
  if (!check_size(msg.slope_x.size(), "slope_x"))
    return false;
  if (!check_size(msg.slope_y.size(), "slope_y"))
    return false;
  if (!check_size(msg.roughness.size(), "roughness"))
    return false;
  if (!check_size(msg.p_obstacle.size(), "p_obstacle"))
    return false;
  if (!check_size(msg.p_drop.size(), "p_drop"))
    return false;
  if (!check_size(msg.step_up.size(), "step_up"))
    return false;
  if (!check_size(msg.p_climbable.size(), "p_climbable"))
    return false;

  return true;
}

bool TerrainGridMapBridge::resolveLayoutPath(const std::string &raw_path,
                                             std::string &resolved_path) const {
  if (raw_path.empty()) {
    return false;
  }
  std::filesystem::path path(raw_path);
  if (path.is_relative()) {
    path = std::filesystem::current_path() / path;
  }
  std::error_code ec;
  const std::filesystem::path normalized =
      std::filesystem::weakly_canonical(path, ec);
  resolved_path = ec ? path.lexically_normal().string() : normalized.string();
  return true;
}

bool TerrainGridMapBridge::loadMfGridLayout(const std::string &path) {
  try {
    std::string resolved_path;
    if (!resolveLayoutPath(path, resolved_path)) {
      mf_layout_status_ = "layout path is empty";
      return false;
    }

    YAML::Node root = YAML::LoadFile(resolved_path);
    if (root["world_layout_file"]) {
      auto referenced = root["world_layout_file"].as<std::string>();
      if (referenced.empty()) {
        mf_layout_status_ = "world_layout_file is empty";
        return false;
      }
      std::filesystem::path nested_path(referenced);
      if (nested_path.is_relative()) {
        nested_path =
            std::filesystem::path(resolved_path).parent_path() / nested_path;
      }
      return loadMfGridLayout(nested_path.string());
    }

    if (!root["meta"]) {
      mf_layout_status_ = "missing meta in layout file";
      return false;
    }
    const YAML::Node meta = root["meta"];
    if (!meta["team"]) {
      mf_layout_status_ = "meta.team required";
      return false;
    }
    mf_layout_team_ = toLower(meta["team"].as<std::string>());
    if (mf_layout_team_.empty()) {
      mf_layout_status_ = "meta.team cannot be empty";
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(mf_state_mutex_);
      if (runtime_team_.empty()) {
        runtime_team_ = mf_layout_team_;
      }
    }

    if (meta["grid_spacing_m"]) {
      mf_grid_spacing_m_ = meta["grid_spacing_m"].as<double>();
    }
    if (!std::isfinite(mf_grid_spacing_m_) || mf_grid_spacing_m_ <= 0.0) {
      mf_layout_status_ = "meta.grid_spacing_m must be finite and > 0";
      return false;
    }
    mf_block_half_extent_m_ = meta["block_half_extent_m"]
                                  ? meta["block_half_extent_m"].as<double>()
                                  : (mf_grid_spacing_m_ * 0.5);
    if (!std::isfinite(mf_block_half_extent_m_) ||
        mf_block_half_extent_m_ <= 0.0) {
      mf_layout_status_ = "meta.block_half_extent_m must be finite and > 0";
      return false;
    }

    if (root["shared_edge_band_width_m"]) {
      shared_edge_band_width_m_ = root["shared_edge_band_width_m"].as<double>();
    } else if (meta["shared_edge_band_width_m"]) {
      shared_edge_band_width_m_ = meta["shared_edge_band_width_m"].as<double>();
    }
    shared_edge_band_width_m_ = std::max(1e-4, shared_edge_band_width_m_);

    const bool use_blocks = root["blocks"] && root["blocks"].IsSequence();
    const YAML::Node cells_node = use_blocks ? root["blocks"] : root["grids"];
    if (!cells_node || !cells_node.IsSequence()) {
      mf_layout_status_ = "missing blocks/grids sequence in layout file";
      return false;
    }

    for (auto &cell : mf_cells_) {
      cell = MfCell{};
    }
    std::array<bool, 13> seen{};
    for (const auto &cell : cells_node) {
      const int id = cell["id"].as<int>();
      if (id < 1 || id > 12) {
        mf_layout_status_ = "grid id out of range";
        return false;
      }
      const double x = cell["x"].as<double>();
      const double y = cell["y"].as<double>();
      if (!std::isfinite(x) || !std::isfinite(y)) {
        mf_layout_status_ = "grid x/y must be finite";
        return false;
      }

      MfCell parsed;
      parsed.x = x;
      parsed.y = y;
      parsed.valid = true;
      if (cell["expected_height_m"]) {
        parsed.expected_height_m = cell["expected_height_m"].as<double>();
        parsed.has_expected_height = std::isfinite(parsed.expected_height_m);
      } else if (cell["depth"]) {
        const int depth = cell["depth"].as<int>();
        if (depth > 0) {
          parsed.expected_height_m = static_cast<double>(depth) * 0.2;
          parsed.has_expected_height = true;
        }
      }
      mf_cells_[static_cast<size_t>(id)] = parsed;
      seen[static_cast<size_t>(id)] = true;
    }

    for (int id = 1; id <= 12; ++id) {
      if (!seen[static_cast<size_t>(id)]) {
        mf_layout_status_ = "grid ids 1..12 must all exist";
        return false;
      }
    }

    auto parse_rectangles = [&](const YAML::Node &zones_node,
                                std::vector<AxisAlignedZone> &out,
                                const std::string &zone_name) -> bool {
      out.clear();
      if (!zones_node) {
        return true;
      }
      if (!zones_node.IsSequence()) {
        mf_layout_status_ = zone_name + " must be a YAML sequence";
        return false;
      }
      for (const auto &zone : zones_node) {
        AxisAlignedZone parsed;
        if (zone["x_min"] && zone["x_max"] && zone["y_min"] && zone["y_max"]) {
          parsed.x_min = zone["x_min"].as<double>();
          parsed.x_max = zone["x_max"].as<double>();
          parsed.y_min = zone["y_min"].as<double>();
          parsed.y_max = zone["y_max"].as<double>();
        } else if (zone["center_x"] && zone["center_y"] && zone["size_x"] &&
                   zone["size_y"]) {
          const double cx = zone["center_x"].as<double>();
          const double cy = zone["center_y"].as<double>();
          const double sx = zone["size_x"].as<double>();
          const double sy = zone["size_y"].as<double>();
          parsed.x_min = cx - 0.5 * sx;
          parsed.x_max = cx + 0.5 * sx;
          parsed.y_min = cy - 0.5 * sy;
          parsed.y_max = cy + 0.5 * sy;
        } else {
          mf_layout_status_ = zone_name +
                              " must contain x_min/x_max/y_min/y_max "
                              "or center_x/center_y/size_x/size_y";
          return false;
        }

        if (!std::isfinite(parsed.x_min) || !std::isfinite(parsed.x_max) ||
            !std::isfinite(parsed.y_min) || !std::isfinite(parsed.y_max) ||
            parsed.x_min > parsed.x_max || parsed.y_min > parsed.y_max) {
          mf_layout_status_ = zone_name + " has invalid rectangle bounds";
          return false;
        }
        out.push_back(parsed);
      }
      return true;
    };

    if (!parse_rectangles(root["ramp_corridors"], ramp_corridor_zones_,
                          "ramp_corridors")) {
      return false;
    }
    if (!parse_rectangles(root["battle_approach_zones"], battle_approach_zones_,
                          "battle_approach_zones")) {
      return false;
    }

    mf_layout_status_ = "ok";
    return true;
  } catch (const std::exception &ex) {
    mf_layout_status_ = std::string("yaml parse error: ") + ex.what();
    return false;
  }
}

std::optional<double> TerrainGridMapBridge::expectedHeightForGridId(
    int grid_id, const std::string &runtime_team) const {
  if (grid_id < 1 || grid_id > 12) {
    return std::nullopt;
  }
  int effective_id = grid_id;
  if (!runtime_team.empty() && !mf_layout_team_.empty() &&
      runtime_team != mf_layout_team_) {
    effective_id = mirrorGridIdAcrossColumns(grid_id);
  }
  const auto &cell = mf_cells_[static_cast<size_t>(effective_id)];
  if (!cell.valid || !cell.has_expected_height) {
    return std::nullopt;
  }
  return cell.expected_height_m;
}

int TerrainGridMapBridge::resolveBlockId(double x_map, double y_map) const {
  for (int id = 1; id <= 12; ++id) {
    const auto &cell = mf_cells_[static_cast<size_t>(id)];
    if (!cell.valid) {
      continue;
    }
    if (std::abs(x_map - cell.x) <= mf_block_half_extent_m_ &&
        std::abs(y_map - cell.y) <= mf_block_half_extent_m_) {
      return id;
    }
  }
  return -1;
}

std::optional<float> TerrainGridMapBridge::sampleKeepoutValue(
    const nav_msgs::msg::OccupancyGrid &grid, double x_map,
    double y_map) const {
  if (grid.info.width == 0U || grid.info.height == 0U ||
      grid.info.resolution <= 0.0f) {
    return std::nullopt;
  }
  if (grid.data.size() < static_cast<size_t>(grid.info.width) *
                             static_cast<size_t>(grid.info.height)) {
    return std::nullopt;
  }

  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;
  const double resolution = static_cast<double>(grid.info.resolution);
  const int px = static_cast<int>(std::floor((x_map - origin_x) / resolution));
  const int py = static_cast<int>(std::floor((y_map - origin_y) / resolution));
  if (px < 0 || py < 0 || px >= static_cast<int>(grid.info.width) ||
      py >= static_cast<int>(grid.info.height)) {
    return std::nullopt;
  }

  const size_t index =
      static_cast<size_t>(py) * static_cast<size_t>(grid.info.width) +
      static_cast<size_t>(px);
  const int8_t value = grid.data[index];
  if (value == 100) {
    return 1.0f;
  }
  if (value == 0) {
    return 0.0f;
  }
  return std::nullopt;
}

void TerrainGridMapBridge::publishDiagnostics(
    const rclcpp::Time &stamp, const DiagnosticsState &state) const {
  if (!pub_diagnostics_) {
    return;
  }

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = this->get_fully_qualified_name();
  status.hardware_id = "R2";

  if (!state.feature_valid || !state.tf_ok) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message =
        state.detail.empty() ? "grid_map_bridge error" : state.detail;
  } else {
    std::vector<std::string> warnings;
    if (!state.keepout_available) {
      warnings.emplace_back(state.keepout_stale ? "keepout stale"
                                                : "keepout unavailable");
    }
    if (!state.mf_layout_ready) {
      warnings.emplace_back("mf layout unavailable");
    }
    if (!state.team_valid) {
      warnings.emplace_back("mf team invalid");
    }

    if (!warnings.empty()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      std::ostringstream oss;
      for (size_t i = 0; i < warnings.size(); ++i) {
        if (i != 0U) {
          oss << "; ";
        }
        oss << warnings[i];
      }
      status.message = oss.str();
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "ok";
    }
  }

  auto add_key_value = [&](const std::string &key, const std::string &value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(kv);
  };

  add_key_value("terrain_features_topic", terrain_features_topic_);
  add_key_value("kfs_mask_topic", kfs_mask_topic_);
  add_key_value("output_topic", output_topic_);
  add_key_value("publish_local_map", publish_local_map_ ? "true" : "false");
  add_key_value("output_topic_local", output_topic_local_);
  add_key_value("fusion_enable", fusion_enable_ ? "true" : "false");
  add_key_value("fusion_publish_raw", fusion_publish_raw_ ? "true" : "false");
  add_key_value("output_topic_raw", output_topic_raw_);
  add_key_value("output_topic_local_raw", output_topic_local_raw_);
  add_key_value("fusion_time_constant_sec",
                std::to_string(fusion_time_constant_sec_));
  add_key_value("fusion_unknown_decay_sec",
                std::to_string(fusion_unknown_decay_sec_));
  add_key_value("map_frame", map_frame_);
  add_key_value("local_frame", local_frame_);
  add_key_value("base_frame", base_frame_);
  add_key_value("keepout_available",
                state.keepout_available ? "true" : "false");
  add_key_value("keepout_stale", state.keepout_stale ? "true" : "false");
  add_key_value("enable_mf_semantics", enable_mf_semantics_ ? "true" : "false");
  add_key_value("mf_kfs_state_topic", mf_kfs_state_topic_);
  add_key_value("mf_kfs_min_confidence",
                std::to_string(mf_kfs_min_confidence_));
  add_key_value("mf_layout_ready", mf_layout_ready_ ? "true" : "false");
  add_key_value("mf_layout_team", mf_layout_team_);
  add_key_value("mf_layout_file", mf_world_layout_file_);
  add_key_value("mf_layout_status", mf_layout_status_);
  add_key_value("shared_edge_band_width_m",
                std::to_string(shared_edge_band_width_m_));
  add_key_value("traversable_edge_height_delta_limit_m",
                std::to_string(traversable_edge_height_delta_limit_m_));
  add_key_value("rule_legality_enforce_ramp_corridor",
                rule_legality_enforce_ramp_corridor_ ? "true" : "false");
  add_key_value("enable_filter_chain", enable_filter_chain_ ? "true" : "false");
  add_key_value("filter_chain_parameter_name", filter_chain_parameter_name_);
  add_key_value("filter_chain_ready", filter_chain_ready_ ? "true" : "false");
  add_key_value("height_error_limit_m", std::to_string(height_error_limit_m_));
  add_key_value("traversability_height_error_weight",
                std::to_string(traversability_height_error_weight_));
  add_key_value("traversability_step_edge_weight",
                std::to_string(traversability_step_edge_weight_));

  diagnostic_msgs::msg::DiagnosticArray diagnostics_msg;
  diagnostics_msg.header.stamp = stamp;
  diagnostics_msg.status.push_back(status);
  pub_diagnostics_->publish(diagnostics_msg);
}

std::string TerrainGridMapBridge::toLower(std::string value) {
  std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

} // namespace rc26_terrain

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc26_terrain::TerrainGridMapBridge)
