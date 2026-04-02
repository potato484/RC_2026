#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rc26_interfaces/msg/operator_status.hpp>
#include <rc26_interfaces/msg/visualization_event_array.hpp>
#include <std_msgs/msg/header.hpp>

namespace rc26_visualization {

constexpr uint8_t kLevelGreen = 0U;
constexpr uint8_t kLevelYellow = 1U;
constexpr uint8_t kLevelOrange = 2U;
constexpr uint8_t kLevelRed = 3U;
constexpr double kInf = std::numeric_limits<double>::infinity();

struct NumericSample {
  bool received{false};
  double age_sec{kInf};
  double value{0.0};
};

struct BoolSample {
  bool received{false};
  double age_sec{kInf};
  bool value{false};
};

struct TextSample {
  bool received{false};
  double age_sec{kInf};
  std::string value;
};

struct LocalizationInput {
  bool received{false};
  double age_sec{kInf};
  uint8_t level{kLevelRed};
  std::string reason{"waiting localization health"};
  std::string state{"UNKNOWN"};
  bool control_degraded{false};
  double sigma_xy{std::numeric_limits<double>::quiet_NaN()};
  double sigma_yaw{std::numeric_limits<double>::quiet_NaN()};
  double degenerate_score{std::numeric_limits<double>::quiet_NaN()};
  double h_min_eig{std::numeric_limits<double>::quiet_NaN()};
};

struct BackendInput {
  bool received{false};
  double age_sec{kInf};
  bool optimizer_ready{false};
  std::string optimizer_state{"unknown"};
  double graph_health{0.0};
  double last_local_reg_age_sec{kInf};
  double last_loop_age_sec{kInf};
  double last_anchor_age_sec{kInf};
  bool imu_spike{false};
};

struct NavSafetyInput {
  bool received{false};
  double age_sec{kInf};
  std::string current_profile{"unknown"};
  std::string reason;
  bool stop_required{false};
  bool timed_out{false};
};

struct MechanismInput {
  bool received{false};
  double age_sec{kInf};
  uint8_t comm_health_level{0U};
};

struct KeepoutInput {
  bool overlay_received{false};
  double overlay_age_sec{kInf};
  bool mask_received{false};
  double mask_age_sec{kInf};
  bool heartbeat_received{false};
  double heartbeat_age_sec{kInf};
  bool heartbeat_enabled{true};
};

struct TerrainInput {
  bool obstacles_received{false};
  double obstacles_age_sec{kInf};
  bool obstacles_active{false};
  bool drop_received{false};
  double drop_age_sec{kInf};
  bool drop_active{false};
  bool grid_received{false};
  double grid_age_sec{kInf};
  double traversability_min{std::numeric_limits<double>::quiet_NaN()};
  bool climbable_active{false};
  bool step_edge_active{false};
  bool speed_limit_received{false};
  double speed_limit_age_sec{kInf};
  bool speed_limited{false};
};

struct TopicWatchInput {
  std::string code_suffix;
  std::string topic_name;
  double max_age_sec{1.0};
  bool required{true};
  bool received{false};
  double age_sec{kInf};
};

struct VisualizationStatusConfig {
  double loc_timeout_sec{0.2};
  double controller_period_ms{33.333};
  double brake_margin_m{0.15};
  double keepout_max_age_ms{300.0};
  double terrain_max_age_ms{1000.0};
  double nav_safety_max_age_ms{2500.0};
  double mechanism_max_age_ms{1000.0};
  double backend_status_max_age_ms{1000.0};
  double backend_graph_health_warn{0.6};
  double backend_graph_health_error{0.3};
  double backend_local_reg_warn_sec{0.75};
  double backend_local_reg_error_sec{1.50};
  uint8_t mechanism_warn_level{1U};
  uint8_t mechanism_error_level{2U};
  uint32_t topics_orange_count{3U};
  std::string localization_health_topic{"/localization/health"};
  std::string localization_backend_topic{"/localization/backend_status"};
  std::string control_degraded_topic{"/control_degraded"};
  std::string control_degenerate_score_topic{"control_degenerate_score"};
  std::string compute_time_ms_topic{"compute_time_ms"};
  std::string pose_age_ms_topic{"pose_age_ms"};
  std::string collision_d_min_topic{"collision_d_min"};
  std::string controller_mode_topic{"/xhu_nav/semantic_gate"};
  std::string nav_mode_state_topic{"/xhu_nav/motion_mode_state"};
  std::string nav_tracking_topic{"/xhu_nav/tracking_state"};
  std::string mechanism_state_topic{"/mechanism/state"};
  std::string block_overlay_topic{"/mf_block_overlay"};
  std::string kfs_filter_mask_topic{"/kfs_filter_mask"};
  std::string kfs_heartbeat_topic{"/kfs_keepout_heartbeat"};
  std::string terrain_obstacles_topic{"terrain_obstacles"};
  std::string terrain_drop_topic{"terrain_drop"};
  std::string terrain_grid_topic{"/terrain_grid_map_local"};
  std::string terrain_speed_limit_topic{"terrain_speed_limit"};
  double terrain_speed_limit_nominal_mps{2.0};
  double terrain_speed_limit_margin_mps{0.05};
  double terrain_climbable_active_threshold{0.20};
  double terrain_step_edge_active_threshold{0.50};
  bool localization_present{true};
  bool controller_present{true};
  bool keepout_present{true};
  bool terrain_present{true};
  bool nav_safety_present{true};
  bool mechanism_present{true};
};

struct EvaluationInput {
  LocalizationInput localization;
  BackendInput backend;
  BoolSample control_degraded;
  NumericSample control_degenerate_score;
  NumericSample compute_time_ms;
  NumericSample pose_age_ms;
  NumericSample collision_d_min;
  TextSample controller_mode;
  NavSafetyInput nav_safety;
  MechanismInput mechanism;
  KeepoutInput keepout;
  TerrainInput terrain;
  std::vector<TopicWatchInput> monitored_topics;
};

class VisualizationStatusCore {
public:
  struct Output {
    diagnostic_msgs::msg::DiagnosticArray summary;
    rc26_interfaces::msg::OperatorStatus operator_status;
    rc26_interfaces::msg::VisualizationEventArray events;
  };

  explicit VisualizationStatusCore(VisualizationStatusConfig config = {});

  void setConfig(const VisualizationStatusConfig& config);
  Output evaluate(const EvaluationInput& input, const std_msgs::msg::Header& header) const;

private:
  VisualizationStatusConfig config_;
};

class TopicTimeoutTracker {
public:
  uint32_t observe(const std::vector<TopicWatchInput>& monitored_topics);
  void resetBaseline(const std::vector<TopicWatchInput>& monitored_topics);
  uint32_t total() const {
    return total_;
  }

private:
  static bool isStale(const TopicWatchInput& topic);
  static std::string topicKey(const TopicWatchInput& topic);

  uint32_t total_{0U};
  std::unordered_map<std::string, bool> stale_states_;
};

}  // namespace rc26_visualization
