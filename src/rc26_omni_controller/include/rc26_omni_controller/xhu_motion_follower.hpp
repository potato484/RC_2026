#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rc26_interfaces/msg/localization_health.hpp"
#include "rc26_interfaces/msg/terrain_feature_grid.hpp"
#include "rc26_interfaces/msg/xhu_motion_mode_state.hpp"
#include "rc26_interfaces/msg/xhu_semantic_corridor.hpp"
#include "rc26_interfaces/msg/xhu_tracking_state.hpp"

namespace rc26_omni_controller {

class XhuMotionFollower : public rclcpp::Node {
public:
  explicit XhuMotionFollower(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  struct TerrainCache {
    bool valid{false};
    float resolution_m{0.0F};
    uint32_t width{0U};
    uint32_t height{0U};
    double origin_x{0.0};
    double origin_y{0.0};
    std::vector<float> p_obstacle;
    std::vector<float> p_drop;
  };

  void onCorridor(const rc26_interfaces::msg::XhuSemanticCorridor::SharedPtr msg);
  void onModeState(const rc26_interfaces::msg::XhuMotionModeState::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onLocalizationHealth(const rc26_interfaces::msg::LocalizationHealth::SharedPtr msg);
  void onTerrainGrid(const rc26_interfaces::msg::TerrainFeatureGrid::SharedPtr msg);

  bool queryRobotPose(double &x, double &y, double &yaw) const;
  bool terrainRiskAt(const std::shared_ptr<const TerrainCache> &cache, double x, double y) const;
  bool terrainRiskAhead(const std::shared_ptr<const TerrainCache> &cache, const nav_msgs::msg::Path &path,
                        size_t start_index, size_t end_index) const;
  size_t findNearestIndex(const nav_msgs::msg::Path &path, double x, double y, size_t hint) const;
  size_t findLookaheadIndex(const nav_msgs::msg::Path &path, size_t start_index,
                            double lookahead_distance) const;
  void publishLookaheadPath(const nav_msgs::msg::Path &source, size_t nearest_index,
                            size_t lookahead_index);
  void publishSemanticGate(const std::string &status);
  void publishRuntimeStateLocked(const rc26_interfaces::msg::XhuSemanticCorridor &corridor,
                                 const std::string &status, bool terminal,
                                 const std::string &reason, float cross_track_error,
                                 float heading_error, float distance_to_goal);
  void publishTerminalStateLocked(const rc26_interfaces::msg::XhuSemanticCorridor &corridor,
                                  const std::string &status, const std::string &reason,
                                  float cross_track_error, float heading_error,
                                  float distance_to_goal);
  void publishHoldStateLocked(const rc26_interfaces::msg::XhuSemanticCorridor &corridor,
                              const rclcpp::Time &stamp, const std::string &reason,
                              float distance_to_goal);
  void publishZeroCommandLocked();
  void controlLoop();

  mutable std::mutex data_mutex_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<rc26_interfaces::msg::XhuSemanticCorridor>::SharedPtr corridor_sub_;
  rclcpp::Subscription<rc26_interfaces::msg::XhuMotionModeState>::SharedPtr mode_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<rc26_interfaces::msg::LocalizationHealth>::SharedPtr loc_health_sub_;
  rclcpp::Subscription<rc26_interfaces::msg::TerrainFeatureGrid>::SharedPtr terrain_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr lookahead_pub_;
  rclcpp::Publisher<rc26_interfaces::msg::XhuTrackingState>::SharedPtr tracking_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr semantic_gate_pub_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  std::shared_ptr<const rc26_interfaces::msg::XhuSemanticCorridor> active_corridor_;
  std::optional<rclcpp::Time> hold_since_;
  size_t nearest_index_{0U};
  rclcpp::Time corridor_start_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_control_stamp_{0, 0, RCL_ROS_TIME};

  bool has_mode_state_{false};
  rc26_interfaces::msg::XhuMotionModeState mode_state_;
  rclcpp::Time mode_state_stamp_{0, 0, RCL_ROS_TIME};
  bool has_odom_{false};
  nav_msgs::msg::Odometry last_odom_;
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  uint8_t loc_health_level_{rc26_interfaces::msg::LocalizationHealth::GREEN};
  std::shared_ptr<const TerrainCache> terrain_cache_;
  geometry_msgs::msg::Twist last_cmd_;

  std::string map_frame_;
  std::string base_frame_;
  double control_frequency_hz_{30.0};
  double lookahead_distance_{0.6};
  double goal_tolerance_xy_{0.15};
  double goal_tolerance_yaw_{0.3};
  double corridor_timeout_sec_{45.0};
  double odom_timeout_sec_{0.3};
  double mode_state_timeout_sec_{1.0};
  double hold_to_abort_sec_{3.0};
  double kp_linear_x_{1.2};
  double kp_linear_y_{1.0};
  double kp_angular_{1.8};
  double default_max_linear_speed_{0.8};
  double default_max_angular_speed_{1.0};
  double default_max_linear_accel_{0.6};
  double default_max_angular_accel_{0.8};
  double lhi_yellow_v_scale_{0.8};
  double lhi_yellow_w_scale_{0.8};
  double lhi_orange_v_scale_{0.5};
  double lhi_orange_w_scale_{0.6};
  double lhi_orange_vy_scale_{0.5};
  double terrain_obstacle_threshold_{0.6};
  double terrain_drop_threshold_{0.8};
  double terrain_sample_spacing_m_{0.12};
  double stop_envelope_half_width_m_{0.20};
  double brake_margin_m_{0.30};
  double max_cross_track_error_m_{0.60};
};

}  // namespace rc26_omni_controller
