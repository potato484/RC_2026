// Copyright 2025 RC2026
// 基于 pb_omni_pid_pursuit_controller 移植
// 原作者: Lihan Chen
//
// Licensed under the Apache License, Version 2.0
// Maintained by DongXuan Chen <2220362462@qq.com>

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "nav2_core/controller.hpp"
#include "rc26_interfaces/msg/localization_health.hpp"
#include "rc26_omni_controller/pid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rc26_omni_controller {

struct CostmapSnapshot {
    std::vector<uint8_t> data;
    unsigned int width{}, height{};
    double origin_x{}, origin_y{}, resolution{};
};

struct TerrainScaleFactors {
    double linear{1.0};
    double lateral{1.0};
    double yaw{1.0};
    bool applied{false};
};

/**
 * @brief 全向轮 PID + Pure Pursuit 路径跟踪控制器
 */
class OmniPidPursuitController : public nav2_core::Controller {
public:
    OmniPidPursuitController() = default;
    ~OmniPidPursuitController() override = default;

    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                   std::shared_ptr<tf2_ros::Buffer> tf,
                   std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                             const geometry_msgs::msg::Twist& velocity,
                                                             nav2_core::GoalChecker* goal_checker) override;

    void setPlan(const nav_msgs::msg::Path& path) override;
    void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

protected:
    nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped& pose);

    bool transformPose(const std::string frame, const geometry_msgs::msg::PoseStamped& in_pose,
                       geometry_msgs::msg::PoseStamped& out_pose) const;

    double getCostmapMaxExtent() const;

    std::unique_ptr<geometry_msgs::msg::PointStamped>
    createCarrotMsg(const geometry_msgs::msg::PoseStamped& carrot_pose);

    geometry_msgs::msg::PoseStamped getLookAheadPoint(const double& lookahead_dist,
                                                      const nav_msgs::msg::Path& transformed_plan);

    geometry_msgs::msg::Point circleSegmentIntersection(const geometry_msgs::msg::Point& p1,
                                                        const geometry_msgs::msg::Point& p2, double r);

    rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

    double getLookAheadDistance(const geometry_msgs::msg::Twist& speed);
    double approachVelocityScalingFactor(const nav_msgs::msg::Path& path) const;
    void applyApproachVelocityScaling(const nav_msgs::msg::Path& path, double& linear_vel) const;
    bool isCollisionDetected(const nav_msgs::msg::Path& path);
    double getMinCollisionDist(const nav_msgs::msg::Path& path, const CostmapSnapshot& snap, double robot_x,
                               double robot_y);

private:
    const CostmapSnapshot& captureCostmapSnapshot();
    void refreshPoseCovSubscription(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);
    void sanitizeLoadedParameters();
    bool validateParameterUpdate(const std::vector<rclcpp::Parameter>& parameters, std::string& reason) const;
    void resetMotionState() noexcept;
    void terrainGridCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
    bool readTerrainLayerValue(const grid_map::GridMap& map,
                               const std::string& layer,
                               const grid_map::Position& pos,
                               float& value) const;
    TerrainScaleFactors evaluateTerrainScales(const nav_msgs::msg::Path& transformed_plan,
                                              int lookahead_end_idx,
                                              const tf2::Transform& transform_global_from_base,
                                              double path_tx,
                                              double path_ty) const;

    double applyCurvatureLimitation(const nav_msgs::msg::Path& path,
                                    const geometry_msgs::msg::PoseStamped& lookahead_pose, double& linear_vel,
                                    double real_dt, double& out_kappa);

    double calculateCurvature(const nav_msgs::msg::Path& path, const geometry_msgs::msg::PoseStamped& lookahead_pose,
                              double forward_dist, double backward_dist) const;

    double calculateCurvatureRadius(const geometry_msgs::msg::Point& near_point,
                                    const geometry_msgs::msg::Point& current_point,
                                    const geometry_msgs::msg::Point& far_point) const;

    void visualizeCurvaturePoints(const geometry_msgs::msg::PoseStamped& backward_pose,
                                  const geometry_msgs::msg::PoseStamped& forward_pose) const;

    std::vector<double> calculateCumulativeDistances(const nav_msgs::msg::Path& path) const;

    geometry_msgs::msg::PoseStamped findPoseAtDistance(const nav_msgs::msg::Path& path,
                                                       const std::vector<double>& cumulative_distances,
                                                       size_t cumulative_offset, double target_distance) const;

private:
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string plugin_name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D* costmap_;
    rclcpp::Logger logger_{rclcpp::get_logger("OmniPidPursuitController")};
    rclcpp::Clock::SharedPtr clock_;
    double last_velocity_scaling_factor_;
    rclcpp::Time last_time_;

    std::shared_ptr<PID> move_pid_;
    std::shared_ptr<PID> heading_pid_;

    // 控制器参数
    double translation_kp_, translation_ki_, translation_kd_;
    bool enable_rotation_;
    double rotation_kp_, rotation_ki_, rotation_kd_;
    double min_max_sum_error_;
    double control_duration_;
    double max_robot_pose_search_dist_;
    bool use_interpolation_;
    double lookahead_dist_;
    bool use_velocity_scaled_lookahead_dist_;
    double min_lookahead_dist_;
    double max_lookahead_dist_;
    double lookahead_time_;
    bool use_rotate_to_heading_;
    double use_rotate_to_heading_threshold_;
    double v_linear_min_;
    double v_linear_max_;
    double configured_v_linear_max_;
    double v_angular_min_;
    double v_angular_max_;
    double a_linear_max_;
    double a_angular_max_;
    double brake_margin_;
    double brake_accel_;
    double lateral_error_gain_;
    double lateral_error_max_;
    bool enable_curvature_ff_;
    double a_lim_x_;
    double a_lim_y_;
    double last_lin_vel_{0.0};
    double last_vx_{0.0};
    double last_vy_{0.0};
    double last_ang_vel_{0.0};
    double min_approach_linear_velocity_;
    double approach_velocity_scaling_dist_;
    double a_lateral_max_;
    double curvature_forward_dist_;
    double curvature_backward_dist_;
    double max_velocity_scaling_factor_rate_;
    double kv_ff_;
    double goal_dist_scale_;
    double wheel_base_;
    double track_width_;
    double wheel_speed_max_;
    double derivative_filter_tau_;
    bool publish_debug_{false};
    bool loc_uncertainty_enable_{true};
    double loc_timeout_sec_{0.2};
    double loc_k_v_{50.0};
    double loc_k_w_{20.0};
    double loc_v_scale_min_{0.2};
    double loc_w_scale_min_{0.3};
    uint32_t collision_check_outside_map_count_{0};
    tf2::Duration transform_tolerance_;
    std::vector<double> plan_cumulative_distances_;
    size_t plan_prune_idx_{0};
    CostmapSnapshot costmap_snapshot_cache_;

    nav_msgs::msg::Path global_plan_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr carrot_pub_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr curvature_points_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr real_dt_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr compute_time_ms_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr pose_age_ms_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr collision_check_ms_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr collision_d_min_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr v_safe_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt32>::SharedPtr collision_check_outside_map_count_pub_;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr terrain_grid_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
    rclcpp::Subscription<rc26_interfaces::msg::LocalizationHealth>::SharedPtr localization_health_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr control_degraded_sub_;
    std::shared_ptr<grid_map::GridMap> terrain_map_;
    rclcpp::Time last_cov_stamp_;
    rclcpp::Time terrain_map_stamp_{0, 0, RCL_ROS_TIME};
    double sigma_xy_{0.0};
    double sigma_yaw_{0.0};
    std::mutex cov_mutex_;
    std::mutex localization_safety_mutex_;
    mutable std::mutex terrain_mutex_;
    uint8_t localization_health_level_{rc26_interfaces::msg::LocalizationHealth::GREEN};
    bool localization_health_control_degraded_{false};
    bool control_degraded_{false};
    rclcpp::Time last_localization_health_stamp_;
    rclcpp::Time last_control_degraded_stamp_;
    double lhi_yellow_v_scale_{0.8};
    double lhi_yellow_w_scale_{0.8};
    double lhi_orange_v_scale_{0.5};
    double lhi_orange_w_scale_{0.6};
    double lhi_orange_vy_scale_{0.5};
    bool lhi_red_stop_enable_{true};
    double degraded_v_scale_{0.3};
    double degraded_w_scale_{0.5};
    bool terrain_enable_{true};
    std::string terrain_grid_topic_{"/terrain_grid_map_local"};
    std::string terrain_traversability_layer_{"traversability"};
    std::string terrain_fresh_layer_{"fresh"};
    std::string terrain_slope_x_layer_{"slope_x"};
    std::string terrain_slope_y_layer_{"slope_y"};
    std::string terrain_roughness_layer_{"roughness"};
    std::string terrain_step_up_layer_{"step_up"};
    std::string terrain_rule_legality_layer_{"rule_legality"};
    std::string terrain_kfs_keepout_layer_{"kfs_keepout"};
    std::string terrain_block_occupied_layer_{"block_occupied"};
    std::string terrain_ramp_corridor_layer_{"ramp_corridor_mask"};
    int terrain_sample_count_{12};
    double terrain_scale_min_{0.35};
    double terrain_lateral_scale_min_{0.2};
    double terrain_yaw_scale_min_{0.25};
    double terrain_slope_limit_{0.45};
    double terrain_roughness_limit_{0.35};
    double terrain_step_up_limit_{0.08};
    double terrain_rule_legality_threshold_{0.5};
    double terrain_keepout_threshold_{0.5};
    double terrain_block_occupied_threshold_{0.5};
    bool terrain_enforce_ramp_corridor_{false};
    double terrain_stale_timeout_sec_{0.4};

    std::recursive_mutex mutex_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace rc26_omni_controller
