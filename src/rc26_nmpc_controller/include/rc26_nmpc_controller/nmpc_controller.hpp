#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rc26_interfaces/msg/localization_backend_status.hpp"
#include "rc26_interfaces/msg/localization_health.hpp"
#include "rc26_interfaces/srv/set_nav_mode.hpp"
#include "rc26_omni_controller/omni_pid_pursuit_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"

namespace rc26_nmpc_controller {

enum class FallbackReason : uint8_t {
    kNone = 0,
    kLocalizationRed,
    kSolverTimeout,
    kSolverInfeasible,
};

struct SolveReport {
    bool success{false};
    bool timed_out{false};
    bool infeasible{false};
    double solve_time_ms{0.0};
    geometry_msgs::msg::Twist command;
};

class NmpcController : public nav2_core::Controller {
public:
    NmpcController() = default;
    ~NmpcController() override = default;

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

private:
    void resetRuntimeState();
    void requestNavProfile(const std::string& profile, const std::string& reason);
    double computeQualityScale(const rclcpp::Time& now, bool& localization_red) const;
    geometry_msgs::msg::Twist applySlewRateLimit(const geometry_msgs::msg::Twist& target, double dt) const;
    void terrainGridCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
    bool readTerrainLayerValue(const grid_map::GridMap& map,
                               const std::string& layer,
                               const grid_map::Position& pos,
                               float& value) const;
    double computeTerrainScale(const geometry_msgs::msg::PoseStamped& pose, const rclcpp::Time& now) const;

    SolveReport solveConstrainedCommand(const geometry_msgs::msg::Twist& reference_cmd,
                                        const std::array<double, 3>& measured_velocity, double dt,
                                        double quality_scale) const;

    std::array<double, 3> readMeasuredVelocity(const geometry_msgs::msg::Twist& nav2_velocity,
                                               const rclcpp::Time& now) const;

    void enterFallbackMode(FallbackReason reason, const rclcpp::Time& now);
    void maybeExitFallbackMode(bool solver_success, bool localization_red, const rclcpp::Time& now);

private:
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D* costmap_{nullptr};
    std::string plugin_name_;

    rclcpp::Logger logger_{rclcpp::get_logger("NmpcController")};
    rclcpp::Clock::SharedPtr clock_;

    std::unique_ptr<rc26_omni_controller::OmniPidPursuitController> fallback_controller_;

    mutable std::mutex state_mutex_;
    nav_msgs::msg::Path global_plan_;
    nav_msgs::msg::Odometry::SharedPtr latest_control_state_;
    rc26_interfaces::msg::LocalizationHealth::SharedPtr latest_health_;
    rc26_interfaces::msg::LocalizationBackendStatus::SharedPtr latest_backend_;
    rclcpp::Time latest_control_state_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time latest_health_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time latest_backend_stamp_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr control_state_sub_;
    rclcpp::Subscription<rc26_interfaces::msg::LocalizationHealth>::SharedPtr health_sub_;
    rclcpp::Subscription<rc26_interfaces::msg::LocalizationBackendStatus>::SharedPtr backend_sub_;
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr terrain_sub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr controller_mode_pub_;
    std::shared_ptr<grid_map::GridMap> terrain_map_;
    rclcpp::Time terrain_map_stamp_{0, 0, RCL_ROS_TIME};
    mutable std::mutex terrain_mutex_;

    rclcpp::Client<rc26_interfaces::srv::SetNavMode>::SharedPtr nav_mode_client_;

    mutable geometry_msgs::msg::Twist last_command_;
    mutable bool has_last_command_{false};
    mutable rclcpp::Time last_command_stamp_{0, 0, RCL_ROS_TIME};

    bool fallback_mode_active_{false};
    FallbackReason fallback_reason_{FallbackReason::kNone};
    rclcpp::Time fallback_enter_stamp_{0, 0, RCL_ROS_TIME};
    std::string last_requested_profile_;

    int consecutive_solver_failures_{0};
    int consecutive_solver_successes_{0};

    // Timing and solver settings
    double control_period_sec_{1.0 / 30.0};
    double solver_time_limit_ms_{3.0};
    int solver_timeout_cycles_{2};
    int fallback_recover_cycles_{6};
    double fallback_min_hold_sec_{0.5};
    bool use_osqp_{true};

    // Cost weights
    double w_ref_vx_{8.0};
    double w_ref_vy_{8.0};
    double w_ref_wz_{6.0};
    double w_smooth_vx_{2.0};
    double w_smooth_vy_{2.0};
    double w_smooth_wz_{1.5};

    // Hard limits
    double vx_min_{-2.5};
    double vx_max_{2.5};
    double vy_min_{-2.5};
    double vy_max_{2.5};
    double wz_min_{-3.0};
    double wz_max_{3.0};
    double ax_max_{3.0};
    double ay_max_{3.0};
    double aw_max_{6.0};

    // Localization-aware conservative scaling
    double lhi_yellow_scale_{0.85};
    double lhi_orange_scale_{0.55};
    double degraded_scale_{0.35};
    double backend_not_ready_scale_{0.7};
    double backend_stale_scale_{0.7};
    double backend_graph_health_ref_{0.7};
    double backend_graph_health_min_scale_{0.4};
    double jump_suppressed_scale_{0.6};
    double imu_spike_scale_{0.6};
    double min_quality_scale_{0.08};
    double health_timeout_sec_{0.25};
    double backend_timeout_sec_{0.5};
    double control_state_timeout_sec_{0.25};

    // Output handover / anti-chatter
    double handover_a_lin_{2.2};
    double handover_a_ang_{4.0};

    // External speed limit from Nav2 costmap filters
    double speed_limit_scale_{1.0};
    double terrain_scale_{1.0};

    // Terrain sampler for first-step NMPC terrain-aware scaling
    bool terrain_enable_{true};
    std::string terrain_grid_topic_{"/terrain_grid_map_local"};
    std::string terrain_traversability_layer_{"traversability"};
    std::string terrain_fresh_layer_{"fresh"};
    std::string terrain_roughness_layer_{"roughness"};
    std::string terrain_step_up_layer_{"step_up"};
    std::string terrain_rule_legality_layer_{"rule_legality"};
    std::string terrain_kfs_keepout_layer_{"kfs_keepout"};
    std::string terrain_block_occupied_layer_{"block_occupied"};
    std::string terrain_ramp_corridor_layer_{"ramp_corridor_mask"};
    int terrain_sample_count_{10};
    int terrain_horizon_points_{24};
    double terrain_scale_min_{0.35};
    double terrain_roughness_limit_{0.35};
    double terrain_step_up_limit_{0.08};
    double terrain_rule_legality_threshold_{0.5};
    double terrain_keepout_threshold_{0.5};
    double terrain_block_occupied_threshold_{0.5};
    bool terrain_enforce_ramp_corridor_{false};
    double terrain_stale_timeout_sec_{0.4};

    // Profiles to trigger when fallback is active
    std::string fallback_controller_id_{"FollowPath"};
    std::string profile_orange_{"loc_orange"};
    std::string profile_red_{"loc_red_hold"};
};

}  // namespace rc26_nmpc_controller
