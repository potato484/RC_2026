#include "rc26_nmpc_controller/nmpc_controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>

#include "grid_map_ros/GridMapRosConverter.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#ifndef RC26_NMPC_HAS_OSQP
#define RC26_NMPC_HAS_OSQP 0
#endif

#if RC26_NMPC_HAS_OSQP
#if __has_include(<osqp/osqp.h>)
#include <osqp/osqp.h>
#elif __has_include(<osqp.h>)
#include <osqp.h>
#else
#error "RC26_NMPC_HAS_OSQP is enabled but osqp headers were not found"
#endif

#ifndef OSQP_SOLVED_INACCURATE
#define OSQP_SOLVED_INACCURATE 2
#endif
#ifndef OSQP_PRIMAL_INFEASIBLE_INACCURATE
#define OSQP_PRIMAL_INFEASIBLE_INACCURATE 3
#endif
#ifndef OSQP_DUAL_INFEASIBLE_INACCURATE
#define OSQP_DUAL_INFEASIBLE_INACCURATE 4
#endif
#ifndef OSQP_TIME_LIMIT_REACHED
#define OSQP_TIME_LIMIT_REACHED (-6)
#endif
#endif

namespace rc26_nmpc_controller {

namespace {

using nav2_util::declare_parameter_if_not_declared;

std::string reasonToString(FallbackReason reason) {
    switch (reason) {
        case FallbackReason::kLocalizationRed:
            return "loc_red";
        case FallbackReason::kSolverTimeout:
            return "solver_timeout";
        case FallbackReason::kSolverInfeasible:
            return "solver_infeasible";
        case FallbackReason::kNone:
        default:
            return "none";
    }
}

double clampScale(double value) {
    return std::clamp(value, 0.0, 1.0);
}

}  // namespace

void NmpcController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                               std::shared_ptr<tf2_ros::Buffer> tf,
                               std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    auto node = parent.lock();
    if (!node) {
        throw nav2_core::PlannerException("Unable to lock lifecycle node in NmpcController::configure");
    }

    node_ = parent;
    tf_ = std::move(tf);
    costmap_ros_ = std::move(costmap_ros);
    costmap_ = costmap_ros_ ? costmap_ros_->getCostmap() : nullptr;
    plugin_name_ = std::move(name);
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    double controller_frequency = 30.0;

    declare_parameter_if_not_declared(node, plugin_name_ + ".solver_time_limit_ms", rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".solver_timeout_cycles", rclcpp::ParameterValue(2));
    declare_parameter_if_not_declared(node, plugin_name_ + ".fallback_recover_cycles", rclcpp::ParameterValue(6));
    declare_parameter_if_not_declared(node, plugin_name_ + ".fallback_min_hold_sec", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".use_osqp", rclcpp::ParameterValue(true));

    declare_parameter_if_not_declared(node, plugin_name_ + ".w_ref_vx", rclcpp::ParameterValue(8.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".w_ref_vy", rclcpp::ParameterValue(8.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".w_ref_wz", rclcpp::ParameterValue(6.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".w_smooth_vx", rclcpp::ParameterValue(2.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".w_smooth_vy", rclcpp::ParameterValue(2.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".w_smooth_wz", rclcpp::ParameterValue(1.5));

    declare_parameter_if_not_declared(node, plugin_name_ + ".vx_min", rclcpp::ParameterValue(-2.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".vx_max", rclcpp::ParameterValue(2.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".vy_min", rclcpp::ParameterValue(-2.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".vy_max", rclcpp::ParameterValue(2.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".wz_min", rclcpp::ParameterValue(-3.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".wz_max", rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".ax_max", rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".ay_max", rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".aw_max", rclcpp::ParameterValue(6.0));

    declare_parameter_if_not_declared(node, plugin_name_ + ".lhi_yellow_scale", rclcpp::ParameterValue(0.85));
    declare_parameter_if_not_declared(node, plugin_name_ + ".lhi_orange_scale", rclcpp::ParameterValue(0.55));
    declare_parameter_if_not_declared(node, plugin_name_ + ".degraded_scale", rclcpp::ParameterValue(0.35));
    declare_parameter_if_not_declared(node, plugin_name_ + ".backend_not_ready_scale", rclcpp::ParameterValue(0.7));
    declare_parameter_if_not_declared(node, plugin_name_ + ".backend_stale_scale", rclcpp::ParameterValue(0.7));
    declare_parameter_if_not_declared(node, plugin_name_ + ".backend_graph_health_ref", rclcpp::ParameterValue(0.7));
    declare_parameter_if_not_declared(node, plugin_name_ + ".backend_graph_health_min_scale", rclcpp::ParameterValue(0.4));
    declare_parameter_if_not_declared(node, plugin_name_ + ".jump_suppressed_scale", rclcpp::ParameterValue(0.6));
    declare_parameter_if_not_declared(node, plugin_name_ + ".imu_spike_scale", rclcpp::ParameterValue(0.6));
    declare_parameter_if_not_declared(node, plugin_name_ + ".min_quality_scale", rclcpp::ParameterValue(0.08));
    declare_parameter_if_not_declared(node, plugin_name_ + ".health_timeout_sec", rclcpp::ParameterValue(0.25));
    declare_parameter_if_not_declared(node, plugin_name_ + ".backend_timeout_sec", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".control_state_timeout_sec", rclcpp::ParameterValue(0.25));

    declare_parameter_if_not_declared(node, plugin_name_ + ".handover_a_lin", rclcpp::ParameterValue(2.2));
    declare_parameter_if_not_declared(node, plugin_name_ + ".handover_a_ang", rclcpp::ParameterValue(4.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".terrain_enable", rclcpp::ParameterValue(true));
    declare_parameter_if_not_declared(node, plugin_name_ + ".terrain_grid_topic",
                                      rclcpp::ParameterValue(std::string("/terrain_grid_map_local")));
    declare_parameter_if_not_declared(node, plugin_name_ + ".terrain_traversability_layer",
                                      rclcpp::ParameterValue(std::string("traversability")));
    declare_parameter_if_not_declared(node, plugin_name_ + ".terrain_fresh_layer",
                                      rclcpp::ParameterValue(std::string("fresh")));
    declare_parameter_if_not_declared(node, plugin_name_ + ".terrain_roughness_layer",
                                      rclcpp::ParameterValue(std::string("roughness")));
    declare_parameter_if_not_declared(node, plugin_name_ + ".terrain_step_up_layer",
                                      rclcpp::ParameterValue(std::string("step_up")));
    declare_parameter_if_not_declared(node, plugin_name_ + ".terrain_rule_legality_layer",
                                      rclcpp::ParameterValue(std::string("rule_legality")));
    declare_parameter_if_not_declared(node, plugin_name_ + ".terrain_kfs_keepout_layer",
                                      rclcpp::ParameterValue(std::string("kfs_keepout")));
    declare_parameter_if_not_declared(node, plugin_name_ + ".terrain_block_occupied_layer",
                                      rclcpp::ParameterValue(std::string("block_occupied")));
    declare_parameter_if_not_declared(node, plugin_name_ + ".terrain_ramp_corridor_layer",
                                      rclcpp::ParameterValue(std::string("ramp_corridor_mask")));
    declare_parameter_if_not_declared(node, plugin_name_ + ".terrain_sample_count", rclcpp::ParameterValue(10));
    declare_parameter_if_not_declared(node, plugin_name_ + ".terrain_horizon_points", rclcpp::ParameterValue(24));
    declare_parameter_if_not_declared(node, plugin_name_ + ".terrain_scale_min", rclcpp::ParameterValue(0.35));
    declare_parameter_if_not_declared(node, plugin_name_ + ".terrain_roughness_limit", rclcpp::ParameterValue(0.35));
    declare_parameter_if_not_declared(node, plugin_name_ + ".terrain_step_up_limit", rclcpp::ParameterValue(0.08));
    declare_parameter_if_not_declared(node, plugin_name_ + ".terrain_rule_legality_threshold", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".terrain_keepout_threshold", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".terrain_block_occupied_threshold", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".terrain_enforce_ramp_corridor", rclcpp::ParameterValue(false));
    declare_parameter_if_not_declared(node, plugin_name_ + ".terrain_stale_timeout_sec", rclcpp::ParameterValue(0.4));

    declare_parameter_if_not_declared(node, plugin_name_ + ".fallback_controller_id",
                                      rclcpp::ParameterValue(std::string("FollowPath")));
    declare_parameter_if_not_declared(node, plugin_name_ + ".profile_orange", rclcpp::ParameterValue(std::string("loc_orange")));
    declare_parameter_if_not_declared(node, plugin_name_ + ".profile_red", rclcpp::ParameterValue(std::string("loc_red_hold")));

    node->get_parameter(plugin_name_ + ".solver_time_limit_ms", solver_time_limit_ms_);
    node->get_parameter(plugin_name_ + ".solver_timeout_cycles", solver_timeout_cycles_);
    node->get_parameter(plugin_name_ + ".fallback_recover_cycles", fallback_recover_cycles_);
    node->get_parameter(plugin_name_ + ".fallback_min_hold_sec", fallback_min_hold_sec_);
    node->get_parameter(plugin_name_ + ".use_osqp", use_osqp_);

    node->get_parameter(plugin_name_ + ".w_ref_vx", w_ref_vx_);
    node->get_parameter(plugin_name_ + ".w_ref_vy", w_ref_vy_);
    node->get_parameter(plugin_name_ + ".w_ref_wz", w_ref_wz_);
    node->get_parameter(plugin_name_ + ".w_smooth_vx", w_smooth_vx_);
    node->get_parameter(plugin_name_ + ".w_smooth_vy", w_smooth_vy_);
    node->get_parameter(plugin_name_ + ".w_smooth_wz", w_smooth_wz_);

    node->get_parameter(plugin_name_ + ".vx_min", vx_min_);
    node->get_parameter(plugin_name_ + ".vx_max", vx_max_);
    node->get_parameter(plugin_name_ + ".vy_min", vy_min_);
    node->get_parameter(plugin_name_ + ".vy_max", vy_max_);
    node->get_parameter(plugin_name_ + ".wz_min", wz_min_);
    node->get_parameter(plugin_name_ + ".wz_max", wz_max_);
    node->get_parameter(plugin_name_ + ".ax_max", ax_max_);
    node->get_parameter(plugin_name_ + ".ay_max", ay_max_);
    node->get_parameter(plugin_name_ + ".aw_max", aw_max_);

    node->get_parameter(plugin_name_ + ".lhi_yellow_scale", lhi_yellow_scale_);
    node->get_parameter(plugin_name_ + ".lhi_orange_scale", lhi_orange_scale_);
    node->get_parameter(plugin_name_ + ".degraded_scale", degraded_scale_);
    node->get_parameter(plugin_name_ + ".backend_not_ready_scale", backend_not_ready_scale_);
    node->get_parameter(plugin_name_ + ".backend_stale_scale", backend_stale_scale_);
    node->get_parameter(plugin_name_ + ".backend_graph_health_ref", backend_graph_health_ref_);
    node->get_parameter(plugin_name_ + ".backend_graph_health_min_scale", backend_graph_health_min_scale_);
    node->get_parameter(plugin_name_ + ".jump_suppressed_scale", jump_suppressed_scale_);
    node->get_parameter(plugin_name_ + ".imu_spike_scale", imu_spike_scale_);
    node->get_parameter(plugin_name_ + ".min_quality_scale", min_quality_scale_);
    node->get_parameter(plugin_name_ + ".health_timeout_sec", health_timeout_sec_);
    node->get_parameter(plugin_name_ + ".backend_timeout_sec", backend_timeout_sec_);
    node->get_parameter(plugin_name_ + ".control_state_timeout_sec", control_state_timeout_sec_);

    node->get_parameter(plugin_name_ + ".handover_a_lin", handover_a_lin_);
    node->get_parameter(plugin_name_ + ".handover_a_ang", handover_a_ang_);
    node->get_parameter(plugin_name_ + ".terrain_enable", terrain_enable_);
    node->get_parameter(plugin_name_ + ".terrain_grid_topic", terrain_grid_topic_);
    node->get_parameter(plugin_name_ + ".terrain_traversability_layer", terrain_traversability_layer_);
    node->get_parameter(plugin_name_ + ".terrain_fresh_layer", terrain_fresh_layer_);
    node->get_parameter(plugin_name_ + ".terrain_roughness_layer", terrain_roughness_layer_);
    node->get_parameter(plugin_name_ + ".terrain_step_up_layer", terrain_step_up_layer_);
    node->get_parameter(plugin_name_ + ".terrain_rule_legality_layer", terrain_rule_legality_layer_);
    node->get_parameter(plugin_name_ + ".terrain_kfs_keepout_layer", terrain_kfs_keepout_layer_);
    node->get_parameter(plugin_name_ + ".terrain_block_occupied_layer", terrain_block_occupied_layer_);
    node->get_parameter(plugin_name_ + ".terrain_ramp_corridor_layer", terrain_ramp_corridor_layer_);
    node->get_parameter(plugin_name_ + ".terrain_sample_count", terrain_sample_count_);
    node->get_parameter(plugin_name_ + ".terrain_horizon_points", terrain_horizon_points_);
    node->get_parameter(plugin_name_ + ".terrain_scale_min", terrain_scale_min_);
    node->get_parameter(plugin_name_ + ".terrain_roughness_limit", terrain_roughness_limit_);
    node->get_parameter(plugin_name_ + ".terrain_step_up_limit", terrain_step_up_limit_);
    node->get_parameter(plugin_name_ + ".terrain_rule_legality_threshold", terrain_rule_legality_threshold_);
    node->get_parameter(plugin_name_ + ".terrain_keepout_threshold", terrain_keepout_threshold_);
    node->get_parameter(plugin_name_ + ".terrain_block_occupied_threshold", terrain_block_occupied_threshold_);
    node->get_parameter(plugin_name_ + ".terrain_enforce_ramp_corridor", terrain_enforce_ramp_corridor_);
    node->get_parameter(plugin_name_ + ".terrain_stale_timeout_sec", terrain_stale_timeout_sec_);

    node->get_parameter(plugin_name_ + ".fallback_controller_id", fallback_controller_id_);
    node->get_parameter(plugin_name_ + ".profile_orange", profile_orange_);
    node->get_parameter(plugin_name_ + ".profile_red", profile_red_);

    node->get_parameter("controller_frequency", controller_frequency);
    control_period_sec_ = 1.0 / std::max(1e-3, controller_frequency);

    solver_time_limit_ms_ = std::max(0.1, solver_time_limit_ms_);
    solver_timeout_cycles_ = std::max(1, solver_timeout_cycles_);
    fallback_recover_cycles_ = std::max(1, fallback_recover_cycles_);
    fallback_min_hold_sec_ = std::max(0.0, fallback_min_hold_sec_);

    lhi_yellow_scale_ = clampScale(lhi_yellow_scale_);
    lhi_orange_scale_ = clampScale(lhi_orange_scale_);
    degraded_scale_ = clampScale(degraded_scale_);
    backend_not_ready_scale_ = clampScale(backend_not_ready_scale_);
    backend_stale_scale_ = clampScale(backend_stale_scale_);
    backend_graph_health_min_scale_ = clampScale(backend_graph_health_min_scale_);
    jump_suppressed_scale_ = clampScale(jump_suppressed_scale_);
    imu_spike_scale_ = clampScale(imu_spike_scale_);
    min_quality_scale_ = clampScale(min_quality_scale_);

    ax_max_ = std::max(0.0, ax_max_);
    ay_max_ = std::max(0.0, ay_max_);
    aw_max_ = std::max(0.0, aw_max_);
    handover_a_lin_ = std::max(0.0, handover_a_lin_);
    handover_a_ang_ = std::max(0.0, handover_a_ang_);
    terrain_sample_count_ = std::clamp(terrain_sample_count_, 3, 64);
    terrain_horizon_points_ = std::clamp(terrain_horizon_points_, 6, 120);
    terrain_scale_min_ = std::clamp(terrain_scale_min_, 0.05, 1.0);
    terrain_roughness_limit_ = std::max(1e-3, terrain_roughness_limit_);
    terrain_step_up_limit_ = std::max(1e-3, terrain_step_up_limit_);
    terrain_rule_legality_threshold_ = std::clamp(terrain_rule_legality_threshold_, 0.0, 1.0);
    terrain_keepout_threshold_ = std::clamp(terrain_keepout_threshold_, 0.0, 1.0);
    terrain_block_occupied_threshold_ = std::clamp(terrain_block_occupied_threshold_, 0.0, 1.0);
    terrain_stale_timeout_sec_ = std::max(0.0, terrain_stale_timeout_sec_);

    control_state_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        "/control_state", rclcpp::SensorDataQoS(), [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_control_state_ = msg;
            latest_control_state_stamp_ = rclcpp::Time(msg->header.stamp, clock_->get_clock_type());
        });

    health_sub_ = node->create_subscription<rc26_interfaces::msg::LocalizationHealth>(
        "/localization/health", rclcpp::SensorDataQoS(), [this](const rc26_interfaces::msg::LocalizationHealth::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_health_ = msg;
            if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
                latest_health_stamp_ = clock_->now();
            } else {
                latest_health_stamp_ = rclcpp::Time(msg->header.stamp, clock_->get_clock_type());
            }
        });

    backend_sub_ = node->create_subscription<rc26_interfaces::msg::LocalizationBackendStatus>(
        "/localization/backend_status", rclcpp::SensorDataQoS(),
        [this](const rc26_interfaces::msg::LocalizationBackendStatus::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_backend_ = msg;
            if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
                latest_backend_stamp_ = clock_->now();
            } else {
                latest_backend_stamp_ = rclcpp::Time(msg->header.stamp, clock_->get_clock_type());
            }
        });

    if (terrain_enable_) {
        rclcpp::QoS terrain_qos(rclcpp::KeepLast(1));
        terrain_qos.reliable();
        terrain_qos.transient_local();
        terrain_sub_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
            terrain_grid_topic_, terrain_qos,
            std::bind(&NmpcController::terrainGridCallback, this, std::placeholders::_1));
    }

    controller_mode_pub_ = node->create_publisher<std_msgs::msg::String>(plugin_name_ + "/mode", rclcpp::QoS(10));

    nav_mode_client_ = node->create_client<rc26_interfaces::srv::SetNavMode>("set_nav_mode");

    fallback_controller_ = std::make_unique<rc26_omni_controller::OmniPidPursuitController>();
    fallback_controller_->configure(parent, fallback_controller_id_, tf_, costmap_ros_);

#if !RC26_NMPC_HAS_OSQP
    if (use_osqp_) {
        RCLCPP_WARN(logger_, "OSQP backend requested but unavailable. Falling back to projected-QP solver.");
    }
#endif

    resetRuntimeState();

    RCLCPP_INFO(logger_, "%s configured. control_period=%.4f sec, solver_limit=%.2f ms",
                plugin_name_.c_str(), control_period_sec_, solver_time_limit_ms_);
}

void NmpcController::cleanup() {
    fallback_controller_.reset();
    control_state_sub_.reset();
    health_sub_.reset();
    backend_sub_.reset();
    terrain_sub_.reset();
    controller_mode_pub_.reset();
    nav_mode_client_.reset();

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        global_plan_.poses.clear();
        latest_control_state_.reset();
        latest_health_.reset();
        latest_backend_.reset();
    }
    {
        std::lock_guard<std::mutex> terrain_lock(terrain_mutex_);
        terrain_map_.reset();
        terrain_map_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    resetRuntimeState();
    tf_.reset();
    costmap_ros_.reset();
    costmap_ = nullptr;
}

void NmpcController::activate() {
    if (fallback_controller_) {
        fallback_controller_->activate();
    }
    if (controller_mode_pub_) {
        controller_mode_pub_->on_activate();
    }
}

void NmpcController::deactivate() {
    if (controller_mode_pub_) {
        controller_mode_pub_->on_deactivate();
    }
    if (fallback_controller_) {
        fallback_controller_->deactivate();
    }
}

void NmpcController::setPlan(const nav_msgs::msg::Path& path) {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        global_plan_ = path;
    }
    if (fallback_controller_) {
        fallback_controller_->setPlan(path);
    }
    consecutive_solver_failures_ = 0;
    consecutive_solver_successes_ = 0;
}

void NmpcController::setSpeedLimit(const double& speed_limit, const bool& percentage) {
    if (fallback_controller_) {
        fallback_controller_->setSpeedLimit(speed_limit, percentage);
    }

    if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
        speed_limit_scale_ = 1.0;
        return;
    }

    if (percentage) {
        speed_limit_scale_ = clampScale(speed_limit / 100.0);
    } else {
        const double ref = std::max(std::abs(vx_max_), 1e-6);
        speed_limit_scale_ = clampScale(speed_limit / ref);
    }
}

geometry_msgs::msg::TwistStamped NmpcController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                                          const geometry_msgs::msg::Twist& velocity,
                                                                          nav2_core::GoalChecker* goal_checker) {
    if (!fallback_controller_) {
        throw nav2_core::PlannerException("Fallback controller is not initialized");
    }

    const rclcpp::Time now = clock_->now();
    const double dt = has_last_command_ ? std::clamp((now - last_command_stamp_).seconds(), 1e-3, 0.2)
                                        : control_period_sec_;

    geometry_msgs::msg::TwistStamped fallback_cmd;
    fallback_cmd = fallback_controller_->computeVelocityCommands(pose, velocity, goal_checker);

    bool localization_red = false;
    const double quality_scale = computeQualityScale(now, localization_red);
    terrain_scale_ = computeTerrainScale(pose, now);
    if (terrain_scale_ <= 1e-3) {
        geometry_msgs::msg::TwistStamped out;
        out.header = pose.header;
        if (out.header.stamp.sec == 0 && out.header.stamp.nanosec == 0) {
            out.header.stamp = now;
        }
        out.twist = geometry_msgs::msg::Twist();
        last_command_ = out.twist;
        has_last_command_ = true;
        last_command_stamp_ = now;
        if (controller_mode_pub_ && controller_mode_pub_->is_activated()) {
            std_msgs::msg::String mode_msg;
            mode_msg.data = "nmpc:terrain_blocked";
            controller_mode_pub_->publish(mode_msg);
        }
        return out;
    }
    const auto measured_velocity = readMeasuredVelocity(velocity, now);

    SolveReport solve_report;
    if (!localization_red) {
        solve_report = solveConstrainedCommand(fallback_cmd.twist, measured_velocity, dt, quality_scale);
    }

    if (solve_report.success) {
        consecutive_solver_failures_ = 0;
        ++consecutive_solver_successes_;
    } else {
        ++consecutive_solver_failures_;
        consecutive_solver_successes_ = 0;
    }

    if (localization_red) {
        enterFallbackMode(FallbackReason::kLocalizationRed, now);
    } else if (!solve_report.success && solve_report.timed_out &&
               consecutive_solver_failures_ > solver_timeout_cycles_) {
        enterFallbackMode(FallbackReason::kSolverTimeout, now);
    } else if (!solve_report.success && solve_report.infeasible &&
               consecutive_solver_failures_ > solver_timeout_cycles_) {
        enterFallbackMode(FallbackReason::kSolverInfeasible, now);
    } else {
        maybeExitFallbackMode(solve_report.success, localization_red, now);
    }

    geometry_msgs::msg::Twist target_cmd = fallback_cmd.twist;
    if (!fallback_mode_active_ && solve_report.success) {
        target_cmd = solve_report.command;
    }

    const auto output_cmd = applySlewRateLimit(target_cmd, dt);

    last_command_ = output_cmd;
    has_last_command_ = true;
    last_command_stamp_ = now;

    if (controller_mode_pub_ && controller_mode_pub_->is_activated()) {
        std_msgs::msg::String mode_msg;
        mode_msg.data = fallback_mode_active_ ? ("fallback:" + reasonToString(fallback_reason_)) : "nmpc";
        controller_mode_pub_->publish(mode_msg);
    }

    geometry_msgs::msg::TwistStamped out;
    out.header = pose.header;
    if (out.header.stamp.sec == 0 && out.header.stamp.nanosec == 0) {
        out.header.stamp = now;
    }
    out.twist = output_cmd;
    return out;
}

void NmpcController::resetRuntimeState() {
    fallback_mode_active_ = false;
    fallback_reason_ = FallbackReason::kNone;
    fallback_enter_stamp_ = clock_ ? clock_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_requested_profile_.clear();

    consecutive_solver_failures_ = 0;
    consecutive_solver_successes_ = 0;

    last_command_ = geometry_msgs::msg::Twist();
    has_last_command_ = false;
    last_command_stamp_ = clock_ ? clock_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
    speed_limit_scale_ = 1.0;
    terrain_scale_ = 1.0;
}

void NmpcController::requestNavProfile(const std::string& profile, const std::string& reason) {
    if (profile.empty() || profile == last_requested_profile_) {
        return;
    }
    if (!nav_mode_client_) {
        return;
    }
    if (!nav_mode_client_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 3000,
                             "set_nav_mode service unavailable, skip profile request '%s'", profile.c_str());
        return;
    }

    auto request = std::make_shared<rc26_interfaces::srv::SetNavMode::Request>();
    request->profile = profile;
    request->timeout = 0.0f;
    request->reason = reason;
    nav_mode_client_->async_send_request(request);
    last_requested_profile_ = profile;
}

void NmpcController::enterFallbackMode(FallbackReason reason, const rclcpp::Time& now) {
    if (!fallback_mode_active_ || fallback_reason_ != reason) {
        fallback_mode_active_ = true;
        fallback_reason_ = reason;
        fallback_enter_stamp_ = now;

        if (reason == FallbackReason::kLocalizationRed) {
            requestNavProfile(profile_red_, "nmpc_fallback_" + reasonToString(reason));
        } else {
            requestNavProfile(profile_orange_, "nmpc_fallback_" + reasonToString(reason));
        }

        RCLCPP_WARN(logger_, "%s switched to fallback mode: %s", plugin_name_.c_str(), reasonToString(reason).c_str());
    }
}

void NmpcController::maybeExitFallbackMode(bool solver_success, bool localization_red, const rclcpp::Time& now) {
    if (!fallback_mode_active_) {
        return;
    }
    if (localization_red || !solver_success) {
        return;
    }
    if (consecutive_solver_successes_ < fallback_recover_cycles_) {
        return;
    }
    if ((now - fallback_enter_stamp_).seconds() < fallback_min_hold_sec_) {
        return;
    }

    fallback_mode_active_ = false;
    fallback_reason_ = FallbackReason::kNone;
    last_requested_profile_.clear();
    RCLCPP_INFO(logger_, "%s returned from fallback mode to NMPC", plugin_name_.c_str());
}

double NmpcController::computeQualityScale(const rclcpp::Time& now, bool& localization_red) const {
    localization_red = false;
    double scale = 1.0;

    rc26_interfaces::msg::LocalizationHealth::SharedPtr health;
    rc26_interfaces::msg::LocalizationBackendStatus::SharedPtr backend;
    rclcpp::Time health_stamp(0, 0, clock_->get_clock_type());
    rclcpp::Time backend_stamp(0, 0, clock_->get_clock_type());

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        health = latest_health_;
        backend = latest_backend_;
        health_stamp = latest_health_stamp_;
        backend_stamp = latest_backend_stamp_;
    }

    const bool health_fresh = health && (now - health_stamp).seconds() <= health_timeout_sec_;
    if (!health_fresh) {
        scale *= lhi_orange_scale_;
    } else {
        if (health->level == rc26_interfaces::msg::LocalizationHealth::RED) {
            localization_red = true;
            return min_quality_scale_;
        }
        if (health->level == rc26_interfaces::msg::LocalizationHealth::ORANGE) {
            scale *= lhi_orange_scale_;
        } else if (health->level == rc26_interfaces::msg::LocalizationHealth::YELLOW) {
            scale *= lhi_yellow_scale_;
        }
        if (health->control_degraded) {
            scale *= degraded_scale_;
        }
    }

    const bool backend_fresh = backend && (now - backend_stamp).seconds() <= backend_timeout_sec_;
    if (!backend_fresh) {
        scale *= backend_stale_scale_;
    } else {
        if (!backend->optimizer_ready) {
            scale *= backend_not_ready_scale_;
        }
        if (backend_graph_health_ref_ > 1e-6 && backend->graph_health < backend_graph_health_ref_) {
            const double ratio = std::clamp(backend->graph_health / backend_graph_health_ref_,
                                            backend_graph_health_min_scale_, 1.0);
            scale *= ratio;
        }
        if (backend->map_to_odom_jump_suppressed) {
            scale *= jump_suppressed_scale_;
        }
        if (backend->imu_spike) {
            scale *= imu_spike_scale_;
        }
    }

    return std::clamp(scale, min_quality_scale_, 1.0);
}

std::array<double, 3> NmpcController::readMeasuredVelocity(const geometry_msgs::msg::Twist& nav2_velocity,
                                                            const rclcpp::Time& now) const {
    nav_msgs::msg::Odometry::SharedPtr control_state;
    rclcpp::Time control_state_stamp(0, 0, clock_->get_clock_type());
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        control_state = latest_control_state_;
        control_state_stamp = latest_control_state_stamp_;
    }

    if (control_state && (now - control_state_stamp).seconds() <= control_state_timeout_sec_) {
        return {
            control_state->twist.twist.linear.x,
            control_state->twist.twist.linear.y,
            control_state->twist.twist.angular.z,
        };
    }

    return {
        nav2_velocity.linear.x,
        nav2_velocity.linear.y,
        nav2_velocity.angular.z,
    };
}

geometry_msgs::msg::Twist NmpcController::applySlewRateLimit(const geometry_msgs::msg::Twist& target, double dt) const {
    if (!has_last_command_) {
        return target;
    }

    geometry_msgs::msg::Twist out = target;
    const double max_dv = handover_a_lin_ * std::max(dt, 1e-3);
    const double max_dw = handover_a_ang_ * std::max(dt, 1e-3);

    out.linear.x = last_command_.linear.x + std::clamp(target.linear.x - last_command_.linear.x, -max_dv, max_dv);
    out.linear.y = last_command_.linear.y + std::clamp(target.linear.y - last_command_.linear.y, -max_dv, max_dv);
    out.angular.z = last_command_.angular.z + std::clamp(target.angular.z - last_command_.angular.z, -max_dw, max_dw);
    return out;
}

void NmpcController::terrainGridCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg) {
    if (!msg) {
        return;
    }

    grid_map::GridMap converted;
    if (!grid_map::GridMapRosConverter::fromMessage(*msg, converted)) {
        if (clock_) {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "NMPC failed to convert terrain grid map message");
        }
        return;
    }

    rclcpp::Time stamp = clock_ ? clock_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
    if (msg->header.stamp.sec != 0 || msg->header.stamp.nanosec != 0) {
        stamp = rclcpp::Time(msg->header.stamp, clock_ ? clock_->get_clock_type() : RCL_ROS_TIME);
    }

    std::lock_guard<std::mutex> lock(terrain_mutex_);
    terrain_map_ = std::make_shared<grid_map::GridMap>(std::move(converted));
    terrain_map_stamp_ = stamp;
}

bool NmpcController::readTerrainLayerValue(const grid_map::GridMap& map,
                                           const std::string& layer,
                                           const grid_map::Position& pos,
                                           float& value) const {
    if (layer.empty() || !map.exists(layer)) {
        return false;
    }

    grid_map::Index index;
    if (!map.getIndex(pos, index) || !map.isValid(index, layer)) {
        return false;
    }
    value = map.at(layer, index);
    return std::isfinite(static_cast<double>(value));
}

double NmpcController::computeTerrainScale(const geometry_msgs::msg::PoseStamped& pose, const rclcpp::Time& now) const {
    if (!terrain_enable_) {
        return 1.0;
    }

    std::shared_ptr<grid_map::GridMap> terrain_map;
    rclcpp::Time terrain_stamp(0, 0, RCL_ROS_TIME);
    {
        std::lock_guard<std::mutex> lock(terrain_mutex_);
        terrain_map = terrain_map_;
        terrain_stamp = terrain_map_stamp_;
    }
    if (!terrain_map) {
        return 1.0;
    }
    if (terrain_stale_timeout_sec_ > 0.0 && terrain_stamp.nanoseconds() > 0) {
        const double age = (now - terrain_stamp).seconds();
        if (age > terrain_stale_timeout_sec_) {
            return 1.0;
        }
    }

    nav_msgs::msg::Path plan;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        plan = global_plan_;
    }
    if (plan.poses.empty()) {
        return 1.0;
    }

    std::string plan_frame = plan.header.frame_id;
    if (plan_frame.empty()) {
        plan_frame = pose.header.frame_id;
    }
    if (plan_frame.empty()) {
        return 1.0;
    }

    geometry_msgs::msg::PoseStamped robot_pose_in_plan = pose;
    robot_pose_in_plan.header.frame_id = pose.header.frame_id;
    if (robot_pose_in_plan.header.frame_id.empty()) {
        robot_pose_in_plan.header.frame_id = plan_frame;
    }
    if (robot_pose_in_plan.header.frame_id != plan_frame) {
        if (!tf_) {
            return 1.0;
        }
        try {
            const auto tf_plan_from_robot = tf_->lookupTransform(
                plan_frame, robot_pose_in_plan.header.frame_id, tf2::TimePointZero);
            tf2::doTransform(robot_pose_in_plan, robot_pose_in_plan, tf_plan_from_robot);
            robot_pose_in_plan.header.frame_id = plan_frame;
        } catch (const tf2::TransformException& ex) {
            if (clock_) {
                RCLCPP_WARN_THROTTLE(
                    logger_, *clock_, 3000, "NMPC terrain sampling pose transform failed: %s", ex.what());
            }
            return 1.0;
        }
    }

    size_t nearest_index = 0;
    double nearest_dist_sq = std::numeric_limits<double>::max();
    for (size_t i = 0; i < plan.poses.size(); ++i) {
        const auto& p = plan.poses[i].pose.position;
        const double dx = p.x - robot_pose_in_plan.pose.position.x;
        const double dy = p.y - robot_pose_in_plan.pose.position.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < nearest_dist_sq) {
            nearest_dist_sq = d2;
            nearest_index = i;
        }
    }

    const size_t end_index = std::min(
        plan.poses.size() - 1, nearest_index + static_cast<size_t>(std::max(1, terrain_horizon_points_)));
    if (end_index < nearest_index) {
        return 1.0;
    }

    const int sample_count = std::clamp(terrain_sample_count_, 3, 64);
    const std::string terrain_frame = terrain_map->getFrameId();
    const bool need_transform = !terrain_frame.empty() && terrain_frame != plan_frame;

    geometry_msgs::msg::TransformStamped tf_terrain_from_plan;
    if (need_transform) {
        if (!tf_) {
            return 1.0;
        }
        try {
            tf_terrain_from_plan = tf_->lookupTransform(terrain_frame, plan_frame, tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            if (clock_) {
                RCLCPP_WARN_THROTTLE(
                    logger_, *clock_, 3000, "NMPC terrain sampling frame transform failed: %s", ex.what());
            }
            return 1.0;
        }
    }

    double min_score = 1.0;
    int valid_samples = 0;
    for (int i = 0; i < sample_count; ++i) {
        const double ratio = sample_count == 1 ? 1.0 : static_cast<double>(i) / static_cast<double>(sample_count - 1);
        const size_t index = nearest_index +
                             static_cast<size_t>(std::lround(ratio * static_cast<double>(end_index - nearest_index)));
        const auto clamped_index = std::min(index, end_index);

        geometry_msgs::msg::PoseStamped sample_pose = plan.poses[clamped_index];
        if (sample_pose.header.frame_id.empty()) {
            sample_pose.header.frame_id = plan_frame;
        }

        geometry_msgs::msg::PoseStamped sample_pose_in_terrain = sample_pose;
        if (need_transform) {
            tf2::doTransform(sample_pose, sample_pose_in_terrain, tf_terrain_from_plan);
        }

        const grid_map::Position sample_pos(sample_pose_in_terrain.pose.position.x, sample_pose_in_terrain.pose.position.y);

        if (!terrain_fresh_layer_.empty() && terrain_map->exists(terrain_fresh_layer_)) {
            float fresh = 0.0f;
            if (!readTerrainLayerValue(*terrain_map, terrain_fresh_layer_, sample_pos, fresh) || fresh < 0.5f) {
                continue;
            }
        }

        float rule_legality = 1.0f;
        if (!terrain_rule_legality_layer_.empty() &&
            readTerrainLayerValue(*terrain_map, terrain_rule_legality_layer_, sample_pos, rule_legality) &&
            rule_legality <= static_cast<float>(terrain_rule_legality_threshold_)) {
            return 0.0;
        }

        float keepout = 0.0f;
        if (!terrain_kfs_keepout_layer_.empty() &&
            readTerrainLayerValue(*terrain_map, terrain_kfs_keepout_layer_, sample_pos, keepout) &&
            keepout >= static_cast<float>(terrain_keepout_threshold_)) {
            return 0.0;
        }

        float block_occupied = 0.0f;
        if (!terrain_block_occupied_layer_.empty() &&
            readTerrainLayerValue(*terrain_map, terrain_block_occupied_layer_, sample_pos, block_occupied) &&
            block_occupied >= static_cast<float>(terrain_block_occupied_threshold_)) {
            return 0.0;
        }

        if (terrain_enforce_ramp_corridor_ && !terrain_ramp_corridor_layer_.empty()) {
            float ramp_mask = 0.0f;
            if (readTerrainLayerValue(*terrain_map, terrain_ramp_corridor_layer_, sample_pos, ramp_mask) &&
                ramp_mask < 0.5f) {
                return 0.0;
            }
        }

        float traversability = 0.0f;
        if (!readTerrainLayerValue(*terrain_map, terrain_traversability_layer_, sample_pos, traversability)) {
            continue;
        }

        double score = std::clamp(static_cast<double>(traversability), 0.0, 1.0);

        float roughness = 0.0f;
        if (readTerrainLayerValue(*terrain_map, terrain_roughness_layer_, sample_pos, roughness)) {
            const double rough = std::max(0.0, static_cast<double>(roughness));
            const double rough_score = std::clamp(1.0 - rough / std::max(terrain_roughness_limit_, 1e-6), 0.0, 1.0);
            score = std::min(score, rough_score);
        }

        float step_up = 0.0f;
        if (readTerrainLayerValue(*terrain_map, terrain_step_up_layer_, sample_pos, step_up)) {
            const double step_score =
                std::clamp(1.0 - std::max(0.0, static_cast<double>(step_up)) / std::max(terrain_step_up_limit_, 1e-6), 0.0, 1.0);
            score = std::min(score, step_score);
        }

        min_score = std::min(min_score, score);
        ++valid_samples;
    }

    if (valid_samples == 0) {
        return 1.0;
    }

    const double scale = terrain_scale_min_ + (1.0 - terrain_scale_min_) * std::clamp(min_score, 0.0, 1.0);
    return std::clamp(scale, terrain_scale_min_, 1.0);
}

SolveReport NmpcController::solveConstrainedCommand(const geometry_msgs::msg::Twist& reference_cmd,
                                                    const std::array<double, 3>& measured_velocity, double dt,
                                                    double quality_scale) const {
    SolveReport report;

    const double scale = std::clamp(quality_scale * speed_limit_scale_ * terrain_scale_, min_quality_scale_, 1.0);

    const std::array<double, 3> u_ref = {
        reference_cmd.linear.x,
        reference_cmd.linear.y,
        reference_cmd.angular.z,
    };
    const std::array<double, 3> u_prev = measured_velocity;

    const std::array<double, 3> v_min = {
        vx_min_ * scale,
        vy_min_ * scale,
        wz_min_ * scale,
    };
    const std::array<double, 3> v_max = {
        vx_max_ * scale,
        vy_max_ * scale,
        wz_max_ * scale,
    };

    const std::array<double, 3> a_max = {
        ax_max_,
        ay_max_,
        aw_max_,
    };

    std::array<double, 3> lower{};
    std::array<double, 3> upper{};
    for (size_t i = 0; i < 3; ++i) {
        const double acc_low = u_prev[i] - a_max[i] * std::max(dt, 1e-3);
        const double acc_high = u_prev[i] + a_max[i] * std::max(dt, 1e-3);
        lower[i] = std::max(v_min[i], acc_low);
        upper[i] = std::min(v_max[i], acc_high);
        if (lower[i] > upper[i]) {
            report.infeasible = true;
            return report;
        }
    }

    const auto solve_start = std::chrono::steady_clock::now();

    std::array<double, 3> solution = {};

#if RC26_NMPC_HAS_OSQP
    bool osqp_solved = false;
    if (use_osqp_) {
        // 1-step RTI-SQP inner QP: min 0.5(u-u_ref)^T W_ref (u-u_ref) + 0.5(u-u_prev)^T W_smooth (u-u_prev)
        // subject to box constraints (velocity + acceleration envelope)
        const c_int n = 3;
        const c_int m = 3;

        c_float p_x[3] = {
            static_cast<c_float>(w_ref_vx_ + w_smooth_vx_),
            static_cast<c_float>(w_ref_vy_ + w_smooth_vy_),
            static_cast<c_float>(w_ref_wz_ + w_smooth_wz_),
        };
        c_int p_i[3] = {0, 1, 2};
        c_int p_p[4] = {0, 1, 2, 3};

        c_float q[3] = {
            static_cast<c_float>(-(w_ref_vx_ * u_ref[0] + w_smooth_vx_ * u_prev[0])),
            static_cast<c_float>(-(w_ref_vy_ * u_ref[1] + w_smooth_vy_ * u_prev[1])),
            static_cast<c_float>(-(w_ref_wz_ * u_ref[2] + w_smooth_wz_ * u_prev[2])),
        };

        c_float a_x[3] = {1.0, 1.0, 1.0};
        c_int a_i[3] = {0, 1, 2};
        c_int a_p[4] = {0, 1, 2, 3};

        c_float l[3] = {
            static_cast<c_float>(lower[0]),
            static_cast<c_float>(lower[1]),
            static_cast<c_float>(lower[2]),
        };
        c_float u[3] = {
            static_cast<c_float>(upper[0]),
            static_cast<c_float>(upper[1]),
            static_cast<c_float>(upper[2]),
        };

        OSQPData data;
        data.n = n;
        data.m = m;
        data.P = csc_matrix(n, n, 3, p_x, p_i, p_p);
        data.q = q;
        data.A = csc_matrix(m, n, 3, a_x, a_i, a_p);
        data.l = l;
        data.u = u;

        OSQPSettings settings;
        osqp_set_default_settings(&settings);
        settings.verbose = false;
        settings.max_iter = 80;
        settings.warm_start = false;
#ifdef PROFILING
        settings.time_limit = std::max(1e-4, solver_time_limit_ms_ / 1000.0);
#endif

        OSQPWorkspace* work = nullptr;
        const c_int setup_status = osqp_setup(&work, &data, &settings);
        if (setup_status == 0 && work != nullptr) {
            const c_int status = osqp_solve(work);
            (void)status;
            const c_int status_val = work->info ? work->info->status_val : OSQP_UNSOLVED;
            const bool solved = status_val == OSQP_SOLVED || status_val == OSQP_SOLVED_INACCURATE;
            const bool timed_out = status_val == OSQP_TIME_LIMIT_REACHED;
            const bool infeasible =
                status_val == OSQP_PRIMAL_INFEASIBLE || status_val == OSQP_PRIMAL_INFEASIBLE_INACCURATE ||
                status_val == OSQP_DUAL_INFEASIBLE || status_val == OSQP_DUAL_INFEASIBLE_INACCURATE;

            if (solved && work->solution && work->solution->x) {
                solution[0] = static_cast<double>(work->solution->x[0]);
                solution[1] = static_cast<double>(work->solution->x[1]);
                solution[2] = static_cast<double>(work->solution->x[2]);
                osqp_solved = true;
            } else {
                report.timed_out = timed_out;
                report.infeasible = infeasible;
                if (!report.timed_out && !report.infeasible) {
                    report.infeasible = true;
                }
            }

            (void)osqp_cleanup(work);
        } else {
            report.infeasible = true;
        }

        if (data.P) {
            c_free(data.P);
        }
        if (data.A) {
            c_free(data.A);
        }
    }

    if (!osqp_solved) {
#endif
        const std::array<double, 3> w_ref = {w_ref_vx_, w_ref_vy_, w_ref_wz_};
        const std::array<double, 3> w_smooth = {w_smooth_vx_, w_smooth_vy_, w_smooth_wz_};

        for (size_t i = 0; i < 3; ++i) {
            const double den = std::max(1e-6, w_ref[i] + w_smooth[i]);
            const double unconstrained = (w_ref[i] * u_ref[i] + w_smooth[i] * u_prev[i]) / den;
            solution[i] = std::clamp(unconstrained, lower[i], upper[i]);
        }
#if RC26_NMPC_HAS_OSQP
    }
#endif

    report.solve_time_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - solve_start).count();

    if (report.solve_time_ms > solver_time_limit_ms_) {
        report.timed_out = true;
        report.success = false;
        return report;
    }

    report.command.linear.x = solution[0];
    report.command.linear.y = solution[1];
    report.command.angular.z = solution[2];
    report.success = true;
    return report;
}

}  // namespace rc26_nmpc_controller

PLUGINLIB_EXPORT_CLASS(rc26_nmpc_controller::NmpcController, nav2_core::Controller)
