// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rc26_omni_controller/omni_pid_pursuit_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <limits>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;
using namespace nav2_costmap_2d;  // NOLINT
using rcl_interfaces::msg::ParameterType;

namespace rc26_omni_controller {

void OmniPidPursuitController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                                         std::shared_ptr<tf2_ros::Buffer> tf,
                                         std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    auto node = parent.lock();
    node_ = parent;
    if (!node) {
        throw nav2_core::PlannerException("Unable to lock node!");
    }

    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    tf_ = tf;
    plugin_name_ = name;
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    double transform_tolerance = 1.0;
    double control_frequency = 20.0;
    max_robot_pose_search_dist_ = getCostmapMaxExtent();

    declare_parameter_if_not_declared(node, plugin_name_ + ".translation_kp", rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".translation_ki", rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(node, plugin_name_ + ".translation_kd", rclcpp::ParameterValue(0.3));
    declare_parameter_if_not_declared(node, plugin_name_ + ".enable_rotation", rclcpp::ParameterValue(true));
    declare_parameter_if_not_declared(node, plugin_name_ + ".rotation_kp", rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".rotation_ki", rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(node, plugin_name_ + ".rotation_kd", rclcpp::ParameterValue(0.3));
    declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(node, plugin_name_ + ".min_max_sum_error", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.3));
    declare_parameter_if_not_declared(node, plugin_name_ + ".use_velocity_scaled_lookahead_dist",
                                      rclcpp::ParameterValue(true));
    declare_parameter_if_not_declared(node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.2));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".use_interpolation", rclcpp::ParameterValue(true));
    declare_parameter_if_not_declared(node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));
    declare_parameter_if_not_declared(node, plugin_name_ + ".use_rotate_to_heading_threshold",
                                      rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(node, plugin_name_ + ".min_approach_linear_velocity",
                                      rclcpp::ParameterValue(0.05));
    declare_parameter_if_not_declared(node, plugin_name_ + ".approach_velocity_scaling_dist",
                                      rclcpp::ParameterValue(0.6));
    declare_parameter_if_not_declared(node, plugin_name_ + ".v_linear_min", rclcpp::ParameterValue(-3.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".v_linear_max", rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".v_angular_min", rclcpp::ParameterValue(-3.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".v_angular_max", rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_robot_pose_search_dist",
                                      rclcpp::ParameterValue(getCostmapMaxExtent()));
    declare_parameter_if_not_declared(node, plugin_name_ + ".a_linear_max", rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".a_angular_max", rclcpp::ParameterValue(6.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".brake_margin", rclcpp::ParameterValue(0.15));
    declare_parameter_if_not_declared(node, plugin_name_ + ".brake_accel", rclcpp::ParameterValue(0.8));
    declare_parameter_if_not_declared(node, plugin_name_ + ".lateral_error_gain", rclcpp::ParameterValue(1.5));
    declare_parameter_if_not_declared(node, plugin_name_ + ".lateral_error_max", rclcpp::ParameterValue(0.3));
    declare_parameter_if_not_declared(node, plugin_name_ + ".enable_curvature_ff", rclcpp::ParameterValue(false));
    declare_parameter_if_not_declared(node, plugin_name_ + ".a_lim_x", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".a_lim_y", rclcpp::ParameterValue(0.6));
    declare_parameter_if_not_declared(node, plugin_name_ + ".a_lateral_max", rclcpp::ParameterValue(3.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".curvature_forward_dist", rclcpp::ParameterValue(0.7));
    declare_parameter_if_not_declared(node, plugin_name_ + ".curvature_backward_dist", rclcpp::ParameterValue(0.3));
    declare_parameter_if_not_declared(node, plugin_name_ + ".max_velocity_scaling_factor_rate",
                                      rclcpp::ParameterValue(0.9));
    declare_parameter_if_not_declared(node, plugin_name_ + ".kv_ff", rclcpp::ParameterValue(0.4));
    declare_parameter_if_not_declared(node, plugin_name_ + ".goal_dist_scale", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".wheel_base", rclcpp::ParameterValue(0.62326));
    declare_parameter_if_not_declared(node, plugin_name_ + ".track_width", rclcpp::ParameterValue(0.7));
    declare_parameter_if_not_declared(node, plugin_name_ + ".wheel_speed_max", rclcpp::ParameterValue(-1.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".derivative_filter_tau", rclcpp::ParameterValue(0.02));
    declare_parameter_if_not_declared(node, plugin_name_ + ".publish_debug", rclcpp::ParameterValue(false));
    declare_parameter_if_not_declared(node, plugin_name_ + ".loc_uncertainty_enable", rclcpp::ParameterValue(true));
    declare_parameter_if_not_declared(node, plugin_name_ + ".loc_timeout_sec", rclcpp::ParameterValue(0.2));
    declare_parameter_if_not_declared(node, plugin_name_ + ".loc_k_v", rclcpp::ParameterValue(50.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".loc_k_w", rclcpp::ParameterValue(20.0));
    declare_parameter_if_not_declared(node, plugin_name_ + ".loc_v_scale_min", rclcpp::ParameterValue(0.2));
    declare_parameter_if_not_declared(node, plugin_name_ + ".loc_w_scale_min", rclcpp::ParameterValue(0.3));

    node->get_parameter(plugin_name_ + ".translation_kp", translation_kp_);
    node->get_parameter(plugin_name_ + ".translation_ki", translation_ki_);
    node->get_parameter(plugin_name_ + ".translation_kd", translation_kd_);
    node->get_parameter(plugin_name_ + ".enable_rotation", enable_rotation_);
    node->get_parameter(plugin_name_ + ".rotation_kp", rotation_kp_);
    node->get_parameter(plugin_name_ + ".rotation_ki", rotation_ki_);
    node->get_parameter(plugin_name_ + ".rotation_kd", rotation_kd_);
    node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
    node->get_parameter(plugin_name_ + ".min_max_sum_error", min_max_sum_error_);
    node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
    node->get_parameter(plugin_name_ + ".use_velocity_scaled_lookahead_dist", use_velocity_scaled_lookahead_dist_);
    node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
    node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
    node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
    node->get_parameter(plugin_name_ + ".use_interpolation", use_interpolation_);
    node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
    node->get_parameter(plugin_name_ + ".use_rotate_to_heading_threshold", use_rotate_to_heading_threshold_);
    node->get_parameter(plugin_name_ + ".min_approach_linear_velocity", min_approach_linear_velocity_);
    node->get_parameter(plugin_name_ + ".approach_velocity_scaling_dist", approach_velocity_scaling_dist_);
    if (approach_velocity_scaling_dist_ > costmap_->getSizeInMetersX() / 2.0) {
        RCLCPP_WARN(logger_, "approach_velocity_scaling_dist is larger than forward costmap extent, "
                             "leading to permanent slowdown");
    }
    node->get_parameter(plugin_name_ + ".v_linear_max", v_linear_max_);
    node->get_parameter(plugin_name_ + ".v_linear_min", v_linear_min_);
    node->get_parameter(plugin_name_ + ".v_angular_max", v_angular_max_);
    node->get_parameter(plugin_name_ + ".v_angular_min", v_angular_min_);
    node->get_parameter(plugin_name_ + ".max_robot_pose_search_dist", max_robot_pose_search_dist_);
    node->get_parameter(plugin_name_ + ".a_linear_max", a_linear_max_);
    node->get_parameter(plugin_name_ + ".a_angular_max", a_angular_max_);
    node->get_parameter(plugin_name_ + ".brake_margin", brake_margin_);
    node->get_parameter(plugin_name_ + ".brake_accel", brake_accel_);
    node->get_parameter(plugin_name_ + ".lateral_error_gain", lateral_error_gain_);
    node->get_parameter(plugin_name_ + ".lateral_error_max", lateral_error_max_);
    node->get_parameter(plugin_name_ + ".enable_curvature_ff", enable_curvature_ff_);
    node->get_parameter(plugin_name_ + ".a_lim_x", a_lim_x_);
    node->get_parameter(plugin_name_ + ".a_lim_y", a_lim_y_);
    node->get_parameter(plugin_name_ + ".a_lateral_max", a_lateral_max_);
    node->get_parameter(plugin_name_ + ".curvature_forward_dist", curvature_forward_dist_);
    node->get_parameter(plugin_name_ + ".curvature_backward_dist", curvature_backward_dist_);
    node->get_parameter(plugin_name_ + ".max_velocity_scaling_factor_rate", max_velocity_scaling_factor_rate_);
    node->get_parameter(plugin_name_ + ".kv_ff", kv_ff_);
    node->get_parameter(plugin_name_ + ".goal_dist_scale", goal_dist_scale_);
    node->get_parameter(plugin_name_ + ".wheel_base", wheel_base_);
    node->get_parameter(plugin_name_ + ".track_width", track_width_);
    node->get_parameter(plugin_name_ + ".wheel_speed_max", wheel_speed_max_);
    node->get_parameter(plugin_name_ + ".derivative_filter_tau", derivative_filter_tau_);
    node->get_parameter(plugin_name_ + ".publish_debug", publish_debug_);
    node->get_parameter(plugin_name_ + ".loc_uncertainty_enable", loc_uncertainty_enable_);
    node->get_parameter(plugin_name_ + ".loc_timeout_sec", loc_timeout_sec_);
    node->get_parameter(plugin_name_ + ".loc_k_v", loc_k_v_);
    node->get_parameter(plugin_name_ + ".loc_k_w", loc_k_w_);
    node->get_parameter(plugin_name_ + ".loc_v_scale_min", loc_v_scale_min_);
    node->get_parameter(plugin_name_ + ".loc_w_scale_min", loc_w_scale_min_);

    node->get_parameter("controller_frequency", control_frequency);

    transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
    control_duration_ = 1.0 / std::max(control_frequency, 1e-3);
    configured_v_linear_max_ = v_linear_max_;

    local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
    carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
    curvature_points_pub_ =
        node_.lock()->create_publisher<visualization_msgs::msg::MarkerArray>(  // 初始化 MarkerArray Publisher
            "curvature_points_marker_array", rclcpp::QoS(10));
    real_dt_pub_ = node->create_publisher<std_msgs::msg::Float64>("real_dt", rclcpp::QoS(10));
    compute_time_ms_pub_ = node->create_publisher<std_msgs::msg::Float64>("compute_time_ms", rclcpp::QoS(10));
    pose_age_ms_pub_ = node->create_publisher<std_msgs::msg::Float64>("pose_age_ms", rclcpp::QoS(10));
    collision_check_ms_pub_ = node->create_publisher<std_msgs::msg::Float64>("collision_check_ms", rclcpp::QoS(10));
    collision_d_min_pub_ = node->create_publisher<std_msgs::msg::Float64>("collision_d_min", rclcpp::QoS(10));
    v_safe_pub_ = node->create_publisher<std_msgs::msg::Float64>("v_safe", rclcpp::QoS(10));
    collision_check_outside_map_count_pub_ =
        node->create_publisher<std_msgs::msg::UInt32>("collision_check_outside_map_count", rclcpp::QoS(10));

    move_pid_ = std::make_shared<PID>(control_duration_, v_linear_max_, v_linear_min_, translation_kp_, translation_kd_,
                                      translation_ki_);
    heading_pid_ = std::make_shared<PID>(control_duration_, v_angular_max_, v_angular_min_, rotation_kp_, rotation_kd_,
                                         rotation_ki_);

    // [M2 修复] 使用 min_max_sum_error_ 参数设置积分限幅
    move_pid_->setIntegralLimits(min_max_sum_error_);
    heading_pid_->setIntegralLimits(min_max_sum_error_);
    move_pid_->setDerivativeFilterTau(derivative_filter_tau_);
    heading_pid_->setDerivativeFilterTau(derivative_filter_tau_);

    last_velocity_scaling_factor_ = v_linear_max_;
    last_time_ = clock_->now();
    collision_check_outside_map_count_ = 0;
    plan_cumulative_distances_.clear();
    plan_prune_idx_ = 0;
    loc_timeout_sec_ = std::max(0.01, loc_timeout_sec_);
    loc_k_v_ = std::max(0.0, loc_k_v_);
    loc_k_w_ = std::max(0.0, loc_k_w_);
    loc_v_scale_min_ = std::clamp(loc_v_scale_min_, 0.0, 1.0);
    loc_w_scale_min_ = std::clamp(loc_w_scale_min_, 0.0, 1.0);
    resetMotionState();
    last_velocity_scaling_factor_ = v_linear_max_;
    refreshPoseCovSubscription(node);
}

void OmniPidPursuitController::cleanup() {
    std::lock_guard<std::recursive_mutex> lock_reinit(mutex_);
    RCLCPP_INFO(logger_,
                "Cleaning up controller: %s of type"
                " omni_pursuit_controller::OmniPidPursuitController",
                plugin_name_.c_str());
    dyn_params_handler_.reset();
    local_path_pub_.reset();
    carrot_pub_.reset();
    curvature_points_pub_.reset();
    real_dt_pub_.reset();
    compute_time_ms_pub_.reset();
    pose_age_ms_pub_.reset();
    collision_check_ms_pub_.reset();
    collision_d_min_pub_.reset();
    v_safe_pub_.reset();
    collision_check_outside_map_count_pub_.reset();
    pose_cov_sub_.reset();
    move_pid_.reset();
    heading_pid_.reset();
    global_plan_.poses.clear();
    plan_cumulative_distances_.clear();
    plan_prune_idx_ = 0;
    costmap_snapshot_cache_.data.clear();
    resetMotionState();
    tf_.reset();
    costmap_ros_.reset();
    costmap_ = nullptr;
}

void OmniPidPursuitController::activate() {
    std::lock_guard<std::recursive_mutex> lock_reinit(mutex_);
    RCLCPP_INFO(logger_,
                "Activating controller: %s of type "
                "omni_pursuit_controller::OmniPidPursuitController",
                plugin_name_.c_str());
    if (local_path_pub_) {
        local_path_pub_->on_activate();
    }
    if (carrot_pub_) {
        carrot_pub_->on_activate();
    }
    if (curvature_points_pub_) {
        curvature_points_pub_->on_activate();
    }
    if (real_dt_pub_) {
        real_dt_pub_->on_activate();
    }
    if (compute_time_ms_pub_) {
        compute_time_ms_pub_->on_activate();
    }
    if (pose_age_ms_pub_) {
        pose_age_ms_pub_->on_activate();
    }
    if (collision_check_ms_pub_) {
        collision_check_ms_pub_->on_activate();
    }
    if (collision_d_min_pub_) {
        collision_d_min_pub_->on_activate();
    }
    if (v_safe_pub_) {
        v_safe_pub_->on_activate();
    }
    if (collision_check_outside_map_count_pub_) {
        collision_check_outside_map_count_pub_->on_activate();
    }
    // Add callback for dynamic parameters
    auto node = node_.lock();
    if (!node) {
        throw nav2_core::PlannerException("Unable to activate controller: node expired.");
    }
    dyn_params_handler_ = node->add_on_set_parameters_callback(
        std::bind(&OmniPidPursuitController::dynamicParametersCallback, this, std::placeholders::_1));
}

void OmniPidPursuitController::deactivate() {
    std::lock_guard<std::recursive_mutex> lock_reinit(mutex_);
    RCLCPP_INFO(logger_,
                "Deactivating controller: %s of type "
                "omni_pursuit_controller::OmniPidPursuitController",
                plugin_name_.c_str());
    if (local_path_pub_) {
        local_path_pub_->on_deactivate();
    }
    if (carrot_pub_) {
        carrot_pub_->on_deactivate();
    }
    if (curvature_points_pub_) {
        curvature_points_pub_->on_deactivate();
    }
    if (real_dt_pub_) {
        real_dt_pub_->on_deactivate();
    }
    if (compute_time_ms_pub_) {
        compute_time_ms_pub_->on_deactivate();
    }
    if (pose_age_ms_pub_) {
        pose_age_ms_pub_->on_deactivate();
    }
    if (collision_check_ms_pub_) {
        collision_check_ms_pub_->on_deactivate();
    }
    if (collision_d_min_pub_) {
        collision_d_min_pub_->on_deactivate();
    }
    if (v_safe_pub_) {
        v_safe_pub_->on_deactivate();
    }
    if (collision_check_outside_map_count_pub_) {
        collision_check_outside_map_count_pub_->on_deactivate();
    }
    dyn_params_handler_.reset();
}

const CostmapSnapshot& OmniPidPursuitController::captureCostmapSnapshot() {
    auto* cm = costmap_ros_ ? costmap_ros_->getCostmap() : nullptr;
    if (cm == nullptr) {
        throw nav2_core::PlannerException("Costmap is not available.");
    }

    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*cm->getMutex());
    costmap_snapshot_cache_.width = cm->getSizeInCellsX();
    costmap_snapshot_cache_.height = cm->getSizeInCellsY();
    costmap_snapshot_cache_.origin_x = cm->getOriginX();
    costmap_snapshot_cache_.origin_y = cm->getOriginY();
    costmap_snapshot_cache_.resolution = cm->getResolution();

    const size_t cell_count = static_cast<size_t>(costmap_snapshot_cache_.width) *
                              static_cast<size_t>(costmap_snapshot_cache_.height);
    costmap_snapshot_cache_.data.resize(cell_count);
    if (cell_count == 0U) {
        return costmap_snapshot_cache_;
    }

    const uint8_t* raw = cm->getCharMap();
    if (raw == nullptr) {
        costmap_snapshot_cache_.data.clear();
        return costmap_snapshot_cache_;
    }

    std::copy_n(raw, cell_count, costmap_snapshot_cache_.data.begin());
    return costmap_snapshot_cache_;
}

void OmniPidPursuitController::refreshPoseCovSubscription(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
    if (!node) {
        return;
    }

    {
        std::lock_guard<std::mutex> lk(cov_mutex_);
        last_cov_stamp_ = node->now() - rclcpp::Duration::from_seconds(loc_timeout_sec_ + 1.0);
        sigma_xy_ = 2.0;
        sigma_yaw_ = 1.0;
    }

    if (!loc_uncertainty_enable_) {
        pose_cov_sub_.reset();
        return;
    }

    if (pose_cov_sub_) {
        return;
    }

    pose_cov_sub_ = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/localization/pose_with_cov", rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            std::lock_guard<std::mutex> lk(cov_mutex_);
            const auto& c = msg->pose.covariance;
            sigma_xy_ = std::sqrt(std::max(0.0, c[0] + c[7]));
            sigma_yaw_ = std::sqrt(std::max(0.0, c[35]));
            last_cov_stamp_ = rclcpp::Time(msg->header.stamp);
        });
}

void OmniPidPursuitController::resetMotionState() noexcept {
    last_lin_vel_ = 0.0;
    last_vx_ = 0.0;
    last_vy_ = 0.0;
    last_ang_vel_ = 0.0;
    last_velocity_scaling_factor_ = 0.0;
    if (move_pid_) {
        move_pid_->setSumError(0.0);
    }
    if (heading_pid_) {
        heading_pid_->setSumError(0.0);
    }
}

geometry_msgs::msg::TwistStamped
OmniPidPursuitController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose,
                                                  const geometry_msgs::msg::Twist& velocity,
                                                  nav2_core::GoalChecker* /*goal_checker*/) {
    std::lock_guard<std::recursive_mutex> lock_reinit(mutex_);
    const auto compute_start = std::chrono::steady_clock::now();
    auto publish_compute_time = [&]() {
        if (!publish_debug_) {
            return;
        }
        std_msgs::msg::Float64 compute_time_msg;
        compute_time_msg.data =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - compute_start).count();
        compute_time_ms_pub_->publish(compute_time_msg);
    };

    try {
        auto now = clock_->now();
        double real_dt = std::clamp((now - last_time_).seconds(), 0.001, 0.1);
        last_time_ = now;
        move_pid_->setDt(real_dt);
        heading_pid_->setDt(real_dt);

        double pose_age_ms = 0.0;
        if (pose.header.stamp.sec != 0 || pose.header.stamp.nanosec != 0) {
            const rclcpp::Time pose_stamp(pose.header.stamp, now.get_clock_type());
            pose_age_ms = std::max(0.0, (now - pose_stamp).seconds() * 1000.0);
        }

        const auto& snap = captureCostmapSnapshot();
        if (snap.width == 0 || snap.height == 0 || snap.data.empty()) {
            throw nav2_core::PlannerException("Costmap snapshot is empty.");
        }

        auto transformed_plan = transformGlobalPlan(pose);
        double lookahead_dist = getLookAheadDistance(velocity);
        auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
        carrot_pub_->publish(createCarrotMsg(carrot_pose));

        double angle_to_goal = tf2::getYaw(carrot_pose.pose.orientation);
        double goal_dist = hypot(transformed_plan.poses.back().pose.position.x,
                                 transformed_plan.poses.back().pose.position.y);

        bool force_rotate_only = false;
        if (use_rotate_to_heading_) {
            angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
            double rotate_to_heading_dist = std::min(approach_velocity_scaling_dist_, lookahead_dist_);
            if (goal_dist < rotate_to_heading_dist && fabs(angle_to_goal) > use_rotate_to_heading_threshold_) {
                force_rotate_only = true;
            }
        }

        const double w_pid = enable_rotation_ ? heading_pid_->calculate(angle_to_goal, 0.0) : 0.0;
        double linear_limit = v_linear_max_;
        double path_kappa = 0.0;
        const double v_kappa =
            applyCurvatureLimitation(transformed_plan, carrot_pose, linear_limit, real_dt, path_kappa);
        applyApproachVelocityScaling(transformed_plan, linear_limit);

        if (force_rotate_only) {
            linear_limit = 0.0;
        }
        last_velocity_scaling_factor_ = linear_limit;

        const double goal_scale = goal_dist_scale_ > 1e-6 ? std::min(1.0, goal_dist / goal_dist_scale_) : 1.0;
        double v_ref = std::min(v_linear_max_, v_kappa) * goal_scale;
        v_ref = std::max(0.0, std::min(v_ref, linear_limit));

        const double v_meas = std::hypot(velocity.linear.x, velocity.linear.y);
        const double v_fb = move_pid_->calculate(v_ref, v_meas);
        auto lin_vel = std::clamp(v_fb + kv_ff_ * v_ref, v_linear_min_, v_linear_max_);
        lin_vel = std::clamp(lin_vel, v_linear_min_, linear_limit);

        double angular_vel = w_pid;
        if (enable_rotation_ && enable_curvature_ff_) {
            angular_vel += std::clamp(lin_vel * path_kappa, -v_angular_max_, v_angular_max_);
        }

        if (force_rotate_only) {
            lin_vel = 0.0;
        }

        geometry_msgs::msg::TransformStamped tf_stamped;
        try {
            tf_stamped = tf_->lookupTransform(costmap_ros_->getGlobalFrameID(), costmap_ros_->getBaseFrameID(),
                                              tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            throw nav2_core::PlannerException(std::string("TF lookup failed: ") + ex.what());
        }
        tf2::Transform transform_map_from_base;
        tf2::fromMsg(tf_stamped.transform, transform_map_from_base);

        nav_msgs::msg::Path costmap_frame_local_plan;
        costmap_frame_local_plan.header.frame_id = costmap_ros_->getGlobalFrameID();
        costmap_frame_local_plan.header.stamp = now;
        const double costmap_resolution = std::max(snap.resolution, 1e-6);
        const int sample_points = std::clamp(static_cast<int>(lookahead_dist / costmap_resolution), 10, 50);
        int lookahead_end_idx = static_cast<int>(transformed_plan.poses.size()) - 1;
        auto lookahead_end_it = std::min_element(
            transformed_plan.poses.begin(), transformed_plan.poses.end(),
            [&carrot_pose](const auto& lhs, const auto& rhs) {
                return std::hypot(lhs.pose.position.x - carrot_pose.pose.position.x,
                                  lhs.pose.position.y - carrot_pose.pose.position.y) <
                       std::hypot(rhs.pose.position.x - carrot_pose.pose.position.x,
                                  rhs.pose.position.y - carrot_pose.pose.position.y);
            });
        if (lookahead_end_it != transformed_plan.poses.end()) {
            lookahead_end_idx = static_cast<int>(std::distance(transformed_plan.poses.begin(), lookahead_end_it));
        }

        costmap_frame_local_plan.poses.reserve(sample_points + 1);
        for (int i = 0; i < sample_points; ++i) {
            const double ratio = sample_points == 1 ? 1.0 : static_cast<double>(i) / (sample_points - 1);
            const int index =
                std::clamp(static_cast<int>(std::round(ratio * lookahead_end_idx)), 0, lookahead_end_idx);
            const auto& src = transformed_plan.poses[index].pose.position;
            const tf2::Vector3 point_map = transform_map_from_base * tf2::Vector3(src.x, src.y, 0.0);
            geometry_msgs::msg::PoseStamped sample_pose;
            sample_pose.header = costmap_frame_local_plan.header;
            sample_pose.pose.position.x = point_map.x();
            sample_pose.pose.position.y = point_map.y();
            costmap_frame_local_plan.poses.push_back(sample_pose);
        }
        {
            const auto& carrot = carrot_pose.pose.position;
            const tf2::Vector3 point_map = transform_map_from_base * tf2::Vector3(carrot.x, carrot.y, 0.0);
            geometry_msgs::msg::PoseStamped sample_pose;
            sample_pose.header = costmap_frame_local_plan.header;
            sample_pose.pose.position.x = point_map.x();
            sample_pose.pose.position.y = point_map.y();
            costmap_frame_local_plan.poses.push_back(sample_pose);
        }

        const tf2::Vector3 robot_map = transform_map_from_base * tf2::Vector3(0.0, 0.0, 0.0);
        const auto collision_check_start = std::chrono::steady_clock::now();
        const double d_min = getMinCollisionDist(costmap_frame_local_plan, snap, robot_map.x(), robot_map.y());
        const double collision_check_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - collision_check_start)
                .count();
        const double brake_margin = std::max(0.0, brake_margin_);
        const double brake_accel = std::max(brake_accel_, 1e-6);
        double v_safe = v_linear_max_;
        if (d_min < std::numeric_limits<double>::max()) {
            const double d_eff = std::max(d_min - brake_margin, 0.0);
            v_safe = std::sqrt(2.0 * brake_accel * d_eff);
            lin_vel = std::min(lin_vel, v_safe);
        }

        if (publish_debug_) {
            std_msgs::msg::Float64 real_dt_msg;
            real_dt_msg.data = real_dt;
            real_dt_pub_->publish(real_dt_msg);

            std_msgs::msg::Float64 pose_age_msg;
            pose_age_msg.data = pose_age_ms;
            pose_age_ms_pub_->publish(pose_age_msg);

            std_msgs::msg::UInt32 collision_count_msg;
            collision_count_msg.data = collision_check_outside_map_count_;
            collision_check_outside_map_count_pub_->publish(collision_count_msg);

            std_msgs::msg::Float64 collision_check_ms_msg;
            collision_check_ms_msg.data = collision_check_ms;
            collision_check_ms_pub_->publish(collision_check_ms_msg);

            std_msgs::msg::Float64 collision_d_min_msg;
            collision_d_min_msg.data = d_min;
            collision_d_min_pub_->publish(collision_d_min_msg);

            std_msgs::msg::Float64 v_safe_msg;
            v_safe_msg.data = v_safe;
            v_safe_pub_->publish(v_safe_msg);
        }

        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header = pose.header;
        if (d_min <= brake_margin) {
            throw nav2_core::PlannerException("Obstacle within safety margin.");
        }

        const int plan_size = static_cast<int>(transformed_plan.poses.size());
        int path_idx = lookahead_end_idx;
        if (plan_size >= 3) {
            path_idx = std::clamp(lookahead_end_idx, 1, plan_size - 2);
        } else if (plan_size > 1) {
            path_idx = std::clamp(lookahead_end_idx, 0, plan_size - 1);
        } else {
            path_idx = 0;
        }

        const int prev_idx = std::max(path_idx - 1, 0);
        const int next_idx = std::min(path_idx + 1, std::max(plan_size - 1, 0));
        const auto& p_prev = transformed_plan.poses[prev_idx].pose.position;
        const auto& p_next = transformed_plan.poses[next_idx].pose.position;
        const double t_dx = p_next.x - p_prev.x;
        const double t_dy = p_next.y - p_prev.y;
        const double t_len = std::hypot(t_dx, t_dy) + 1e-9;
        const double tx = t_dx / t_len;
        const double ty = t_dy / t_len;

        const double lateral_error_max = std::max(0.0, lateral_error_max_);
        const double e_perp = std::clamp(-(carrot_pose.pose.position.x * ty) + (carrot_pose.pose.position.y * tx),
                                         -lateral_error_max, lateral_error_max);
        double vx = lin_vel * tx - lateral_error_gain_ * e_perp * ty;
        double vy = lin_vel * ty + lateral_error_gain_ * e_perp * tx;
        double wz = angular_vel;

        if (loc_uncertainty_enable_) {
            double sx = 2.0;
            double sy = 1.0;
            {
                std::lock_guard<std::mutex> lk(cov_mutex_);
                const double age = (clock_->now() - last_cov_stamp_).seconds();
                sx = (age > loc_timeout_sec_) ? 2.0 : sigma_xy_;
                sy = (age > loc_timeout_sec_) ? 1.0 : sigma_yaw_;
            }
            const double scale_v = std::clamp(std::exp(-loc_k_v_ * sx), loc_v_scale_min_, 1.0);
            const double scale_w = std::clamp(std::exp(-loc_k_w_ * sy), loc_w_scale_min_, 1.0);
            vx *= scale_v;
            vy *= scale_v;
            wz *= scale_w;
        }

        const double dvx_max = std::max(0.0, a_lim_x_) * real_dt;
        const double dvy_max = std::max(0.0, a_lim_y_) * real_dt;
        vx = last_vx_ + std::clamp(vx - last_vx_, -dvx_max, dvx_max);
        vy = last_vy_ + std::clamp(vy - last_vy_, -dvy_max, dvy_max);
        const double max_dw = std::max(0.0, a_angular_max_) * real_dt;
        wz = std::clamp(wz, last_ang_vel_ - max_dw, last_ang_vel_ + max_dw);
        if (wheel_speed_max_ > 0.0) {
            const double lw = (wheel_base_ + track_width_) / 2.0;
            const double speeds[4] = {
                vx - vy - wz * lw,  // FL
                vx + vy + wz * lw,  // FR
                vx + vy - wz * lw,  // RL
                vx - vy + wz * lw   // RR
            };
            double peak = 0.0;
            for (double wheel_speed : speeds) {
                peak = std::max(peak, std::abs(wheel_speed));
            }
            if (peak > wheel_speed_max_) {
                const double scale = wheel_speed_max_ / peak;
                vx *= scale;
                vy *= scale;
                wz *= scale;
            }
        }

        last_vx_ = vx;
        last_vy_ = vy;
        cmd_vel.twist.linear.x = vx;
        cmd_vel.twist.linear.y = vy;
        cmd_vel.twist.angular.z = wz;
        last_lin_vel_ = lin_vel;
        last_ang_vel_ = wz;

        publish_compute_time();
        return cmd_vel;
    } catch (const nav2_core::PlannerException&) {
        resetMotionState();
        publish_compute_time();
        throw;
    } catch (const std::exception& ex) {
        resetMotionState();
        publish_compute_time();
        throw nav2_core::PlannerException(std::string("computeVelocityCommands failed: ") + ex.what());
    }
}

void OmniPidPursuitController::setPlan(const nav_msgs::msg::Path& path) {
    std::lock_guard<std::recursive_mutex> lock_reinit(mutex_);
    global_plan_ = path;
    plan_cumulative_distances_ = calculateCumulativeDistances(global_plan_);
    plan_prune_idx_ = 0;
    if (move_pid_) {
        move_pid_->setSumError(0);
    }
    if (heading_pid_) {
        heading_pid_->setSumError(0);
    }
    last_velocity_scaling_factor_ = v_linear_max_;
    last_lin_vel_ = 0.0;
    last_vx_ = 0.0;
    last_vy_ = 0.0;
    last_ang_vel_ = 0.0;
}

void OmniPidPursuitController::setSpeedLimit(const double& speed_limit, const bool& percentage) {
    std::lock_guard<std::recursive_mutex> lock_reinit(mutex_);
    if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT) {
        v_linear_max_ = configured_v_linear_max_;
    } else {
        v_linear_max_ = percentage ? configured_v_linear_max_ * speed_limit / 100.0 : speed_limit;
    }
    v_linear_max_ = std::max(v_linear_min_, v_linear_max_);
    last_velocity_scaling_factor_ = std::clamp(last_velocity_scaling_factor_, 0.0, std::max(0.0, v_linear_max_));
    if (move_pid_) {
        move_pid_->setOutputLimits(v_linear_min_, v_linear_max_);
    }
}

nav_msgs::msg::Path OmniPidPursuitController::transformGlobalPlan(const geometry_msgs::msg::PoseStamped& pose) {
    if (global_plan_.poses.empty()) {
        throw nav2_core::PlannerException("Received plan with zero length");
    }

    // let's get the pose of the robot in the frame of the plan
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
        throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
    }

    // We'll discard points on the plan that are outside the local costmap
    double max_costmap_extent = getCostmapMaxExtent();

    auto closest_pose_upper_bound = nav2_util::geometry_utils::first_after_integrated_distance(
        global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

    // First find the closest pose on the path to the robot
    // bounded by when the path turns around (if it does) so we don't get a pose from a later
    // portion of the path
    auto transformation_begin = nav2_util::geometry_utils::min_by(
        global_plan_.poses.begin(), closest_pose_upper_bound,
        [&robot_pose](const geometry_msgs::msg::PoseStamped& ps) { return euclidean_distance(robot_pose, ps); });

    // Find points up to max_transform_dist so we only transform them.
    auto transformation_end = std::find_if(transformation_begin, global_plan_.poses.end(), [&](const auto& pose) {
        return euclidean_distance(pose, robot_pose) > max_costmap_extent;
    });

    // Lambda to transform a PoseStamped from global frame to local
    auto transform_global_pose_to_local = [&](const auto& global_plan_pose) {
        geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
        stamped_pose.header.frame_id = global_plan_.header.frame_id;
        stamped_pose.header.stamp = robot_pose.header.stamp;
        stamped_pose.pose = global_plan_pose.pose;
        if (!transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose)) {
            throw nav2_core::PlannerException("Unable to transform global plan pose into robot frame");
        }
        transformed_pose.pose.position.z = 0.0;
        return transformed_pose;
    };

    const size_t transformation_begin_offset = static_cast<size_t>(
        std::distance(global_plan_.poses.begin(), transformation_begin));

    // Transform the near part of the global plan into the robot's frame of reference.
    nav_msgs::msg::Path transformed_plan;
    std::transform(transformation_begin, transformation_end, std::back_inserter(transformed_plan.poses),
                   transform_global_pose_to_local);
    transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
    transformed_plan.header.stamp = robot_pose.header.stamp;

    // Remove the portion of the global plan that we've already passed so we don't
    // process it on the next iteration (this is called path pruning)
    global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
    if (!plan_cumulative_distances_.empty()) {
        plan_prune_idx_ = std::min(plan_prune_idx_ + transformation_begin_offset, plan_cumulative_distances_.size());
    } else {
        plan_prune_idx_ = 0;
    }
    local_path_pub_->publish(transformed_plan);

    if (transformed_plan.poses.empty()) {
        throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
    }

    return transformed_plan;
}

std::unique_ptr<geometry_msgs::msg::PointStamped>
OmniPidPursuitController::createCarrotMsg(const geometry_msgs::msg::PoseStamped& carrot_pose) {
    auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
    carrot_msg->header = carrot_pose.header;
    carrot_msg->point.x = carrot_pose.pose.position.x;
    carrot_msg->point.y = carrot_pose.pose.position.y;
    carrot_msg->point.z = 0.01;  // publish right over map to stand out
    return carrot_msg;
}

geometry_msgs::msg::PoseStamped
OmniPidPursuitController::getLookAheadPoint(const double& lookahead_dist, const nav_msgs::msg::Path& transformed_plan) {
    // Find the first pose which is at a distance greater than the lookahead distance
    auto goal_pose_it = std::find_if(transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto& ps) {
        return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
    });

    // If the no pose is not far enough, take the last pose
    if (goal_pose_it == transformed_plan.poses.end()) {
        goal_pose_it = std::prev(transformed_plan.poses.end());
    } else if (use_interpolation_ && goal_pose_it != transformed_plan.poses.begin()) {
        // Find the point on the line segment between the two poses
        // that is exactly the lookahead distance away from the robot pose (the origin)
        // This can be found with a closed form for the intersection of a segment and a circle
        // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
        // and goal_pose is guaranteed to be outside the circle.
        auto prev_pose_it = std::prev(goal_pose_it);
        auto point =
            circleSegmentIntersection(prev_pose_it->pose.position, goal_pose_it->pose.position, lookahead_dist);
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = prev_pose_it->header.frame_id;
        pose.header.stamp = goal_pose_it->header.stamp;
        pose.pose.position = point;
        pose.pose.orientation = goal_pose_it->pose.orientation;
        return pose;
    }

    return *goal_pose_it;
}

geometry_msgs::msg::Point OmniPidPursuitController::circleSegmentIntersection(const geometry_msgs::msg::Point& p1,
                                                                              const geometry_msgs::msg::Point& p2,
                                                                              double r) {
    // Formula for intersection of a line with a circle centered at the origin,
    // modified to always return the point that is on the segment between the two points.
    // https://mathworld.wolfram.com/Circle-LineIntersection.html
    // This works because the poses are transformed into the robot frame.
    // This can be derived from solving the system of equations of a line and a circle
    // which results in something that is just a reformulation of the quadratic formula.
    // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
    // https://www.desmos.com/calculator/td5cwbuocd
    double x1 = p1.x;
    double x2 = p2.x;
    double y1 = p1.y;
    double y2 = p2.y;

    double dx = x2 - x1;
    double dy = y2 - y1;
    double dr2 = dx * dx + dy * dy;
    double d = x1 * y2 - x2 * y1;

    // 安全检查: 避免 dr2 为零或判别式为负
    if (dr2 < 1e-9) {
        // 两点重合，返回 p2
        return p2;
    }

    double discriminant = r * r * dr2 - d * d;
    if (discriminant < 0) {
        // 无交点，返回距离圆心较远的点 (p2)
        return p2;
    }

    // Augmentation to only return point within segment
    double d1 = x1 * x1 + y1 * y1;
    double d2 = x2 * x2 + y2 * y2;
    double dd = d2 - d1;

    geometry_msgs::msg::Point p;
    double sqrt_term = std::sqrt(discriminant);
    p.x = (d * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
    p.y = (-d * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
    return p;
}

double OmniPidPursuitController::getCostmapMaxExtent() const {
    const double max_costmap_dim_meters = std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
    return max_costmap_dim_meters / 2.0;
}
bool OmniPidPursuitController::transformPose(const std::string frame, const geometry_msgs::msg::PoseStamped& in_pose,
                                             geometry_msgs::msg::PoseStamped& out_pose) const {
    if (in_pose.header.frame_id == frame) {
        out_pose = in_pose;
        return true;
    }

    try {
        tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
        return true;
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
    }
    return false;
}

bool OmniPidPursuitController::isCollisionDetected(const nav_msgs::msg::Path& path) {
    auto costmap = costmap_ros_->getCostmap();
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
    for (const auto& pose_stamped : path.poses) {
        const auto& pose = pose_stamped.pose;
        unsigned int mx, my;
        if (costmap->worldToMap(pose.position.x, pose.position.y, mx, my)) {
            if (costmap->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                return true;
            }
        } else {
            ++collision_check_outside_map_count_;
            return true;
        }
    }
    return false;
}

double OmniPidPursuitController::getMinCollisionDist(const nav_msgs::msg::Path& path, const CostmapSnapshot& snap,
                                                     double robot_x, double robot_y) {
    double d_min = std::numeric_limits<double>::max();
    const double resolution = std::max(snap.resolution, 1e-6);
    for (const auto& pose_stamped : path.poses) {
        const int mx = static_cast<int>((pose_stamped.pose.position.x - snap.origin_x) / resolution);
        const int my = static_cast<int>((pose_stamped.pose.position.y - snap.origin_y) / resolution);
        if (mx < 0 || my < 0 || mx >= static_cast<int>(snap.width) || my >= static_cast<int>(snap.height)) {
            ++collision_check_outside_map_count_;
            return 0.0;
        }
        const size_t index = static_cast<size_t>(my) * snap.width + static_cast<size_t>(mx);
        if (snap.data[index] >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            d_min = std::min(d_min,
                             std::hypot(pose_stamped.pose.position.x - robot_x, pose_stamped.pose.position.y - robot_y));
        }
    }
    return d_min;
}

double OmniPidPursuitController::getLookAheadDistance(const geometry_msgs::msg::Twist& speed) {
    // If using velocity-scaled look ahead distances, find and clamp the dist
    // Else, use the static look ahead distance
    double lookahead_dist = lookahead_dist_;

    if (use_velocity_scaled_lookahead_dist_) {
        lookahead_dist = hypot(speed.linear.x, speed.linear.y) * lookahead_time_;
        lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
    }

    return lookahead_dist;
}

double OmniPidPursuitController::approachVelocityScalingFactor(const nav_msgs::msg::Path& transformed_path) const {
    // Waiting to apply the threshold based on integrated distance ensures we don't
    // erroneously apply approach scaling on curvy paths that are contained in a large local costmap.
    double remaining_distance = nav2_util::geometry_utils::calculate_path_length(transformed_path);
    if (remaining_distance < approach_velocity_scaling_dist_ && approach_velocity_scaling_dist_ > 1e-9) {
        auto& last = transformed_path.poses.back();
        // Here we will use a regular euclidean distance from the robot frame (origin)
        // to get smooth scaling, regardless of path density.
        double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
        return distance_to_last_pose / approach_velocity_scaling_dist_;
    } else {
        return 1.0;
    }
}

void OmniPidPursuitController::applyApproachVelocityScaling(const nav_msgs::msg::Path& path, double& linear_vel) const {
    double approach_vel = linear_vel;
    double velocity_scaling = approachVelocityScalingFactor(path);
    double unbounded_vel = approach_vel * velocity_scaling;
    if (unbounded_vel < min_approach_linear_velocity_) {
        approach_vel = min_approach_linear_velocity_;
    } else {
        approach_vel *= velocity_scaling;
    }

    // Use the lowest velocity between approach and other constraints, if all overlapping
    linear_vel = std::min(linear_vel, approach_vel);
}

double OmniPidPursuitController::applyCurvatureLimitation(const nav_msgs::msg::Path& path,
                                                          const geometry_msgs::msg::PoseStamped& lookahead_pose,
                                                          double& linear_vel, double real_dt, double& out_kappa) {
    const double kappa_raw = calculateCurvature(path, lookahead_pose, curvature_forward_dist_, curvature_backward_dist_);
    const double kappa = std::abs(kappa_raw);
    out_kappa = kappa_raw;
    if (path.poses.size() >= 3) {
        const auto lookahead_it = std::min_element(
            path.poses.begin(), path.poses.end(), [&lookahead_pose](const auto& lhs, const auto& rhs) {
                const double lhs_dist = std::hypot(lhs.pose.position.x - lookahead_pose.pose.position.x,
                                                   lhs.pose.position.y - lookahead_pose.pose.position.y);
                const double rhs_dist = std::hypot(rhs.pose.position.x - lookahead_pose.pose.position.x,
                                                   rhs.pose.position.y - lookahead_pose.pose.position.y);
                return lhs_dist < rhs_dist;
            });
        if (lookahead_it != path.poses.end()) {
            size_t idx = static_cast<size_t>(std::distance(path.poses.begin(), lookahead_it));
            if (idx == 0) {
                idx = 1;
            } else if (idx + 1 >= path.poses.size()) {
                idx = path.poses.size() - 2;
            }
            const auto& pa = path.poses[idx - 1].pose.position;
            const auto& pb = path.poses[idx].pose.position;
            const auto& pc = path.poses[idx + 1].pose.position;
            const double cross = (pb.x - pa.x) * (pc.y - pb.y) - (pb.y - pa.y) * (pc.x - pb.x);
            if (std::abs(cross) > 1e-9) {
                out_kappa = std::copysign(kappa, cross);
            }
        }
    }
    const double lateral_limit = std::max(1e-6, a_lateral_max_);
    const double v_kappa = std::sqrt(lateral_limit / (kappa + 1e-6));
    const double target_vel = std::min(linear_vel, v_kappa);
    double scaled_linear_vel =
        last_velocity_scaling_factor_ + std::clamp(target_vel - last_velocity_scaling_factor_,
                                                   -max_velocity_scaling_factor_rate_ * real_dt,
                                                   max_velocity_scaling_factor_rate_ * real_dt);
    scaled_linear_vel = std::max(scaled_linear_vel, 2.0 * min_approach_linear_velocity_);
    linear_vel = std::min(linear_vel, scaled_linear_vel);
    return v_kappa;
}

double OmniPidPursuitController::calculateCurvature(const nav_msgs::msg::Path& path,
                                                    const geometry_msgs::msg::PoseStamped& lookahead_pose,
                                                    double forward_dist, double backward_dist) const {
    if (path.poses.size() < 2) {
        return 0.0;
    }

    std::vector<double> fallback_cumulative_distances;
    const std::vector<double>* cumulative_distances = nullptr;
    size_t cumulative_offset = plan_prune_idx_;
    const bool use_cached_distances =
        !plan_cumulative_distances_.empty() && cumulative_offset < plan_cumulative_distances_.size() &&
        cumulative_offset + path.poses.size() <= plan_cumulative_distances_.size();

    if (use_cached_distances) {
        cumulative_distances = &plan_cumulative_distances_;
    } else {
        fallback_cumulative_distances = calculateCumulativeDistances(path);
        cumulative_distances = &fallback_cumulative_distances;
        cumulative_offset = 0;
    }

    const size_t begin_idx = cumulative_offset;
    const size_t end_idx = cumulative_offset + path.poses.size() - 1;
    const double start_distance = (*cumulative_distances)[begin_idx];
    const double lookahead_guess_distance =
        start_distance + std::hypot(lookahead_pose.pose.position.x, lookahead_pose.pose.position.y);

    auto begin_it = cumulative_distances->begin() + static_cast<std::ptrdiff_t>(begin_idx);
    auto end_it = cumulative_distances->begin() + static_cast<std::ptrdiff_t>(end_idx + 1);
    auto lookahead_it = std::lower_bound(begin_it, end_it, lookahead_guess_distance);

    size_t lookahead_idx = begin_idx;
    if (lookahead_it == end_it) {
        lookahead_idx = end_idx;
    } else {
        lookahead_idx = static_cast<size_t>(std::distance(cumulative_distances->begin(), lookahead_it));
        if (lookahead_idx > begin_idx) {
            const double high_err = std::abs((*cumulative_distances)[lookahead_idx] - lookahead_guess_distance);
            const double low_err = std::abs((*cumulative_distances)[lookahead_idx - 1] - lookahead_guess_distance);
            if (low_err < high_err) {
                --lookahead_idx;
            }
        }
    }

    const double lookahead_pose_cumulative_distance = (*cumulative_distances)[lookahead_idx];
    const auto backward_pose = findPoseAtDistance(path, *cumulative_distances, cumulative_offset,
                                                  lookahead_pose_cumulative_distance - backward_dist);
    const auto forward_pose = findPoseAtDistance(path, *cumulative_distances, cumulative_offset,
                                                 lookahead_pose_cumulative_distance + forward_dist);

    const double curvature_radius = calculateCurvatureRadius(backward_pose.pose.position, lookahead_pose.pose.position,
                                                             forward_pose.pose.position);
    const double curvature = 1.0 / curvature_radius;
    visualizeCurvaturePoints(backward_pose, forward_pose);
    return curvature;
}

double OmniPidPursuitController::calculateCurvatureRadius(const geometry_msgs::msg::Point& near_point,
                                                          const geometry_msgs::msg::Point& current_point,
                                                          const geometry_msgs::msg::Point& far_point) const {
    double x1 = near_point.x, y1 = near_point.y;
    double x2 = current_point.x, y2 = current_point.y;
    double x3 = far_point.x, y3 = far_point.y;

    double denominator = 2.0 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
    if (std::abs(denominator) < 1e-9) {
        return 1e9;
    }

    double center_x =
        ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) + (x3 * x3 + y3 * y3) * (y1 - y2)) /
        denominator;
    double center_y =
        ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) + (x3 * x3 + y3 * y3) * (x2 - x1)) /
        denominator;
    double radius = std::hypot(x2 - center_x, y2 - center_y);
    if (std::isnan(radius) || std::isinf(radius) || radius < 1e-9) {
        return 1e9;
    }
    return radius;
}

void OmniPidPursuitController::visualizeCurvaturePoints(const geometry_msgs::msg::PoseStamped& backward_pose,
                                                        const geometry_msgs::msg::PoseStamped& forward_pose) const {
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker near_marker;
    near_marker.header = backward_pose.header;
    near_marker.ns = "curvature_points";
    near_marker.id = 0;
    near_marker.type = visualization_msgs::msg::Marker::SPHERE;
    near_marker.action = visualization_msgs::msg::Marker::ADD;
    near_marker.pose = backward_pose.pose;
    near_marker.scale.x = near_marker.scale.y = near_marker.scale.z = 0.1;
    near_marker.color.g = 1.0;
    near_marker.color.a = 1.0;

    visualization_msgs::msg::Marker far_marker;
    far_marker.header = forward_pose.header;
    far_marker.ns = "curvature_points";
    far_marker.id = 1;
    far_marker.type = visualization_msgs::msg::Marker::SPHERE;
    far_marker.action = visualization_msgs::msg::Marker::ADD;
    far_marker.pose = forward_pose.pose;
    far_marker.scale.x = far_marker.scale.y = far_marker.scale.z = 0.1;
    far_marker.color.r = 1.0;
    far_marker.color.a = 1.0;

    marker_array.markers.push_back(near_marker);
    marker_array.markers.push_back(far_marker);

    curvature_points_pub_->publish(marker_array);
}

std::vector<double> OmniPidPursuitController::calculateCumulativeDistances(const nav_msgs::msg::Path& path) const {
    std::vector<double> cumulative_distances;
    if (path.poses.empty()) {
        return cumulative_distances;
    }

    cumulative_distances.reserve(path.poses.size());
    cumulative_distances.push_back(0.0);

    for (size_t i = 1; i < path.poses.size(); ++i) {
        const auto& prev_pose = path.poses[i - 1].pose.position;
        const auto& curr_pose = path.poses[i].pose.position;
        double distance = hypot(curr_pose.x - prev_pose.x, curr_pose.y - prev_pose.y);
        cumulative_distances.push_back(cumulative_distances.back() + distance);
    }
    return cumulative_distances;
}

geometry_msgs::msg::PoseStamped OmniPidPursuitController::findPoseAtDistance(
    const nav_msgs::msg::Path& path, const std::vector<double>& cumulative_distances, size_t cumulative_offset,
    double target_distance) const {
    if (path.poses.empty() || cumulative_distances.empty()) {
        return geometry_msgs::msg::PoseStamped();
    }

    const size_t begin_idx = std::min(cumulative_offset, cumulative_distances.size() - 1);
    const size_t end_idx = std::min(cumulative_offset + path.poses.size() - 1, cumulative_distances.size() - 1);
    if (end_idx < begin_idx) {
        return path.poses.front();
    }

    const double begin_dist = cumulative_distances[begin_idx];
    const double end_dist = cumulative_distances[end_idx];
    if (target_distance <= begin_dist) {
        return path.poses.front();
    }
    if (target_distance >= end_dist) {
        return path.poses[end_idx - begin_idx];
    }

    auto begin_it = cumulative_distances.begin() + static_cast<std::ptrdiff_t>(begin_idx);
    auto end_it = cumulative_distances.begin() + static_cast<std::ptrdiff_t>(end_idx + 1);
    auto it = std::lower_bound(begin_it, end_it, target_distance);
    size_t global_index = static_cast<size_t>(std::distance(cumulative_distances.begin(), it));

    if (global_index <= begin_idx) {
        return path.poses.front();
    }
    if (global_index > end_idx) {
        return path.poses[end_idx - begin_idx];
    }

    const size_t local_index = global_index - begin_idx;

    // Prevent division by zero when two consecutive points are at the same distance.
    double denominator = cumulative_distances[global_index] - cumulative_distances[global_index - 1];
    if (std::abs(denominator) < 1e-9) {
        return path.poses[local_index];
    }
    double ratio = (target_distance - cumulative_distances[global_index - 1]) / denominator;
    geometry_msgs::msg::PoseStamped pose1 = path.poses[local_index - 1];
    geometry_msgs::msg::PoseStamped pose2 = path.poses[local_index];

    geometry_msgs::msg::PoseStamped interpolated_pose;
    interpolated_pose.header = pose2.header;
    interpolated_pose.pose.position.x = pose1.pose.position.x + ratio * (pose2.pose.position.x - pose1.pose.position.x);
    interpolated_pose.pose.position.y = pose1.pose.position.y + ratio * (pose2.pose.position.y - pose1.pose.position.y);
    interpolated_pose.pose.position.z = pose1.pose.position.z + ratio * (pose2.pose.position.z - pose1.pose.position.z);
    interpolated_pose.pose.orientation = pose2.pose.orientation;

    return interpolated_pose;
}

rcl_interfaces::msg::SetParametersResult
OmniPidPursuitController::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    std::lock_guard<std::recursive_mutex> lock_reinit(mutex_);

    for (const auto& parameter : parameters) {
        const auto& type = parameter.get_type();
        const auto& name = parameter.get_name();

        if (type == ParameterType::PARAMETER_DOUBLE) {
            if (name == plugin_name_ + ".translation_kp") {
                translation_kp_ = parameter.as_double();
            } else if (name == plugin_name_ + ".translation_ki") {
                translation_ki_ = parameter.as_double();
            } else if (name == plugin_name_ + ".translation_kd") {
                translation_kd_ = parameter.as_double();
            } else if (name == plugin_name_ + ".rotation_kp") {
                rotation_kp_ = parameter.as_double();
            } else if (name == plugin_name_ + ".rotation_ki") {
                rotation_ki_ = parameter.as_double();
            } else if (name == plugin_name_ + ".rotation_kd") {
                rotation_kd_ = parameter.as_double();
            } else if (name == plugin_name_ + ".transform_tolerance") {
                double transform_tolerance = parameter.as_double();
                transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
            } else if (name == plugin_name_ + ".min_max_sum_error") {
                min_max_sum_error_ = parameter.as_double();
            } else if (name == plugin_name_ + ".lookahead_dist") {
                lookahead_dist_ = parameter.as_double();
            } else if (name == plugin_name_ + ".min_lookahead_dist") {
                min_lookahead_dist_ = parameter.as_double();
            } else if (name == plugin_name_ + ".max_lookahead_dist") {
                max_lookahead_dist_ = parameter.as_double();
            } else if (name == plugin_name_ + ".lookahead_time") {
                lookahead_time_ = parameter.as_double();
            } else if (name == plugin_name_ + ".use_rotate_to_heading_threshold") {
                use_rotate_to_heading_threshold_ = parameter.as_double();
            } else if (name == plugin_name_ + ".min_approach_linear_velocity") {
                min_approach_linear_velocity_ = parameter.as_double();
            } else if (name == plugin_name_ + ".approach_velocity_scaling_dist") {
                approach_velocity_scaling_dist_ = parameter.as_double();
            } else if (name == plugin_name_ + ".v_linear_max") {
                v_linear_max_ = parameter.as_double();
                configured_v_linear_max_ = v_linear_max_;
            } else if (name == plugin_name_ + ".v_linear_min") {
                v_linear_min_ = parameter.as_double();
            } else if (name == plugin_name_ + ".v_angular_max") {
                v_angular_max_ = parameter.as_double();
            } else if (name == plugin_name_ + ".v_angular_min") {
                v_angular_min_ = parameter.as_double();
            } else if (name == plugin_name_ + ".a_linear_max") {
                a_linear_max_ = parameter.as_double();
            } else if (name == plugin_name_ + ".a_angular_max") {
                a_angular_max_ = parameter.as_double();
            } else if (name == plugin_name_ + ".brake_margin") {
                brake_margin_ = parameter.as_double();
            } else if (name == plugin_name_ + ".brake_accel") {
                brake_accel_ = parameter.as_double();
            } else if (name == plugin_name_ + ".lateral_error_gain") {
                lateral_error_gain_ = parameter.as_double();
            } else if (name == plugin_name_ + ".lateral_error_max") {
                lateral_error_max_ = parameter.as_double();
            } else if (name == plugin_name_ + ".a_lim_x") {
                a_lim_x_ = parameter.as_double();
            } else if (name == plugin_name_ + ".a_lim_y") {
                a_lim_y_ = parameter.as_double();
            } else if (name == plugin_name_ + ".a_lateral_max") {
                a_lateral_max_ = parameter.as_double();
            } else if (name == plugin_name_ + ".curvature_forward_dist") {
                curvature_forward_dist_ = parameter.as_double();
            } else if (name == plugin_name_ + ".curvature_backward_dist") {
                curvature_backward_dist_ = parameter.as_double();
            } else if (name == plugin_name_ + ".max_velocity_scaling_factor_rate") {
                max_velocity_scaling_factor_rate_ = parameter.as_double();
            } else if (name == plugin_name_ + ".kv_ff") {
                kv_ff_ = parameter.as_double();
            } else if (name == plugin_name_ + ".goal_dist_scale") {
                goal_dist_scale_ = parameter.as_double();
            } else if (name == plugin_name_ + ".wheel_base") {
                wheel_base_ = parameter.as_double();
            } else if (name == plugin_name_ + ".track_width") {
                track_width_ = parameter.as_double();
            } else if (name == plugin_name_ + ".wheel_speed_max") {
                wheel_speed_max_ = parameter.as_double();
            } else if (name == plugin_name_ + ".derivative_filter_tau") {
                derivative_filter_tau_ = parameter.as_double();
            } else if (name == plugin_name_ + ".loc_timeout_sec") {
                loc_timeout_sec_ = parameter.as_double();
            } else if (name == plugin_name_ + ".loc_k_v") {
                loc_k_v_ = parameter.as_double();
            } else if (name == plugin_name_ + ".loc_k_w") {
                loc_k_w_ = parameter.as_double();
            } else if (name == plugin_name_ + ".loc_v_scale_min") {
                loc_v_scale_min_ = parameter.as_double();
            } else if (name == plugin_name_ + ".loc_w_scale_min") {
                loc_w_scale_min_ = parameter.as_double();
            }
        } else if (type == ParameterType::PARAMETER_BOOL) {
            if (name == plugin_name_ + ".use_velocity_scaled_lookahead_dist") {
                use_velocity_scaled_lookahead_dist_ = parameter.as_bool();
            } else if (name == plugin_name_ + ".use_interpolation") {
                use_interpolation_ = parameter.as_bool();
            } else if (name == plugin_name_ + ".use_rotate_to_heading") {
                use_rotate_to_heading_ = parameter.as_bool();
            } else if (name == plugin_name_ + ".enable_curvature_ff") {
                enable_curvature_ff_ = parameter.as_bool();
            } else if (name == plugin_name_ + ".publish_debug") {
                publish_debug_ = parameter.as_bool();
            } else if (name == plugin_name_ + ".loc_uncertainty_enable") {
                loc_uncertainty_enable_ = parameter.as_bool();
            }
        }
    }

    refreshPoseCovSubscription(node_.lock());

    // 同步更新 PID 控制器参数
    if (move_pid_) {
        move_pid_->setGains(translation_kp_, translation_kd_, translation_ki_);
        move_pid_->setOutputLimits(v_linear_min_, v_linear_max_);
        move_pid_->setIntegralLimits(min_max_sum_error_);  // [M2 修复]
        move_pid_->setDerivativeFilterTau(derivative_filter_tau_);
    }
    if (heading_pid_) {
        heading_pid_->setGains(rotation_kp_, rotation_kd_, rotation_ki_);
        heading_pid_->setOutputLimits(v_angular_min_, v_angular_max_);
        heading_pid_->setIntegralLimits(min_max_sum_error_);  // [M2 修复]
        heading_pid_->setDerivativeFilterTau(derivative_filter_tau_);
    }

    result.successful = true;
    return result;
}

};  // namespace rc26_omni_controller
// Register this controller as a nav2_core plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rc26_omni_controller::OmniPidPursuitController, nav2_core::Controller)
