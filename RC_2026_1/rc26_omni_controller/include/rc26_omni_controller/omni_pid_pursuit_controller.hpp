// Copyright 2025 RC2026
// 基于 pb_omni_pid_pursuit_controller 移植
// 原作者: Lihan Chen
//
// Licensed under the Apache License, Version 2.0

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "rc26_omni_controller/pid.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rc26_omni_controller {

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

private:
    void applyCurvatureLimitation(const nav_msgs::msg::Path& path,
                                  const geometry_msgs::msg::PoseStamped& lookahead_pose, double& linear_vel);

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
                                                       double target_distance) const;

private:
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string plugin_name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D* costmap_;
    rclcpp::Logger logger_{rclcpp::get_logger("OmniPidPursuitController")};
    rclcpp::Clock::SharedPtr clock_;
    double last_velocity_scaling_factor_;

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
    double v_angular_min_;
    double v_angular_max_;
    double min_approach_linear_velocity_;
    double approach_velocity_scaling_dist_;
    double curvature_min_;
    double curvature_max_;
    double reduction_ratio_at_high_curvature_;
    double curvature_forward_dist_;
    double curvature_backward_dist_;
    double max_velocity_scaling_factor_rate_;
    tf2::Duration transform_tolerance_;

    nav_msgs::msg::Path global_plan_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr carrot_pub_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr curvature_points_pub_;

    std::mutex mutex_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
};

}  // namespace rc26_omni_controller
