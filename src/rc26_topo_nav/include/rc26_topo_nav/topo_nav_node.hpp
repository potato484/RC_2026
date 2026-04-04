#pragma once

#include "rc26_topo_nav/graph_loader.hpp"
#include "rc26_topo_nav/body_planning.hpp"
#include "rc26_topo_nav/overlay_reducer.hpp"
#include "rc26_topo_nav/planner.hpp"
#include "rc26_topo_nav/edge_executor.hpp"
#include "rc26_topo_nav/diagnostics.hpp"
#include "rc26_topo_nav/surface_route.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rc26_interfaces/action/navigate_surface_route.hpp>
#include <rc26_interfaces/action/navigate_topo_target.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <atomic>
#include <mutex>
#include <optional>
#include <thread>

namespace rc26_topo_nav {

class TopoNavNode : public rclcpp::Node {
public:
    using NavigateTopoTarget = rc26_interfaces::action::NavigateTopoTarget;
    using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateTopoTarget>;
    using NavigateSurfaceRoute = rc26_interfaces::action::NavigateSurfaceRoute;
    using SurfaceGoalHandle = rclcpp_action::ServerGoalHandle<NavigateSurfaceRoute>;

    explicit TopoNavNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~TopoNavNode() override;

private:
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const NavigateTopoTarget::Goal> goal);

    rclcpp_action::CancelResponse handleCancel(std::shared_ptr<GoalHandle> goal_handle);

    void handleAccepted(std::shared_ptr<GoalHandle> goal_handle);

    void execute(std::shared_ptr<GoalHandle> goal_handle);

    rclcpp_action::GoalResponse handleSurfaceGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const NavigateSurfaceRoute::Goal> goal);

    rclcpp_action::CancelResponse handleSurfaceCancel(std::shared_ptr<SurfaceGoalHandle> goal_handle);

    void handleSurfaceAccepted(std::shared_ptr<SurfaceGoalHandle> goal_handle);

    void executeSurface(std::shared_ptr<SurfaceGoalHandle> goal_handle);

    std::string findNearestNode() const;
    bool currentRobotPose(Pose3& pose_out) const;

    FieldGraph graph_;
    FieldGraph surface_graph_;
    PlannerWeights weights_;
    std::unique_ptr<OverlayReducer> overlay_reducer_;
    std::unique_ptr<EdgeExecutor> edge_executor_;
    std::unique_ptr<Diagnostics> diagnostics_;

    rclcpp_action::Server<NavigateTopoTarget>::SharedPtr action_server_;
    rclcpp_action::Server<NavigateSurfaceRoute>::SharedPtr surface_action_server_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::mutex worker_mutex_;
    std::thread worker_thread_;
    std::mutex execution_mutex_;
    std::atomic<bool> goal_active_{false};

    std::string team_;
    std::optional<RobotGeometryProfile> robot_geometry_;
    SurfaceBodyPlanningConfig surface_body_planning_;
    double surface_start_match_xy_m_ = 0.30;
    double surface_start_match_z_m_ = 0.10;
    double surface_start_match_yaw_deg_ = 25.0;
    double surface_anchor_radius_m_ = 0.30;
    static constexpr int MAX_REPLAN_ATTEMPTS = 3;
};

}  // namespace rc26_topo_nav
