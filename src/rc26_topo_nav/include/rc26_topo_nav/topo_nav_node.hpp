#pragma once

#include "rc26_topo_nav/graph_loader.hpp"
#include "rc26_topo_nav/overlay_reducer.hpp"
#include "rc26_topo_nav/planner.hpp"
#include "rc26_topo_nav/edge_executor.hpp"
#include "rc26_topo_nav/diagnostics.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rc26_interfaces/action/navigate_topo_target.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <atomic>
#include <mutex>
#include <thread>

namespace rc26_topo_nav {

class TopoNavNode : public rclcpp::Node {
public:
    using NavigateTopoTarget = rc26_interfaces::action::NavigateTopoTarget;
    using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateTopoTarget>;

    explicit TopoNavNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~TopoNavNode() override;

private:
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const NavigateTopoTarget::Goal> goal);

    rclcpp_action::CancelResponse handleCancel(std::shared_ptr<GoalHandle> goal_handle);

    void handleAccepted(std::shared_ptr<GoalHandle> goal_handle);

    void execute(std::shared_ptr<GoalHandle> goal_handle);

    std::string findNearestNode() const;

    FieldGraph graph_;
    PlannerWeights weights_;
    std::unique_ptr<OverlayReducer> overlay_reducer_;
    std::unique_ptr<EdgeExecutor> edge_executor_;
    std::unique_ptr<Diagnostics> diagnostics_;

    rclcpp_action::Server<NavigateTopoTarget>::SharedPtr action_server_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::mutex worker_mutex_;
    std::thread worker_thread_;
    std::mutex execution_mutex_;
    std::atomic<bool> goal_active_{false};

    std::string team_;
    static constexpr int MAX_REPLAN_ATTEMPTS = 3;
};

}  // namespace rc26_topo_nav
