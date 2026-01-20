#pragma once

#include <atomic>
#include <mutex>
#include <string>

#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "rc26_interfaces/msg/nav_safety_mode.hpp"
#include "rc26_interfaces/srv/set_nav_mode.hpp"

namespace rc26_nav_mode_manager {

class NavModeManager : public rclcpp::Node {
public:
    using NavSafetyMode = rc26_interfaces::msg::NavSafetyMode;
    using SetNavMode = rc26_interfaces::srv::SetNavMode;
    using ClearEntireCostmap = nav2_msgs::srv::ClearEntireCostmap;

    explicit NavModeManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void handleSetMode(const SetNavMode::Request::SharedPtr request,
                       SetNavMode::Response::SharedPtr response);

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void obstaclesCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    bool checkRobotStopped() const;
    bool setDropLayerEnabled(bool enabled);
    bool clearCostmap();
    bool waitCostmapRebuild(const rclcpp::Time& clear_time, double timeout_sec);

    void onTimeout();
    void publishState(bool stop_required = false, bool timed_out = false);

    uint8_t current_mode_{NavSafetyMode::NORMAL};
    rclcpp::Time mode_switch_time_;
    std::string mode_reason_;
    mutable std::mutex state_mutex_;

    rclcpp::TimerBase::SharedPtr timeout_timer_;
    double default_timeout_sec_{5.0};
    std::atomic<double> current_timeout_sec_{5.0};

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    double stop_linear_eps_mps_{0.05};
    double stop_angular_eps_rps_{0.05};
    std::atomic<double> last_linear_speed_mps_{0.0};
    std::atomic<double> last_angular_speed_rps_{0.0};
    rclcpp::Time last_odom_stamp_;
    mutable std::mutex odom_mutex_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_obs_sub_;
    rclcpp::Time last_obstacles_obs_stamp_;
    mutable std::mutex obstacles_mutex_;

    rclcpp::Service<SetNavMode>::SharedPtr set_mode_srv_;
    rclcpp::Publisher<NavSafetyMode>::SharedPtr state_pub_;

    rclcpp::AsyncParametersClient::SharedPtr costmap_param_client_;
    rclcpp::Client<ClearEntireCostmap>::SharedPtr clear_costmap_client_;

    std::string costmap_node_name_;
    std::string odom_topic_;
    std::string obstacles_topic_;
    double param_timeout_sec_{2.0};
    double clear_timeout_sec_{2.0};
    double rebuild_timeout_sec_{0.5};
};

}  // namespace rc26_nav_mode_manager
