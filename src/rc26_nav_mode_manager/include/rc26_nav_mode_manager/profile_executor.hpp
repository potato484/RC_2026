#pragma once

#include <atomic>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rc26_nav_mode_manager/profile_db.hpp"
#include "rc26_nav_mode_manager/profile_types.hpp"

namespace rc26_nav_mode_manager {

class ProfileExecutor {
public:
    struct SwitchResult {
        bool success{false};
        std::string message;
    };

    using ClearEntireCostmap = nav2_msgs::srv::ClearEntireCostmap;

    explicit ProfileExecutor(rclcpp::Node* node, ProfileDB* db);

    SwitchResult execute(const NavProfile& profile, const std::string& reason);
    SwitchResult executeForFallback(const NavProfile& profile, const std::string& reason);
    uint64_t cancel();
    std::string getCurrentProfile() const;

private:
    bool stepValidate(const NavProfile& profile, std::string& error);
    bool stepPrecheck(const NavProfile& profile, std::string& error);
    bool stepCostmap(const NavProfile& profile, std::string& error);
    bool stepParams(const NavProfile& profile, std::string& error);
    void stepState(const NavProfile& profile, const std::string& reason);

    bool isCancelled(uint64_t epoch) const;
    bool checkRobotStopped() const;
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    bool rollbackParams();
    bool captureDefaults();
    std::string deriveLocalCostmapClearServiceName(const std::string& costmap_node_name) const;

    SwitchResult executeInternal(const NavProfile& profile, const std::string& reason, uint64_t epoch);

    rclcpp::Node* node_;
    ProfileDB* db_;

    std::string current_profile_{"safe"};
    std::string current_reason_;
    mutable std::mutex state_mutex_;
    mutable std::mutex execution_mutex_;

    std::atomic<uint64_t> cancel_epoch_{0};

    // Odom
    std::string odom_topic_{"odom"};
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::atomic<double> last_linear_speed_{0.0};
    std::atomic<double> last_angular_speed_{0.0};
    std::atomic<bool> odom_received_{false};
    double stop_linear_eps_{0.05};
    double stop_angular_eps_{0.05};
    static constexpr int kOdomWindowSize = 10;
    std::deque<double> linear_window_;
    std::deque<double> angular_window_;
    rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
    mutable std::mutex odom_window_mutex_;

    // Costmap
    std::string costmap_node_name_{"local_costmap/local_costmap"};
    double clear_timeout_sec_{2.0};
    rclcpp::Client<ClearEntireCostmap>::SharedPtr clear_costmap_client_;

    // Controller params
    std::string controller_server_node_{"controller_server"};
    double param_timeout_sec_{2.0};
    rclcpp::AsyncParametersClient::SharedPtr controller_param_client_;
    std::unordered_map<std::string, double> param_defaults_;
    bool defaults_captured_{false};
};

}  // namespace rc26_nav_mode_manager
