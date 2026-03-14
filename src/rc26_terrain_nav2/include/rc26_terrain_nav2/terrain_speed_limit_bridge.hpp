#pragma once

#include <mutex>
#include <string>

#include "nav2_msgs/msg/speed_limit.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace rc26_terrain_nav2 {

class TerrainSpeedLimitBridge : public rclcpp::Node {
public:
    explicit TerrainSpeedLimitBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void watchdogTimerCallback();
    void inputCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void publishSpeedLimit(double speed_limit, bool percentage);
    static std::string normalizePolicy(std::string policy);

    std::string input_topic_{"terrain_speed_limit"};
    std::string output_topic_{"controller_server/speed_limit"};
    std::string output_topic_compat_{"speed_limit"};
    double min_speed_limit_{0.0};
    double max_speed_limit_{2.5};
    bool publish_no_limit_on_nan_{true};
    double republish_period_sec_{0.2};
    double stale_timeout_sec_{0.0};
    std::string stale_policy_{"keep_last"};

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr input_sub_;
    rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr output_pub_;
    rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr output_pub_compat_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    std::mutex state_mutex_;
    rclcpp::Time last_input_stamp_{0, 0, RCL_ROS_TIME};
    double last_speed_limit_{0.0};
    bool last_percentage_{false};
    bool has_output_state_{false};
    bool stale_policy_applied_{false};
};

}  // namespace rc26_terrain_nav2
