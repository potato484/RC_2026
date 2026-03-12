#pragma once

#include <string>

#include "nav2_msgs/msg/speed_limit.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace rc26_terrain_nav2 {

class TerrainSpeedLimitBridge : public rclcpp::Node {
public:
    explicit TerrainSpeedLimitBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void inputCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void publishSpeedLimit(double speed_limit, bool percentage);

    std::string input_topic_{"terrain_speed_limit"};
    std::string output_topic_{"/controller_server/speed_limit"};
    std::string output_topic_compat_{"/speed_limit"};
    double min_speed_limit_{0.0};
    double max_speed_limit_{2.5};
    bool publish_no_limit_on_nan_{true};

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr input_sub_;
    rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr output_pub_;
    rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr output_pub_compat_;
};

}  // namespace rc26_terrain_nav2
