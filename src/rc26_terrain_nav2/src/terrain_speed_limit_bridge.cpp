#include "rc26_terrain_nav2/terrain_speed_limit_bridge.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace rc26_terrain_nav2 {

TerrainSpeedLimitBridge::TerrainSpeedLimitBridge(const rclcpp::NodeOptions& options)
    : Node("terrain_speed_limit_bridge", options) {
    this->declare_parameter<std::string>("input_topic", input_topic_);
    this->declare_parameter<std::string>("output_topic", output_topic_);
    this->declare_parameter<std::string>("output_topic_compat", output_topic_compat_);
    this->declare_parameter<double>("min_speed_limit", min_speed_limit_);
    this->declare_parameter<double>("max_speed_limit", max_speed_limit_);
    this->declare_parameter<bool>("publish_no_limit_on_nan", publish_no_limit_on_nan_);

    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("output_topic_compat", output_topic_compat_);
    this->get_parameter("min_speed_limit", min_speed_limit_);
    this->get_parameter("max_speed_limit", max_speed_limit_);
    this->get_parameter("publish_no_limit_on_nan", publish_no_limit_on_nan_);

    min_speed_limit_ = std::max(0.0, min_speed_limit_);
    max_speed_limit_ = std::max(min_speed_limit_, max_speed_limit_);

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();
    qos.durability(rclcpp::DurabilityPolicy::Volatile);

    output_pub_ = this->create_publisher<nav2_msgs::msg::SpeedLimit>(output_topic_, qos);
    if (!output_topic_compat_.empty() && output_topic_compat_ != output_topic_) {
        output_pub_compat_ = this->create_publisher<nav2_msgs::msg::SpeedLimit>(output_topic_compat_, qos);
    }
    input_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        input_topic_, qos, std::bind(&TerrainSpeedLimitBridge::inputCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
        this->get_logger(),
        "TerrainSpeedLimitBridge started: input_topic=%s output_topic=%s output_topic_compat=%s "
        "min=%.3f max=%.3f publish_no_limit_on_nan=%s",
        input_topic_.c_str(),
        output_topic_.c_str(),
        output_topic_compat_.c_str(),
        min_speed_limit_,
        max_speed_limit_,
        publish_no_limit_on_nan_ ? "true" : "false");
}

void TerrainSpeedLimitBridge::inputCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    if (!msg) {
        return;
    }

    const double value = static_cast<double>(msg->data);
    if (!std::isfinite(value)) {
        if (publish_no_limit_on_nan_) {
            publishSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);
        }
        return;
    }

    const double clamped = std::clamp(value, min_speed_limit_, max_speed_limit_);
    publishSpeedLimit(clamped, false);
}

void TerrainSpeedLimitBridge::publishSpeedLimit(double speed_limit, bool percentage) {
    nav2_msgs::msg::SpeedLimit msg;
    msg.percentage = percentage;
    msg.speed_limit = speed_limit;
    if (output_pub_) {
        output_pub_->publish(msg);
    }
    if (output_pub_compat_) {
        output_pub_compat_->publish(msg);
    }
}

}  // namespace rc26_terrain_nav2
