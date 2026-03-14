#include "rc26_terrain_nav2/terrain_speed_limit_bridge.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

namespace {

constexpr double kMinRepublishPeriodSec = 0.02;

}  // namespace

namespace rc26_terrain_nav2 {

std::string TerrainSpeedLimitBridge::normalizePolicy(std::string policy) {
    std::transform(policy.begin(), policy.end(), policy.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return policy;
}

TerrainSpeedLimitBridge::TerrainSpeedLimitBridge(const rclcpp::NodeOptions& options)
    : Node("terrain_speed_limit_bridge", options) {
    this->declare_parameter<std::string>("input_topic", input_topic_);
    this->declare_parameter<std::string>("output_topic", output_topic_);
    this->declare_parameter<std::string>("output_topic_compat", output_topic_compat_);
    this->declare_parameter<double>("min_speed_limit", min_speed_limit_);
    this->declare_parameter<double>("max_speed_limit", max_speed_limit_);
    this->declare_parameter<bool>("publish_no_limit_on_nan", publish_no_limit_on_nan_);
    this->declare_parameter<double>("republish_period_sec", republish_period_sec_);
    this->declare_parameter<double>("stale_timeout_sec", stale_timeout_sec_);
    this->declare_parameter<std::string>("stale_policy", stale_policy_);

    this->get_parameter("input_topic", input_topic_);
    this->get_parameter("output_topic", output_topic_);
    this->get_parameter("output_topic_compat", output_topic_compat_);
    this->get_parameter("min_speed_limit", min_speed_limit_);
    this->get_parameter("max_speed_limit", max_speed_limit_);
    this->get_parameter("publish_no_limit_on_nan", publish_no_limit_on_nan_);
    this->get_parameter("republish_period_sec", republish_period_sec_);
    this->get_parameter("stale_timeout_sec", stale_timeout_sec_);
    this->get_parameter("stale_policy", stale_policy_);

    min_speed_limit_ = std::max(0.0, min_speed_limit_);
    max_speed_limit_ = std::max(min_speed_limit_, max_speed_limit_);
    republish_period_sec_ = std::max(kMinRepublishPeriodSec, republish_period_sec_);
    stale_timeout_sec_ = std::max(0.0, stale_timeout_sec_);
    stale_policy_ = normalizePolicy(stale_policy_);
    if (stale_policy_ != "keep_last" && stale_policy_ != "no_limit" &&
        stale_policy_ != "zero_limit") {
        RCLCPP_WARN(this->get_logger(),
                    "Unknown stale_policy '%s', fallback to 'keep_last'",
                    stale_policy_.c_str());
        stale_policy_ = "keep_last";
    }

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliable();
    qos.durability(rclcpp::DurabilityPolicy::Volatile);

    output_pub_ = this->create_publisher<nav2_msgs::msg::SpeedLimit>(output_topic_, qos);
    if (!output_topic_compat_.empty() && output_topic_compat_ != output_topic_) {
        output_pub_compat_ = this->create_publisher<nav2_msgs::msg::SpeedLimit>(output_topic_compat_, qos);
    }
    input_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        input_topic_, qos, std::bind(&TerrainSpeedLimitBridge::inputCallback, this, std::placeholders::_1));
    watchdog_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(republish_period_sec_),
        std::bind(&TerrainSpeedLimitBridge::watchdogTimerCallback, this));

    RCLCPP_INFO(
        this->get_logger(),
        "TerrainSpeedLimitBridge started: input_topic=%s output_topic=%s output_topic_compat=%s "
        "min=%.3f max=%.3f publish_no_limit_on_nan=%s republish_period=%.3f stale_timeout=%.3f "
        "stale_policy=%s",
        input_topic_.c_str(),
        output_topic_.c_str(),
        output_topic_compat_.c_str(),
        min_speed_limit_,
        max_speed_limit_,
        publish_no_limit_on_nan_ ? "true" : "false",
        republish_period_sec_,
        stale_timeout_sec_,
        stale_policy_.c_str());
}

void TerrainSpeedLimitBridge::inputCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    if (!msg) {
        return;
    }

    const double value = static_cast<double>(msg->data);
    if (!std::isfinite(value)) {
        if (publish_no_limit_on_nan_) {
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                last_input_stamp_ = this->now();
                last_speed_limit_ = nav2_costmap_2d::NO_SPEED_LIMIT;
                last_percentage_ = false;
                has_output_state_ = true;
                stale_policy_applied_ = false;
            }
            publishSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);
        }
        return;
    }

    const double clamped = std::clamp(value, min_speed_limit_, max_speed_limit_);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        last_input_stamp_ = this->now();
        last_speed_limit_ = clamped;
        last_percentage_ = false;
        has_output_state_ = true;
        stale_policy_applied_ = false;
    }
    publishSpeedLimit(clamped, false);
}

void TerrainSpeedLimitBridge::watchdogTimerCallback() {
    double speed_limit = 0.0;
    bool percentage = false;
    bool should_publish = false;
    bool log_stale = false;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!has_output_state_) {
            return;
        }

        speed_limit = last_speed_limit_;
        percentage = last_percentage_;
        should_publish = true;

        if (stale_timeout_sec_ > 0.0 && last_input_stamp_.nanoseconds() > 0) {
            const double age_sec = (this->now() - last_input_stamp_).seconds();
            if (age_sec > stale_timeout_sec_) {
                if (!stale_policy_applied_) {
                    log_stale = true;
                }

                if (stale_policy_ == "no_limit") {
                    speed_limit = nav2_costmap_2d::NO_SPEED_LIMIT;
                    percentage = false;
                    last_speed_limit_ = speed_limit;
                    last_percentage_ = percentage;
                } else if (stale_policy_ == "zero_limit") {
                    speed_limit = 0.0;
                    percentage = false;
                    last_speed_limit_ = speed_limit;
                    last_percentage_ = percentage;
                }

                stale_policy_applied_ = true;
            } else {
                stale_policy_applied_ = false;
            }
        }
    }

    if (log_stale) {
        RCLCPP_WARN(
            this->get_logger(),
            "No terrain speed limit update received within %.3f s, applying stale_policy=%s",
            stale_timeout_sec_,
            stale_policy_.c_str());
    }
    if (should_publish) {
        publishSpeedLimit(speed_limit, percentage);
    }
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
