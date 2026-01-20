#include "rc26_nav_mode_manager/nav_mode_manager.hpp"

#include <chrono>
#include <cmath>

namespace rc26_nav_mode_manager {

NavModeManager::NavModeManager(const rclcpp::NodeOptions& options)
    : Node("nav_mode_manager", options) {
    this->declare_parameter<std::string>("costmap_node_name", "local_costmap/local_costmap");
    this->declare_parameter<std::string>("odom_topic", "odom");
    this->declare_parameter<std::string>("obstacles_topic", "terrain_obstacles");
    this->declare_parameter<double>("default_timeout_sec", 5.0);
    this->declare_parameter<double>("stop_linear_eps_mps", 0.05);
    this->declare_parameter<double>("stop_angular_eps_rps", 0.05);
    this->declare_parameter<double>("param_timeout_sec", 2.0);
    this->declare_parameter<double>("clear_timeout_sec", 2.0);
    this->declare_parameter<double>("rebuild_timeout_sec", 0.5);

    costmap_node_name_ = this->get_parameter("costmap_node_name").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    obstacles_topic_ = this->get_parameter("obstacles_topic").as_string();
    default_timeout_sec_ = this->get_parameter("default_timeout_sec").as_double();
    stop_linear_eps_mps_ = this->get_parameter("stop_linear_eps_mps").as_double();
    stop_angular_eps_rps_ = this->get_parameter("stop_angular_eps_rps").as_double();
    param_timeout_sec_ = this->get_parameter("param_timeout_sec").as_double();
    clear_timeout_sec_ = this->get_parameter("clear_timeout_sec").as_double();
    rebuild_timeout_sec_ = this->get_parameter("rebuild_timeout_sec").as_double();

    current_timeout_sec_ = default_timeout_sec_;
    mode_switch_time_ = this->now();
    last_odom_stamp_ = this->now();
    last_obstacles_obs_stamp_ = this->now();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&NavModeManager::odomCallback, this, std::placeholders::_1));

    obstacles_obs_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        obstacles_topic_, rclcpp::SensorDataQoS(),
        std::bind(&NavModeManager::obstaclesCallback, this, std::placeholders::_1));

    state_pub_ = this->create_publisher<NavSafetyMode>("nav_safety/state", 10);

    set_mode_srv_ = this->create_service<SetNavMode>(
        "nav_safety/set_mode",
        std::bind(&NavModeManager::handleSetMode, this, std::placeholders::_1, std::placeholders::_2));

    costmap_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, costmap_node_name_);

    std::string clear_service_name = costmap_node_name_ + "/clear_entirely_local_costmap";
    size_t pos = clear_service_name.find("local_costmap/local_costmap");
    if (pos != std::string::npos) {
        clear_service_name = clear_service_name.substr(0, pos) + "local_costmap/clear_entirely_local_costmap";
    }
    clear_costmap_client_ = this->create_client<ClearEntireCostmap>(clear_service_name);

    RCLCPP_INFO(this->get_logger(), "NavModeManager initialized");
    RCLCPP_INFO(this->get_logger(), "  costmap_node: %s", costmap_node_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "  clear_service: %s", clear_service_name.c_str());

    publishState();
}

void NavModeManager::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    last_odom_stamp_ = msg->header.stamp;
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    last_linear_speed_mps_ = std::sqrt(vx * vx + vy * vy);
    last_angular_speed_rps_ = std::abs(msg->twist.twist.angular.z);
}

void NavModeManager::obstaclesCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    last_obstacles_obs_stamp_ = msg->header.stamp;
}

bool NavModeManager::checkRobotStopped() const {
    double lin = last_linear_speed_mps_.load();
    double ang = last_angular_speed_rps_.load();
    return (lin < stop_linear_eps_mps_) && (ang < stop_angular_eps_rps_);
}

bool NavModeManager::setDropLayerEnabled(bool enabled) {
    if (!costmap_param_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Parameter service not available for %s", costmap_node_name_.c_str());
        return false;
    }

    std::string param_name = "drop_layer.enabled";
    auto future = costmap_param_client_->set_parameters({rclcpp::Parameter(param_name, enabled)});

    auto status = future.wait_for(std::chrono::duration<double>(param_timeout_sec_));
    if (status != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Timeout setting parameter %s", param_name.c_str());
        return false;
    }

    auto results = future.get();
    if (results.empty() || !results[0].successful) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set parameter %s", param_name.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Set %s = %s", param_name.c_str(), enabled ? "true" : "false");
    return true;
}

bool NavModeManager::clearCostmap() {
    if (!clear_costmap_client_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Clear costmap service not available");
        return false;
    }

    auto request = std::make_shared<ClearEntireCostmap::Request>();
    auto future = clear_costmap_client_->async_send_request(request);

    auto status = future.wait_for(std::chrono::duration<double>(clear_timeout_sec_));
    if (status != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Timeout clearing costmap");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Costmap cleared");
    return true;
}

bool NavModeManager::waitCostmapRebuild(const rclcpp::Time& clear_time, double timeout_sec) {
    auto start = this->now();
    rclcpp::Rate rate(50);

    while (rclcpp::ok()) {
        auto elapsed = (this->now() - start).seconds();
        if (elapsed > timeout_sec) {
            RCLCPP_WARN(this->get_logger(), "Timeout waiting for costmap rebuild");
            return false;
        }

        rclcpp::Time obs_stamp;
        {
            std::lock_guard<std::mutex> lock(obstacles_mutex_);
            obs_stamp = last_obstacles_obs_stamp_;
        }

        if (obs_stamp > clear_time) {
            RCLCPP_DEBUG(this->get_logger(), "Costmap rebuild confirmed (obstacle obs received)");
            return true;
        }

        rate.sleep();
    }
    return false;
}

void NavModeManager::onTimeout() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (current_mode_ == NavSafetyMode::MF_TRAVERSE || current_mode_ == NavSafetyMode::MF_EXIT) {
        RCLCPP_ERROR(this->get_logger(), "Mode timeout! Reverting to MF_SAFE and requesting STOP");

        setDropLayerEnabled(true);
        current_mode_ = NavSafetyMode::MF_SAFE;
        mode_reason_ = "timeout_revert";
        mode_switch_time_ = this->now();

        publishState(true, true);
    }

    if (timeout_timer_) {
        timeout_timer_->cancel();
        timeout_timer_.reset();
    }
}

void NavModeManager::publishState(bool stop_required, bool timed_out) {
    NavSafetyMode msg;
    msg.mode = current_mode_;
    msg.stamp = this->now();
    msg.reason = mode_reason_;
    msg.stop_required = stop_required;
    msg.timed_out = timed_out;
    state_pub_->publish(msg);
}

void NavModeManager::handleSetMode(const SetNavMode::Request::SharedPtr request,
                                   SetNavMode::Response::SharedPtr response) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    uint8_t target_mode = request->mode;
    double timeout = (request->timeout > 0.0f) ? static_cast<double>(request->timeout) : default_timeout_sec_;
    std::string reason = request->reason;

    if (target_mode > NavSafetyMode::MF_EXIT) {
        response->success = false;
        response->message = "Invalid mode value";
        return;
    }

    bool need_drop_off = (target_mode == NavSafetyMode::MF_TRAVERSE || target_mode == NavSafetyMode::MF_EXIT);
    bool need_drop_on = (target_mode == NavSafetyMode::NORMAL || target_mode == NavSafetyMode::MF_SAFE);

    if (need_drop_off && !checkRobotStopped()) {
        response->success = false;
        response->message = "Robot must be stopped before switching to TRAVERSE/EXIT mode";
        RCLCPP_WARN(this->get_logger(), "Rejected mode switch: robot not stopped (lin=%.3f, ang=%.3f)",
                    last_linear_speed_mps_.load(), last_angular_speed_rps_.load());
        return;
    }

    if (need_drop_off) {
        if (!setDropLayerEnabled(false)) {
            response->success = false;
            response->message = "Failed to disable drop_layer";
            return;
        }

        rclcpp::Time clear_time = this->now();
        if (!clearCostmap()) {
            setDropLayerEnabled(true);
            response->success = false;
            response->message = "Failed to clear costmap";
            return;
        }

        if (!waitCostmapRebuild(clear_time, rebuild_timeout_sec_)) {
            RCLCPP_WARN(this->get_logger(), "Costmap rebuild wait timed out, proceeding anyway");
        }

        if (timeout_timer_) {
            timeout_timer_->cancel();
        }
        timeout_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(timeout),
            std::bind(&NavModeManager::onTimeout, this));
        current_timeout_sec_ = timeout;

    } else if (need_drop_on) {
        if (!setDropLayerEnabled(true)) {
            response->success = false;
            response->message = "Failed to enable drop_layer";
            return;
        }

        if (timeout_timer_) {
            timeout_timer_->cancel();
            timeout_timer_.reset();
        }
    }

    current_mode_ = target_mode;
    mode_reason_ = reason;
    mode_switch_time_ = this->now();

    publishState();

    response->success = true;
    response->message = "Mode switched successfully";

    const char* mode_names[] = {"NORMAL", "MF_SAFE", "MF_TRAVERSE", "MF_EXIT"};
    RCLCPP_INFO(this->get_logger(), "Mode switched to %s (reason: %s, timeout: %.1fs)",
                mode_names[target_mode], reason.c_str(), timeout);
}

}  // namespace rc26_nav_mode_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc26_nav_mode_manager::NavModeManager)
