#include "rc26_nav_mode_manager/nav_mode_manager.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace rc26_nav_mode_manager {

NavModeManager::NavModeManager(const rclcpp::NodeOptions& options)
    : Node("nav_mode_manager", options) {

    this->declare_parameter<std::string>("profiles_file", "");
    profiles_file_ = this->get_parameter("profiles_file").as_string();

    if (profiles_file_.empty()) {
        auto pkg_dir = ament_index_cpp::get_package_share_directory("rc26_nav_mode_manager");
        // Default profile database used by SmartWaypointNavigator via `set_nav_mode` service.
        profiles_file_ = pkg_dir + "/config/nav_profiles.yaml";
    }

    auto load_result = ProfileLoader::loadFromFile(profiles_file_);
    if (!load_result.success) {
        RCLCPP_FATAL(this->get_logger(), "Failed to load profiles: %s",
                     load_result.error_message.c_str());
        throw std::runtime_error("Profile load failed: " + load_result.error_message);
    }

    profile_db_ = std::make_unique<ProfileDB>();
    profile_db_->load(std::move(load_result.profiles));
    RCLCPP_INFO(this->get_logger(), "Loaded %zu profiles from %s",
                profile_db_->getAllNames().size(), profiles_file_.c_str());

    executor_ = std::make_unique<ProfileExecutor>(this, profile_db_.get());
    watchdog_ = std::make_unique<WatchdogTimer>(this);

    // Safety state is consumed by SmartWaypointNavigator; it will abort navigation when stop_required/timed_out is set.
    state_pub_ = this->create_publisher<NavSafetyState>("nav_safety_state", 10);

    set_mode_srv_ = this->create_service<SetNavMode>(
        "set_nav_mode",
        std::bind(&NavModeManager::handleSetMode, this,
                  std::placeholders::_1, std::placeholders::_2));

    publish_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() { publishState(); });

    publishState();
    RCLCPP_INFO(this->get_logger(), "NavModeManager initialized (profile-driven)");
}

void NavModeManager::publishState(bool stop_required, bool timed_out) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    NavSafetyState msg;
    msg.header.stamp = this->now();
    msg.current_profile = current_profile_;
    msg.reason = current_reason_;
    msg.stop_required = stop_required_ || stop_required;
    msg.timed_out = timed_out_ || timed_out;
    state_pub_->publish(msg);
}

void NavModeManager::onWatchdogTimeout() {
    std::string profile_name;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        profile_name = current_profile_;
    }
    RCLCPP_WARN(this->get_logger(), "Watchdog timeout for profile '%s'", profile_name.c_str());

    executor_->cancel();

    auto profile_opt = profile_db_->get(profile_name);
    if (profile_opt) {
        bool stop_on_timeout = profile_opt->watchdog.stop_required_on_timeout;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            timed_out_ = true;
            stop_required_ = stop_on_timeout;
        }
        executeFallback(profile_name);
        publishState(stop_on_timeout, true);
    }
}

void NavModeManager::executeFallback(const std::string& from_profile) {
    auto profile_opt = profile_db_->get(from_profile);
    if (!profile_opt) return;

    std::string fallback_name = profile_opt->fallback_profile;
    if (fallback_name == from_profile) {
        RCLCPP_INFO(this->get_logger(), "Already at terminal fallback '%s'", fallback_name.c_str());
        return;
    }

    auto fallback_opt = profile_db_->get(fallback_name);
    if (!fallback_opt) {
        RCLCPP_ERROR(this->get_logger(), "Fallback profile '%s' not found", fallback_name.c_str());
        return;
    }

    auto result = executor_->executeForFallback(*fallback_opt, "watchdog_timeout");
    if (result.success) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_profile_ = fallback_name;
        current_reason_ = "watchdog_timeout";
        RCLCPP_INFO(this->get_logger(), "Fallback to '%s' succeeded", fallback_name.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "Fallback to '%s' failed: %s, trying next",
                    fallback_name.c_str(), result.message.c_str());
        executeFallback(fallback_name);
    }
}

void NavModeManager::handleSetMode(const SetNavMode::Request::SharedPtr request,
                                   SetNavMode::Response::SharedPtr response) {
    std::string profile_name = request->profile;
    double timeout = request->timeout;
    std::string reason = request->reason;

    if (profile_name.empty()) {
        response->success = false;
        response->message = "Profile name cannot be empty";
        return;
    }

    auto profile_opt = profile_db_->get(profile_name);
    if (!profile_opt) {
        response->success = false;
        response->message = "Profile '" + profile_name + "' not found";
        return;
    }

    watchdog_->cancel();

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        timed_out_ = false;
        stop_required_ = false;
    }

    auto result = executor_->execute(*profile_opt, reason);
    response->success = result.success;
    response->message = result.message;

    if (result.success) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_profile_ = profile_name;
            current_reason_ = reason;
        }

        double effective_timeout = (timeout > 0) ? timeout : profile_opt->watchdog.timeout_sec;
        if (effective_timeout > 0) {
            watchdog_->start(effective_timeout,
                             std::bind(&NavModeManager::onWatchdogTimeout, this));
        }

        publishState();
    }
}

}  // namespace rc26_nav_mode_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc26_nav_mode_manager::NavModeManager)
