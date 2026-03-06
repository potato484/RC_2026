#include "rc26_nav_mode_manager/nav_mode_manager.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>

namespace rc26_nav_mode_manager {

namespace {

std::chrono::steady_clock::time_point makeDeadline(double timeout_sec) {
    return std::chrono::steady_clock::now() +
           std::chrono::duration_cast<std::chrono::steady_clock::duration>(
               std::chrono::duration<double>(timeout_sec));
}

}  // namespace

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

    exclusive_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    executor_ = std::make_unique<ProfileExecutor>(this, profile_db_.get());
    watchdog_ = std::make_unique<WatchdogTimer>(this);

    // Safety state is consumed by SmartWaypointNavigator; it will abort navigation when stop_required/timed_out is set.
    state_pub_ = this->create_publisher<NavSafetyState>("nav_safety_state", 10);

    set_mode_srv_ = this->create_service<SetNavMode>(
        "set_nav_mode",
        std::bind(&NavModeManager::handleSetMode, this,
                  std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        exclusive_group_);

    publish_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() { publishState(); });

    publishState();
    RCLCPP_INFO(this->get_logger(), "NavModeManager initialized (profile-driven)");
}

void NavModeManager::armWatchdog(double timeout_sec) {
    watchdog_->start(timeout_sec,
                     std::bind(&NavModeManager::onWatchdogTimeout, this),
                     exclusive_group_);

    std::lock_guard<std::mutex> lock(state_mutex_);
    current_watchdog_timeout_sec_ = timeout_sec;
    watchdog_deadline_ = timeout_sec > 0.0 ? makeDeadline(timeout_sec)
                                           : std::chrono::steady_clock::time_point{};
}

void NavModeManager::disarmWatchdog() {
    watchdog_->cancel();

    std::lock_guard<std::mutex> lock(state_mutex_);
    current_watchdog_timeout_sec_ = 0.0;
    watchdog_deadline_ = std::chrono::steady_clock::time_point{};
}

double NavModeManager::getRemainingWatchdogSecLocked() const {
    if (current_watchdog_timeout_sec_ <= 0.0) {
        return 0.0;
    }

    const auto now = std::chrono::steady_clock::now();
    if (watchdog_deadline_ <= now) {
        return 0.0;
    }

    return std::chrono::duration<double>(watchdog_deadline_ - now).count();
}

void NavModeManager::publishState(bool stop_required, bool timed_out) {
    NavSafetyState msg;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        msg.header.stamp = this->now();
        msg.current_profile = current_profile_;
        msg.reason = current_reason_;
        msg.stop_required = stop_required_ || stop_required;
        msg.timed_out = timed_out_ || timed_out;
    }
    state_pub_->publish(msg);
}

void NavModeManager::onWatchdogTimeout() {
    std::string profile_name;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        profile_name = current_profile_;
        current_watchdog_timeout_sec_ = 0.0;
        watchdog_deadline_ = std::chrono::steady_clock::time_point{};
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

    const std::string& fallback_name = profile_opt->fallback_profile;
    if (fallback_name == from_profile) {
        RCLCPP_INFO(this->get_logger(), "Already at terminal fallback '%s'", fallback_name.c_str());
        return;
    }

    auto fallback_opt = profile_db_->get(fallback_name);
    if (!fallback_opt) {
        RCLCPP_ERROR(this->get_logger(), "Fallback profile '%s' not found", fallback_name.c_str());
        return;
    }

    ProfileExecutor::SwitchResult result;
    try {
        result = executor_->executeForFallback(*fallback_opt, "watchdog_timeout");
    } catch (const std::exception& e) {
        result.message = "Fallback execution threw exception: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "%s", result.message.c_str());
    }

    if (result.success) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_profile_ = fallback_name;
            current_reason_ = "watchdog_timeout";
        }
        RCLCPP_INFO(this->get_logger(), "Fallback to '%s' succeeded", fallback_name.c_str());
        if (fallback_opt->watchdog.timeout_sec > 0) {
            try {
                armWatchdog(fallback_opt->watchdog.timeout_sec);
            } catch (const std::exception& e) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                current_watchdog_timeout_sec_ = 0.0;
                watchdog_deadline_ = std::chrono::steady_clock::time_point{};
                RCLCPP_ERROR(this->get_logger(),
                             "Fallback watchdog arm failed for '%s': %s",
                             fallback_name.c_str(), e.what());
            }
        } else {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_watchdog_timeout_sec_ = 0.0;
            watchdog_deadline_ = std::chrono::steady_clock::time_point{};
        }
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
    std::string previous_profile;
    std::string previous_reason;
    bool previous_stop_required = false;
    bool previous_timed_out = false;
    double previous_watchdog_remaining = 0.0;

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

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        previous_profile = current_profile_;
        previous_reason = current_reason_;
        previous_stop_required = stop_required_;
        previous_timed_out = timed_out_;
        previous_watchdog_remaining = getRemainingWatchdogSecLocked();
    }

    disarmWatchdog();

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        timed_out_ = false;
        stop_required_ = false;
    }

    ProfileExecutor::SwitchResult result;
    try {
        result = executor_->execute(*profile_opt, reason);
    } catch (const std::exception& e) {
        result.message = "Profile execution threw exception: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "%s", result.message.c_str());
    }

    response->success = result.success;
    response->message = result.message;

    if (result.success) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_profile_ = profile_name;
            current_reason_ = reason;
        }

        double effective_timeout = (timeout > 0) ? timeout : profile_opt->watchdog.timeout_sec;
        if (effective_timeout > 0.0) {
            try {
                armWatchdog(effective_timeout);
            } catch (const std::exception& e) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                current_watchdog_timeout_sec_ = 0.0;
                watchdog_deadline_ = std::chrono::steady_clock::time_point{};
                RCLCPP_ERROR(this->get_logger(),
                             "Mode '%s' switched but watchdog arm failed: %s",
                             profile_name.c_str(), e.what());
            }
        }

        publishState();
        return;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_profile_ = previous_profile;
        current_reason_ = previous_reason;
        stop_required_ = previous_stop_required;
        timed_out_ = previous_timed_out;
    }
    if (previous_watchdog_remaining > 0.0) {
        try {
            armWatchdog(previous_watchdog_remaining);
            RCLCPP_WARN(this->get_logger(),
                        "Mode switch to '%s' failed, restored watchdog for '%s' with %.3fs remaining",
                        profile_name.c_str(), previous_profile.c_str(), previous_watchdog_remaining);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                         "Mode switch to '%s' failed and watchdog restore for '%s' also failed: %s",
                         profile_name.c_str(), previous_profile.c_str(), e.what());
        }
    }
    publishState();
}

}  // namespace rc26_nav_mode_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rc26_nav_mode_manager::NavModeManager)
