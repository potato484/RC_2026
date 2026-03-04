#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "rc26_interfaces/msg/nav_safety_state.hpp"
#include "rc26_interfaces/srv/set_nav_mode.hpp"
#include "rc26_nav_mode_manager/profile_db.hpp"
#include "rc26_nav_mode_manager/profile_executor.hpp"
#include "rc26_nav_mode_manager/profile_loader.hpp"
#include "rc26_nav_mode_manager/watchdog_timer.hpp"

namespace rc26_nav_mode_manager {

class NavModeManager : public rclcpp::Node {
public:
    using NavSafetyState = rc26_interfaces::msg::NavSafetyState;
    using SetNavMode = rc26_interfaces::srv::SetNavMode;

    explicit NavModeManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void handleSetMode(const SetNavMode::Request::SharedPtr request,
                       SetNavMode::Response::SharedPtr response);
    void onWatchdogTimeout();
    void publishState(bool stop_required = false, bool timed_out = false);
    void executeFallback(const std::string& from_profile);

    std::unique_ptr<ProfileDB> profile_db_;
    std::unique_ptr<ProfileExecutor> executor_;
    std::unique_ptr<WatchdogTimer> watchdog_;

    std::string current_profile_{"safe"};
    std::string current_reason_;
    bool stop_required_{false};
    bool timed_out_{false};
    mutable std::mutex state_mutex_;

    rclcpp::Service<SetNavMode>::SharedPtr set_mode_srv_;
    rclcpp::Publisher<NavSafetyState>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::string profiles_file_;
    rclcpp::CallbackGroup::SharedPtr exclusive_group_;
};

}  // namespace rc26_nav_mode_manager
