#pragma once

#include <string>
#include <optional>

namespace rc26_nav_mode_manager {

struct WatchdogConfig {
    double timeout_sec{0.0};
    bool stop_required_on_timeout{false};
};

struct PrecheckConfig {
    bool require_stopped{false};
};

struct CostmapConfig {
    bool clear_on_switch{false};
};

struct ControllerConfig {
    std::optional<double> v_linear_max;
    std::optional<double> v_angular_max;
    std::optional<double> v_linear_min;
    std::optional<double> acc_linear;
    std::optional<double> acc_angular;
    int transition_timeout_ms{500};
};

struct NavProfile {
    std::string name;
    std::string fallback_profile;
    WatchdogConfig watchdog;
    PrecheckConfig precheck;
    CostmapConfig costmap;
    ControllerConfig controller;
};

}  // namespace rc26_nav_mode_manager
