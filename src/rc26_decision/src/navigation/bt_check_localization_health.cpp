#include "rc26_decision/navigation/bt_check_localization_health.hpp"

#include <algorithm>
#include <string>

#include "rc26_interfaces/msg/localization_health.hpp"
#include "rc26_interfaces/msg/route_observability.hpp"

namespace rc26_decision {

namespace {

int profilePriority(const std::string& profile) {
    if (profile == "loc_red_hold") {
        return 3;
    }
    if (profile == "loc_orange") {
        return 2;
    }
    if (profile == "loc_yellow") {
        return 1;
    }
    return 0;
}

std::string pickHigherPriorityProfile(const std::string& base_profile, const std::string& candidate_profile) {
    return profilePriority(candidate_profile) > profilePriority(base_profile) ? candidate_profile : base_profile;
}

}  // namespace

CheckLocalizationHealth::CheckLocalizationHealth(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config) {}

BT::PortsList CheckLocalizationHealth::providedPorts() {
    return {};
}

BT::NodeStatus CheckLocalizationHealth::tick() {
    int loc_level = static_cast<int>(rc26_interfaces::msg::LocalizationHealth::GREEN);
    int route_risk = static_cast<int>(rc26_interfaces::msg::RouteObservability::LOW);
    bool loc_control_degraded = false;
    bool optimizer_ready = true;
    std::string route_profile;
    std::string loc_reason = "unknown";

    (void)config().blackboard->get("loc_level", loc_level);
    (void)config().blackboard->get("loc_route_risk_level", route_risk);
    (void)config().blackboard->get("loc_control_degraded", loc_control_degraded);
    (void)config().blackboard->get("loc_optimizer_ready", optimizer_ready);
    (void)config().blackboard->get("loc_recommended_profile", route_profile);
    (void)config().blackboard->get("loc_reason", loc_reason);

    std::string profile_normal = "normal";
    std::string profile_yellow = "loc_yellow";
    std::string profile_orange = "loc_orange";
    std::string profile_red = "loc_red_hold";
    (void)config().blackboard->get("loc_profile_normal", profile_normal);
    (void)config().blackboard->get("loc_profile_yellow", profile_yellow);
    (void)config().blackboard->get("loc_profile_orange", profile_orange);
    (void)config().blackboard->get("loc_profile_red", profile_red);

    std::string recommended_profile = profile_normal;
    if (loc_level >= static_cast<int>(rc26_interfaces::msg::LocalizationHealth::RED) || !optimizer_ready) {
        recommended_profile = profile_red;
    } else if (loc_level >= static_cast<int>(rc26_interfaces::msg::LocalizationHealth::ORANGE) || loc_control_degraded) {
        recommended_profile = profile_orange;
    } else if (loc_level >= static_cast<int>(rc26_interfaces::msg::LocalizationHealth::YELLOW) ||
               route_risk >= static_cast<int>(rc26_interfaces::msg::RouteObservability::MEDIUM)) {
        recommended_profile = profile_yellow;
    }

    if (!route_profile.empty()) {
        recommended_profile = pickHigherPriorityProfile(recommended_profile, route_profile);
    }
    config().blackboard->set("loc_recommended_profile", recommended_profile);

    const bool route_high = route_risk >= static_cast<int>(rc26_interfaces::msg::RouteObservability::HIGH);
    const bool route_medium = route_risk >= static_cast<int>(rc26_interfaces::msg::RouteObservability::MEDIUM);
    const bool red_or_worse = loc_level >= static_cast<int>(rc26_interfaces::msg::LocalizationHealth::RED) || !optimizer_ready;
    const bool orange_with_high_risk =
        loc_level >= static_cast<int>(rc26_interfaces::msg::LocalizationHealth::ORANGE) && route_high;
    const bool degraded_with_high_risk = loc_control_degraded && route_high;

    const bool guard_required = red_or_worse || orange_with_high_risk || degraded_with_high_risk;
    config().blackboard->set("loc_guard_required", guard_required);
    if (guard_required) {
        config().blackboard->set("loc_guard_reason", loc_reason);
        return BT::NodeStatus::SUCCESS;
    }

    // 非抢占条件下，若中风险则只做外层 profile 保守，不进入恢复分支。
    if (route_medium && loc_level >= static_cast<int>(rc26_interfaces::msg::LocalizationHealth::YELLOW)) {
        config().blackboard->set("loc_guard_reason", std::string("profile_only"));
    }
    return BT::NodeStatus::FAILURE;
}

}  // namespace rc26_decision
