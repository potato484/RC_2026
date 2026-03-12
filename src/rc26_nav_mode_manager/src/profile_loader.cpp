#include "rc26_nav_mode_manager/profile_loader.hpp"

#include <cmath>
#include <fstream>

#include <yaml-cpp/yaml.h>

namespace rc26_nav_mode_manager {

namespace {

bool assignCompatibleControllerLimit(const YAML::Node& ctrl,
                                     const std::string& profile_name,
                                     const char* legacy_key,
                                     const char* canonical_key,
                                     std::optional<double>& output,
                                     std::string& error) {
    const auto legacy = ctrl[legacy_key];
    const auto canonical = ctrl[canonical_key];
    if (!legacy && !canonical) {
        return true;
    }

    if (legacy && canonical) {
        const double legacy_value = legacy.as<double>();
        const double canonical_value = canonical.as<double>();
        if (std::abs(legacy_value - canonical_value) > 1e-9) {
            error = "Profile '" + profile_name + "' has conflicting controller fields '" +
                    std::string(legacy_key) + "' and '" + canonical_key + "'";
            return false;
        }
        output = canonical_value;
        return true;
    }

    output = canonical ? canonical.as<double>() : legacy.as<double>();
    return true;
}

}  // namespace

bool ProfileLoader::validateProfile(const NavProfile& profile, std::string& error) {
    if (profile.fallback_profile.empty()) {
        error = "Profile '" + profile.name + "' missing required field 'fallback_profile'";
        return false;
    }

    if (profile.watchdog.timeout_sec < 0) {
        error = "Profile '" + profile.name + "' has invalid timeout_sec (must be >= 0)";
        return false;
    }

    return true;
}

bool ProfileLoader::dfsDetectCycle(
    const std::string& current,
    const std::unordered_map<std::string, NavProfile>& profiles,
    std::unordered_set<std::string>& visiting,
    std::unordered_set<std::string>& visited) {
    if (visited.count(current)) return false;
    if (visiting.count(current)) return true;

    visiting.insert(current);

    auto it = profiles.find(current);
    if (it != profiles.end()) {
        const auto& fallback = it->second.fallback_profile;
        if (fallback != current && dfsDetectCycle(fallback, profiles, visiting, visited)) {
            return true;
        }
    }

    visiting.erase(current);
    visited.insert(current);
    return false;
}

bool ProfileLoader::detectCycle(
    const std::unordered_map<std::string, NavProfile>& profiles,
    std::string& error) {
    std::unordered_set<std::string> visiting, visited;

    for (const auto& [name, profile] : profiles) {
        if (dfsDetectCycle(name, profiles, visiting, visited)) {
            error = "Cycle detected in fallback chain involving '" + name + "'";
            return true;
        }
    }
    return false;
}

ProfileLoader::LoadResult ProfileLoader::loadFromFile(const std::string& file_path) {
    LoadResult result;

    YAML::Node root;
    try {
        root = YAML::LoadFile(file_path);
    } catch (const YAML::Exception& e) {
        result.error_message = "Failed to parse YAML: " + std::string(e.what());
        return result;
    }

    if (!root["profiles"]) {
        result.error_message = "Missing 'profiles' key in config";
        return result;
    }

    for (const auto& item : root["profiles"]) {
        std::string name;
        try {
            name = item.first.as<std::string>();
        } catch (const YAML::Exception& e) {
            result.error_message = "Invalid profile name: " + std::string(e.what());
            return result;
        }
        const auto& node = item.second;

        NavProfile profile;
        profile.name = name;

        try {
            if (!node["fallback_profile"]) {
                result.error_message = "Profile '" + name + "' missing 'fallback_profile'";
                return result;
            }
            profile.fallback_profile = node["fallback_profile"].as<std::string>();

            if (node["watchdog"]) {
                const auto& wd = node["watchdog"];
                if (wd["timeout_sec"]) {
                    profile.watchdog.timeout_sec = wd["timeout_sec"].as<double>();
                }
                if (wd["stop_required_on_timeout"]) {
                    profile.watchdog.stop_required_on_timeout = wd["stop_required_on_timeout"].as<bool>();
                }
            }

            if (node["precheck"]) {
                const auto& pc = node["precheck"];
                if (pc["require_stopped"]) {
                    profile.precheck.require_stopped = pc["require_stopped"].as<bool>();
                }
            }

            if (node["costmap"]) {
                const auto& cm = node["costmap"];
                if (cm["clear_on_switch"]) {
                    profile.costmap.clear_on_switch = cm["clear_on_switch"].as<bool>();
                }
            }

            if (node["controller"]) {
                const auto& ctrl = node["controller"];
                if (ctrl["v_linear_max"]) profile.controller.v_linear_max = ctrl["v_linear_max"].as<double>();
                if (ctrl["v_angular_max"]) profile.controller.v_angular_max = ctrl["v_angular_max"].as<double>();
                if (ctrl["v_linear_min"]) profile.controller.v_linear_min = ctrl["v_linear_min"].as<double>();
                std::string compatibility_error;
                if (!assignCompatibleControllerLimit(ctrl,
                                                     name,
                                                     "acc_linear",
                                                     "a_linear_max",
                                                     profile.controller.acc_linear,
                                                     compatibility_error)) {
                    result.error_message = compatibility_error;
                    return result;
                }
                if (!assignCompatibleControllerLimit(ctrl,
                                                     name,
                                                     "acc_angular",
                                                     "a_angular_max",
                                                     profile.controller.acc_angular,
                                                     compatibility_error)) {
                    result.error_message = compatibility_error;
                    return result;
                }
                if (ctrl["transition_timeout_ms"]) {
                    int v = ctrl["transition_timeout_ms"].as<int>();
                    profile.controller.transition_timeout_ms = (v > 0) ? v : 500;
                }
            }
        } catch (const YAML::Exception& e) {
            result.error_message = "Profile '" + name + "' has invalid field type: " + std::string(e.what());
            return result;
        }

        std::string error;
        if (!validateProfile(profile, error)) {
            result.error_message = error;
            return result;
        }

        result.profiles[name] = std::move(profile);
    }

    // Check safe profile exists and self-references
    auto safe_it = result.profiles.find("safe");
    if (safe_it == result.profiles.end()) {
        result.error_message = "Required 'safe' profile not found";
        return result;
    }
    if (safe_it->second.fallback_profile != "safe") {
        result.error_message = "'safe' profile must have fallback_profile pointing to itself";
        return result;
    }

    // Check all fallback_profile references exist
    for (const auto& [name, profile] : result.profiles) {
        if (result.profiles.find(profile.fallback_profile) == result.profiles.end()) {
            result.error_message = "Profile '" + name + "' references unknown fallback '" + profile.fallback_profile + "'";
            return result;
        }
    }

    // Detect cycles
    std::string cycle_error;
    if (detectCycle(result.profiles, cycle_error)) {
        result.error_message = cycle_error;
        return result;
    }

    result.success = true;
    return result;
}

}  // namespace rc26_nav_mode_manager
