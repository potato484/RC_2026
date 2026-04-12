#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rc26_xhu_nav/mode_manager/profile_types.hpp"

namespace rc26_xhu_nav::mode_manager {

class ProfileLoader {
public:
    struct LoadResult {
        bool success{false};
        std::string error_message;
        std::unordered_map<std::string, NavProfile> profiles;
    };

    static LoadResult loadFromFile(const std::string& file_path);

private:
    static bool validateProfile(const NavProfile& profile, std::string& error);
    static bool detectCycle(
        const std::unordered_map<std::string, NavProfile>& profiles,
        std::string& error);
    static bool dfsDetectCycle(
        const std::string& current,
        const std::unordered_map<std::string, NavProfile>& profiles,
        std::unordered_set<std::string>& visiting,
        std::unordered_set<std::string>& visited);
};

}  // namespace rc26_xhu_nav::mode_manager
