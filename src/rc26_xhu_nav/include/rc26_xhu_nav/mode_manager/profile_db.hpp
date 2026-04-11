#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "rc26_xhu_nav/mode_manager/profile_types.hpp"

namespace rc26_xhu_nav::mode_manager {

class ProfileDB {
public:
    void load(std::unordered_map<std::string, NavProfile> profiles);
    std::optional<NavProfile> get(const std::string& name) const;
    bool exists(const std::string& name) const;
    std::vector<std::string> getAllNames() const;

private:
    std::unordered_map<std::string, NavProfile> profiles_;
};

}  // namespace rc26_xhu_nav::mode_manager
