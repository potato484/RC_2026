#include "rc26_xhu_nav/mode_manager/profile_db.hpp"

namespace rc26_xhu_nav::mode_manager {

void ProfileDB::load(std::unordered_map<std::string, NavProfile> profiles) {
    profiles_ = std::move(profiles);
}

std::optional<NavProfile> ProfileDB::get(const std::string& name) const {
    auto it = profiles_.find(name);
    if (it != profiles_.end()) {
        return it->second;
    }
    return std::nullopt;
}

bool ProfileDB::exists(const std::string& name) const {
    return profiles_.count(name) > 0;
}

std::vector<std::string> ProfileDB::getAllNames() const {
    std::vector<std::string> names;
    names.reserve(profiles_.size());
    for (const auto& [name, _] : profiles_) {
        names.push_back(name);
    }
    return names;
}

}  // namespace rc26_xhu_nav::mode_manager
