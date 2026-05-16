#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "rc26_vision/runtime/model_profile.hpp"

namespace rc26_vision {

struct VisionConfig {
    std::string default_model;
    std::unordered_map<std::string, ModelProfile> profiles;
};

class ProfileLoader {
public:
    static VisionConfig loadFromYaml(const std::string& yaml_path);
    static std::vector<std::string> loadLabelsFromFile(const std::string& path);
    static void validate(const VisionConfig& config);
};

}  // namespace rc26_vision
