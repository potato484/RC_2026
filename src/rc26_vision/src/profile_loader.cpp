#include "rc26_vision/profile_loader.hpp"

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdexcept>
#include <filesystem>

namespace rc26_vision {

std::vector<std::string> ProfileLoader::loadLabelsFromFile(const std::string& path) {
    std::vector<std::string> labels;
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open labels file: " + path);
    }
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty()) {
            labels.push_back(line);
        }
    }
    return labels;
}

VisionConfig ProfileLoader::loadFromYaml(const std::string& yaml_path) {
    VisionConfig config;
    YAML::Node root = YAML::LoadFile(yaml_path);

    if (!root["vision"]) {
        throw std::runtime_error("Missing 'vision' section in config");
    }
    auto vision = root["vision"];

    if (vision["default_model"]) {
        config.default_model = vision["default_model"].as<std::string>();
    }

    if (!vision["models"]) {
        throw std::runtime_error("Missing 'vision.models' section");
    }

    for (const auto& item : vision["models"]) {
        ModelProfile profile;
        profile.id = item.first.as<std::string>();
        auto node = item.second;

        // engine type (required)
        if (!node["engine"]) {
            throw std::runtime_error("Profile '" + profile.id + "' missing required field: engine");
        }
        std::string engine_str = node["engine"].as<std::string>();
        if (engine_str == "aidlite") {
            profile.engine = EngineType::AidLite;
        } else if (engine_str == "onnxruntime") {
            profile.engine = EngineType::OnnxRuntime;
        } else {
            throw std::runtime_error("Profile '" + profile.id + "' has unknown engine: " + engine_str);
        }

        profile.model_path = node["model_path"].as<std::string>();
        if (!node["conf_thresh"]) {
            throw std::runtime_error("Profile '" + profile.id + "' missing required field: conf_thresh");
        }
        profile.conf_thresh = node["conf_thresh"].as<float>();
        profile.iou_thresh = node["iou_thresh"].as<float>(0.45f);
        profile.input_w = node["input_w"].as<int>(640);
        profile.input_h = node["input_h"].as<int>(640);

        // labels
        if (node["labels"]) {
            profile.labels = node["labels"].as<std::vector<std::string>>();
        } else if (node["labels_path"]) {
            profile.labels = loadLabelsFromFile(node["labels_path"].as<std::string>());
        }

        // aidlite config
        if (profile.engine == EngineType::AidLite && node["aidlite"]) {
            AidLiteConfig aidcfg;
            auto aidnode = node["aidlite"];
            aidcfg.framework_type = aidnode["framework_type"].as<std::string>();
            aidcfg.accelerate_type = aidnode["accelerate_type"].as<std::string>();
            if (aidnode["input_shape"]) {
                aidcfg.input_shape = aidnode["input_shape"].as<std::vector<int>>();
            }
            if (aidnode["output_shapes"]) {
                for (const auto& shape : aidnode["output_shapes"]) {
                    aidcfg.output_shapes.push_back(shape.as<std::vector<int>>());
                }
            }
            profile.aidlite = aidcfg;
        }

        config.profiles[profile.id] = std::move(profile);
    }

    return config;
}

void ProfileLoader::validate(const VisionConfig& config) {
    if (config.profiles.empty()) {
        throw std::runtime_error("No model profiles defined");
    }

    if (!config.default_model.empty()) {
        if (config.profiles.find(config.default_model) == config.profiles.end()) {
            throw std::runtime_error("default_model '" + config.default_model + "' not found");
        }
    }

    for (const auto& [id, profile] : config.profiles) {
        if (profile.model_path.empty()) {
            throw std::runtime_error("Profile '" + id + "' missing model_path");
        }
        if (profile.labels.empty()) {
            throw std::runtime_error("Profile '" + id + "' missing labels");
        }
        if (profile.input_w <= 0 || profile.input_h <= 0) {
            throw std::runtime_error("Profile '" + id + "' has invalid input size");
        }
        if (profile.engine == EngineType::AidLite && !profile.aidlite) {
            throw std::runtime_error("Profile '" + id + "' missing aidlite config");
        }
    }
}

}  // namespace rc26_vision
