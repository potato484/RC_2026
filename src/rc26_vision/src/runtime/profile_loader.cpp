#include "rc26_vision/runtime/profile_loader.hpp"

#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <stdexcept>

namespace rc26_vision {

namespace {

std::string scalarToString(const YAML::Node& node, const std::string& fallback = "") {
    if (!node || !node.IsScalar()) {
        return fallback;
    }
    return node.as<std::string>();
}

std::string expandEnvironmentVariables(const std::string& input) {
    std::string output;
    output.reserve(input.size());

    for (std::size_t index = 0; index < input.size(); ) {
        if (input[index] != '$') {
            output.push_back(input[index++]);
            continue;
        }

        if (index + 1 < input.size() && input[index + 1] == '{') {
            const auto end = input.find('}', index + 2);
            if (end == std::string::npos) {
                output.push_back(input[index++]);
                continue;
            }

            const auto name = input.substr(index + 2, end - index - 2);
            if (const char* value = std::getenv(name.c_str())) {
                output += value;
            }
            index = end + 1;
            continue;
        }

        std::size_t end = index + 1;
        if (end >= input.size() || (!std::isalpha(static_cast<unsigned char>(input[end])) && input[end] != '_')) {
            output.push_back(input[index++]);
            continue;
        }

        while (end < input.size()) {
            const unsigned char ch = static_cast<unsigned char>(input[end]);
            if (!std::isalnum(ch) && input[end] != '_') {
                break;
            }
            ++end;
        }

        const auto name = input.substr(index + 1, end - index - 1);
        if (const char* value = std::getenv(name.c_str())) {
            output += value;
        }
        index = end;
    }

    return output;
}

std::filesystem::path resolvePath(const std::string& raw_path, const std::filesystem::path& base_dir) {
    std::string path_str = expandEnvironmentVariables(raw_path);

    if (!path_str.empty() && path_str.front() == '~') {
        if (const char* home = std::getenv("HOME")) {
            if (path_str.size() == 1) {
                path_str = home;
            } else if (path_str[1] == '/') {
                path_str = std::string(home) + path_str.substr(1);
            }
        }
    }

    std::filesystem::path resolved(path_str);
    if (resolved.is_relative()) {
        resolved = base_dir / resolved;
    }
    return resolved.lexically_normal();
}

}  // namespace

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
    const auto yaml_file = std::filesystem::absolute(yaml_path);
    const auto yaml_dir = yaml_file.parent_path();
    YAML::Node root = YAML::LoadFile(yaml_file.string());

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
        if (engine_str == "auto") {
            profile.engine = EngineType::Auto;
        } else if (engine_str == "aidlite") {
            profile.engine = EngineType::AidLite;
        } else if (engine_str == "aidlite_qnn_yolo" || engine_str == "aidlite_qnn231_yolo" ||
                   engine_str == "aidlite_qnn231") {
            profile.engine = EngineType::AidLiteQnnYolo;
        } else if (engine_str == "opencv_onnx" || engine_str == "onnxruntime") {
            profile.engine = EngineType::LocalOnnx;
        } else {
            throw std::runtime_error("Profile '" + profile.id + "' has unknown engine: " + engine_str);
        }

        profile.model_path = resolvePath(node["model_path"].as<std::string>(), yaml_dir).string();
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
            const auto labels_path = resolvePath(node["labels_path"].as<std::string>(), yaml_dir);
            profile.labels = loadLabelsFromFile(labels_path.string());
        }

        // aidlite config
        if ((profile.engine == EngineType::Auto ||
             profile.engine == EngineType::AidLite ||
             profile.engine == EngineType::AidLiteQnnYolo) && node["aidlite"]) {
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
            aidcfg.input_name = scalarToString(aidnode["input_name"]);
            aidcfg.output_name = scalarToString(aidnode["output_name"]);
            aidcfg.resize_mode = scalarToString(aidnode["resize_mode"], "stretch");
            aidcfg.padding_color = scalarToString(aidnode["padding_color"], "114");
            aidcfg.input_scale = aidnode["input_scale"].as<double>(aidcfg.input_scale);
            aidcfg.input_offset = aidnode["input_offset"].as<double>(aidcfg.input_offset);
            aidcfg.output_scale = aidnode["output_scale"].as<double>(aidcfg.output_scale);
            aidcfg.output_offset = aidnode["output_offset"].as<double>(aidcfg.output_offset);
            aidcfg.split_output_quantization =
                aidnode["split_output_quantization"].as<bool>(aidcfg.split_output_quantization);
            aidcfg.bbox_output_scale =
                aidnode["bbox_output_scale"].as<double>(aidcfg.bbox_output_scale);
            aidcfg.bbox_output_offset =
                aidnode["bbox_output_offset"].as<double>(aidcfg.bbox_output_offset);
            aidcfg.score_output_scale =
                aidnode["score_output_scale"].as<double>(aidcfg.score_output_scale);
            aidcfg.score_output_offset =
                aidnode["score_output_offset"].as<double>(aidcfg.score_output_offset);
            aidcfg.num_classes = aidnode["num_classes"].as<int>(aidcfg.num_classes);
            aidcfg.use_dsp = aidnode["use_dsp"].as<bool>(aidcfg.use_dsp);
            aidcfg.enable_cpu_fallback =
                aidnode["enable_cpu_fallback"].as<bool>(aidcfg.enable_cpu_fallback);
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
        if ((profile.engine == EngineType::AidLite ||
             profile.engine == EngineType::AidLiteQnnYolo) && !profile.aidlite) {
            throw std::runtime_error("Profile '" + id + "' missing aidlite config");
        }
    }
}

}  // namespace rc26_vision
