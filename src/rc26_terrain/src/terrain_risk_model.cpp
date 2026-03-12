#include "rc26_terrain/terrain_risk_model.hpp"

#include <algorithm>
#include <cmath>

#include "yaml-cpp/yaml.h"

namespace rc26_terrain {

bool TerrainRiskModel::loadFromFile(const std::string& path, std::string& error) {
    error.clear();
    try {
        YAML::Node root = YAML::LoadFile(path);
        if (root["terrain_risk_model"]) {
            root = root["terrain_risk_model"];
        }
        if (!root || !root.IsMap()) {
            error = "terrain_risk_model root missing or invalid";
            enabled_ = false;
            return false;
        }

        enabled_ = root["enabled"] ? root["enabled"].as<bool>() : true;
        auto load_head = [](const YAML::Node& node) {
            LogisticHead head;
            if (!node || !node.IsMap()) {
                return head;
            }
            head.enabled = node["enabled"] ? node["enabled"].as<bool>() : true;
            head.intercept = node["intercept"] ? node["intercept"].as<double>() : 0.0;
            const auto coeff_node = node["coefficients"];
            if (coeff_node && coeff_node.IsMap()) {
                for (const auto it : coeff_node) {
                    const std::string key = it.first.as<std::string>();
                    head.coefficients[key] = it.second.as<double>();
                }
            }
            return head;
        };
        obstacle_ = load_head(root["obstacle"]);
        drop_ = load_head(root["drop"]);

        if (!obstacle_.enabled && !drop_.enabled) {
            enabled_ = false;
        }
        return true;
    } catch (const std::exception& ex) {
        error = ex.what();
        enabled_ = false;
        obstacle_ = LogisticHead{};
        drop_ = LogisticHead{};
        return false;
    }
}

bool TerrainRiskModel::enabled() const {
    return enabled_;
}

float TerrainRiskModel::predictObstacle(const TerrainRiskFeatures& features, float fallback_prob) const {
    return predict(obstacle_, features, fallback_prob);
}

float TerrainRiskModel::predictDrop(const TerrainRiskFeatures& features, float fallback_prob) const {
    return predict(drop_, features, fallback_prob);
}

float TerrainRiskModel::sigmoid(double value) {
    if (value >= 30.0) {
        return 1.0f;
    }
    if (value <= -30.0) {
        return 0.0f;
    }
    return static_cast<float>(1.0 / (1.0 + std::exp(-value)));
}

float TerrainRiskModel::clampProb(float value) {
    if (!std::isfinite(static_cast<double>(value))) {
        return 0.0f;
    }
    return std::clamp(value, 0.0f, 1.0f);
}

double TerrainRiskModel::featureValue(const TerrainRiskFeatures& features, const std::string& name) {
    if (name == "slope_abs") {
        return static_cast<double>(features.slope_abs);
    }
    if (name == "roughness") {
        return static_cast<double>(features.roughness);
    }
    if (name == "sigma_h") {
        return static_cast<double>(features.sigma_h);
    }
    if (name == "step_up") {
        return static_cast<double>(features.step_up);
    }
    if (name == "height_span") {
        return static_cast<double>(features.height_span);
    }
    if (name == "obstacle_proxy") {
        return static_cast<double>(features.obstacle_proxy);
    }
    if (name == "drop_proxy") {
        return static_cast<double>(features.drop_proxy);
    }
    if (name == "climbable_prob") {
        return static_cast<double>(features.climbable_prob);
    }
    return 0.0;
}

float TerrainRiskModel::predict(const LogisticHead& head,
                                const TerrainRiskFeatures& features,
                                float fallback_prob) const {
    if (!enabled_ || !head.enabled || head.coefficients.empty()) {
        return clampProb(fallback_prob);
    }

    double logit = head.intercept;
    for (const auto& [name, coeff] : head.coefficients) {
        logit += coeff * featureValue(features, name);
    }
    return clampProb(sigmoid(logit));
}

}  // namespace rc26_terrain
