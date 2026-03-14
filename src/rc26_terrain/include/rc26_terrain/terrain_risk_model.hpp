#pragma once

#include <string>
#include <unordered_map>

namespace rc26_terrain {

struct TerrainRiskFeatures {
    float slope_abs{0.0f};
    float roughness{0.0f};
    float sigma_h{0.0f};
    float step_up{0.0f};
    float height_span{0.0f};
    float obstacle_proxy{0.0f};
    float drop_proxy{0.0f};
    float climbable_prob{0.0f};
};

class TerrainRiskModel {
public:
    bool loadFromFile(const std::string& path, std::string& error);
    bool enabled() const;
    float predictObstacle(const TerrainRiskFeatures& features, float fallback_prob) const;
    float predictDrop(const TerrainRiskFeatures& features, float fallback_prob) const;

private:
    struct LogisticHead {
        bool enabled{false};
        double intercept{0.0};
        std::unordered_map<std::string, double> coefficients;
    };

    static float sigmoid(double value);
    static float clampProb(float value);
    static double featureValue(const TerrainRiskFeatures& features, const std::string& name);
    float predict(const LogisticHead& head,
                  const TerrainRiskFeatures& features,
                  float fallback_prob) const;

    bool enabled_{false};
    LogisticHead obstacle_;
    LogisticHead drop_;
};

}  // namespace rc26_terrain
