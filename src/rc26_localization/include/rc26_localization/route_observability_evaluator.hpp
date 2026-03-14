// Copyright 2026 RC2026

#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

namespace rc26_localization {

struct RouteObservabilityInput {
    std::vector<Eigen::Vector2d> window_points_map;
    std::vector<Eigen::Vector2d> map_points_xy;
    bool control_degraded{false};
    bool imu_spike{false};
    bool uwb_available{false};
    bool retry_zone_enable{false};
    Eigen::Vector2d retry_zone_xy{Eigen::Vector2d::Zero()};
    bool graph_status_valid{false};
    double last_loop_age_sec{-1.0};
};

struct RouteObservabilityResult {
    double score{1.0};
    uint8_t risk_level{0U};  // 0=LOW 1=MEDIUM 2=HIGH
    double repeat_structure_risk{0.0};
    double dynamic_risk{0.0};
    double loop_opportunity_score{0.0};
    double anchor_opportunity_score{0.0};
    std::string recommended_nav_profile{"normal"};
};

class RouteObservabilityEvaluator {
public:
    struct Config {
        double map_near_dist_m{0.7};
        double medium_risk_threshold{0.45};
        double high_risk_threshold{0.7};
        double anchor_effective_dist_m{2.0};
        double loop_recent_age_sec{8.0};
    };

    RouteObservabilityEvaluator();
    explicit RouteObservabilityEvaluator(const Config& cfg);

    void setConfig(const Config& cfg);
    const Config& config() const { return config_; }

    RouteObservabilityResult evaluate(const RouteObservabilityInput& input) const;

private:
    static double clamp01(double value);
    static double computePathAnisotropy(const std::vector<Eigen::Vector2d>& points);
    static double computeMeanNearestDistance(const std::vector<Eigen::Vector2d>& samples,
                                             const std::vector<Eigen::Vector2d>& map_points);
    static double computePathLoopHint(const std::vector<Eigen::Vector2d>& points);
    static double computeMinDistanceToPoint(const std::vector<Eigen::Vector2d>& points,
                                            const Eigen::Vector2d& target);

    Config config_;
};

}  // namespace rc26_localization
