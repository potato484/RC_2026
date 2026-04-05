#include "rc26_localization/route_observability_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Eigenvalues>

namespace rc26_localization {

RouteObservabilityEvaluator::RouteObservabilityEvaluator() = default;

RouteObservabilityEvaluator::RouteObservabilityEvaluator(const Config& cfg) : config_(cfg) {}

void RouteObservabilityEvaluator::setConfig(const Config& cfg) {
    config_ = cfg;
}

RouteObservabilityResult RouteObservabilityEvaluator::evaluate(const RouteObservabilityInput& input) const {
    RouteObservabilityResult result;
    if (input.window_points_map.size() < 2) {
        result.score = 0.5;
        result.risk_level = 1U;
        result.repeat_structure_risk = 0.5;
        result.dynamic_risk = input.control_degraded ? 0.8 : 0.4;
        result.loop_opportunity_score = 0.0;
        result.anchor_opportunity_score = input.uwb_available ? 0.7 : 0.0;
        result.recommended_nav_profile = (result.dynamic_risk > 0.6) ? "loc_orange" : "loc_yellow";
        return result;
    }

    const double anisotropy = computePathAnisotropy(input.window_points_map);
    const double mean_nearest_distance = computeMeanNearestDistance(input.window_points_map, input.map_points_xy);
    const double density_score = 1.0 - clamp01(mean_nearest_distance / std::max(1e-3, config_.map_near_dist_m * 2.0));

    result.repeat_structure_risk = clamp01(0.65 * anisotropy + 0.35 * (1.0 - density_score));
    result.dynamic_risk = clamp01((input.control_degraded ? 0.55 : 0.2) + (input.imu_spike ? 0.25 : 0.0) +
                                  0.2 * anisotropy);

    if (input.graph_status_valid && input.last_loop_age_sec >= 0.0) {
        result.loop_opportunity_score =
            clamp01(1.0 - input.last_loop_age_sec / std::max(1e-3, config_.loop_recent_age_sec));
    } else {
        result.loop_opportunity_score = computePathLoopHint(input.window_points_map);
    }

    if (input.retry_zone_enable) {
        const double min_dist_retry = computeMinDistanceToPoint(input.window_points_map, input.retry_zone_xy);
        result.anchor_opportunity_score =
            std::exp(-min_dist_retry / std::max(1e-3, config_.anchor_effective_dist_m));
    } else {
        result.anchor_opportunity_score = 0.0;
    }
    if (input.uwb_available) {
        result.anchor_opportunity_score = std::max(result.anchor_opportunity_score, 0.7);
    }
    result.anchor_opportunity_score = clamp01(result.anchor_opportunity_score);

    const double risk_raw =
        0.45 * result.repeat_structure_risk + 0.35 * result.dynamic_risk +
        0.20 * (1.0 - result.loop_opportunity_score) - 0.15 * result.anchor_opportunity_score;
    const double risk = clamp01(risk_raw);
    result.score = clamp01(1.0 - risk);

    if (risk >= config_.high_risk_threshold) {
        result.risk_level = 2U;
        result.recommended_nav_profile = "loc_orange";
    } else if (risk >= config_.medium_risk_threshold) {
        result.risk_level = 1U;
        result.recommended_nav_profile = "loc_yellow";
    } else {
        result.risk_level = 0U;
        result.recommended_nav_profile = "normal";
    }

    return result;
}

double RouteObservabilityEvaluator::clamp01(const double value) {
    return std::clamp(value, 0.0, 1.0);
}

double RouteObservabilityEvaluator::computePathAnisotropy(const std::vector<Eigen::Vector2d>& points) {
    if (points.size() < 2) {
        return 0.0;
    }

    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (const auto& pt : points) {
        mean += pt;
    }
    mean /= static_cast<double>(points.size());

    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (const auto& pt : points) {
        const Eigen::Vector2d centered = pt - mean;
        cov.noalias() += centered * centered.transpose();
    }
    cov /= static_cast<double>(std::max<size_t>(1, points.size() - 1));

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
    if (solver.info() != Eigen::Success) {
        return 0.0;
    }

    const Eigen::Vector2d eig = solver.eigenvalues();
    const double max_eig = std::max(eig(1), 1e-9);
    const double min_eig = std::max(eig(0), 0.0);
    return clamp01(1.0 - min_eig / max_eig);
}

double RouteObservabilityEvaluator::computeMeanNearestDistance(const std::vector<Eigen::Vector2d>& samples,
                                                               const std::vector<Eigen::Vector2d>& map_points) {
    if (samples.empty() || map_points.empty()) {
        return std::numeric_limits<double>::infinity();
    }

    double acc = 0.0;
    for (const auto& sample : samples) {
        double best_sq = std::numeric_limits<double>::infinity();
        for (const auto& map_pt : map_points) {
            const double dx = sample.x() - map_pt.x();
            const double dy = sample.y() - map_pt.y();
            const double dist_sq = dx * dx + dy * dy;
            if (dist_sq < best_sq) {
                best_sq = dist_sq;
            }
        }
        acc += std::sqrt(best_sq);
    }
    return acc / static_cast<double>(samples.size());
}

double RouteObservabilityEvaluator::computePathLoopHint(const std::vector<Eigen::Vector2d>& points) {
    if (points.size() < 3) {
        return 0.0;
    }

    const double end_to_start = (points.back() - points.front()).norm();
    const double compactness = std::exp(-end_to_start / 2.0);
    return clamp01(compactness);
}

double RouteObservabilityEvaluator::computeMinDistanceToPoint(const std::vector<Eigen::Vector2d>& points,
                                                              const Eigen::Vector2d& target) {
    if (points.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    double best_sq = std::numeric_limits<double>::infinity();
    for (const auto& pt : points) {
        const double dx = pt.x() - target.x();
        const double dy = pt.y() - target.y();
        const double dist_sq = dx * dx + dy * dy;
        if (dist_sq < best_sq) {
            best_sq = dist_sq;
        }
    }
    return std::sqrt(best_sq);
}

}  // namespace rc26_localization
