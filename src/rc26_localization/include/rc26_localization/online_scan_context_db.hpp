// Copyright 2025 RC2026

#pragma once

#include <cstdint>
#include <mutex>
#include <vector>

#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"

namespace rc26_localization {

struct OnlineScanContextRecord {
    uint32_t keyframe_id{0U};
    rclcpp::Time stamp;
    Eigen::Vector2d center_xy{Eigen::Vector2d::Zero()};
    Eigen::MatrixXf descriptor;
    Eigen::VectorXf ring_key;
};

class OnlineScanContextDB {
public:
    struct MatchCandidate {
        uint32_t keyframe_id{0U};
        double ring_distance{0.0};
        double similarity{-1.0};
        int sector_shift{0};
    };

    void clear();
    void addRecord(const OnlineScanContextRecord& record);
    std::vector<MatchCandidate> query(const Eigen::VectorXf& query_ring, const Eigen::MatrixXf& query_desc,
                                      int topk) const;
    size_t size() const;

private:
    static double bestSectorSimilarity(const Eigen::MatrixXf& query_desc, const Eigen::MatrixXf& target_desc,
                                       int& best_shift);

    mutable std::mutex mutex_;
    std::vector<OnlineScanContextRecord> records_;
};

}  // namespace rc26_localization
