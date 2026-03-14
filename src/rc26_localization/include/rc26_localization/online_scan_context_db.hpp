// Copyright 2025 RC2026

#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include <Eigen/Dense>
#include "rc26_localization/scan_context_ring_index.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rc26_localization {

struct OnlineScanContextRecord {
    uint32_t keyframe_id{0U};
    rclcpp::Time stamp;
    Eigen::Vector2d center_xy{Eigen::Vector2d::Zero()};
    Eigen::MatrixXf descriptor;
    Eigen::VectorXf ring_key;
    Eigen::VectorXf sector_key;
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
    static Eigen::VectorXf makeSectorKey(const Eigen::MatrixXf& descriptor);
    static int fastAlignUsingSectorKey(const Eigen::VectorXf& query_sector, const Eigen::VectorXf& target_sector);
    static double bestSectorSimilarityWindowed(const Eigen::MatrixXf& query_desc, const Eigen::MatrixXf& target_desc,
                                               int coarse_shift, int window_radius, int& best_shift);
    void rebuildIndexLocked();

    mutable std::mutex mutex_;
    std::vector<OnlineScanContextRecord> records_;
    std::shared_ptr<ScanContextRingKeyIndex> ring_index_;
    size_t indexed_record_count_{0U};
    size_t pending_index_rebuild_{0U};
};

}  // namespace rc26_localization
