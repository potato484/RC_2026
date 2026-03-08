#include "rc26_localization/online_scan_context_db.hpp"

#include <algorithm>
#include <cmath>

namespace rc26_localization {

void OnlineScanContextDB::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    records_.clear();
}

void OnlineScanContextDB::addRecord(const OnlineScanContextRecord& record) {
    if (record.ring_key.size() == 0 || record.descriptor.size() == 0) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    records_.push_back(record);
}

std::vector<OnlineScanContextDB::MatchCandidate> OnlineScanContextDB::query(
    const Eigen::VectorXf& query_ring, const Eigen::MatrixXf& query_desc, int topk) const {
    std::vector<MatchCandidate> matches;
    if (query_ring.size() == 0 || query_desc.size() == 0 || topk <= 0) {
        return matches;
    }

    std::vector<std::pair<double, size_t>> ring_ranked;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ring_ranked.reserve(records_.size());
        for (size_t i = 0; i < records_.size(); ++i) {
            const auto& rec = records_[i];
            if (rec.ring_key.size() != query_ring.size()) {
                continue;
            }
            const double ring_distance = (rec.ring_key - query_ring).norm();
            ring_ranked.emplace_back(ring_distance, i);
        }
    }

    if (ring_ranked.empty()) {
        return matches;
    }

    std::sort(ring_ranked.begin(), ring_ranked.end(),
              [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
    if (static_cast<int>(ring_ranked.size()) > topk) {
        ring_ranked.resize(static_cast<size_t>(topk));
    }

    matches.reserve(ring_ranked.size());
    for (const auto& [ring_distance, index] : ring_ranked) {
        OnlineScanContextRecord rec;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (index >= records_.size()) {
                continue;
            }
            rec = records_[index];
        }

        int best_shift = 0;
        const double sim = bestSectorSimilarity(query_desc, rec.descriptor, best_shift);
        if (sim < 0.0) {
            continue;
        }
        matches.push_back(MatchCandidate{
            rec.keyframe_id,
            ring_distance,
            sim,
            best_shift,
        });
    }

    std::sort(matches.begin(), matches.end(), [](const MatchCandidate& lhs, const MatchCandidate& rhs) {
        if (std::abs(lhs.similarity - rhs.similarity) > 1e-9) {
            return lhs.similarity > rhs.similarity;
        }
        return lhs.ring_distance < rhs.ring_distance;
    });
    return matches;
}

size_t OnlineScanContextDB::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return records_.size();
}

double OnlineScanContextDB::bestSectorSimilarity(const Eigen::MatrixXf& query_desc, const Eigen::MatrixXf& target_desc,
                                                 int& best_shift) {
    best_shift = 0;
    if (query_desc.rows() != target_desc.rows() || query_desc.cols() != target_desc.cols() || query_desc.size() == 0) {
        return -1.0;
    }

    const int rows = query_desc.rows();
    const int cols = query_desc.cols();

    double query_sq = 0.0;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            const double q = query_desc(r, c);
            query_sq += q * q;
        }
    }
    if (query_sq <= 1e-12) {
        return -1.0;
    }

    double best_sim = -1.0;
    for (int shift = 0; shift < cols; ++shift) {
        double dot = 0.0;
        double target_sq = 0.0;
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                const double q = query_desc(r, c);
                const double t = target_desc(r, (c + shift) % cols);
                dot += q * t;
                target_sq += t * t;
            }
        }

        if (target_sq <= 1e-12) {
            continue;
        }

        const double sim = dot / (std::sqrt(query_sq) * std::sqrt(target_sq));
        if (sim > best_sim) {
            best_sim = sim;
            best_shift = shift;
        }
    }
    return best_sim;
}

}  // namespace rc26_localization
