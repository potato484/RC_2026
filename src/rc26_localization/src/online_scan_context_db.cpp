#include "rc26_localization/online_scan_context_db.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rc26_localization {

namespace {
constexpr int kSectorSearchWindow = 3;
constexpr size_t kIndexRebuildBatch = 32U;

int wrapShift(int shift, int cols) {
    if (cols <= 0) {
        return 0;
    }
    int wrapped = shift % cols;
    if (wrapped < 0) {
        wrapped += cols;
    }
    return wrapped;
}
}  // namespace

void OnlineScanContextDB::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    records_.clear();
    ring_index_.reset();
    indexed_record_count_ = 0U;
    pending_index_rebuild_ = 0U;
}

void OnlineScanContextDB::addRecord(const OnlineScanContextRecord& record) {
    if (record.ring_key.size() == 0 || record.descriptor.size() == 0) {
        return;
    }

    OnlineScanContextRecord normalized = record;
    if (normalized.sector_key.size() == 0) {
        normalized.sector_key = makeSectorKey(normalized.descriptor);
    }
    if (normalized.sector_key.size() == 0) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    records_.push_back(std::move(normalized));
    ++pending_index_rebuild_;
    if (!ring_index_ || pending_index_rebuild_ >= kIndexRebuildBatch) {
        rebuildIndexLocked();
    }
}

std::vector<OnlineScanContextDB::MatchCandidate> OnlineScanContextDB::query(
    const Eigen::VectorXf& query_ring, const Eigen::MatrixXf& query_desc, int topk) const {
    std::vector<MatchCandidate> matches;
    if (query_ring.size() == 0 || query_desc.size() == 0 || topk <= 0) {
        return matches;
    }

    std::vector<OnlineScanContextRecord> records_snapshot;
    std::shared_ptr<ScanContextRingKeyIndex> index_snapshot;
    size_t indexed_record_count_snapshot = 0U;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        records_snapshot = records_;
        index_snapshot = ring_index_;
        indexed_record_count_snapshot = std::min(indexed_record_count_, records_.size());
    }
    if (records_snapshot.empty()) {
        return matches;
    }

    std::vector<std::pair<double, size_t>> ring_ranked;
    const size_t capped_topk = std::min<size_t>(static_cast<size_t>(topk), records_snapshot.size());

    if (index_snapshot && !index_snapshot->empty() && indexed_record_count_snapshot > 0U &&
        index_snapshot->dimension() == query_ring.size()) {
        const auto neighbors = index_snapshot->knnSearch(query_ring, std::min(capped_topk, indexed_record_count_snapshot));
        ring_ranked.reserve(neighbors.size());
        for (const auto& neighbor : neighbors) {
            if (neighbor.index >= records_snapshot.size()) {
                continue;
            }
            ring_ranked.emplace_back(neighbor.distance, neighbor.index);
        }
    }

    const size_t linear_scan_begin = ring_ranked.empty() ? 0U : indexed_record_count_snapshot;
    if (linear_scan_begin < records_snapshot.size()) {
        ring_ranked.reserve(std::max(ring_ranked.size(), records_snapshot.size() - linear_scan_begin));
        for (size_t i = linear_scan_begin; i < records_snapshot.size(); ++i) {
            const auto& rec = records_snapshot[i];
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
    if (ring_ranked.size() > capped_topk) {
        ring_ranked.resize(capped_topk);
    }

    const Eigen::VectorXf query_sector = makeSectorKey(query_desc);
    matches.reserve(ring_ranked.size());
    for (const auto& [ring_distance, index] : ring_ranked) {
        if (index >= records_snapshot.size()) {
            continue;
        }
        const auto& rec = records_snapshot[index];

        const bool use_sector_seed =
            query_sector.size() > 0 && rec.sector_key.size() > 0 && query_sector.size() == rec.sector_key.size();
        const int coarse_shift = use_sector_seed ? fastAlignUsingSectorKey(query_sector, rec.sector_key) : 0;
        const int window_radius =
            use_sector_seed ? kSectorSearchWindow : std::max<int>(0, static_cast<int>(rec.descriptor.cols() / 2));

        int best_shift = coarse_shift;
        const double sim =
            bestSectorSimilarityWindowed(query_desc, rec.descriptor, coarse_shift, window_radius, best_shift);
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

Eigen::VectorXf OnlineScanContextDB::makeSectorKey(const Eigen::MatrixXf& descriptor) {
    if (descriptor.rows() == 0 || descriptor.cols() == 0) {
        return Eigen::VectorXf();
    }
    return descriptor.colwise().mean().transpose();
}

int OnlineScanContextDB::fastAlignUsingSectorKey(const Eigen::VectorXf& query_sector,
                                                 const Eigen::VectorXf& target_sector) {
    if (query_sector.size() == 0 || target_sector.size() == 0 || query_sector.size() != target_sector.size()) {
        return 0;
    }

    const int cols = query_sector.size();
    const double query_norm = std::sqrt(std::max(0.0, static_cast<double>(query_sector.squaredNorm())));
    if (query_norm <= 1e-12) {
        return 0;
    }

    int best_shift = 0;
    double best_sim = -1.0;
    for (int shift = 0; shift < cols; ++shift) {
        double dot = 0.0;
        double target_sq = 0.0;
        for (int c = 0; c < cols; ++c) {
            const double q = query_sector(c);
            const double t = target_sector((c + shift) % cols);
            dot += q * t;
            target_sq += t * t;
        }
        if (target_sq <= 1e-12) {
            continue;
        }
        const double sim = dot / (query_norm * std::sqrt(target_sq));
        if (sim > best_sim) {
            best_sim = sim;
            best_shift = shift;
        }
    }
    return best_shift;
}

double OnlineScanContextDB::bestSectorSimilarityWindowed(const Eigen::MatrixXf& query_desc,
                                                         const Eigen::MatrixXf& target_desc, int coarse_shift,
                                                         int window_radius, int& best_shift) {
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

    const int bounded_window = std::clamp(window_radius, 0, cols / 2);
    double best_sim = -1.0;
    for (int delta = -bounded_window; delta <= bounded_window; ++delta) {
        const int shift = wrapShift(coarse_shift + delta, cols);
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

void OnlineScanContextDB::rebuildIndexLocked() {
    auto index = std::make_shared<ScanContextRingKeyIndex>();
    std::vector<Eigen::VectorXf> keys;
    keys.reserve(records_.size());
    for (const auto& rec : records_) {
        keys.push_back(rec.ring_key);
    }

    if (!keys.empty() && index->build(keys)) {
        ring_index_ = std::move(index);
        indexed_record_count_ = records_.size();
    } else {
        ring_index_.reset();
        indexed_record_count_ = 0U;
    }
    pending_index_rebuild_ = 0U;
}

}  // namespace rc26_localization
