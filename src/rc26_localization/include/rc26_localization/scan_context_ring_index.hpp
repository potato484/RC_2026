// Copyright 2025 RC2026

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <numeric>
#include <queue>
#include <utility>
#include <vector>

#include <Eigen/Dense>

namespace rc26_localization {

// 轻量 ring_key KD-tree（header-only），用于静态/在线 Scan Context 近邻召回。
class ScanContextRingKeyIndex {
public:
    struct Neighbor {
        size_t index{0U};
        double distance{0.0};
    };

    bool build(const std::vector<Eigen::VectorXf>& keys) {
        clear();
        if (keys.empty()) {
            return false;
        }

        const int dim = keys.front().size();
        if (dim <= 0) {
            return false;
        }
        for (const auto& key : keys) {
            if (key.size() != dim) {
                clear();
                return false;
            }
        }

        dimension_ = dim;
        points_ = keys;

        std::vector<size_t> indices(points_.size(), 0U);
        std::iota(indices.begin(), indices.end(), 0U);
        root_ = buildRecursive(indices.begin(), indices.end(), 0);
        return root_ != nullptr;
    }

    void clear() {
        root_.reset();
        points_.clear();
        dimension_ = 0;
    }

    bool empty() const {
        return !root_ || points_.empty() || dimension_ <= 0;
    }

    int dimension() const {
        return dimension_;
    }

    std::vector<Neighbor> knnSearch(const Eigen::VectorXf& query, size_t k) const {
        std::vector<Neighbor> neighbors;
        if (k == 0 || empty() || query.size() != dimension_) {
            return neighbors;
        }

        const size_t query_k = std::min(k, points_.size());
        std::priority_queue<HeapEntry, std::vector<HeapEntry>, HeapCompare> heap;
        knnSearchRecursive(root_.get(), query, query_k, heap);

        neighbors.reserve(heap.size());
        while (!heap.empty()) {
            const auto [distance_sq, index] = heap.top();
            heap.pop();
            neighbors.push_back(Neighbor{index, std::sqrt(std::max(0.0, distance_sq))});
        }
        std::reverse(neighbors.begin(), neighbors.end());
        return neighbors;
    }

private:
    struct Node {
        size_t point_index{0U};
        int axis{0};
        std::unique_ptr<Node> left;
        std::unique_ptr<Node> right;
    };

    using HeapEntry = std::pair<double, size_t>;  // (distance_sq, index)
    struct HeapCompare {
        bool operator()(const HeapEntry& lhs, const HeapEntry& rhs) const {
            return lhs.first < rhs.first;
        }
    };

    std::unique_ptr<Node> buildRecursive(std::vector<size_t>::iterator begin,
                                         std::vector<size_t>::iterator end,
                                         int depth) {
        if (begin >= end || dimension_ <= 0) {
            return nullptr;
        }

        const auto count = static_cast<size_t>(std::distance(begin, end));
        const auto mid = begin + static_cast<std::ptrdiff_t>(count / 2U);
        const int axis = depth % dimension_;
        std::nth_element(begin, mid, end, [this, axis](size_t lhs, size_t rhs) {
            return points_[lhs](axis) < points_[rhs](axis);
        });

        auto node = std::make_unique<Node>();
        node->point_index = *mid;
        node->axis = axis;
        node->left = buildRecursive(begin, mid, depth + 1);
        node->right = buildRecursive(mid + 1, end, depth + 1);
        return node;
    }

    static double squaredDistance(const Eigen::VectorXf& lhs, const Eigen::VectorXf& rhs) {
        return static_cast<double>((lhs - rhs).squaredNorm());
    }

    void knnSearchRecursive(const Node* node, const Eigen::VectorXf& query, size_t k,
                            std::priority_queue<HeapEntry, std::vector<HeapEntry>, HeapCompare>& heap) const {
        if (!node) {
            return;
        }

        const auto& point = points_[node->point_index];
        const double distance_sq = squaredDistance(query, point);
        if (heap.size() < k) {
            heap.emplace(distance_sq, node->point_index);
        } else if (distance_sq < heap.top().first) {
            heap.pop();
            heap.emplace(distance_sq, node->point_index);
        }

        const double axis_delta = static_cast<double>(query(node->axis) - point(node->axis));
        const Node* near_branch = (axis_delta <= 0.0) ? node->left.get() : node->right.get();
        const Node* far_branch = (axis_delta <= 0.0) ? node->right.get() : node->left.get();

        knnSearchRecursive(near_branch, query, k, heap);

        const double worst_dist = heap.empty() ? std::numeric_limits<double>::infinity() : heap.top().first;
        if (far_branch && (heap.size() < k || axis_delta * axis_delta < worst_dist)) {
            knnSearchRecursive(far_branch, query, k, heap);
        }
    }

    int dimension_{0};
    std::vector<Eigen::VectorXf> points_;
    std::unique_ptr<Node> root_;
};

}  // namespace rc26_localization
