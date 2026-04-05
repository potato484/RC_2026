#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include "small_gicp/pcl/pcl_point.hpp"

namespace rc26_localization {

class StaticVoxelFilter {
public:
    struct Config {
        double voxel_size{0.3};
        int window_size{10};
        int stable_threshold{5};
    };

    struct VoxelKey {
        int x{0};
        int y{0};
        int z{0};

        bool operator==(const VoxelKey& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct VoxelKeyHash {
        std::size_t operator()(const VoxelKey& key) const noexcept {
            const std::size_t h1 = std::hash<int>{}(key.x);
            const std::size_t h2 = std::hash<int>{}(key.y);
            const std::size_t h3 = std::hash<int>{}(key.z);
            return h1 ^ (h2 << 1U) ^ (h3 << 2U);
        }
    };

    StaticVoxelFilter() = default;

    explicit StaticVoxelFilter(const Config& cfg) : cfg_(cfg) {}

    void setConfig(const Config& cfg) {
        cfg_ = cfg;
        reset();
    }

    void reset() {
        stability_map_.clear();
        frame_window_.clear();
    }

    void update(const pcl::PointCloud<pcl::PointCovariance>::Ptr& cloud, const Eigen::Matrix4d& T_map_odom) {
        if (!cloud || cloud->empty() || cfg_.voxel_size <= 0.0) {
            return;
        }

        std::unordered_set<VoxelKey, VoxelKeyHash> frame_keys;
        frame_keys.reserve(cloud->size());
        for (const auto& pt : cloud->points) {
            const Eigen::Vector4d p_odom(pt.x, pt.y, pt.z, 1.0);
            const Eigen::Vector3d p_map = (T_map_odom * p_odom).head<3>();
            frame_keys.insert(makeKey(p_map));
        }

        for (const auto& key : frame_keys) {
            ++stability_map_[key];
        }
        frame_window_.push_back(std::move(frame_keys));

        while (static_cast<int>(frame_window_.size()) > cfg_.window_size) {
            const auto& oldest = frame_window_.front();
            for (const auto& key : oldest) {
                auto it = stability_map_.find(key);
                if (it == stability_map_.end()) {
                    continue;
                }
                --it->second;
                if (it->second <= 0) {
                    stability_map_.erase(it);
                }
            }
            frame_window_.pop_front();
        }
    }

    std::vector<bool> computeStaticMask(const pcl::PointCloud<pcl::PointCovariance>::Ptr& cloud,
                                        const Eigen::Matrix4d& T_map_odom) {
        std::vector<bool> mask;
        if (!cloud || cloud->empty()) {
            return mask;
        }

        update(cloud, T_map_odom);
        mask.resize(cloud->size(), true);
        for (size_t i = 0; i < cloud->size(); ++i) {
            const auto& pt = cloud->points[i];
            const Eigen::Vector4d p_odom(pt.x, pt.y, pt.z, 1.0);
            const Eigen::Vector3d p_map = (T_map_odom * p_odom).head<3>();
            const VoxelKey key = makeKey(p_map);
            const auto it = stability_map_.find(key);
            mask[i] = (it != stability_map_.end() && it->second >= cfg_.stable_threshold);
        }
        return mask;
    }

private:
    VoxelKey makeKey(const Eigen::Vector3d& point) const {
        const double inv = 1.0 / cfg_.voxel_size;
        return VoxelKey{
            static_cast<int>(std::floor(point.x() * inv)),
            static_cast<int>(std::floor(point.y() * inv)),
            static_cast<int>(std::floor(point.z() * inv)),
        };
    }

    Config cfg_;
    std::unordered_map<VoxelKey, int, VoxelKeyHash> stability_map_;
    std::deque<std::unordered_set<VoxelKey, VoxelKeyHash>> frame_window_;
};

}  // namespace rc26_localization
