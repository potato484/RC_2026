#pragma once

#include <cstdint>
#include <string>

namespace rc26_terrain {

struct TerrainStats {
    int kfs_occupied_cells{0};
    int obstacle_cells{0};
    int drop_cells{0};
    int climbable_cells{0};
};

class SafetyGuard {
public:
    void setFailSafeStrategy(std::string strategy) { fail_safe_strategy_ = std::move(strategy); }
    void updateStats(const TerrainStats& stats) { stats_ = stats; }

    const TerrainStats& stats() const { return stats_; }
    const std::string& failSafeStrategy() const { return fail_safe_strategy_; }

private:
    std::string fail_safe_strategy_{"virtual_fence"};
    TerrainStats stats_{};
};

}  // namespace rc26_terrain
