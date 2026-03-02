#include "rc26_terrain/grid_evaluator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <queue>

namespace rc26_terrain {

namespace {

float quantileInplace(std::vector<float>& values, double q) {
    if (values.empty()) {
        return 0.0F;
    }
    q = std::clamp(q, 0.0, 1.0);
    const size_t idx = static_cast<size_t>(q * static_cast<double>(values.size() - 1));
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(idx), values.end());
    return values[idx];
}

}  // namespace

GridEvaluator::GridEvaluator(const GridConfig& cfg) : cfg_(cfg) {
    if (cfg_.width <= 0) {
        cfg_.width = 1;
    }
    num_cells_ = cfg_.width * cfg_.width;

    ground_z_filtered_.assign(static_cast<size_t>(num_cells_), 0.0F);
    top_z_.assign(static_cast<size_t>(num_cells_), 0.0F);
    last_seen_sec_.assign(static_cast<size_t>(num_cells_), -1.0);
    obstacle_state_.assign(static_cast<size_t>(num_cells_), 0U);
    drop_state_.assign(static_cast<size_t>(num_cells_), 0U);
    kfs_occupied_.assign(static_cast<size_t>(num_cells_), 0U);
    cell_z_samples_.assign(static_cast<size_t>(num_cells_), {});
    states_.assign(static_cast<size_t>(num_cells_), TerrainCellState::UNKNOWN);
}

bool GridEvaluator::validIndex(int idx) const {
    return idx >= 0 && idx < num_cells_;
}

void GridEvaluator::feedSamples(int idx, float z) {
    if (!validIndex(idx) || !std::isfinite(z)) {
        return;
    }
    cell_z_samples_[static_cast<size_t>(idx)].push_back(z);
}

void GridEvaluator::clearSamples() {
    for (auto& bucket : cell_z_samples_) {
        bucket.clear();
    }
}

void GridEvaluator::setKfsOccupied(int idx, bool occupied) {
    if (!validIndex(idx)) {
        return;
    }
    kfs_occupied_[static_cast<size_t>(idx)] = occupied ? 1U : 0U;
}

void GridEvaluator::evaluate(
    double stamp_sec, double /*linear_speed_mps*/, bool /*base_ground_stable*/, double /*pitch_rad*/) {
    for (int idx = 0; idx < num_cells_; ++idx) {
        const size_t uidx = static_cast<size_t>(idx);
        auto& samples = cell_z_samples_[uidx];

        if (!samples.empty()) {
            const float ground = quantileInplace(samples, cfg_.ground_quantile);
            const float top = quantileInplace(samples, cfg_.top_quantile);
            const float height_delta = std::max(0.0F, top - ground);

            if (last_seen_sec_[uidx] < 0.0) {
                ground_z_filtered_[uidx] = ground;
            } else {
                ground_z_filtered_[uidx] = static_cast<float>(cfg_.ground_ema_alpha) * ground +
                                           static_cast<float>(1.0 - cfg_.ground_ema_alpha) *
                                               ground_z_filtered_[uidx];
            }
            top_z_[uidx] = top;
            last_seen_sec_[uidx] = stamp_sec;

            obstacle_state_[uidx] = (height_delta > static_cast<float>(cfg_.h_obstacle_m)) ? 1U : 0U;
            drop_state_[uidx] = 0U;
            states_[uidx] = obstacle_state_[uidx] ? TerrainCellState::OBSTACLE : TerrainCellState::GROUND;
        } else if (last_seen_sec_[uidx] >= 0.0 && (stamp_sec - last_seen_sec_[uidx]) > cfg_.decay_time_sec) {
            last_seen_sec_[uidx] = -1.0;
            ground_z_filtered_[uidx] = 0.0F;
            top_z_[uidx] = 0.0F;
            obstacle_state_[uidx] = 0U;
            drop_state_[uidx] = 0U;
            states_[uidx] = TerrainCellState::UNKNOWN;
        }
    }

    runBfsDenoiseObstacle();

    for (int idx = 0; idx < num_cells_; ++idx) {
        const size_t uidx = static_cast<size_t>(idx);
        if (kfs_occupied_[uidx]) {
            states_[uidx] = TerrainCellState::KFS_OCCUPIED;
        } else if (obstacle_state_[uidx]) {
            states_[uidx] = TerrainCellState::OBSTACLE;
        } else if (last_seen_sec_[uidx] >= 0.0) {
            states_[uidx] = TerrainCellState::GROUND;
        } else {
            states_[uidx] = TerrainCellState::UNKNOWN;
        }
    }

    clearSamples();
}

TerrainCellState GridEvaluator::stateOf(int idx) const {
    if (!validIndex(idx)) {
        return TerrainCellState::UNKNOWN;
    }
    if (kfs_occupied_[static_cast<size_t>(idx)]) {
        return TerrainCellState::KFS_OCCUPIED;
    }
    return states_[static_cast<size_t>(idx)];
}

float GridEvaluator::groundZ(int idx) const {
    if (!validIndex(idx)) {
        return 0.0F;
    }
    return ground_z_filtered_[static_cast<size_t>(idx)];
}

float GridEvaluator::topZ(int idx) const {
    if (!validIndex(idx)) {
        return 0.0F;
    }
    return top_z_[static_cast<size_t>(idx)];
}

void GridEvaluator::runBfsDenoiseObstacle() {
    const int min_area = std::max(1, cfg_.min_obstacle_area_cells);
    if (min_area <= 1) {
        return;
    }

    std::vector<uint8_t> visited(static_cast<size_t>(num_cells_), 0U);
    const auto width = cfg_.width;
    const std::array<std::pair<int, int>, 4> dirs{{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};

    for (int seed = 0; seed < num_cells_; ++seed) {
        const size_t seed_idx = static_cast<size_t>(seed);
        if (!obstacle_state_[seed_idx] || visited[seed_idx]) {
            continue;
        }

        std::queue<int> q;
        std::vector<int> component;
        q.push(seed);
        visited[seed_idx] = 1U;

        while (!q.empty()) {
            const int cell = q.front();
            q.pop();
            component.push_back(cell);

            const int x = cell / width;
            const int y = cell % width;
            for (const auto& [dx, dy] : dirs) {
                const int nx = x + dx;
                const int ny = y + dy;
                if (nx < 0 || nx >= width || ny < 0 || ny >= width) {
                    continue;
                }
                const int n = nx * width + ny;
                const size_t nidx = static_cast<size_t>(n);
                if (obstacle_state_[nidx] && !visited[nidx]) {
                    visited[nidx] = 1U;
                    q.push(n);
                }
            }
        }

        if (static_cast<int>(component.size()) < min_area) {
            for (const int cell : component) {
                obstacle_state_[static_cast<size_t>(cell)] = 0U;
            }
        }
    }
}

}  // namespace rc26_terrain
