#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "rc26_terrain/terrain_types.hpp"

namespace rc26_terrain {

struct GridConfig {
    int width{0};
    int half_width{0};
    double resolution_m{0.1};
    int min_points_per_cell{1};
    double ground_quantile{0.25};
    double top_quantile{0.95};
    double ground_ema_alpha{0.6};
    double ground_ema_alpha_slow{0.25};
    double jump_thresh_m{0.23};
    int freeze_max_frames{3};
    double h_climb_m{0.30};
    double h_obstacle_m{0.33};
    double h_drop_m{0.15};
    double climbable_min_dz_m{0.05};
    bool enable_hysteresis{true};
    int score_max{10};
    int score_inc{2};
    int score_dec{1};
    int obstacle_on_score{6};
    int obstacle_off_score{3};
    int drop_on_score{6};
    int drop_off_score{3};
    double decay_time_sec{2.0};
    double stale_time_sec{0.7};
    int min_obstacle_area_cells{1};
    std::string obstacle_neighbor_mode{"edge4"};
    std::string drop_neighbor_mode{"edge8"};
};

class GridEvaluator {
public:
    explicit GridEvaluator(const GridConfig& cfg);

    void feedSamples(int idx, float z);
    void clearSamples();
    void evaluate(double stamp_sec, double linear_speed_mps, bool base_ground_stable, double pitch_rad);
    void setKfsOccupied(int idx, bool occupied);

    TerrainCellState stateOf(int idx) const;
    float groundZ(int idx) const;
    float topZ(int idx) const;

private:
    void runBfsDenoiseObstacle();
    bool validIndex(int idx) const;

    GridConfig cfg_;
    int num_cells_{0};

    std::vector<float> ground_z_filtered_;
    std::vector<float> top_z_;
    std::vector<double> last_seen_sec_;
    std::vector<uint8_t> obstacle_state_;
    std::vector<uint8_t> drop_state_;
    std::vector<uint8_t> kfs_occupied_;
    std::vector<std::vector<float>> cell_z_samples_;
    std::vector<TerrainCellState> states_;
};

}  // namespace rc26_terrain
