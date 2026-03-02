#include <memory>

#include "gtest/gtest.h"

#include "rc26_terrain/grid_evaluator.hpp"

namespace rc26_terrain {

class GridEvaluatorTest : public ::testing::Test {
protected:
    void SetUp() override {
        GridConfig cfg{};
        cfg.width = 3;
        cfg.half_width = 1;
        cfg.resolution_m = 0.1;
        cfg.min_points_per_cell = 1;
        cfg.ground_quantile = 0.25;
        cfg.top_quantile = 0.95;
        cfg.ground_ema_alpha = 0.6;
        cfg.ground_ema_alpha_slow = 0.25;
        cfg.jump_thresh_m = 0.23;
        cfg.freeze_max_frames = 3;
        cfg.h_climb_m = 0.30;
        cfg.h_obstacle_m = 0.33;
        cfg.h_drop_m = 0.15;
        cfg.enable_hysteresis = false;
        cfg.obstacle_neighbor_mode = "edge4";
        cfg.drop_neighbor_mode = "edge8";
        cfg.min_obstacle_area_cells = 1;
        cfg.stale_time_sec = 1.0;
        cfg.decay_time_sec = 5.0;
        evaluator_ = std::make_unique<GridEvaluator>(cfg);
    }

    std::unique_ptr<GridEvaluator> evaluator_;
};

TEST_F(GridEvaluatorTest, KfsOccupiedIndependentOfObstacleState) {
    evaluator_->setKfsOccupied(4, true);
    EXPECT_EQ(evaluator_->stateOf(4), TerrainCellState::KFS_OCCUPIED);
}

TEST_F(GridEvaluatorTest, ClearKfsDoesNotCrash) {
    evaluator_->setKfsOccupied(4, true);
    evaluator_->setKfsOccupied(4, false);
    EXPECT_NE(evaluator_->stateOf(4), TerrainCellState::KFS_OCCUPIED);
}

TEST_F(GridEvaluatorTest, DecayResetsStaleCell) {
    evaluator_->feedSamples(4, 0.0F);
    evaluator_->evaluate(0.0, 0.0, true, 0.0);
    EXPECT_EQ(evaluator_->stateOf(4), TerrainCellState::GROUND);

    evaluator_->evaluate(10.0, 0.0, true, 0.0);
    EXPECT_EQ(evaluator_->stateOf(4), TerrainCellState::UNKNOWN);
}

TEST(GridEvaluatorStandaloneTest, BfsDenoiseRemovesIsolatedObstacle) {
    GridConfig cfg{};
    cfg.width = 3;
    cfg.half_width = 1;
    cfg.min_points_per_cell = 1;
    cfg.h_obstacle_m = 0.33;
    cfg.min_obstacle_area_cells = 2;
    cfg.decay_time_sec = 5.0;

    GridEvaluator ev(cfg);
    ev.feedSamples(4, 0.0F);
    ev.feedSamples(4, 1.0F);  // 高差足够触发 obstacle
    ev.evaluate(0.0, 0.0, true, 0.0);
    EXPECT_NE(ev.stateOf(4), TerrainCellState::OBSTACLE);
}

}  // namespace rc26_terrain
