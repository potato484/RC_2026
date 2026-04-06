#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <tuple>

#include "rc26_local_3d_planner/planner_core.hpp"

namespace rc26_local_3d_planner {
namespace {

geometry_msgs::msg::Quaternion quaternionFromYaw(const double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

PlannerInput makeBaseInput() {
  PlannerInput input;
  input.has_pose = true;
  input.robot_x = 0.0;
  input.robot_y = 0.0;
  input.robot_yaw = 0.0;
  input.current_vx = 0.0;
  input.current_wz = 0.0;
  input.has_mode_state = true;
  input.mode_state.active_mode = "normal";
  input.mode_state.max_linear_speed = 0.5F;
  input.mode_state.max_angular_speed = 1.0F;
  input.mode_state.max_linear_accel = 0.6F;
  input.mode_state.max_angular_accel = 0.8F;
  input.corridor.corridor_id = "test_corridor";
  input.corridor.edge_id = "test_edge";
  input.corridor.header.frame_id = "map";
  input.corridor.path.header.frame_id = "map";
  input.corridor.preferred_linear_speed = 0.3F;
  input.corridor.max_linear_speed = 0.5F;
  input.corridor.max_angular_speed = 1.0F;
  input.corridor.allow_reverse = false;
  input.corridor.allow_in_place_rotate = true;
  for (const auto [x, y, yaw] :
       {std::tuple<double, double, double>{0.0, 0.0, 0.0},
        std::tuple<double, double, double>{0.4, 0.0, 0.0},
        std::tuple<double, double, double>{0.9, 0.0, 0.0}}) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation = quaternionFromYaw(yaw);
    input.corridor.path.poses.push_back(pose);
  }
  input.has_semantic_summary = true;
  input.semantic_summary.revision = 1U;
  input.semantic_summary.terrain_available = true;
  input.semantic_summary.keepout_available = true;
  input.has_terrain_grid = true;
  input.terrain_grid.resolution_m = 0.15F;
  input.terrain_grid.width = 8U;
  input.terrain_grid.height = 8U;
  input.terrain_grid.origin.position.x = -0.5;
  input.terrain_grid.origin.position.y = -0.5;
  const auto cell_count = static_cast<std::size_t>(input.terrain_grid.width) *
                          static_cast<std::size_t>(input.terrain_grid.height);
  input.terrain_grid.p_obstacle.assign(cell_count, 0.0F);
  input.terrain_grid.p_drop.assign(cell_count, 0.0F);
  return input;
}

TEST(PlannerCoreTest, PassScenarioProducesTraceAndPreview) {
  PlannerCore planner;
  PlannerInput input = makeBaseInput();
  PlannerTrace trace;

  const auto result = planner.plan(input, &trace);

  EXPECT_EQ(result.status, "PASS");
  EXPECT_TRUE(result.has_solution);
  EXPECT_FALSE(result.preview_path.poses.empty());
  EXPECT_EQ(trace.final_status, "PASS");
  EXPECT_FALSE(trace.candidates.empty());
  EXPECT_EQ(trace.semantic_revision, 1U);
}

TEST(PlannerCoreTest, BlockedKeepoutBecomesWaitingOnBlock) {
  PlannerCore planner;
  PlannerInput input = makeBaseInput();
  std::fill(input.terrain_grid.p_obstacle.begin(), input.terrain_grid.p_obstacle.end(), 0.95F);
  input.semantic_summary.blocked_cells = 4U;
  input.corridor.allow_in_place_rotate = false;

  const auto result = planner.plan(input);

  EXPECT_EQ(result.status, "WAITING_ON_BLOCK");
  EXPECT_TRUE(result.blocked_by_keepout);
  EXPECT_TRUE(result.blocked_by_terrain);
}

TEST(PlannerCoreTest, RotateRecoverySuggestedWhenHeadingMismatchRemains) {
  PlannerCore planner;
  PlannerInput input = makeBaseInput();
  std::fill(input.terrain_grid.p_obstacle.begin(), input.terrain_grid.p_obstacle.end(), 0.95F);
  input.corridor.path.poses.back().pose.orientation = quaternionFromYaw(1.57);
  input.semantic_summary.blocked_cells = 0U;
  input.corridor.allow_in_place_rotate = true;

  const auto result = planner.plan(input);

  EXPECT_EQ(result.status, "RECOVERY_RUNNING");
  EXPECT_TRUE(result.should_rotate_recovery);
  EXPECT_EQ(result.recovery_state.recovery_name, "rotate_in_place");
}

TEST(PlannerCoreTest, TerrainCollisionFallsBackToLocalCollisionBlocked) {
  PlannerCore planner;
  PlannerInput input = makeBaseInput();
  std::fill(input.terrain_grid.p_obstacle.begin(), input.terrain_grid.p_obstacle.end(), 0.95F);
  input.semantic_summary.blocked_cells = 0U;
  input.corridor.allow_in_place_rotate = false;

  const auto result = planner.plan(input);

  EXPECT_EQ(result.status, "LOCAL_COLLISION_BLOCKED");
  EXPECT_TRUE(result.blocked_by_terrain);
  EXPECT_FALSE(result.blocked_by_keepout);
}

}  // namespace
}  // namespace rc26_local_3d_planner
