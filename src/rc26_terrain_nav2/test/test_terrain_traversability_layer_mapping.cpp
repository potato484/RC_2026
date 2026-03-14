#include "gtest/gtest.h"

#include "nav2_costmap_2d/cost_values.hpp"
#include "rc26_terrain_nav2/terrain_traversability_layer.hpp"

namespace {

TEST(TerrainTraversabilityLayerMappingTest, AppliesThresholdAndGradientMapping) {
    constexpr double kLethal = 0.25;
    constexpr double kInscribed = 0.45;

    EXPECT_EQ(
        rc26_terrain_nav2::mapTraversabilityToCost(0.10f, kLethal, kInscribed),
        nav2_costmap_2d::LETHAL_OBSTACLE);
    EXPECT_EQ(
        rc26_terrain_nav2::mapTraversabilityToCost(0.35f, kLethal, kInscribed),
        nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    EXPECT_EQ(
        rc26_terrain_nav2::mapTraversabilityToCost(1.0f, kLethal, kInscribed),
        static_cast<unsigned char>(0));

    const auto cost = rc26_terrain_nav2::mapTraversabilityToCost(0.60f, kLethal, kInscribed);
    EXPECT_GT(cost, 0);
    EXPECT_LT(cost, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

}  // namespace
