#include "rc26_xhu_nav/body_planner/planner.hpp"

#include <gtest/gtest.h>

#include <cmath>

using namespace rc26_xhu_nav::body_planner;

namespace {

SurfaceGraph makeGraph() {
    SurfaceGraph graph;
    const double kHalfPi = std::acos(-1.0) * 0.5;
    graph.nodes["a"] = SurfaceNode{"a", {0.0, 0.0, 0.0, 0.0}, 0.50, 0.0};
    graph.nodes["b"] = SurfaceNode{"b", {1.0, 0.0, 0.0, 0.0}, 0.50, 0.0};
    graph.nodes["c"] = SurfaceNode{"c", {1.0, 1.0, 0.0, kHalfPi}, 0.50, 0.0};

    graph.edges.push_back(SurfaceEdge{
        "ab", "a", "b", "plane_move", "mf_traverse", 1.0, 0.0, 1.0, 0.0, 0.50, 0.0, true});
    graph.edges.push_back(SurfaceEdge{
        "bc", "b", "c", "plane_move", "mf_traverse", 1.0, 0.0, 1.0, 0.0, 0.50, kHalfPi, true});
    graph.edges.push_back(SurfaceEdge{
        "ac", "a", "c", "plane_move", "mf_traverse", 4.0, 0.0, 1.4, 0.0, 0.50, kHalfPi, true});

    graph.adjacency["a"] = {0, 2};
    graph.adjacency["b"] = {1};
    return graph;
}

RobotGeometry makeGeometry() {
    RobotGeometry geometry;
    geometry.half_length_m = 0.30;
    geometry.half_width_m = 0.20;
    return geometry;
}

PlannerConfig makeConfig() {
    PlannerConfig config;
    config.heading_bin_count = 16;
    config.max_heading_change_deg = 95.0;
    config.turn_cost_weight = 0.25;
    config.node_turn_clearance_gain = 1.0;
    config.edge_turn_clearance_gain = 0.75;
    return config;
}

}  // namespace

TEST(SurfaceBodyPlannerTest, FindsHeadingAwarePath) {
    const auto graph = makeGraph();
    PlanRequest request;
    request.start_node_id = "a";
    request.goal_node_id = "c";
    request.start_yaw = 0.0;

    const PlannerWeights weights;
    const std::unordered_map<std::string, NodeOverlay> node_overlays;
    const std::unordered_map<std::string, EdgeOverlay> edge_overlays;
    const auto result = planRoute(
        graph, request, node_overlays, edge_overlays, weights, makeGeometry(), makeConfig());

    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.node_path.size(), 3u);
    EXPECT_EQ(result.node_path.front(), "a");
    EXPECT_EQ(result.node_path.back(), "c");
    ASSERT_EQ(result.edge_path.size(), 2u);
    EXPECT_EQ(result.edge_path[0], "ab");
    EXPECT_EQ(result.edge_path[1], "bc");
}

TEST(SurfaceBodyPlannerTest, BlocksTurnWhenNodeClearanceIsTooSmall) {
    auto graph = makeGraph();
    graph.nodes["b"].center_clearance_m = 0.18;

    PlanRequest request;
    request.start_node_id = "a";
    request.goal_node_id = "c";
    request.start_yaw = 0.0;

    const PlannerWeights weights;
    const std::unordered_map<std::string, NodeOverlay> node_overlays;
    const std::unordered_map<std::string, EdgeOverlay> edge_overlays;
    const auto result = planRoute(
        graph, request, node_overlays, edge_overlays, weights, makeGeometry(), makeConfig());

    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.edge_path.size(), 1u);
    EXPECT_EQ(result.edge_path.front(), "ac");
}

TEST(SurfaceBodyPlannerTest, ReportsTransitionConstraintWhenAllRoutesFail) {
    auto graph = makeGraph();
    graph.nodes["b"].center_clearance_m = 0.05;
    graph.edges.resize(2);
    graph.adjacency["a"] = {0};
    graph.adjacency["b"] = {1};

    PlanRequest request;
    request.start_node_id = "a";
    request.goal_node_id = "c";
    request.start_yaw = 0.0;

    const PlannerWeights weights;
    const std::unordered_map<std::string, NodeOverlay> node_overlays;
    const std::unordered_map<std::string, EdgeOverlay> edge_overlays;
    const auto result = planRoute(
        graph, request, node_overlays, edge_overlays, weights, makeGeometry(), makeConfig());

    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.failure_reason.empty());
    EXPECT_FALSE(result.blocked_transition_edge_id.empty());
}
