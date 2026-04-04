#include <gtest/gtest.h>
#include "rc26_topo_nav/body_planning.hpp"
#include "rc26_topo_nav/planner.hpp"
#include "rc26_topo_nav/graph_loader.hpp"
#include "rc26_topo_nav/surface_route.hpp"
#include <fstream>

using namespace rc26_topo_nav;

class PlannerTest : public ::testing::Test {
protected:
    FieldGraph graph_;
    PlannerWeights weights_;
    std::unordered_map<std::string, NodeOverlay> n_ov_;
    std::unordered_map<std::string, EdgeOverlay> e_ov_;

    void SetUp() override {
        // Build a simple 3-node linear graph: A -> B -> C
        graph_.grid_spacing_m = 1.2;

        GraphNode a{"a", "staging", {0,0,0,0}, 0, 0xFF, 0, 0, "", "", "", "", -1.0, -1.0};
        GraphNode b{"b", "mf_edge_pose", {1.2,0,0,0}, 1, 0xFF, 1, 1.0, "grab", "", "", "", -1.0, -1.0};
        GraphNode c{"c", "mf_edge_pose", {2.4,0,0,0}, 1, 0xFF, 2, 1.0, "grab", "", "", "", -1.0, -1.0};
        graph_.nodes["a"] = a;
        graph_.nodes["b"] = b;
        graph_.nodes["c"] = c;

        GraphEdge e1{"e1", "a", "b", "plane_move", 0, "mf_traverse", false, false, 0xFF, 1.0, {}};
        GraphEdge e2{"e2", "b", "c", "plane_move", 0, "mf_traverse", false, false, 0xFF, 1.0, {}};
        GraphEdge e3{"e3", "b", "a", "plane_move", 0, "mf_traverse", false, false, 0xFF, 1.0, {}};
        GraphEdge e4{"e4", "a", "c", "plane_move", 0, "mf_traverse", false, false, 0xFF, 5.0, {}};
        graph_.edges = {e1, e2, e3, e4};

        graph_.adjacency["a"].push_back(0);
        graph_.adjacency["b"].push_back(1);
        graph_.adjacency["b"].push_back(2);
        graph_.adjacency["a"].push_back(3);

        TaskDef task{"grab", {"b", "c"}, "min_total_cost"};
        graph_.tasks.push_back(task);

        RouteDef route{"entry_route", {"a", "b", "c"}};
        graph_.routes[route.route_tag] = route;

        n_ov_["a"] = {NodeState::FREE, 0};
        n_ov_["b"] = {NodeState::FREE, 0};
        n_ov_["c"] = {NodeState::FREE, 0};
        e_ov_["e1"] = {EdgeState::ENABLED, 0};
        e_ov_["e2"] = {EdgeState::ENABLED, 0};
        e_ov_["e3"] = {EdgeState::ENABLED, 0};
        e_ov_["e4"] = {EdgeState::ENABLED, 0};
    }
};

TEST_F(PlannerTest, FindsDirectPath) {
    auto result = planRoute(graph_, "a", "c", n_ov_, e_ov_, weights_);
    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.node_path.size(), 3u);
    EXPECT_EQ(result.node_path[0], "a");
    EXPECT_EQ(result.node_path[1], "b");
    EXPECT_EQ(result.node_path[2], "c");
}

TEST_F(PlannerTest, FastRouteMatchesTraceRouteResult) {
    PlannerRunOptions run_options;
    run_options.heuristic_scale = 0.5;

    PlannerTraceOptions trace_options;
    trace_options.heuristic_scale = run_options.heuristic_scale;

    const auto fast = planRoute(graph_, "a", "c", n_ov_, e_ov_, weights_, run_options);
    const auto trace = planRouteTrace(graph_, "a", "c", n_ov_, e_ov_, weights_, trace_options);

    ASSERT_TRUE(fast.success);
    ASSERT_TRUE(trace.result.success);
    EXPECT_EQ(fast.node_path, trace.result.node_path);
    EXPECT_EQ(fast.edge_indices, trace.result.edge_indices);
    EXPECT_DOUBLE_EQ(fast.total_cost, trace.result.total_cost);
}

TEST_F(PlannerTest, BlockedNodeAvoidance) {
    n_ov_["b"].state = NodeState::BLOCKED;
    // Can't reach blocked node b directly
    auto result = planRoute(graph_, "a", "b", n_ov_, e_ov_, weights_);
    EXPECT_FALSE(result.success);
}

TEST_F(PlannerTest, BlockedEdgeAvoidance) {
    e_ov_["e1"].state = EdgeState::BLOCKED;
    auto result = planRoute(graph_, "a", "b", n_ov_, e_ov_, weights_);
    EXPECT_FALSE(result.success);
}

TEST_F(PlannerTest, TaskPicksMinCost) {
    auto result = planToTask(graph_, "a", "grab", n_ov_, e_ov_, weights_);
    ASSERT_TRUE(result.success);
    // 'b' is closer than 'c', should pick 'b'
    EXPECT_EQ(result.node_path.back(), "b");
}

TEST_F(PlannerTest, FastTaskMatchesTraceTaskResult) {
    PlannerRunOptions run_options;
    run_options.heuristic_scale = 0.5;

    PlannerTraceOptions trace_options;
    trace_options.heuristic_scale = run_options.heuristic_scale;

    const auto fast = planToTask(graph_, "a", "grab", n_ov_, e_ov_, weights_, run_options);
    const auto trace = planToTaskTrace(graph_, "a", "grab", n_ov_, e_ov_, weights_, trace_options);

    ASSERT_TRUE(fast.success);
    ASSERT_TRUE(trace.result.success);
    EXPECT_EQ(fast.node_path, trace.result.node_path);
    EXPECT_EQ(fast.edge_indices, trace.result.edge_indices);
    EXPECT_DOUBLE_EQ(fast.total_cost, trace.result.total_cost);
}

TEST_F(PlannerTest, TaskSkipsBlockedCandidate) {
    n_ov_["b"].state = NodeState::BLOCKED;
    auto result = planToTask(graph_, "a", "grab", n_ov_, e_ov_, weights_);
    ASSERT_TRUE(result.success);
    EXPECT_EQ(result.node_path.back(), "c");
}

TEST_F(PlannerTest, MissingNodeFails) {
    auto result = planRoute(graph_, "a", "missing", n_ov_, e_ov_, weights_);
    EXPECT_FALSE(result.success);
}

TEST_F(PlannerTest, RouteTagFollowsDeclaredSequence) {
    auto result = planRouteTag(graph_, "a", "entry_route", n_ov_, e_ov_, weights_);
    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.node_path.size(), 3u);
    EXPECT_EQ(result.node_path[0], "a");
    EXPECT_EQ(result.node_path[1], "b");
    EXPECT_EQ(result.node_path[2], "c");
}

TEST_F(PlannerTest, RouteTagFailsWhenDeclaredEdgeBlocked) {
    e_ov_["e2"].state = EdgeState::BLOCKED;
    auto result = planRouteTag(graph_, "a", "entry_route", n_ov_, e_ov_, weights_);
    EXPECT_FALSE(result.success);
}

TEST_F(PlannerTest, TraceCapturesPlannerFramesAndPath) {
    auto trace = planRouteTrace(graph_, "a", "c", n_ov_, e_ov_, weights_);
    ASSERT_TRUE(trace.result.success);
    EXPECT_EQ(trace.result.node_path.size(), 3u);
    ASSERT_GE(trace.frames.size(), 4u);
    EXPECT_EQ(trace.frames.front().event, TraceEventType::INIT);
    EXPECT_EQ(trace.frames.back().event, TraceEventType::GOAL);
    EXPECT_EQ(trace.frames.back().best_path.back(), "c");
}

TEST_F(PlannerTest, TraceWithHeuristicStillFindsOptimalPath) {
    PlannerTraceOptions options;
    options.heuristic_scale = 1.0;

    auto trace = planRouteTrace(graph_, "a", "c", n_ov_, e_ov_, weights_, options);
    ASSERT_TRUE(trace.result.success);
    EXPECT_EQ(trace.result.node_path.size(), 3u);
    EXPECT_EQ(trace.result.node_path[1], "b");
}

TEST_F(PlannerTest, EstimatedHeuristicScaleMatchesMinimumEdgeDensity) {
    const double scale = estimateAdmissibleHeuristicScale(graph_, weights_);
    EXPECT_NEAR(scale, (1.0 / 1.2) * 0.99, 1e-9);
}

TEST_F(PlannerTest, TaskTraceRecordsCandidateSelection) {
    auto trace = planToTaskTrace(graph_, "a", "grab", n_ov_, e_ov_, weights_);
    ASSERT_TRUE(trace.result.success);
    ASSERT_EQ(trace.candidate_results.size(), 2u);
    EXPECT_EQ(trace.selected_candidate, "b");
    EXPECT_EQ(trace.frames.back().event, TraceEventType::CANDIDATE_SELECTED);
}

TEST_F(PlannerTest, SurfaceRouteHonorsRuntimeOverlays) {
    Pose3 requested_start{0.05, 0.0, 0.0, 0.0};
    Pose3 requested_goal{2.35, 0.0, 0.0, 0.0};
    e_ov_["e1"].state = EdgeState::BLOCKED;

    const auto plan = planSurfaceRoute(
        graph_,
        requested_start,
        requested_goal,
        n_ov_,
        e_ov_,
        weights_,
        1.0,
        rclcpp::Time(0),
        "map");

    ASSERT_TRUE(plan.success);
    ASSERT_EQ(plan.plan.edge_indices.size(), 1u);
    EXPECT_EQ(graph_.edges[plan.plan.edge_indices.front()].id, "e4");
    EXPECT_EQ(plan.segments.size(), 1u);
}

TEST_F(PlannerTest, SurfaceBodyPlanningBlocksNarrowAndSteepGraphElements) {
    for (auto& [_, node] : graph_.nodes) {
        node.type = "surface_point";
        node.center_clearance_m = 1.0;
        node.surface_pitch_deg = 0.0;
    }
    for (auto& edge : graph_.edges) {
        edge.center_clearance_m = 1.0;
        edge.horizontal_length_m = 1.2;
        edge.slope_deg = 0.0;
    }
    graph_.nodes["b"].center_clearance_m = 0.19;
    graph_.nodes["c"].surface_pitch_deg = 41.0;
    graph_.edges[0].center_clearance_m = 0.19;
    graph_.edges[1].slope_deg = 42.0;
    graph_.edges[1].horizontal_length_m = 0.10;
    graph_.edges[1].height_change = 0.22;

    RobotGeometryProfile geometry;
    geometry.name = "test";
    geometry.half_length_m = 0.30;
    geometry.half_width_m = 0.20;

    SurfaceBodyPlanningConfig config;
    config.enabled = true;
    config.require_annotated_surface_graph = true;
    config.clearance_margin_m = 0.02;
    config.max_surface_pitch_deg = 35.0;
    config.max_edge_slope_deg = 35.0;
    config.max_step_height_m = 0.18;

    std::unordered_map<std::string, NodeOverlay> node_overlays;
    std::unordered_map<std::string, EdgeOverlay> edge_overlays;
    std::string error;
    const auto stats = applySurfaceBodyPlanningOverlays(
        graph_,
        geometry,
        config,
        node_overlays,
        edge_overlays,
        &error);

    EXPECT_TRUE(error.empty());
    EXPECT_TRUE(stats.annotations_available);
    EXPECT_NE(node_overlays["b"].extra_cost, 0.0);
    EXPECT_EQ(stats.penalized_nodes_clearance, 1u);
    EXPECT_EQ(node_overlays["c"].state, NodeState::BLOCKED);
    EXPECT_EQ(edge_overlays["e1"].state, EdgeState::BLOCKED);
    EXPECT_EQ(edge_overlays["e2"].state, EdgeState::BLOCKED);
}
