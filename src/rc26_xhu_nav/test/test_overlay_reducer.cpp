#include "rc26_xhu_nav/topology/overlay_reducer.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

namespace rc26_xhu_nav::topology {
namespace {

FieldGraph makeSurfaceGraph() {
    FieldGraph graph;
    graph.nodes["a"] = GraphNode{"a", "surface_point", {0.0, 0.0, 0.0, 0.0}, 0, 0xFF, 0, 0.0, "", "", "", "", 0.50, 0.0};
    graph.nodes["b"] = GraphNode{"b", "surface_point", {1.0, 0.0, 0.0, 0.0}, 0, 0xFF, 0, 0.0, "", "", "", "", 0.50, 0.0};
    graph.edges.push_back(GraphEdge{
        "e1", "a", "b", "plane_move", 0.0, "mf_traverse", false, false, 0xFF, 1.0, {}, 1.0, 0.0, 0.50, 0.0, true});
    graph.adjacency["a"] = {0};
    return graph;
}

class OverlayReducerTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() {
        if (!rclcpp::ok()) {
            int argc = 0;
            rclcpp::init(argc, nullptr);
        }
    }

    static void TearDownTestSuite() {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    void SetUp() override {
        node_ = std::make_shared<rclcpp::Node>("overlay_reducer_test_node");
        node_->declare_parameter("team", "blue");
        reducer_ = std::make_unique<OverlayReducer>(node_.get());
        dynamic_overlay_pub_ =
            node_->create_publisher<rc26_interfaces::msg::SurfaceGraphOverlay>("/surface_graph_overlay", 10);
        executor_.add_node(node_);
    }

    void TearDown() override {
        executor_.remove_node(node_);
        reducer_.reset();
        dynamic_overlay_pub_.reset();
        node_.reset();
    }

    void spinFor(const std::chrono::milliseconds duration) {
        const auto deadline = std::chrono::steady_clock::now() + duration;
        while (std::chrono::steady_clock::now() < deadline) {
            executor_.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<OverlayReducer> reducer_;
    rclcpp::Publisher<rc26_interfaces::msg::SurfaceGraphOverlay>::SharedPtr dynamic_overlay_pub_;
    rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(OverlayReducerTest, DynamicSurfaceOverlayAffectsAllModeAndExpiresByTtl) {
    const auto graph = makeSurfaceGraph();
    const auto initial_snapshot = reducer_->snapshot();

    rc26_interfaces::msg::SurfaceGraphOverlay overlay;
    overlay.team = "blue";
    overlay.source = "unit_test_dynamic";
    overlay.ttl.sec = 0;
    overlay.ttl.nanosec = 200000000;
    overlay.blocked_edge_ids = {"e1"};
    dynamic_overlay_pub_->publish(overlay);
    spinFor(std::chrono::milliseconds(50));

    const auto active_snapshot = reducer_->snapshot();
    EXPECT_GT(active_snapshot.version, initial_snapshot.version);
    ASSERT_EQ(active_snapshot.active_dynamic_sources.size(), 1u);
    EXPECT_EQ(active_snapshot.active_dynamic_sources.front(), "unit_test_dynamic");
    EXPECT_EQ(active_snapshot.dynamic_blocked_edges.at("e1"), "unit_test_dynamic");

    std::unordered_map<std::string, NodeOverlay> static_node_overlays;
    std::unordered_map<std::string, EdgeOverlay> static_edge_overlays;
    reducer_->applyOverlays(
        graph,
        static_node_overlays,
        static_edge_overlays,
        OverlayApplicationMode::STATIC_ONLY);
    EXPECT_EQ(static_edge_overlays["e1"].state, EdgeState::ENABLED);

    std::unordered_map<std::string, NodeOverlay> all_node_overlays;
    std::unordered_map<std::string, EdgeOverlay> all_edge_overlays;
    reducer_->applyOverlays(
        graph,
        all_node_overlays,
        all_edge_overlays,
        OverlayApplicationMode::ALL);
    EXPECT_EQ(all_edge_overlays["e1"].state, EdgeState::BLOCKED);

    std::this_thread::sleep_for(std::chrono::milliseconds(220));
    spinFor(std::chrono::milliseconds(30));
    const auto expired_snapshot = reducer_->snapshot();
    EXPECT_TRUE(expired_snapshot.active_dynamic_sources.empty());
    EXPECT_TRUE(expired_snapshot.dynamic_blocked_edges.empty());
    EXPECT_GT(expired_snapshot.version, active_snapshot.version);
}

}  // namespace
}  // namespace rc26_xhu_nav::topology
