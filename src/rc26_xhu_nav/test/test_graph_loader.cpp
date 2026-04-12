#include <gtest/gtest.h>
#include "rc26_xhu_nav/topology/graph_loader.hpp"
#include <fstream>

using namespace rc26_xhu_nav::topology;

class GraphLoaderTest : public ::testing::Test {
protected:
    std::string tmp_path_;

    void SetUp() override {
        tmp_path_ = "/tmp/test_graph_" + std::to_string(getpid()) + ".yaml";
    }

    void TearDown() override {
        std::remove(tmp_path_.c_str());
    }

    void writeYaml(const std::string& content) {
        std::ofstream f(tmp_path_);
        f << content;
    }
};

TEST_F(GraphLoaderTest, LoadsValidGraph) {
    writeYaml(R"(
meta:
  team: "blue"
  schema_version: "1.0"
  grid_spacing_m: 1.2
nodes:
  - {id: "a", type: "staging", pose: {x: 0, y: 0, z: 0, yaw: 0}, level: 0, phase_mask: 255, block_id: 0, base_cost: 0, operation_tag: ""}
  - {id: "b", type: "mf_edge_pose", pose: {x: 1.2, y: 0, z: 0, yaw: 0}, level: 1, phase_mask: 255, block_id: 1, base_cost: 1, operation_tag: "grab"}
edges:
  - {id: "e1", from: "a", to: "b", motion_type: "plane_move", height_change: 0, required_mode: "mf_traverse", requires_confirmation: false, can_block: false, phase_mask: 255, base_cost: 1.0}
tasks:
  - {task_tag: "grab", candidate_nodes: ["b"], selection_policy: "min_total_cost"}
routes:
  - {route_tag: "entry", nodes: ["a", "b"]}
)");

    auto lr = loadFieldGraph(tmp_path_);
    ASSERT_TRUE(lr.success) << lr.error;
    EXPECT_EQ(lr.graph.nodes.size(), 2u);
    EXPECT_EQ(lr.graph.edges.size(), 1u);
    EXPECT_EQ(lr.graph.tasks.size(), 1u);
    EXPECT_EQ(lr.graph.routes.size(), 1u);
    EXPECT_EQ(lr.graph.team, "blue");
}

TEST_F(GraphLoaderTest, DetectsMissingEndpoint) {
    writeYaml(R"(
meta:
  team: "blue"
  grid_spacing_m: 1.2
nodes:
  - {id: "a", type: "staging", pose: {x: 0, y: 0, z: 0, yaw: 0}, level: 0, phase_mask: 255, block_id: 0, base_cost: 0, operation_tag: ""}
edges:
  - {id: "e1", from: "a", to: "missing", motion_type: "plane_move", height_change: 0, required_mode: "m", requires_confirmation: false, can_block: false, phase_mask: 255, base_cost: 1.0}
tasks: []
)");

    auto lr = loadFieldGraph(tmp_path_);
    ASSERT_TRUE(lr.success);
    auto vr = validateGraph(lr.graph);
    EXPECT_FALSE(vr.valid);
    EXPECT_GE(vr.errors.size(), 1u);
}

TEST_F(GraphLoaderTest, DetectsDiagonalEdge) {
    writeYaml(R"(
meta:
  team: "blue"
  grid_spacing_m: 1.2
nodes:
  - {id: "b1", type: "mf_edge_pose", pose: {x: 0, y: 0, z: 0, yaw: 0}, level: 1, phase_mask: 255, block_id: 1, base_cost: 1, operation_tag: ""}
  - {id: "b5", type: "mf_edge_pose", pose: {x: 1.2, y: 1.2, z: 0, yaw: 0}, level: 1, phase_mask: 255, block_id: 5, base_cost: 1, operation_tag: ""}
edges:
  - {id: "diag", from: "b1", to: "b5", motion_type: "plane_move", height_change: 0, required_mode: "m", requires_confirmation: false, can_block: true, phase_mask: 255, base_cost: 1.0}
tasks: []
)");

    auto lr = loadFieldGraph(tmp_path_);
    ASSERT_TRUE(lr.success);
    auto vr = validateGraph(lr.graph);
    EXPECT_FALSE(vr.valid);
}

TEST_F(GraphLoaderTest, FailsOnBadYaml) {
    writeYaml("{{{{invalid yaml");
    auto lr = loadFieldGraph(tmp_path_);
    EXPECT_FALSE(lr.success);
}

TEST_F(GraphLoaderTest, DetectsBrokenRoute) {
    writeYaml(R"(
meta:
  team: "blue"
  grid_spacing_m: 1.2
nodes:
  - {id: "a", type: "staging", pose: {x: 0, y: 0, z: 0, yaw: 0}, level: 0, phase_mask: 255, block_id: 0, base_cost: 0, operation_tag: ""}
  - {id: "b", type: "mf_edge_pose", pose: {x: 1.2, y: 0, z: 0, yaw: 0}, level: 1, phase_mask: 255, block_id: 1, base_cost: 1, operation_tag: "grab"}
edges: []
tasks: []
routes:
  - {route_tag: "broken", nodes: ["a", "b"]}
)");

    auto lr = loadFieldGraph(tmp_path_);
    ASSERT_TRUE(lr.success);
    auto vr = validateGraph(lr.graph);
    EXPECT_FALSE(vr.valid);
}
