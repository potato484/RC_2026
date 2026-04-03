#!/usr/bin/env python3
"""Regression tests for the topo simulation adapter."""

from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


PKG_ROOT = Path(__file__).resolve().parents[1]
SIM_ROOT = PKG_ROOT / "sim_assets"

GRAPH_BLUE = PKG_ROOT / "config" / "r2_field_graph_blue.yaml"
SURFACE_GRAPH_BLUE = PKG_ROOT / "config" / "r2_surface_graph_blue.yaml"
SIM_WORLD = SIM_ROOT / "worlds" / "robocon2026_v2_aligned.world"
SIM_KFS = SIM_ROOT / "config" / "kfs_config_v2_aligned.yaml"

SERVER = load_module("topo_sim_server", PKG_ROOT / "scripts" / "topo_sim_server.py")


class TopoSimServerTest(unittest.TestCase):
    def test_scene_manifest_contains_world_mesh_and_graph(self):
        manifest = SERVER.build_scene_manifest(
            team="blue",
            graph_file=GRAPH_BLUE,
            world_file=SIM_WORLD,
            kfs_config_file=SIM_KFS,
            include_full_geometry=True,
        )

        self.assertGreater(len(manifest["sceneFeatures"]), 400)
        self.assertGreater(len(manifest["graphNodes"]), 10)
        self.assertGreater(len(manifest["graphEdges"]), 10)
        self.assertEqual(manifest["meta"]["team"], "blue")
        structural_vertical_faces = [
            feature
            for feature in manifest["sceneFeatures"]
            if feature["name"] == "主地图" and feature["area_xy"] < 1e-4 and feature["z_span"] >= 0.35
        ]
        self.assertGreater(len(structural_vertical_faces), 0)
        self.assertEqual(manifest["meta"]["surface_graph_file"], str(SURFACE_GRAPH_BLUE))

    def test_structural_vertical_viewer_faces_do_not_become_2d_keepouts(self):
        rects = SERVER.build_obstacle_rects(
            [
                {
                    "id": "stair-riser",
                    "render_class": "world-ground",
                    "area_xy": 0.0,
                    "z_span": 0.4,
                    "points": [
                        {"x": 1.2, "y": 0.0, "z": 0.0},
                        {"x": 1.2, "y": 0.0, "z": 0.4},
                        {"x": 1.2, "y": 0.4, "z": 0.4},
                        {"x": 1.2, "y": 0.4, "z": 0.0},
                    ],
                },
                {
                    "id": "fence-panel",
                    "render_class": "world-fence",
                    "area_xy": 0.0,
                    "z_span": 0.4,
                    "points": [
                        {"x": 2.0, "y": 0.0, "z": 0.0},
                        {"x": 2.0, "y": 0.0, "z": 0.4},
                        {"x": 2.0, "y": 0.4, "z": 0.4},
                        {"x": 2.0, "y": 0.4, "z": 0.0},
                    ],
                },
            ]
        )

        rect_ids = {rect["id"] for rect in rects}
        self.assertNotIn("stair-riser", rect_ids)
        self.assertIn("fence-panel", rect_ids)

    def test_offline_astar_run_uses_runtime_trace_cli(self):
        manifest = SERVER.build_scene_manifest(
            team="blue",
            graph_file=GRAPH_BLUE,
            world_file=SIM_WORLD,
            kfs_config_file=SIM_KFS,
            include_full_geometry=True,
        )
        request = SERVER.PlannerRunRequest(
            algorithm="astar",
            mode="offline-sim",
            team="blue",
            start_node="ramp_entry_south",
            goal_node="ramp_exit_north",
        )

        snapshot = SERVER.run_offline_request(request, manifest)

        self.assertTrue(snapshot["success"])
        self.assertEqual(snapshot["algorithm"], "astar")
        self.assertGreater(len(snapshot["frames"]), 10)
        self.assertGreater(len(snapshot["path_points"]), 3)

    def test_rrt_and_dwa_runs_emit_frames(self):
        manifest = SERVER.build_scene_manifest(
            team="blue",
            graph_file=GRAPH_BLUE,
            world_file=SIM_WORLD,
            kfs_config_file=SIM_KFS,
            include_full_geometry=True,
        )

        rrt_request = SERVER.PlannerRunRequest(
            algorithm="rrt",
            mode="offline-sim",
            team="blue",
            start_node="mf_entry_staging",
            goal_node="mf_b8",
        )
        rrt_snapshot = SERVER.run_offline_request(rrt_request, manifest)
        self.assertGreater(len(rrt_snapshot["frames"]), 1)

        dwa_request = SERVER.PlannerRunRequest(
            algorithm="dwa",
            mode="offline-sim",
            team="blue",
            start_node="mf_entry_staging",
            goal_node="mf_b8",
        )
        dwa_snapshot = SERVER.run_offline_request(dwa_request, manifest)
        self.assertGreater(len(dwa_snapshot["frames"]), 1)

    def test_surface_route_preview_returns_projected_path(self):
        request = SERVER.SurfaceRoutePreviewRequest(
            team="blue",
            surface_graph_file=str(SURFACE_GRAPH_BLUE),
            start_pick_world={"x": -1.29, "y": 3.77, "z": 0.41, "yaw": 0.0},
            goal_pick_world={"x": -1.11, "y": 3.03, "z": 0.01, "yaw": 0.0},
        )

        preview = SERVER.preview_surface_route(request)

        self.assertTrue(preview["success"])
        self.assertEqual(preview["team"], "blue")
        self.assertEqual(preview["surface_graph_file"], str(SURFACE_GRAPH_BLUE))
        self.assertTrue(preview["projected_start_node_id"])
        self.assertTrue(preview["projected_goal_node_id"])
        self.assertEqual(preview["projected_start"]["x"], -1.29)
        self.assertEqual(preview["projected_goal"]["y"], 3.03)
        self.assertGreater(len(preview["path_points"]), 20)
        self.assertGreater(len(preview["segments"]), 1)

    def test_surface_route_trace_returns_search_frames(self):
        request = SERVER.SurfaceRouteTraceRequest(
            team="blue",
            surface_graph_file=str(SURFACE_GRAPH_BLUE),
            start_pick_world={"x": -1.29, "y": 3.77, "z": 0.41, "yaw": 0.0},
            goal_pick_world={"x": -1.11, "y": 3.03, "z": 0.01, "yaw": 0.0},
        )

        trace = SERVER.trace_surface_route(request)

        self.assertTrue(trace["success"])
        self.assertGreater(len(trace["frames"]), 5)
        self.assertEqual(trace["summary"]["projectedStartNodeId"], trace["projected_start_node_id"])
        self.assertEqual(trace["summary"]["projectedGoalNodeId"], trace["projected_goal_node_id"])
        self.assertEqual(trace["frames"][-1]["metrics"]["traceMode"], "surface_route")
        self.assertGreater(len(trace["frames"][-1]["bestPath"]["nodeIds"]), 2)
        self.assertIn("node_poses", trace)
        self.assertIn(trace["projected_start_node_id"], trace["node_poses"])
        self.assertNotIn("pose", trace["frames"][0]["openSet"][0])
        self.assertNotIn("points", trace["frames"][-1]["bestPath"])

    def test_surface_route_preview_rejects_non_traversable_point(self):
        request = SERVER.SurfaceRoutePreviewRequest(
            team="blue",
            surface_graph_file=str(SURFACE_GRAPH_BLUE),
            start_pick_world={"x": 99.0, "y": 99.0, "z": 0.0, "yaw": 0.0},
            goal_pick_world={"x": -1.11, "y": 3.03, "z": 0.01, "yaw": 0.0},
        )

        preview = SERVER.preview_surface_route(request)

        self.assertFalse(preview["success"])
        self.assertEqual(preview["failure_code"], "POINT_NOT_TRAVERSABLE")

    def test_normalize_astar_trace_document_preserves_sampled_frame_metadata(self):
        graph_document = {
            "nodes": [
                {"id": "n1", "pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}},
                {"id": "n2", "pose": {"x": 1.0, "y": 0.0, "z": 0.0, "yaw": 0.0}},
            ],
            "edges": [],
        }
        raw_trace = {
            "success": True,
            "goal_kind": "node",
            "goal_value": "n2",
            "total_cost": 1.0,
            "selected_candidate": None,
            "candidate_results": [],
            "node_path": ["n1", "n2"],
            "edge_path": [],
            "frame_count_total": 1200,
            "frame_count_emitted": 120,
            "frames_sampled": True,
            "frames": [
                {
                    "event": "init",
                    "step_index": 0,
                    "node_id": "n1",
                    "from_node": "",
                    "edge_id": "",
                    "g_cost": 0.0,
                    "f_cost": 1.0,
                    "step_cost": 0.0,
                    "message": "init",
                    "frontier": [{"node_id": "n1", "g_cost": 0.0, "f_cost": 1.0}],
                    "best_path": ["n1"],
                    "expanded_nodes": [],
                },
                {
                    "event": "goal",
                    "step_index": 1199,
                    "node_id": "n2",
                    "from_node": "n1",
                    "edge_id": "",
                    "g_cost": 1.0,
                    "f_cost": 1.0,
                    "step_cost": 1.0,
                    "message": "goal",
                    "frontier": [],
                    "best_path": ["n1", "n2"],
                    "expanded_nodes": ["n2"],
                },
            ],
        }

        normalized = SERVER.normalize_astar_trace_document(
            raw_trace,
            graph_document,
            node_pose_map={
                "n1": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
                "n2": {"x": 1.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
            },
            trace_mode="surface_route",
        )

        self.assertEqual(normalized["summary"]["framesCount"], 1200)
        self.assertEqual(normalized["summary"]["returnedFramesCount"], 120)
        self.assertTrue(normalized["summary"]["framesSampled"])
        self.assertEqual(len(normalized["frames"]), 2)
        self.assertNotIn("pose", normalized["frames"][0]["openSet"][0])
        self.assertNotIn("points", normalized["frames"][1]["bestPath"])


if __name__ == "__main__":
    unittest.main()
