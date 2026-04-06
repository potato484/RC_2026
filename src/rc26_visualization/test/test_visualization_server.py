#!/usr/bin/env python3
"""Regression tests for the visualization web adapter."""

from __future__ import annotations

import importlib.util
import json
import unittest
from pathlib import Path
from unittest import mock


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


PKG_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PKG_ROOT.parents[1]
TOPO_NAV_ROOT = REPO_ROOT / "src" / "rc26_topo_nav"
SIM_ROOT = TOPO_NAV_ROOT / "sim_assets"

GRAPH_BLUE = TOPO_NAV_ROOT / "config" / "r2_field_graph_blue.yaml"
SURFACE_GRAPH_BLUE = TOPO_NAV_ROOT / "config" / "r2_surface_graph_blue.yaml"
SIM_WORLD = SIM_ROOT / "worlds" / "robocon2026_v2_aligned.world"
SIM_KFS = SIM_ROOT / "config" / "kfs_config_v2_aligned.yaml"

SERVER = load_module("visualization_server", PKG_ROOT / "scripts" / "visualization_server.py")


class VisualizationServerTest(unittest.TestCase):
    _cache: dict[str, object] = {}

    @classmethod
    def _full_manifest(cls) -> dict[str, object]:
        manifest = cls._cache.get("full_manifest")
        if manifest is None:
            manifest = SERVER.build_scene_manifest(
                team="blue",
                graph_file=GRAPH_BLUE,
                world_file=SIM_WORLD,
                kfs_config_file=SIM_KFS,
                include_full_geometry=True,
            )
            cls._cache["full_manifest"] = manifest
        return manifest  # type: ignore[return-value]

    @classmethod
    def _surface_preview_request(cls) -> "SERVER.SurfaceRoutePreviewRequest":
        return SERVER.SurfaceRoutePreviewRequest(
            team="blue",
            surface_graph_file=str(SURFACE_GRAPH_BLUE),
            start_pick_world={"x": -1.29, "y": 3.77, "z": 0.41, "yaw": 0.0},
            goal_pick_world={"x": -1.11, "y": 3.03, "z": 0.01, "yaw": 0.0},
        )

    @classmethod
    def _cached_surface_preview(cls) -> dict[str, object]:
        preview = cls._cache.get("surface_preview")
        if preview is None:
            preview = SERVER.preview_surface_route(cls._surface_preview_request())
            cls._cache["surface_preview"] = preview
        return preview  # type: ignore[return-value]

    @classmethod
    def _cached_invalid_surface_preview(cls) -> dict[str, object]:
        preview = cls._cache.get("invalid_surface_preview")
        if preview is None:
            preview = SERVER.preview_surface_route(
                SERVER.SurfaceRoutePreviewRequest(
                    team="blue",
                    surface_graph_file=str(SURFACE_GRAPH_BLUE),
                    start_pick_world={"x": 99.0, "y": 99.0, "z": 0.0, "yaw": 0.0},
                    goal_pick_world={"x": -1.11, "y": 3.03, "z": 0.01, "yaw": 0.0},
                )
            )
            cls._cache["invalid_surface_preview"] = preview
        return preview  # type: ignore[return-value]

    def test_scene_manifest_contains_world_mesh_and_graph(self):
        manifest = self._full_manifest()

        self.assertGreater(len(manifest["sceneFeatures"]), 400)
        self.assertGreater(len(manifest["graphNodes"]), 10)
        self.assertGreater(len(manifest["graphEdges"]), 10)
        self.assertEqual(manifest["meta"]["team"], "blue")
        self.assertEqual(manifest["viewerMeta"]["viewer_title"], "RC26 全局比赛场地闭环可视化平台")
        self.assertGreaterEqual(len(manifest["semanticZones"]), 4)
        self.assertIn("operator", {preset["id"] for preset in manifest["layoutPresets"]})
        self.assertIn("scene", {display["id"] for display in manifest["displayCatalog"]})
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

    def test_surface_route_preview_returns_projected_path(self):
        preview = self._cached_surface_preview()

        self.assertTrue(preview["success"])
        self.assertEqual(preview["team"], "blue")
        self.assertEqual(preview["surface_graph_file"], str(SURFACE_GRAPH_BLUE))
        self.assertTrue(preview["projected_start_node_id"])
        self.assertTrue(preview["projected_goal_node_id"])
        self.assertEqual(preview["projected_start"]["x"], -1.29)
        self.assertEqual(preview["projected_goal"]["y"], 3.03)
        self.assertGreater(len(preview["path_points"]), 20)
        self.assertGreaterEqual(len(preview["segments"]), 1)
        self.assertIn("planning_logs", preview)
        self.assertEqual(preview["planning_logs"][0]["stage"], "request")
        self.assertEqual(preview["planning_logs"][-1]["stage"], "surface_route_cli")
        self.assertEqual(preview["planning_logs"][0]["message"], "已准备表面图投影与路径规划输入")
        self.assertEqual(preview["planning_logs"][-1]["title"], "表面路线预览")
        self.assertGreaterEqual(preview["planning_timing_ms"]["surfaceProjection"], 0.0)
        self.assertGreaterEqual(preview["planning_timing_ms"]["surfacePlanning"], 0.0)
        self.assertGreaterEqual(preview["planning_timing_ms"]["surfacePathExpand"], 0.0)
        self.assertGreaterEqual(preview["planning_timing_ms"]["surfaceSegmentBuild"], 0.0)
        self.assertGreaterEqual(preview["planning_timing_ms"]["surfaceCompletePlanning"], 0.0)
        self.assertGreater(preview["planning_timing_ms"]["surfaceRouteCli"], 0.0)
        self.assertNotIn("N/A", json.dumps(preview["planning_logs"], ensure_ascii=False))

    def test_surface_route_trace_returns_search_frames(self):
        fake_preview = {
            "success": True,
            "surface_graph_file": str(SURFACE_GRAPH_BLUE),
            "projected_start_node_id": "sf_test_start",
            "projected_goal_node_id": "sf_test_goal",
            "path_points": [
                {"x": -1.29, "y": 3.77, "z": 0.41, "yaw": 0.0},
                {"x": -1.22, "y": 3.55, "z": 0.32, "yaw": 0.0},
                {"x": -1.11, "y": 3.03, "z": 0.01, "yaw": 0.0},
            ],
            "segments": [{"id": "segment_0"}],
            "planning_logs": [
                {"stage": "request", "title": "收到路线请求", "level": "info", "message": "mock request"},
                {"stage": "surface_route_cli", "title": "表面路线预览", "level": "info", "message": "mock preview"},
            ],
            "planning_timing_ms": {
                "surfaceProjection": 1.0,
                "surfacePlanning": 2.0,
                "surfacePathExpand": 0.5,
                "surfaceSegmentBuild": 0.4,
                "surfaceCompletePlanning": 3.0,
                "surfaceRouteCli": 3.5,
            },
        }
        fake_trace = {
            "success": True,
            "frames": [
                {
                    "stepIndex": 0,
                    "metrics": {"traceMode": "surface_route"},
                    "bestPath": {"nodeIds": ["sf_test_start"]},
                    "openSet": [{"nodeId": "sf_test_start"}],
                },
                {
                    "stepIndex": 1,
                    "metrics": {"traceMode": "surface_route"},
                    "bestPath": {"nodeIds": ["sf_test_start", "sf_test_goal"]},
                    "openSet": [],
                },
            ],
            "node_poses": {
                "sf_test_start": {"x": -1.29, "y": 3.77, "z": 0.41, "yaw": 0.0},
                "sf_test_goal": {"x": -1.11, "y": 3.03, "z": 0.01, "yaw": 0.0},
            },
            "summary": {
                "framesCount": 12,
                "returnedFramesCount": 2,
                "framesSampled": False,
                "tracePlanningMs": 4.0,
                "traceElapsedMs": 4.5,
            },
            "planning_timing_ms": {
                "tracePlanning": 4.0,
                "plannerTraceCli": 4.5,
            },
            "planning_logs": [
                {"stage": "planner_trace_cli", "title": "搜索回放生成", "level": "info", "message": "mock trace"},
            ],
            "projected_start_node_id": "sf_test_start",
            "projected_goal_node_id": "sf_test_goal",
            "failure_code": "",
            "failure_reason": "",
        }
        request = SERVER.SurfaceRouteTraceRequest(
            team="blue",
            surface_graph_file=str(SURFACE_GRAPH_BLUE),
            start_pick_world={"x": -1.29, "y": 3.77, "z": 0.41, "yaw": 0.0},
            goal_pick_world={"x": -1.11, "y": 3.03, "z": 0.01, "yaw": 0.0},
        )

        with mock.patch.object(SERVER, "preview_surface_route", return_value=fake_preview), mock.patch.object(
            SERVER, "trace_surface_route_from_nodes", return_value=fake_trace
        ):
            trace = SERVER.trace_surface_route(request)

        self.assertTrue(trace["success"])
        self.assertEqual(len(trace["frames"]), 2)
        self.assertEqual(trace["summary"]["projectedStartNodeId"], trace["projected_start_node_id"])
        self.assertEqual(trace["summary"]["projectedGoalNodeId"], trace["projected_goal_node_id"])
        self.assertEqual(trace["frames"][-1]["metrics"]["traceMode"], "surface_route")
        self.assertGreaterEqual(len(trace["frames"][-1]["bestPath"]["nodeIds"]), 2)
        self.assertIn("node_poses", trace)
        self.assertIn(trace["projected_start_node_id"], trace["node_poses"])
        self.assertNotIn("pose", trace["frames"][0]["openSet"][0])
        self.assertNotIn("points", trace["frames"][-1]["bestPath"])
        self.assertIn("planning_logs", trace)
        self.assertEqual(
            [entry["stage"] for entry in trace["planning_logs"]],
            ["request", "surface_route_cli", "planner_trace_cli", "trace_pipeline"],
        )
        self.assertGreaterEqual(trace["summary"]["surfaceProjectionMs"], 0.0)
        self.assertGreaterEqual(trace["summary"]["surfacePlanningMs"], 0.0)
        self.assertGreaterEqual(trace["summary"]["surfacePathExpandMs"], 0.0)
        self.assertGreaterEqual(trace["summary"]["surfaceSegmentBuildMs"], 0.0)
        self.assertGreaterEqual(trace["summary"]["surfaceCompletePlanningMs"], 0.0)
        self.assertGreater(trace["summary"]["tracePlanningMs"], 0.0)
        self.assertGreater(trace["summary"]["previewElapsedMs"], 0.0)
        self.assertGreater(trace["summary"]["traceElapsedMs"], 0.0)
        self.assertGreater(trace["summary"]["totalElapsedMs"], trace["summary"]["traceElapsedMs"])
        self.assertGreaterEqual(trace["planning_timing_ms"]["surfaceProjection"], 0.0)
        self.assertGreaterEqual(trace["planning_timing_ms"]["surfacePlanning"], 0.0)
        self.assertGreaterEqual(trace["planning_timing_ms"]["surfacePathExpand"], 0.0)
        self.assertGreaterEqual(trace["planning_timing_ms"]["surfaceSegmentBuild"], 0.0)
        self.assertGreaterEqual(trace["planning_timing_ms"]["surfaceCompletePlanning"], 0.0)
        self.assertGreater(trace["planning_timing_ms"]["tracePlanning"], 0.0)
        self.assertGreater(trace["planning_timing_ms"]["plannerTraceCli"], 0.0)
        self.assertEqual(trace["planning_logs"][1]["title"], "表面路线预览")
        self.assertEqual(trace["planning_logs"][2]["title"], "搜索回放生成")
        self.assertEqual(trace["planning_logs"][-1]["title"], "回放整合结果")

    def test_surface_route_trace_from_nodes_returns_background_replay(self):
        trace = SERVER.trace_surface_route_from_nodes(
            SERVER.SurfaceRouteTraceFromNodesRequest(
                team="blue",
                surface_graph_file=str(GRAPH_BLUE),
                start_node_id="ramp_entry_south",
                goal_node_id="ramp_exit_north",
                requested_start={"x": -1.29, "y": 3.77, "z": 0.41, "yaw": 0.0},
                requested_goal={"x": -1.11, "y": 3.03, "z": 0.01, "yaw": 0.0},
            )
        )

        self.assertTrue(trace["success"])
        self.assertEqual(trace["projected_start_node_id"], "ramp_entry_south")
        self.assertEqual(trace["projected_goal_node_id"], "ramp_exit_north")
        self.assertGreater(len(trace["frames"]), 5)
        self.assertEqual([entry["stage"] for entry in trace["planning_logs"]], ["planner_trace_cli"])
        self.assertEqual(trace["planning_logs"][0]["title"], "搜索回放生成")
        self.assertGreater(trace["summary"]["tracePlanningMs"], 0.0)
        self.assertGreater(trace["planning_timing_ms"]["plannerTraceCli"], 0.0)

    def test_surface_route_preview_rejects_non_traversable_point(self):
        preview = self._cached_invalid_surface_preview()

        self.assertFalse(preview["success"])
        self.assertEqual(preview["failure_code"], "START_POINT_NOT_PROJECTABLE")
        self.assertIn("planning_logs", preview)
        self.assertEqual(preview["planning_logs"][-1]["level"], "error")
        self.assertEqual(preview["planning_logs"][-1]["stage"], "surface_route_cli")
        self.assertEqual(preview["planning_logs"][-1]["title"], "表面路线预览")

    def test_local_planner_scenarios_list_contains_trace_fixtures(self):
        scenarios = SERVER.list_local_planner_scenarios()
        scenario_names = {item["name"] for item in scenarios}

        self.assertIn("pass_straight", scenario_names)
        self.assertIn("waiting_on_block", scenario_names)
        self.assertIn("rotate_recovery", scenario_names)
        self.assertIn("local_collision_blocked", scenario_names)

    def test_live_bridge_state_includes_local_planner_runtime_fields(self):
        bridge = SERVER.LiveRosBridge()

        self.assertIn("controlState", bridge.state)
        self.assertIn("motionModeState", bridge.state)
        self.assertIn("localPlannerPreviewPath", bridge.state)
        self.assertIn("localPlannerState", bridge.state)
        self.assertIn("recoveryState", bridge.state)
        self.assertIn("semanticSummary", bridge.state)
        self.assertIn("operatorStatus", bridge.state)
        self.assertIn("mechanismState", bridge.state)
        self.assertIn("btSnapshot", bridge.state)
        self.assertIn("btEvents", bridge.state)

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
