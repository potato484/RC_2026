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


if __name__ == "__main__":
    unittest.main()
