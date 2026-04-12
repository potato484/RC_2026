#!/usr/bin/env python3
"""Regression tests for topo + simulation HTML rendering."""

from __future__ import annotations

import importlib.util
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


PKG_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PKG_ROOT.parents[1]
SIM_ROOT = PKG_ROOT / "sim_assets"

GRAPH_BLUE = PKG_ROOT / "config" / "r2_field_graph_blue.yaml"
SIM_WORLD = SIM_ROOT / "worlds" / "robocon2026_v2_aligned.world"
SIM_KFS = SIM_ROOT / "config" / "kfs_config_v2_aligned.yaml"
SIM_MODEL_ROOT = SIM_ROOT / "models"

REN = load_module("render_graph_sim_html", PKG_ROOT / "scripts" / "render_graph_sim_html.py")


class Args:
    blocked_node: list[str] = []
    blocked_edge: list[str] = []
    slow_edge: list[str] = []
    confirm_edge: list[str] = []
    edge_extra_cost: list[str] = []


class RenderGraphSimHtmlTest(unittest.TestCase):
    def test_world_projection_filters_mesh_noise(self):
        world_context = REN.parse_world_context(SIM_WORLD, SIM_MODEL_ROOT)
        scene_features = world_context["scene_features"]
        names = {feature["name"] for feature in scene_features}

        self.assertLess(len(scene_features), 320)
        self.assertNotIn("柱体", names)
        self.assertNotIn("杆架", names)
        self.assertIn("主地图", names)
        self.assertIn("区域三放置架", names)

    def test_viewer_3d_mode_preserves_structural_vertical_faces(self):
        projection_context = REN.parse_world_context(
            SIM_WORLD,
            SIM_MODEL_ROOT,
            filter_noise=False,
        )
        viewer_context = REN.parse_world_context(
            SIM_WORLD,
            SIM_MODEL_ROOT,
            geometry_mode=REN.WORLD_GEOMETRY_MODE_VIEWER_3D,
            filter_noise=False,
        )

        projection_vertical_faces = [
            feature
            for feature in projection_context["scene_features"]
            if feature["name"] == "主地图" and feature["area_xy"] < 1e-4 and feature["z_span"] >= 0.35
        ]
        viewer_vertical_faces = [
            feature
            for feature in viewer_context["scene_features"]
            if feature["name"] == "主地图" and feature["area_xy"] < 1e-4 and feature["z_span"] >= 0.35
        ]

        self.assertEqual(projection_vertical_faces, [])
        self.assertGreater(len(viewer_vertical_faces), 0)
        self.assertGreater(len(viewer_context["scene_features"]), len(projection_context["scene_features"]))

    def test_blue_graph_aligns_to_blue_meilin_slots(self):
        document = REN.load_yaml(GRAPH_BLUE)
        sim_config = REN.load_yaml(SIM_KFS)
        alignment = REN.derive_graph_alignment(document, sim_config, "blue")

        self.assertEqual(alignment["dx"], -4.2)
        self.assertEqual(alignment["dy"], -2.2)
        self.assertEqual(alignment["max_error"], 0.0)

    def test_default_demo_route_reaches_exit_ramp(self):
        document = REN.load_yaml(GRAPH_BLUE)
        overlay_state = REN.build_overlay_state(Args(), document)
        result = REN.plan_route_trace(
            document,
            "ramp_entry_south",
            "ramp_exit_north",
            overlay_state,
            capture_frames=True,
        )

        self.assertTrue(result["success"])
        self.assertAlmostEqual(result["total_cost"], 23.5)
        self.assertEqual(result["node_path"][0], "ramp_entry_south")
        self.assertEqual(result["node_path"][-1], "ramp_exit_north")
        self.assertIn("e_ramp_up_south", result["edge_path"])
        self.assertIn("e_ramp_down_north", result["edge_path"])
        self.assertGreater(len(result["frames"]), 20)

    def test_html_contains_world_alignment_and_trace(self):
        document = REN.load_yaml(GRAPH_BLUE)
        sim_config = REN.load_yaml(SIM_KFS)
        alignment = REN.derive_graph_alignment(document, sim_config, "blue")
        world_context = REN.parse_world_context(SIM_WORLD, SIM_MODEL_ROOT)
        meilin_slots = REN.build_meilin_slots(sim_config, "blue")
        overlay_state = REN.build_overlay_state(Args(), document)
        planning = REN.run_planning(
            document,
            type(
                "PlanningArgs",
                (),
                {
                    "start": None,
                    "goal_node": None,
                    "goal_task": None,
                    "goal_route": None,
                },
            )(),
            overlay_state,
        )
        render_model = REN.build_render_model(
            document,
            alignment=alignment,
            scene_features=world_context["scene_features"],
            meilin_slots=meilin_slots,
            planning=planning,
        )
        html_text = REN.render_html(
            GRAPH_BLUE,
            document,
            render_model,
            planning,
            overlay_state,
            alignment,
            world_context,
            type("PageArgs", (), {"title": None})(),
        )

        self.assertIn("RC26 拓扑导航仿真观察页", html_text)
        self.assertIn("拓扑图到仿真场地的自动对齐结果", html_text)
        self.assertIn("ramp_entry_south", html_text)
        self.assertIn("ramp_exit_north", html_text)
        self.assertIn("主地图", html_text)
        self.assertIn("三维高度语义", html_text)
        self.assertIn("高度变化 dZ", html_text)
        self.assertIn("\"frames\"", html_text)

    def test_cli_writes_sim_html_file(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            out = Path(tmp_dir) / "graph_sim.html"
            completed = subprocess.run(
                [
                    sys.executable,
                    str(PKG_ROOT / "scripts" / "render_graph_sim_html.py"),
                    "--graph",
                    str(GRAPH_BLUE),
                    "--world",
                    str(SIM_WORLD),
                    "--kfs-config",
                    str(SIM_KFS),
                    "--out",
                    str(out),
                ],
                cwd=WORKSPACE_ROOT,
                capture_output=True,
                text=True,
            )
            self.assertEqual(
                completed.returncode,
                0,
                msg=f"render failed:\nstdout:\n{completed.stdout}\nstderr:\n{completed.stderr}",
            )
            self.assertTrue(out.is_file())
            html_text = out.read_text(encoding="utf-8")
            self.assertIn("<!DOCTYPE html>", html_text)
            self.assertIn("RC26 拓扑导航仿真观察页", html_text)
            self.assertIn("蓝方", html_text)
            self.assertIn("拓扑图到仿真场地的自动对齐结果", html_text)
            self.assertIn("ramp_exit_north", html_text)


if __name__ == "__main__":
    unittest.main()
