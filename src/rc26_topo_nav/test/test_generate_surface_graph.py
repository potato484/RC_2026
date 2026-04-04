#!/usr/bin/env python3
"""Regression tests for dense surface graph generation."""

from __future__ import annotations

import importlib.util
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import yaml


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


PKG_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PKG_ROOT.parents[1]

GEN = load_module("generate_surface_graph", PKG_ROOT / "scripts" / "generate_surface_graph.py")
VAL = load_module("validate_graph", PKG_ROOT / "scripts" / "validate_graph.py")

WORLD_FILE = PKG_ROOT / "sim_assets" / "worlds" / "robocon2026_v2_aligned.world"
OVERLAY = PKG_ROOT / "config" / "r2_surface_graph_overlay.yaml"
EXPECTED_BLUE = PKG_ROOT / "config" / "r2_surface_graph_blue.yaml"
EXPECTED_RED = PKG_ROOT / "config" / "r2_surface_graph_red.yaml"


def load_yaml(path: Path):
    with path.open(encoding="utf-8") as handle:
        return yaml.safe_load(handle)


class GenerateSurfaceGraphTest(unittest.TestCase):
    def test_checked_in_surface_graphs_validate_cleanly(self):
        for expected in (EXPECTED_BLUE, EXPECTED_RED):
            errors = VAL.validate_graph(load_yaml(expected))
            self.assertEqual(errors, [], f"{expected.name} validation errors: {errors}")

    def test_dump_graph_keeps_expected_meta_shape(self):
        document = GEN.dump_graph(
            team="blue",
            world_file=WORLD_FILE,
            overlay_file=OVERLAY,
            nodes=[],
            edges=[],
        )

        self.assertEqual(document["meta"]["team"], "blue")
        self.assertEqual(document["meta"]["source"], "robocon2026_surface_graph_body_v1")
        self.assertEqual(document["meta"]["grid_spacing_m"], 0.18)
        self.assertEqual(document["tasks"], [])
        self.assertEqual(document["routes"], [])

    def test_build_nodes_adds_clearance_and_pitch_annotations(self):
        scene_features = [
            {
                "id": "surface-a",
                "name": "test-surface",
                "render_class": "world-ground",
                "area_xy": 1.0,
                "points": [
                    {"x": 0.0, "y": 0.0, "z": 0.0},
                    {"x": 1.0, "y": 0.0, "z": 0.0},
                    {"x": 1.0, "y": 1.0, "z": 0.0},
                    {"x": 0.0, "y": 1.0, "z": 0.0},
                ],
            }
        ]
        nodes = GEN.build_nodes(
            scene_features,
            spacing_m=0.5,
            min_surface_area_xy=0.01,
            traversable_render_classes={"world-ground"},
            excluded_feature_names=set(),
        )
        self.assertGreaterEqual(len(nodes), 1)
        self.assertIn("center_clearance_m", nodes[0])
        self.assertIn("surface_pitch_deg", nodes[0])
        self.assertGreater(nodes[0]["center_clearance_m"], 0.0)
        self.assertEqual(nodes[0]["surface_pitch_deg"], 0.0)

    def test_build_edges_adds_body_aware_annotations(self):
        scene_features = [
            {
                "id": "surface-a",
                "name": "test-surface",
                "render_class": "world-ground",
                "area_xy": 1.0,
                "points": [
                    {"x": 0.0, "y": 0.0, "z": 0.0},
                    {"x": 1.0, "y": 0.0, "z": 0.0},
                    {"x": 1.0, "y": 1.0, "z": 0.0},
                    {"x": 0.0, "y": 1.0, "z": 0.0},
                ],
            }
        ]
        surface_records, surface_regions = GEN.build_surface_regions(
            scene_features,
            min_surface_area_xy=0.01,
            traversable_render_classes={"world-ground"},
            excluded_feature_names=set(),
        )
        nodes = [
            {
                "id": "sf_0_0",
                "type": "surface_point",
                "pose": {"x": 0.25, "y": 0.25, "z": 0.0, "yaw": 0.0},
                "surface_id": "surface-a",
            },
            {
                "id": "sf_0_1",
                "type": "surface_point",
                "pose": {"x": 0.75, "y": 0.25, "z": 0.0, "yaw": 0.0},
                "surface_id": "surface-a",
            },
        ]
        edges = GEN.build_edges(
            nodes,
            surface_records,
            surface_regions,
            neighbor_radius_m=0.8,
            transition_radius_m=0.4,
            max_transition_height_m=0.2,
            plane_height_epsilon_m=0.05,
            clearance_sample_spacing_m=0.05,
        )
        self.assertEqual(len(edges), 2)
        self.assertIn("center_clearance_m", edges[0])
        self.assertIn("slope_deg", edges[0])
        self.assertIn("horizontal_length_m", edges[0])
        self.assertTrue(edges[0]["same_surface"])
        self.assertGreater(edges[0]["center_clearance_m"], 0.0)

    def test_cli_generates_expected_blue_surface_graph(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            out = Path(tmp_dir) / "surface_blue.yaml"
            completed = subprocess.run(
                [
                    sys.executable,
                    str(PKG_ROOT / "scripts" / "generate_surface_graph.py"),
                    "--team",
                    "blue",
                    "--world",
                    str(WORLD_FILE),
                    "--overlay",
                    str(OVERLAY),
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
                msg=f"surface graph generate failed:\nstdout:\n{completed.stdout}\nstderr:\n{completed.stderr}",
            )

            generated = load_yaml(out)
            expected = load_yaml(EXPECTED_BLUE)
            self.assertEqual(generated["meta"]["team"], "blue")
            self.assertEqual(len(generated["nodes"]), 4758)
            self.assertEqual(len(generated["edges"]), 33990)
            self.assertIn("center_clearance_m", generated["nodes"][0])
            self.assertIn("surface_pitch_deg", generated["nodes"][0])
            self.assertIn("center_clearance_m", generated["edges"][0])
            self.assertIn("slope_deg", generated["edges"][0])
            self.assertEqual(generated["nodes"][0], expected["nodes"][0])
            self.assertEqual(generated["nodes"][-1], expected["nodes"][-1])
            self.assertEqual(generated["edges"][0], expected["edges"][0])
            self.assertEqual(generated["edges"][-1], expected["edges"][-1])
            self.assertEqual(VAL.validate_graph(generated), [])

    def test_red_surface_graph_keeps_same_density(self):
        red = load_yaml(EXPECTED_RED)
        self.assertEqual(red["meta"]["team"], "red")
        self.assertEqual(len(red["nodes"]), 4758)
        self.assertEqual(len(red["edges"]), 33990)


if __name__ == "__main__":
    unittest.main()
