#!/usr/bin/env python3
"""Regression tests for dense surface graph generation."""

from __future__ import annotations

import importlib.util
import io
import os
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from unittest import mock

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
RUN_FULL_SURFACE_GRAPH_REGEN = os.environ.get("RC26_RUN_FULL_SURFACE_GRAPH_REGEN") == "1"


def load_yaml(path: Path):
    with path.open(encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def scan_graph_manifest(path: Path):
    manifest = {
        "team": None,
        "schema_version": None,
        "source": None,
        "node_count": 0,
        "edge_count": 0,
        "node_clearance_annotations": 0,
        "node_pitch_annotations": 0,
        "edge_clearance_annotations": 0,
        "edge_slope_annotations": 0,
    }
    in_nodes = False
    in_edges = False

    with path.open(encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.rstrip("\n")
            stripped = line.strip()

            if stripped == "nodes:":
                in_nodes = True
                in_edges = False
                continue
            if stripped == "edges:":
                in_nodes = False
                in_edges = True
                continue
            if stripped in {"tasks:", "routes:"}:
                in_nodes = False
                in_edges = False

            if manifest["team"] is None and stripped.startswith("team:"):
                manifest["team"] = stripped.split(":", 1)[1].strip().strip("'\"")
            if manifest["schema_version"] is None and stripped.startswith("schema_version:"):
                manifest["schema_version"] = stripped.split(":", 1)[1].strip().strip("'\"")
            if manifest["source"] is None and stripped.startswith("source:"):
                manifest["source"] = stripped.split(":", 1)[1].strip().strip("'\"")

            if in_nodes and line.startswith("- id:"):
                manifest["node_count"] += 1
            elif in_edges and line.startswith("- id:"):
                manifest["edge_count"] += 1

            if in_nodes and "center_clearance_m:" in stripped:
                manifest["node_clearance_annotations"] += 1
            if in_nodes and "surface_pitch_deg:" in stripped:
                manifest["node_pitch_annotations"] += 1
            if in_edges and "center_clearance_m:" in stripped:
                manifest["edge_clearance_annotations"] += 1
            if in_edges and "slope_deg:" in stripped:
                manifest["edge_slope_annotations"] += 1

    return manifest


class GenerateSurfaceGraphTest(unittest.TestCase):
    @staticmethod
    def make_smoke_world_context():
        return {
            "field_pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
            "includes": [],
            "scene_features": [
                {
                    "id": "surface-a",
                    "name": "smoke-floor",
                    "render_class": "world-ground",
                    "area_xy": 1.0,
                    "points": [
                        {"x": 0.0, "y": 0.0, "z": 0.0},
                        {"x": 1.0, "y": 0.0, "z": 0.0},
                        {"x": 1.0, "y": 1.0, "z": 0.0},
                        {"x": 0.0, "y": 1.0, "z": 0.0},
                    ],
                }
            ],
        }

    def run_main_with_stub_world(self, *args: object):
        stdout = io.StringIO()
        stderr = io.StringIO()
        with mock.patch.object(
            GEN.RENDER,
            "parse_world_context",
            return_value=self.make_smoke_world_context(),
        ), mock.patch.object(
            sys,
            "argv",
            ["generate_surface_graph.py", *[str(arg) for arg in args]],
        ), redirect_stdout(stdout), redirect_stderr(stderr):
            exit_code = GEN.main()
        return exit_code, stdout.getvalue(), stderr.getvalue()

    @unittest.skipUnless(
        RUN_FULL_SURFACE_GRAPH_REGEN,
        "full checked-in surface graph parse/validate is opt-in; set RC26_RUN_FULL_SURFACE_GRAPH_REGEN=1",
    )
    def test_checked_in_surface_graphs_validate_cleanly(self):
        for expected in (EXPECTED_BLUE, EXPECTED_RED):
            errors = VAL.validate_graph(load_yaml(expected))
            self.assertEqual(errors, [], f"{expected.name} validation errors: {errors}")

    def test_checked_in_surface_graph_manifests_match_expected_density(self):
        blue = scan_graph_manifest(EXPECTED_BLUE)
        red = scan_graph_manifest(EXPECTED_RED)

        self.assertEqual(blue["team"], "blue")
        self.assertEqual(red["team"], "red")
        self.assertEqual(blue["schema_version"], "1.1")
        self.assertEqual(red["schema_version"], "1.1")
        self.assertEqual(blue["source"], "robocon2026_surface_graph_body_v1")
        self.assertEqual(red["source"], "robocon2026_surface_graph_body_v1")
        self.assertEqual(blue["node_count"], 4758)
        self.assertEqual(red["node_count"], 4758)
        self.assertEqual(blue["edge_count"], 33990)
        self.assertEqual(red["edge_count"], 33990)
        self.assertGreater(blue["node_clearance_annotations"], 0)
        self.assertGreater(red["node_clearance_annotations"], 0)
        self.assertGreater(blue["node_pitch_annotations"], 0)
        self.assertGreater(red["node_pitch_annotations"], 0)
        self.assertGreater(blue["edge_clearance_annotations"], 0)
        self.assertGreater(red["edge_clearance_annotations"], 0)
        self.assertGreater(blue["edge_slope_annotations"], 0)
        self.assertGreater(red["edge_slope_annotations"], 0)

    @unittest.skipUnless(
        RUN_FULL_SURFACE_GRAPH_REGEN,
        "full checked-in surface graph summary parse is opt-in; set RC26_RUN_FULL_SURFACE_GRAPH_REGEN=1",
    )
    def test_validate_graph_summary_reports_surface_density(self):
        summary = VAL.graph_summary(load_yaml(EXPECTED_BLUE))
        self.assertEqual(summary["team"], "blue")
        self.assertEqual(summary["schema_version"], "1.1")
        self.assertEqual(summary["node_count"], 4758)
        self.assertEqual(summary["edge_count"], 33990)
        self.assertEqual(summary["surface_node_count"], 4758)
        self.assertEqual(summary["surface_edge_count"], 33990)
        self.assertIn("schema=1.1", VAL.format_graph_summary(summary))

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

    def test_canonicalize_surface_graph_document_sorts_graph_collections(self):
        document = {
            "meta": {"team": "blue", "schema_version": "1.1"},
            "nodes": [{"id": "n2"}, {"id": "n1"}],
            "edges": [{"id": "e2"}, {"id": "e1"}],
            "tasks": [{"task_tag": "task_b"}, {"task_tag": "task_a"}],
            "routes": [{"route_tag": "route_b"}, {"route_tag": "route_a"}],
        }

        canonical = GEN.canonicalize_graph_document(document)

        self.assertEqual([item["id"] for item in canonical["nodes"]], ["n1", "n2"])
        self.assertEqual([item["id"] for item in canonical["edges"]], ["e1", "e2"])
        self.assertEqual([item["task_tag"] for item in canonical["tasks"]], ["task_a", "task_b"])
        self.assertEqual([item["route_tag"] for item in canonical["routes"]], ["route_a", "route_b"])

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

    def test_cli_smoke_generates_and_checks_existing_small_surface_graph(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            out = Path(tmp_dir) / "surface_blue.yaml"
            fake_world = Path(tmp_dir) / "synthetic.world"

            completed_code, completed_stdout, completed_stderr = self.run_main_with_stub_world(
                "--team",
                "blue",
                "--world",
                fake_world,
                "--overlay",
                OVERLAY,
                "--out",
                out,
            )
            self.assertEqual(
                completed_code,
                0,
                msg=(
                    "surface graph smoke generate failed:\n"
                    f"stdout:\n{completed_stdout}\nstderr:\n{completed_stderr}"
                ),
            )

            generated = load_yaml(out)
            self.assertEqual(generated["meta"]["team"], "blue")
            self.assertGreater(len(generated["nodes"]), 0)
            self.assertGreater(len(generated["edges"]), 0)
            self.assertIn("center_clearance_m", generated["nodes"][0])
            self.assertIn("surface_pitch_deg", generated["nodes"][0])
            self.assertIn("center_clearance_m", generated["edges"][0])
            self.assertIn("slope_deg", generated["edges"][0])
            self.assertEqual(VAL.validate_graph(generated), [])
            self.assertIn("Generated", completed_stdout)

            check_code, check_stdout, check_stderr = self.run_main_with_stub_world(
                "--team",
                "blue",
                "--world",
                fake_world,
                "--overlay",
                OVERLAY,
                "--out",
                out,
                "--check-existing",
            )
            self.assertEqual(
                check_code,
                0,
                msg=(
                    "surface graph smoke --check-existing failed:\n"
                    f"stdout:\n{check_stdout}\nstderr:\n{check_stderr}"
                ),
            )
            self.assertIn("Surface graph matches existing file", check_stdout)

    @unittest.skipUnless(
        RUN_FULL_SURFACE_GRAPH_REGEN,
        "full surface graph regeneration is opt-in; set RC26_RUN_FULL_SURFACE_GRAPH_REGEN=1",
    )
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

            check_existing = subprocess.run(
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
                    "--check-existing",
                ],
                cwd=WORKSPACE_ROOT,
                capture_output=True,
                text=True,
            )
            self.assertEqual(
                check_existing.returncode,
                0,
                msg=(
                    "surface graph --check-existing failed:\n"
                    f"stdout:\n{check_existing.stdout}\nstderr:\n{check_existing.stderr}"
                ),
            )
            self.assertIn("Surface graph matches existing file", check_existing.stdout)
            self.assertIn("nodes=4758", check_existing.stdout)

    def test_red_surface_graph_keeps_same_density(self):
        red = scan_graph_manifest(EXPECTED_RED)
        self.assertEqual(red["team"], "red")
        self.assertEqual(red["node_count"], 4758)
        self.assertEqual(red["edge_count"], 33990)


if __name__ == "__main__":
    unittest.main()
