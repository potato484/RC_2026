#!/usr/bin/env python3
"""Regression tests for topo graph generation."""

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

GEN = load_module("generate_graph", PKG_ROOT / "scripts" / "generate_graph.py")
VAL = load_module("validate_graph", PKG_ROOT / "scripts" / "validate_graph.py")

WORLD_LAYOUT = WORKSPACE_ROOT / "src" / "rc26_kfs_keepout" / "config" / "r2_mf_world.yaml"
OVERLAY = PKG_ROOT / "config" / "r2_field_graph_overlay.yaml"
EXPECTED_BLUE = PKG_ROOT / "config" / "r2_field_graph_blue.yaml"
EXPECTED_RED = PKG_ROOT / "config" / "r2_field_graph_red.yaml"


def load_yaml(path: Path):
    with path.open(encoding="utf-8") as f:
        return yaml.safe_load(f)


class GenerateGraphTest(unittest.TestCase):
    def test_manual_semantic_overrides_survive_generation(self):
        generated = GEN.build_graph_document(
            load_yaml(WORLD_LAYOUT),
            load_yaml(OVERLAY),
            team="blue",
            world_layout_name=WORLD_LAYOUT.name,
            overlay_name=OVERLAY.name,
        )
        nodes = {node["id"]: node for node in generated["nodes"]}
        edges = {edge["id"]: edge for edge in generated["edges"]}

        self.assertEqual(nodes["mf_b2"]["base_cost"], 0.5)
        self.assertEqual(nodes["mf_b4"]["base_cost"], 1.0)
        self.assertEqual(nodes["mf_b6"]["base_cost"], 1.5)
        self.assertEqual(nodes["mf_b8"]["base_cost"], 1.5)
        self.assertEqual(nodes["mf_b10"]["base_cost"], 1.0)
        self.assertEqual(nodes["mf_b12"]["base_cost"], 1.0)
        self.assertEqual(edges["e_b3_b6"]["base_cost"], 1.5)
        self.assertEqual(edges["e_b3_b6"]["height_change"], 0.2)
        self.assertEqual(edges["e_b6_b3"]["base_cost"], 1.5)
        self.assertEqual(edges["e_b6_b3"]["height_change"], 0.2)

    def test_generated_blue_graph_matches_checked_in_output(self):
        generated = GEN.build_graph_document(
            load_yaml(WORLD_LAYOUT),
            load_yaml(OVERLAY),
            team="blue",
            world_layout_name=WORLD_LAYOUT.name,
            overlay_name=OVERLAY.name,
        )
        expected = load_yaml(EXPECTED_BLUE)
        self.assertEqual(
            GEN.canonicalize_graph_document(generated),
            GEN.canonicalize_graph_document(expected),
        )

    def test_generated_red_graph_matches_checked_in_output(self):
        generated = GEN.build_graph_document(
            load_yaml(WORLD_LAYOUT),
            load_yaml(OVERLAY),
            team="red",
            world_layout_name=WORLD_LAYOUT.name,
            overlay_name=OVERLAY.name,
        )
        expected = load_yaml(EXPECTED_RED)
        self.assertEqual(
            GEN.canonicalize_graph_document(generated),
            GEN.canonicalize_graph_document(expected),
        )

    def test_generated_graph_validates_cleanly(self):
        for team in ("blue", "red"):
            generated = GEN.build_graph_document(
                load_yaml(WORLD_LAYOUT),
                load_yaml(OVERLAY),
                team=team,
                world_layout_name=WORLD_LAYOUT.name,
                overlay_name=OVERLAY.name,
            )
            errors = VAL.validate_graph(generated)
            self.assertEqual(errors, [], f"{team} graph validation errors: {errors}")

    def test_cli_check_existing_succeeds_for_checked_in_graphs(self):
        for team, expected in (("blue", EXPECTED_BLUE), ("red", EXPECTED_RED)):
            completed = subprocess.run(
                [
                    sys.executable,
                    str(PKG_ROOT / "scripts" / "generate_graph.py"),
                    "--world-layout",
                    str(WORLD_LAYOUT),
                    "--overlay",
                    str(OVERLAY),
                    "--team",
                    team,
                    "--out",
                    str(expected),
                    "--check-existing",
                ],
                cwd=WORKSPACE_ROOT,
                capture_output=True,
                text=True,
            )
            self.assertEqual(
                completed.returncode,
                0,
                msg=f"{team} check failed:\nstdout:\n{completed.stdout}\nstderr:\n{completed.stderr}",
            )

    def test_cli_generates_valid_file(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            out = Path(tmp_dir) / "generated.yaml"
            completed = subprocess.run(
                [
                    sys.executable,
                    str(PKG_ROOT / "scripts" / "generate_graph.py"),
                    "--world-layout",
                    str(WORLD_LAYOUT),
                    "--overlay",
                    str(OVERLAY),
                    "--team",
                    "blue",
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
                msg=f"generate failed:\nstdout:\n{completed.stdout}\nstderr:\n{completed.stderr}",
            )
            errors = VAL.validate_graph(load_yaml(out))
            self.assertEqual(errors, [])


if __name__ == "__main__":
    unittest.main()
