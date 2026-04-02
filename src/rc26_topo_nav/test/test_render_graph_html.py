#!/usr/bin/env python3
"""Regression tests for static topo graph HTML rendering."""

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
GRAPH_BLUE = PKG_ROOT / "config" / "r2_field_graph_blue.yaml"

REN = load_module("render_graph_html", PKG_ROOT / "scripts" / "render_graph_html.py")


class RenderGraphHtmlTest(unittest.TestCase):
    def test_render_model_has_no_route_warnings_for_checked_in_graph(self):
        document = REN.load_yaml(GRAPH_BLUE)
        model = REN.build_render_model(document, GRAPH_BLUE.name)
        self.assertEqual(model["warnings"], [])
        self.assertEqual(len(model["rendered_nodes"]), len(document["nodes"]))
        self.assertEqual(len(model["rendered_edges"]), len(document["edges"]))

    def test_html_contains_core_graph_sections(self):
        document = REN.load_yaml(GRAPH_BLUE)
        html_text = REN.render_html_document(document, GRAPH_BLUE.name)

        self.assertIn("<svg", html_text)
        self.assertIn("mf_b1", html_text)
        self.assertIn("e_b3_b6", html_text)
        self.assertIn("mf_grab", html_text)
        self.assertIn("mf_entry_default", html_text)
        self.assertIn("RC26 Topo Graph Viewer", html_text)

    def test_cli_writes_html_file(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            out = Path(tmp_dir) / "graph.html"
            completed = subprocess.run(
                [
                    sys.executable,
                    str(PKG_ROOT / "scripts" / "render_graph_html.py"),
                    "--graph",
                    str(GRAPH_BLUE),
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
            self.assertIn("r2_field_graph_blue.yaml", html_text)
            self.assertIn("data-route-tag=\"mf_entry_default\"", html_text)
            self.assertIn("data-task-tag=\"mf_grab\"", html_text)


if __name__ == "__main__":
    unittest.main()
