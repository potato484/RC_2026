#!/usr/bin/env python3
"""Smoke tests for the local planner trace CLI."""

from __future__ import annotations

import json
import subprocess
import sys
import unittest
from pathlib import Path


EXPECTED_SCENARIOS = {
    "pass_straight": {
        "status": "PASS",
        "has_solution": True,
    },
    "waiting_on_block": {
        "status": "WAITING_ON_BLOCK",
        "has_solution": False,
    },
    "rotate_recovery": {
        "status": "RECOVERY_RUNNING",
        "has_solution": False,
    },
    "local_collision_blocked": {
        "status": "LOCAL_COLLISION_BLOCKED",
        "has_solution": False,
    },
}


class LocalPlannerTraceCliTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        if len(sys.argv) != 3:
            raise RuntimeError(
                "usage: test_local_planner_trace_cli.py <cli_binary> <scenarios_dir>"
            )
        cls.cli_binary = Path(sys.argv[1]).resolve()
        cls.scenarios_dir = Path(sys.argv[2]).resolve()
        if not cls.cli_binary.is_file():
            raise FileNotFoundError(f"local_planner_trace_cli not found: {cls.cli_binary}")
        if not cls.scenarios_dir.is_dir():
            raise FileNotFoundError(f"scenario directory not found: {cls.scenarios_dir}")

    def run_trace(self, scenario_name: str) -> dict:
        snapshot_file = self.scenarios_dir / f"{scenario_name}.yaml"
        completed = subprocess.run(
            [str(self.cli_binary), "--snapshot", str(snapshot_file)],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(
            completed.returncode,
            0,
            msg=(
                f"trace cli failed for {scenario_name}\n"
                f"stdout:\n{completed.stdout}\n"
                f"stderr:\n{completed.stderr}"
            ),
        )
        try:
            return json.loads(completed.stdout)
        except json.JSONDecodeError as exc:  # pragma: no cover - assertion already explains context
            raise AssertionError(
                f"trace cli returned invalid json for {scenario_name}:\n{completed.stdout}"
            ) from exc

    def test_trace_cli_emits_expected_status_per_scenario(self) -> None:
        for scenario_name, expected in EXPECTED_SCENARIOS.items():
            with self.subTest(scenario=scenario_name):
                payload = self.run_trace(scenario_name)
                self.assertTrue(payload["success"])
                self.assertEqual(payload["snapshotLabel"], scenario_name)
                self.assertEqual(payload["traceMode"], "local_planner")
                self.assertEqual(payload["result"]["status"], expected["status"])
                self.assertEqual(payload["summary"]["finalStatus"], expected["status"])
                self.assertEqual(payload["result"]["hasSolution"], expected["has_solution"])
                self.assertGreaterEqual(payload["summary"]["candidateCount"], 1)
                self.assertGreaterEqual(len(payload["frames"]), 1)
                self.assertEqual(payload["frames"][-1]["algorithm"], "local_planner")
                self.assertEqual(payload["frames"][-1]["phase"], "result")

    def test_trace_cli_requires_snapshot_argument(self) -> None:
        completed = subprocess.run(
            [str(self.cli_binary)],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertNotEqual(completed.returncode, 0)
        self.assertIn("Missing required --snapshot", completed.stderr)


if __name__ == "__main__":
    unittest.main(argv=sys.argv[:1])
