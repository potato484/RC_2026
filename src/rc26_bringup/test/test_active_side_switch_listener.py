from __future__ import annotations

import importlib.util
import os
import tempfile
import unittest
from pathlib import Path

import yaml


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "active_side_switch_listener.py"
)
SPEC = importlib.util.spec_from_file_location(
    "active_side_switch_listener", SCRIPT_PATH
)
assert SPEC is not None
assert SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


SELECTOR_TEMPLATE = """\
# selector
active_side: {side}
preselection_mode: first
first_preselection_mc_repeat_enable: true
first_preselection_mc_repeat_max_count: 2
first_preselection_mc_repeat_forward_x_step_m:
  red: 0.2
  blue: -0.2
runtime_configs:
  red: "r2_red.yaml"
  blue: "r2_blue.yaml"
"""


class ActiveSideSwitchListenerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.directory = Path(self.temp_dir.name)
        self.selector = self.directory / "r2_active_side.yaml"
        self.selector.write_text(
            SELECTOR_TEMPLATE.format(side="red"), encoding="utf-8"
        )

    def tearDown(self) -> None:
        self.temp_dir.cleanup()

    def _side(self) -> str:
        data = yaml.safe_load(self.selector.read_text(encoding="utf-8"))
        return str(data["active_side"])

    def test_toggle_commits_and_cleans_transaction_files(self) -> None:
        old_side, new_side = MODULE.toggle_active_side_file(self.selector)

        self.assertEqual(("red", "blue"), (old_side, new_side))
        self.assertEqual("blue", self._side())
        self.assertEqual(
            [],
            list(self.directory.glob(".r2_active_side.yaml.*")),
        )

    def test_recovers_durable_pending_file(self) -> None:
        pending = self.directory / ".r2_active_side.yaml.pending"
        pending.write_text(
            SELECTOR_TEMPLATE.format(side="blue"), encoding="utf-8"
        )

        recovered = MODULE.recover_interrupted_selector_write(self.selector)

        self.assertEqual(("red", "blue"), recovered)
        self.assertEqual("blue", self._side())
        self.assertFalse(pending.exists())

    def test_recovers_legacy_complete_temp_file(self) -> None:
        staged = self.directory / ".r2_active_side.yaml.crash.tmp"
        staged.write_text(
            SELECTOR_TEMPLATE.format(side="blue"), encoding="utf-8"
        )
        target_mtime = self.selector.stat().st_mtime_ns
        os.utime(staged, ns=(target_mtime + 1, target_mtime + 1))

        recovered = MODULE.recover_interrupted_selector_write(self.selector)

        self.assertEqual(("red", "blue"), recovered)
        self.assertEqual("blue", self._side())
        self.assertFalse(staged.exists())

    def test_recovers_incomplete_newer_temp_as_toggle_intent(self) -> None:
        staged = self.directory / ".r2_active_side.yaml.crash.tmp"
        staged.write_text("active_side:", encoding="utf-8")
        target_mtime = self.selector.stat().st_mtime_ns
        os.utime(staged, ns=(target_mtime + 1, target_mtime + 1))

        recovered = MODULE.recover_interrupted_selector_write(self.selector)

        self.assertEqual(("red", "blue"), recovered)
        self.assertEqual("blue", self._side())
        self.assertFalse(staged.exists())

    def test_ignores_stale_temp_older_than_target(self) -> None:
        staged = self.directory / ".r2_active_side.yaml.stale.tmp"
        staged.write_text(
            SELECTOR_TEMPLATE.format(side="blue"), encoding="utf-8"
        )
        target_mtime = self.selector.stat().st_mtime_ns
        os.utime(staged, ns=(target_mtime - 1, target_mtime - 1))

        recovered = MODULE.recover_interrupted_selector_write(self.selector)

        self.assertIsNone(recovered)
        self.assertEqual("red", self._side())


if __name__ == "__main__":
    unittest.main()
