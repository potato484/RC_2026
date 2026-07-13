import importlib.util
import subprocess
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "bringup.launch.py"


def _load_bringup_launch_module():
    spec = importlib.util.spec_from_file_location("rc26_bringup_launch", LAUNCH_FILE)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _decision_params(config_path):
    with config_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    return data["r2_runtime"]["decision"]["ros__parameters"]


def test_default_red_blue_configs_disable_global_startup_odom_gate():
    for name in ("r2_red.yaml", "r2_blue.yaml"):
        params = _decision_params(PACKAGE_ROOT / "config" / name)
        assert params["startup_wait_for_odom"] is False


def test_bringup_honors_runtime_gate_setting_and_keeps_legacy_fallback():
    module = _load_bringup_launch_module()

    assert module._resolve_startup_wait_for_odom(
        {"decision_params": {"startup_wait_for_odom": False}}, True
    ) is False
    assert module._resolve_startup_wait_for_odom(
        {"decision_params": {"startup_wait_for_odom": "true"}}, True
    ) is True
    assert module._resolve_startup_wait_for_odom({"decision_params": {}}, True) is True
    assert module._resolve_startup_wait_for_odom({"decision_params": {}}, False) is False


def test_start_r2_auto_dry_run_reports_disabled_startup_gate():
    result = subprocess.run(
        [str(WORKSPACE_ROOT / "start_r2_auto.sh"), "--dry-run"],
        cwd=WORKSPACE_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )

    assert "Startup odom gate: false" in result.stdout
