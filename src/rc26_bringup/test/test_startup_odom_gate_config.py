import importlib.util
import subprocess
from pathlib import Path

import yaml
from launch import LaunchContext


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


def test_default_configs_expose_compat_pickup_to_final_delay():
    for name in ("r2_red.yaml", "r2_blue.yaml"):
        params = _decision_params(PACKAGE_ROOT / "config" / name)
        assert (
            params["second_preselect_climb_place_compat_pickup_to_final_delay_s"]
            == 10.0
        )


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


def test_managed_preselection_tree_selects_compat_profile_only_for_second():
    module = _load_bringup_launch_module()
    default_tree = "/tmp/custom_tree.xml"

    assert (
        module._managed_preselection_tree(default_tree, "second", False)
        == module.SECOND_PRESELECTION_TREE
    )
    assert (
        module._managed_preselection_tree(default_tree, "second", True)
        == module.SECOND_PRESELECTION_KFS_COMPAT_TREE
    )
    assert (
        module._managed_preselection_tree(default_tree, "first", True)
        == module.FIRST_PRESELECTION_TREE
    )
    assert module._managed_preselection_tree(default_tree, None, True) == default_tree


def test_active_side_selector_defaults_compatibility_off():
    with (PACKAGE_ROOT / "config" / "r2_active_side.yaml").open(
        "r", encoding="utf-8"
    ) as stream:
        selector = yaml.safe_load(stream) or {}
    assert selector["second_preselection_kfs_search_compat_enable"] is False


def test_compat_selector_reads_true_and_is_disabled_by_runtime_override(tmp_path):
    module = _load_bringup_launch_module()
    selector_path = tmp_path / "r2_active_side.yaml"
    selector_path.write_text(
        "second_preselection_kfs_search_compat_enable: true\n",
        encoding="utf-8",
    )
    context = LaunchContext()
    context.launch_configurations["runtime_config_file"] = ""
    context.launch_configurations["side_config_file"] = str(selector_path)

    assert module._resolve_second_preselection_kfs_search_compat_enable(
        context, str(PACKAGE_ROOT)
    ) is True

    context.launch_configurations["runtime_config_file"] = "/tmp/explicit.yaml"
    assert module._resolve_second_preselection_kfs_search_compat_enable(
        context, str(PACKAGE_ROOT)
    ) is False


def test_start_r2_auto_dry_run_reports_disabled_startup_gate():
    result = subprocess.run(
        [str(WORKSPACE_ROOT / "start_r2_auto.sh"), "--dry-run"],
        cwd=WORKSPACE_ROOT,
        check=True,
        capture_output=True,
        text=True,
    )

    assert "Startup odom gate: false" in result.stdout
    assert "Second KFS search compatibility: false" in result.stdout
