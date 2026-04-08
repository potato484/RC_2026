#!/usr/bin/env python3

import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

REQUIRED_GROUPS = {
    "navigation_operator.rviz": ["场景基础", "导航态势", "地形安全"],
    "navigation_engineering.rviz": ["场景基础", "导航态势", "点云诊断", "地形安全"],
    "navigation_diagnostic.rviz": ["场景基础", "导航态势", "点云诊断", "地形安全"],
    "slam_operator.rviz": ["场景基础", "建图点云", "轨迹对比", "地形安全"],
    "slam_engineering.rviz": ["场景基础", "建图点云", "轨迹对比", "点云诊断", "地形安全"],
    "slam_diagnostic.rviz": ["场景基础", "建图点云", "点云诊断", "地形安全"],
}

EXPECTATIONS = {
    "navigation_operator.rviz": [
        "Fixed Frame: map",
        "Frame Rate: 30",
        "Value: /xhu_nav/route",
        "Value: /xhu_nav/corridor",
        "Value: /xhu_nav/lookahead_path",
    ],
    "navigation_engineering.rviz": [
        "Fixed Frame: map",
        "Frame Rate: 20",
        "Value: registered_scan",
        "Value: terrain_obstacles",
        "Value: terrain_drop",
    ],
    "navigation_diagnostic.rviz": [
        "Fixed Frame: map",
        "Frame Rate: 15",
        "Value: laser_map_full",
        "Name: 保持禁行区",
    ],
    "slam_operator.rviz": [
        "Fixed Frame: odom",
        "Frame Rate: 30",
        "Value: registered_scan",
        "Value: terrain_grid_map_local",
    ],
    "slam_engineering.rviz": [
        "Fixed Frame: odom",
        "Frame Rate: 20",
        "Name: TF",
        "Value: curvature_points_marker_array",
    ],
    "slam_diagnostic.rviz": [
        "Fixed Frame: odom",
        "Frame Rate: 15",
        "Name: 保持禁行区",
        "Value: terrain_drop",
    ],
}


def main() -> int:
    config_dir = Path(get_package_share_directory("rc26_xhu_viewer")) / "config"
    failures = []

    all_files = set(EXPECTATIONS.keys()) | set(REQUIRED_GROUPS.keys())
    for filename in sorted(all_files):
        file_path = config_dir / filename
        if not file_path.is_file():
            failures.append(f"missing file: {file_path}")
            continue
        content = file_path.read_text(encoding="utf-8")

        for group_name in REQUIRED_GROUPS.get(filename, []):
            if f"Name: {group_name}" not in content:
                failures.append(f"{filename}: missing group '{group_name}'")

        for token in EXPECTATIONS.get(filename, []):
            if token not in content:
                failures.append(f"{filename}: missing token '{token}'")

    if failures:
        for item in failures:
            print(f"[FAIL] {item}")
        return 1

    print(f"[PASS] validated {len(all_files)} rc26_xhu_viewer configs in {config_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
