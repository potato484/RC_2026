#!/usr/bin/env python3

import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


EXPECTATIONS = {
    "navigation_operator.rviz": [
        "Fixed Frame: map",
        "Name: 全局路线",
        "Value: /xhu_nav/route",
        "Value: /xhu_nav/corridor",
        "Value: /xhu_nav/lookahead_path",
    ],
    "navigation_engineering.rviz": [
        "Fixed Frame: map",
        "Name: 点云配准",
        "Value: registered_scan",
        "Value: terrain_obstacles",
        "Value: terrain_drop",
    ],
    "navigation_diagnostic.rviz": [
        "Fixed Frame: map",
        "Name: 地形障碍",
        "Name: 保持禁行区",
        "Value: laser_map_full",
    ],
    "slam_operator.rviz": [
        "Fixed Frame: odom",
        "Name: 匹配点云",
        "Value: registered_scan",
        "Value: terrain_grid_map_local",
    ],
    "slam_engineering.rviz": [
        "Fixed Frame: odom",
        "Name: TF",
        "Name: 曲率标记",
        "Value: curvature_points_marker_array",
    ],
    "slam_diagnostic.rviz": [
        "Fixed Frame: odom",
        "Name: 地形跌落",
        "Name: 保持禁行区",
        "Value: terrain_drop",
    ],
}


def main() -> int:
    config_dir = Path(get_package_share_directory("rc26_xhu_viewer")) / "config"
    failures = []

    for filename, tokens in EXPECTATIONS.items():
        file_path = config_dir / filename
        if not file_path.is_file():
            failures.append(f"missing file: {file_path}")
            continue
        content = file_path.read_text(encoding="utf-8")
        for token in tokens:
            if token not in content:
                failures.append(f"{filename}: missing token '{token}'")

    if failures:
        for item in failures:
            print(f"[FAIL] {item}")
        return 1

    print(f"[PASS] validated {len(EXPECTATIONS)} rc26_xhu_viewer configs in {config_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
