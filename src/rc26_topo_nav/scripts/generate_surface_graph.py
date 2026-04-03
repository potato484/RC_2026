#!/usr/bin/env python3
"""Generate a dense traversable surface graph from the bundled field mesh."""

from __future__ import annotations

import argparse
import math
from collections import defaultdict
from pathlib import Path
from typing import Any

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent

import sys

if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import render_graph_sim_html as RENDER  # noqa: E402


def point_in_polygon(point: tuple[float, float], polygon: list[tuple[float, float]]) -> bool:
    x, y = point
    inside = False
    count = len(polygon)
    for index in range(count):
        x1, y1 = polygon[index]
        x2, y2 = polygon[(index + 1) % count]
        if ((y1 > y) != (y2 > y)) and (
            x < (x2 - x1) * (y - y1) / max((y2 - y1), 1e-9) + x1
        ):
            inside = not inside
    return inside


def polygon_centroid(points: list[dict[str, float]]) -> tuple[float, float]:
    if not points:
        return (0.0, 0.0)
    return (
        sum(point["x"] for point in points) / len(points),
        sum(point["y"] for point in points) / len(points),
    )


def fit_plane(points: list[dict[str, float]]) -> tuple[float, float, float, float] | None:
    if len(points) < 3:
        return None
    p0 = points[0]
    for left_index in range(1, len(points) - 1):
        p1 = points[left_index]
        for right_index in range(left_index + 1, len(points)):
            p2 = points[right_index]
            ux = p1["x"] - p0["x"]
            uy = p1["y"] - p0["y"]
            uz = p1["z"] - p0["z"]
            vx = p2["x"] - p0["x"]
            vy = p2["y"] - p0["y"]
            vz = p2["z"] - p0["z"]
            a = uy * vz - uz * vy
            b = uz * vx - ux * vz
            c = ux * vy - uy * vx
            if abs(a) < 1e-9 and abs(b) < 1e-9 and abs(c) < 1e-9:
                continue
            d = -(a * p0["x"] + b * p0["y"] + c * p0["z"])
            return (a, b, c, d)
    return None


def interpolate_z(points: list[dict[str, float]], x: float, y: float) -> float:
    plane = fit_plane(points)
    if plane is None or abs(plane[2]) < 1e-6:
        return sum(point["z"] for point in points) / len(points)
    a, b, c, d = plane
    return -(a * x + b * y + d) / c


def sample_feature(
    feature: dict[str, Any],
    *,
    spacing_m: float,
    min_surface_area_xy: float,
) -> list[dict[str, Any]]:
    if float(feature.get("area_xy", 0.0)) < min_surface_area_xy:
        return []

    polygon = [(float(point["x"]), float(point["y"])) for point in feature["points"]]
    xs = [item[0] for item in polygon]
    ys = [item[1] for item in polygon]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    samples: list[dict[str, Any]] = []
    y = min_y + spacing_m * 0.5
    while y <= max_y + 1e-6:
        x = min_x + spacing_m * 0.5
        while x <= max_x + 1e-6:
            if point_in_polygon((x, y), polygon):
                z = interpolate_z(feature["points"], x, y)
                samples.append(
                    {
                        "x": round(x, 4),
                        "y": round(y, 4),
                        "z": round(z, 4),
                    }
                )
            x += spacing_m
        y += spacing_m

    if samples:
        return samples

    centroid_x, centroid_y = polygon_centroid(feature["points"])
    return [
        {
            "x": round(centroid_x, 4),
            "y": round(centroid_y, 4),
            "z": round(interpolate_z(feature["points"], centroid_x, centroid_y), 4),
        }
    ]


def classify_motion(
    left: dict[str, Any],
    right: dict[str, Any],
    *,
    plane_height_epsilon_m: float,
) -> tuple[str, str, float]:
    dz = float(right["z"]) - float(left["z"])
    if abs(dz) <= plane_height_epsilon_m:
        return ("plane_move", "plane_move", abs(dz))
    if dz > 0.0:
        return ("ramp_up", "stair_up", abs(dz))
    return ("ramp_down", "stair_down", abs(dz))


def build_nodes(
    scene_features: list[dict[str, Any]],
    *,
    spacing_m: float,
    min_surface_area_xy: float,
    traversable_render_classes: set[str],
    excluded_feature_names: set[str],
) -> list[dict[str, Any]]:
    nodes: list[dict[str, Any]] = []
    for feature_index, feature in enumerate(scene_features):
        if feature.get("render_class") not in traversable_render_classes:
            continue
        if feature.get("name") in excluded_feature_names:
            continue
        if float(feature.get("area_xy", 0.0)) <= 1e-6:
            continue
        for sample_index, sample in enumerate(
            sample_feature(
                feature,
                spacing_m=spacing_m,
                min_surface_area_xy=min_surface_area_xy,
            )
        ):
            nodes.append(
                {
                    "id": f"sf_{feature_index}_{sample_index}",
                    "type": "surface_point",
                    "pose": {
                        "x": sample["x"],
                        "y": sample["y"],
                        "z": sample["z"],
                        "yaw": 0.0,
                    },
                    "level": max(0, int(round(sample["z"] * 10.0))),
                    "phase_mask": 255,
                    "block_id": 0,
                    "base_cost": 0.0,
                    "operation_tag": "",
                    "surface_id": str(feature["id"]),
                    "surface_name": str(feature["name"]),
                    "render_class": str(feature["render_class"]),
                }
            )
    return nodes


def build_edges(
    nodes: list[dict[str, Any]],
    *,
    neighbor_radius_m: float,
    transition_radius_m: float,
    max_transition_height_m: float,
    plane_height_epsilon_m: float,
) -> list[dict[str, Any]]:
    buckets: dict[tuple[int, int], list[int]] = defaultdict(list)
    bucket_size = neighbor_radius_m
    for index, node in enumerate(nodes):
        x = float(node["pose"]["x"])
        y = float(node["pose"]["y"])
        buckets[(int(math.floor(x / bucket_size)), int(math.floor(y / bucket_size)))].append(index)

    pair_seen: set[tuple[int, int]] = set()
    edges: list[dict[str, Any]] = []
    for index, node in enumerate(nodes):
        x = float(node["pose"]["x"])
        y = float(node["pose"]["y"])
        z = float(node["pose"]["z"])
        feature_id = str(node["surface_id"])
        cell_x = int(math.floor(x / bucket_size))
        cell_y = int(math.floor(y / bucket_size))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for other_index in buckets.get((cell_x + dx, cell_y + dy), []):
                    if other_index <= index:
                        continue
                    pair = (index, other_index)
                    if pair in pair_seen:
                        continue
                    pair_seen.add(pair)
                    other = nodes[other_index]
                    ox = float(other["pose"]["x"])
                    oy = float(other["pose"]["y"])
                    oz = float(other["pose"]["z"])
                    horizontal = math.hypot(ox - x, oy - y)
                    if horizontal < 1e-6 or horizontal > neighbor_radius_m:
                        continue
                    same_feature = feature_id == str(other["surface_id"])
                    if not same_feature and horizontal > transition_radius_m:
                        continue
                    if abs(oz - z) > max_transition_height_m:
                        continue
                    forward_motion, forward_mode, height_change = classify_motion(
                        node["pose"],
                        other["pose"],
                        plane_height_epsilon_m=plane_height_epsilon_m,
                    )
                    backward_motion, backward_mode, _ = classify_motion(
                        other["pose"],
                        node["pose"],
                        plane_height_epsilon_m=plane_height_epsilon_m,
                    )
                    base_cost = round(math.sqrt(horizontal * horizontal + (oz - z) * (oz - z)), 4)
                    edges.append(
                        {
                            "id": f"se_{index}_{other_index}",
                            "from": node["id"],
                            "to": other["id"],
                            "motion_type": forward_motion,
                            "height_change": round(float(other["pose"]["z"]) - float(node["pose"]["z"]), 4),
                            "required_mode": forward_mode,
                            "requires_confirmation": False,
                            "can_block": False,
                            "phase_mask": 255,
                            "base_cost": base_cost,
                            "control_points": [],
                        }
                    )
                    edges.append(
                        {
                            "id": f"se_{other_index}_{index}",
                            "from": other["id"],
                            "to": node["id"],
                            "motion_type": backward_motion,
                            "height_change": round(float(node["pose"]["z"]) - float(other["pose"]["z"]), 4),
                            "required_mode": backward_mode,
                            "requires_confirmation": False,
                            "can_block": False,
                            "phase_mask": 255,
                            "base_cost": base_cost,
                            "control_points": [],
                        }
                    )
    return edges


def dump_graph(
    *,
    team: str,
    world_file: Path,
    overlay_file: Path,
    nodes: list[dict[str, Any]],
    edges: list[dict[str, Any]],
) -> dict[str, Any]:
    return {
        "meta": {
            "team": team.lower(),
            "schema_version": "1.0",
            "source": "robocon2026_surface_graph_v1",
            "coordinate_frame": "map",
            "generated": True,
            "world_file": str(world_file),
            "overlay_file": str(overlay_file),
            "grid_spacing_m": 0.18,
        },
        "nodes": nodes,
        "edges": edges,
        "tasks": [],
        "routes": [],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate a dense traversable surface graph")
    default_world = SCRIPT_DIR.parent / "sim_assets" / "worlds" / "robocon2026_v2_aligned.world"
    default_overlay = SCRIPT_DIR.parent / "config" / "r2_surface_graph_overlay.yaml"
    parser.add_argument("--team", default="blue", choices=["blue", "red"])
    parser.add_argument("--world", type=Path, default=default_world)
    parser.add_argument("--overlay", type=Path, default=default_overlay)
    parser.add_argument("--out", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    overlay = RENDER.load_yaml(args.overlay)
    sampling = overlay.get("sampling", {})
    scene_root = args.world.parent.parent / "models"
    world_context = RENDER.parse_world_context(
        args.world,
        scene_root,
        geometry_mode=RENDER.WORLD_GEOMETRY_MODE_VIEWER_3D,
        filter_noise=True,
    )

    nodes = build_nodes(
        world_context["scene_features"],
        spacing_m=float(sampling.get("spacing_m", 0.18)),
        min_surface_area_xy=float(sampling.get("min_surface_area_xy", 0.025)),
        traversable_render_classes=set(overlay.get("traversable_render_classes", [])),
        excluded_feature_names=set(overlay.get("excluded_feature_names", [])),
    )
    edges = build_edges(
        nodes,
        neighbor_radius_m=float(sampling.get("neighbor_radius_m", 0.27)),
        transition_radius_m=float(sampling.get("transition_radius_m", 0.24)),
        max_transition_height_m=float(sampling.get("max_transition_height_m", 0.32)),
        plane_height_epsilon_m=float(sampling.get("plane_height_epsilon_m", 0.05)),
    )
    graph = dump_graph(
        team=args.team,
        world_file=args.world,
        overlay_file=args.overlay,
        nodes=nodes,
        edges=edges,
    )

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(graph, handle, sort_keys=False, allow_unicode=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
