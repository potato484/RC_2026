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
    def point_on_segment(
        query: tuple[float, float],
        left: tuple[float, float],
        right: tuple[float, float],
        epsilon: float = 1e-6,
    ) -> bool:
        qx, qy = query
        lx, ly = left
        rx, ry = right
        cross = (qx - lx) * (ry - ly) - (qy - ly) * (rx - lx)
        if abs(cross) > epsilon:
            return False
        dot = (qx - lx) * (qx - rx) + (qy - ly) * (qy - ry)
        return dot <= epsilon

    x, y = point
    inside = False
    count = len(polygon)
    for index in range(count):
        x1, y1 = polygon[index]
        x2, y2 = polygon[(index + 1) % count]
        if point_on_segment(point, (x1, y1), (x2, y2)):
            return True
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


def plane_normal(points: list[dict[str, float]]) -> tuple[float, float, float] | None:
    plane = fit_plane(points)
    if plane is None:
        return None
    a, b, c, _ = plane
    norm = math.sqrt(a * a + b * b + c * c)
    if norm <= 1e-9:
        return None
    if c < 0.0:
        norm = -norm
    return (a / norm, b / norm, c / norm)


def surface_pitch_deg(points: list[dict[str, float]]) -> float:
    normal = plane_normal(points)
    if normal is None:
        return 0.0
    nz = max(-1.0, min(1.0, normal[2]))
    return math.degrees(math.acos(abs(nz)))


def point_to_segment_distance(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    px, py = point
    sx, sy = start
    ex, ey = end
    dx = ex - sx
    dy = ey - sy
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return math.hypot(px - sx, py - sy)
    t = ((px - sx) * dx + (py - sy) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    proj_x = sx + t * dx
    proj_y = sy + t * dy
    return math.hypot(px - proj_x, py - proj_y)


def polygon_boundary_clearance(
    point: tuple[float, float],
    polygon: list[tuple[float, float]],
) -> float:
    if not polygon:
        return 0.0
    clearance = float("inf")
    for index in range(len(polygon)):
        start = polygon[index]
        end = polygon[(index + 1) % len(polygon)]
        clearance = min(clearance, point_to_segment_distance(point, start, end))
    return 0.0 if clearance == float("inf") else clearance


def boundary_segment_clearance(
    point: tuple[float, float],
    boundary_segments: list[tuple[tuple[float, float], tuple[float, float]]],
) -> float:
    if not boundary_segments:
        return 0.0
    clearance = float("inf")
    for start, end in boundary_segments:
        clearance = min(clearance, point_to_segment_distance(point, start, end))
    return 0.0 if clearance == float("inf") else clearance


def point_supported_by_polygons(
    point: tuple[float, float],
    polygons: list[list[tuple[float, float]]],
) -> bool:
    return any(point_in_polygon(point, polygon) for polygon in polygons)


def quantize_point(point: tuple[float, float], precision: int = 4) -> tuple[int, int]:
    scale = 10 ** precision
    return (int(round(point[0] * scale)), int(round(point[1] * scale)))


def segment_key(
    start: tuple[float, float],
    end: tuple[float, float],
    precision: int = 4,
) -> tuple[tuple[int, int], tuple[int, int]]:
    left = quantize_point(start, precision)
    right = quantize_point(end, precision)
    return (left, right) if left <= right else (right, left)


def polygon_segments(
    polygon: list[tuple[float, float]],
) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    return [
        (polygon[index], polygon[(index + 1) % len(polygon)])
        for index in range(len(polygon))
    ]


class DisjointSet:
    def __init__(self, items: list[str]):
        self.parent = {item: item for item in items}

    def find(self, item: str) -> str:
        parent = self.parent[item]
        if parent != item:
            self.parent[item] = self.find(parent)
        return self.parent[item]

    def union(self, left: str, right: str) -> None:
        left_root = self.find(left)
        right_root = self.find(right)
        if left_root == right_root:
            return
        if left_root < right_root:
            self.parent[right_root] = left_root
        else:
            self.parent[left_root] = right_root


def normal_dot(
    left: tuple[float, float, float] | None,
    right: tuple[float, float, float] | None,
) -> float:
    if left is None or right is None:
        return 1.0
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]


def build_surface_regions(
    scene_features: list[dict[str, Any]],
    *,
    min_surface_area_xy: float,
    traversable_render_classes: set[str],
    excluded_feature_names: set[str],
) -> tuple[dict[str, dict[str, Any]], dict[str, dict[str, Any]]]:
    feature_records: dict[str, dict[str, Any]] = {}
    for feature in scene_features:
        if feature.get("render_class") not in traversable_render_classes:
            continue
        if feature.get("name") in excluded_feature_names:
            continue
        if float(feature.get("area_xy", 0.0)) < min_surface_area_xy:
            continue
        polygon = [(float(point["x"]), float(point["y"])) for point in feature["points"]]
        if len(polygon) < 3:
            continue
        feature_id = str(feature["id"])
        feature_records[feature_id] = {
            "feature_id": feature_id,
            "render_class": str(feature.get("render_class", "")),
            "polygon": polygon,
            "segments": polygon_segments(polygon),
            "normal": plane_normal(feature["points"]),
        }

    dsu = DisjointSet(sorted(feature_records.keys()))
    edge_to_features: dict[tuple[tuple[int, int], tuple[int, int]], list[str]] = defaultdict(list)
    for feature_id, record in feature_records.items():
        for start, end in record["segments"]:
            edge_to_features[segment_key(start, end)].append(feature_id)

    normal_cos_threshold = math.cos(math.radians(10.0))
    for shared_feature_ids in edge_to_features.values():
        if len(shared_feature_ids) < 2:
            continue
        for left_index in range(len(shared_feature_ids) - 1):
            for right_index in range(left_index + 1, len(shared_feature_ids)):
                left_id = shared_feature_ids[left_index]
                right_id = shared_feature_ids[right_index]
                left_record = feature_records[left_id]
                right_record = feature_records[right_id]
                if left_record["render_class"] != right_record["render_class"]:
                    continue
                if normal_dot(left_record["normal"], right_record["normal"]) < normal_cos_threshold:
                    continue
                dsu.union(left_id, right_id)

    region_members: dict[str, list[str]] = defaultdict(list)
    for feature_id in feature_records:
        region_members[dsu.find(feature_id)].append(feature_id)

    surface_regions: dict[str, dict[str, Any]] = {}
    for region_root, member_ids in region_members.items():
        boundary_counts: dict[tuple[tuple[int, int], tuple[int, int]], int] = defaultdict(int)
        boundary_segments: dict[
            tuple[tuple[int, int], tuple[int, int]],
            tuple[tuple[float, float], tuple[float, float]],
        ] = {}
        polygons: list[list[tuple[float, float]]] = []
        for feature_id in member_ids:
            record = feature_records[feature_id]
            polygons.append(record["polygon"])
            for start, end in record["segments"]:
                key = segment_key(start, end)
                boundary_counts[key] += 1
                boundary_segments[key] = (start, end)
        surface_regions[region_root] = {
            "region_id": region_root,
            "feature_ids": sorted(member_ids),
            "polygons": polygons,
            "boundary_segment_map": {
                key: boundary_segments[key]
                for key, count in boundary_counts.items()
                if count == 1
            },
        }

    for feature_id, record in feature_records.items():
        record["region_id"] = dsu.find(feature_id)

    return feature_records, surface_regions


def merge_region_support(
    region_ids: list[str],
    surface_regions: dict[str, dict[str, Any]],
) -> tuple[list[list[tuple[float, float]]], list[tuple[tuple[float, float], tuple[float, float]]]]:
    polygons: list[list[tuple[float, float]]] = []
    boundary_counts: dict[tuple[tuple[int, int], tuple[int, int]], int] = defaultdict(int)
    boundary_segments: dict[
        tuple[tuple[int, int], tuple[int, int]],
        tuple[tuple[float, float], tuple[float, float]],
    ] = {}
    for region_id in sorted(set(region_ids)):
        region = surface_regions.get(region_id)
        if region is None:
            continue
        polygons.extend(region["polygons"])
        for key, segment in region["boundary_segment_map"].items():
            boundary_counts[key] += 1
            boundary_segments[key] = segment
    return (
        polygons,
        [
            boundary_segments[key]
            for key, count in boundary_counts.items()
            if count == 1
        ],
    )


def sample_segment_clearance(
    start: tuple[float, float],
    end: tuple[float, float],
    polygons: list[list[tuple[float, float]]],
    *,
    sample_spacing_m: float,
) -> float:
    def support_distance_along_direction(
        origin: tuple[float, float],
        direction: tuple[float, float],
        *,
        probe_step_m: float,
        max_distance_m: float,
    ) -> float:
        distance = 0.0
        while distance + probe_step_m <= max_distance_m + 1e-9:
            distance += probe_step_m
            probe = (
                origin[0] + direction[0] * distance,
                origin[1] + direction[1] * distance,
            )
            if not point_supported_by_polygons(probe, polygons):
                return max(0.0, distance - probe_step_m)
        return max_distance_m

    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    steps = max(1, int(math.ceil(distance / max(sample_spacing_m, 1e-3))))
    min_clearance = float("inf")
    if distance <= 1e-9:
        return 0.0
    direction = ((end[0] - start[0]) / distance, (end[1] - start[1]) / distance)
    normal = (-direction[1], direction[0])
    probe_step_m = min(0.02, max(sample_spacing_m * 0.5, 0.01))
    max_probe_distance_m = 2.0
    for step in range(steps + 1):
        ratio = step / steps
        sample = (
            start[0] + (end[0] - start[0]) * ratio,
            start[1] + (end[1] - start[1]) * ratio,
        )
        if not point_supported_by_polygons(sample, polygons):
            return 0.0
        left_clearance = support_distance_along_direction(
            sample,
            normal,
            probe_step_m=probe_step_m,
            max_distance_m=max_probe_distance_m,
        )
        right_clearance = support_distance_along_direction(
            sample,
            (-normal[0], -normal[1]),
            probe_step_m=probe_step_m,
            max_distance_m=max_probe_distance_m,
        )
        min_clearance = min(
            min_clearance,
            min(left_clearance, right_clearance),
        )
    return 0.0 if min_clearance == float("inf") else min_clearance


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
    surface_records: dict[str, dict[str, Any]] | None = None,
    surface_regions: dict[str, dict[str, Any]] | None = None,
) -> list[dict[str, Any]]:
    if surface_records is None or surface_regions is None:
        surface_records, surface_regions = build_surface_regions(
            scene_features,
            min_surface_area_xy=min_surface_area_xy,
            traversable_render_classes=traversable_render_classes,
            excluded_feature_names=excluded_feature_names,
        )

    nodes: list[dict[str, Any]] = []
    for feature_index, feature in enumerate(scene_features):
        if feature.get("render_class") not in traversable_render_classes:
            continue
        if feature.get("name") in excluded_feature_names:
            continue
        if float(feature.get("area_xy", 0.0)) <= 1e-6:
            continue
        feature_id = str(feature["id"])
        surface_record = surface_records.get(feature_id)
        if surface_record is None:
            continue
        region = surface_regions[surface_record["region_id"]]
        boundary_segments = list(region["boundary_segment_map"].values())
        pitch_deg = surface_pitch_deg(feature["points"])
        for sample_index, sample in enumerate(
            sample_feature(
                feature,
                spacing_m=spacing_m,
                min_surface_area_xy=min_surface_area_xy,
            )
        ):
            center_clearance_m = boundary_segment_clearance(
                (sample["x"], sample["y"]),
                boundary_segments,
            )
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
                    "center_clearance_m": round(center_clearance_m, 4),
                    "surface_pitch_deg": round(pitch_deg, 3),
                }
            )
    return nodes


def build_edges(
    nodes: list[dict[str, Any]],
    surface_records: dict[str, dict[str, Any]],
    surface_regions: dict[str, dict[str, Any]],
    *,
    neighbor_radius_m: float,
    transition_radius_m: float,
    max_transition_height_m: float,
    plane_height_epsilon_m: float,
    clearance_sample_spacing_m: float,
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
        surface_record = surface_records.get(feature_id)
        if surface_record is None:
            continue
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
                    other_feature_id = str(other["surface_id"])
                    other_surface_record = surface_records.get(other_feature_id)
                    if other_surface_record is None:
                        continue
                    same_surface = surface_record["region_id"] == other_surface_record["region_id"]
                    if not same_surface and horizontal > transition_radius_m:
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
                    polygons, _ = merge_region_support(
                        [surface_record["region_id"], other_surface_record["region_id"]],
                        surface_regions,
                    )
                    center_clearance_m = sample_segment_clearance(
                        (x, y),
                        (ox, oy),
                        [polygon for polygon in polygons if polygon],
                        sample_spacing_m=clearance_sample_spacing_m,
                    )
                    slope_deg = math.degrees(math.atan2(abs(oz - z), max(horizontal, 1e-6)))
                    nominal_yaw = math.atan2(oy - y, ox - x)
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
                            "horizontal_length_m": round(horizontal, 4),
                            "slope_deg": round(slope_deg, 3),
                            "center_clearance_m": round(center_clearance_m, 4),
                            "nominal_yaw": round(nominal_yaw, 4),
                            "same_surface": same_surface,
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
                            "horizontal_length_m": round(horizontal, 4),
                            "slope_deg": round(slope_deg, 3),
                            "center_clearance_m": round(center_clearance_m, 4),
                            "nominal_yaw": round(math.atan2(y - oy, x - ox), 4),
                            "same_surface": same_surface,
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
            "schema_version": "1.1",
            "source": "robocon2026_surface_graph_body_v1",
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
    surface_records, surface_regions = build_surface_regions(
        world_context["scene_features"],
        min_surface_area_xy=float(sampling.get("min_surface_area_xy", 0.025)),
        traversable_render_classes=set(overlay.get("traversable_render_classes", [])),
        excluded_feature_names=set(overlay.get("excluded_feature_names", [])),
    )

    nodes = build_nodes(
        world_context["scene_features"],
        spacing_m=float(sampling.get("spacing_m", 0.18)),
        min_surface_area_xy=float(sampling.get("min_surface_area_xy", 0.025)),
        traversable_render_classes=set(overlay.get("traversable_render_classes", [])),
        excluded_feature_names=set(overlay.get("excluded_feature_names", [])),
        surface_records=surface_records,
        surface_regions=surface_regions,
    )
    edges = build_edges(
        nodes,
        surface_records,
        surface_regions,
        neighbor_radius_m=float(sampling.get("neighbor_radius_m", 0.27)),
        transition_radius_m=float(sampling.get("transition_radius_m", 0.24)),
        max_transition_height_m=float(sampling.get("max_transition_height_m", 0.32)),
        plane_height_epsilon_m=float(sampling.get("plane_height_epsilon_m", 0.05)),
        clearance_sample_spacing_m=float(sampling.get("clearance_sample_spacing_m", 0.05)),
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
