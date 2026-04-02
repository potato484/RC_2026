#!/usr/bin/env python3
"""Render a topo graph together with RC_Sim_001 world context and planner trace."""

from __future__ import annotations

import argparse
import heapq
import html
import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml


SVG_WIDTH = 1280
SVG_HEIGHT = 920
SVG_PADDING = 88
MEILIN_PAD_SIZE = 0.92

EDGE_COLORS = {
    "plane_move": "#355070",
    "ramp_up": "#2a9d8f",
    "ramp_down": "#d62828",
}

NODE_COLORS = {
    "mf_edge_pose": "#f4a261",
    "staging": "#4ea8de",
    "ramp_entry": "#2a9d8f",
    "ramp_exit": "#e76f51",
}

WORLD_FEATURE_FILL = [
    "#f4f1de",
    "#dcefe8",
    "#ebe7ff",
    "#ffe8d6",
    "#d8f3dc",
]

HIDDEN_WORLD_NODES = {"空物体", "Spear", "hand", "fist", "平面.005"}
NOISE_WORLD_NODES = {"柱体", "杆架", "主地图.002"}
SUBTLE_WORLD_NODES = {"平面.006"}

TEAM_LABELS = {
    "blue": "蓝方",
    "red": "红方",
    "unknown": "未知",
}

MOTION_TYPE_LABELS = {
    "plane_move": "平面移动",
    "ramp_up": "坡道上行",
    "ramp_down": "坡道下行",
}

NODE_TYPE_LABELS = {
    "mf_edge_pose": "主区导航点",
    "staging": "等待点",
    "ramp_entry": "坡道入口点",
    "ramp_exit": "坡道出口点",
}

GOAL_KIND_LABELS = {
    "node": "导航点",
    "task": "任务",
    "route": "预设路线",
}

FRAME_TYPE_LABELS = {
    "init": "初始化前沿",
    "skip_stale": "跳过过期队列项",
    "pop": "取出当前最优点",
    "blocked_node": "节点阻塞",
    "edge_blocked": "边阻塞",
    "relax": "更新更优路径",
    "keep_best": "保留已有更优路径",
    "failed": "搜索失败",
    "goal": "到达目标",
    "route_tag": "展开预设路线",
}

WORLD_RENDER_CLASS_LABELS = {
    "world-ground": "地面分区",
    "world-marking": "标线/起始区",
    "world-platform": "平台/台面",
    "world-fence": "围栏轮廓",
}

TASK_REASON_LABELS = {
    "candidate_blocked": "候选点已被阻塞",
}

BLOCK_NODE_RE = re.compile(r"^mf_b(\d+)$")


def team_label(team: str) -> str:
    return TEAM_LABELS.get(team.lower(), team.upper())


def motion_type_label(motion_type: str) -> str:
    return MOTION_TYPE_LABELS.get(motion_type, motion_type)


def node_type_label(node_type: str) -> str:
    return NODE_TYPE_LABELS.get(node_type, node_type)


def goal_kind_label(goal_kind: str) -> str:
    return GOAL_KIND_LABELS.get(goal_kind, goal_kind)


def frame_type_label(event_type: str) -> str:
    return FRAME_TYPE_LABELS.get(event_type, event_type)


def world_render_class_label(render_class: str) -> str:
    return WORLD_RENDER_CLASS_LABELS.get(render_class, render_class)


def with_raw_label(display_name: str, raw_value: str) -> str:
    return display_name if display_name == raw_value else f"{display_name}（{raw_value}）"


def short_node_label(node_id: str) -> str:
    match = BLOCK_NODE_RE.fullmatch(node_id)
    if match:
        return f"块{match.group(1)}"
    special = {
        "mf_entry_staging": "入口等待",
        "mf_exit_staging": "出口等待",
        "ramp_entry_south": "南坡入口",
        "ramp_exit_north": "北坡出口",
        "ramp_entry_north": "北坡入口",
        "ramp_exit_south": "南坡出口",
    }
    return special.get(node_id, node_id)


def full_node_label(node_id: str) -> str:
    match = BLOCK_NODE_RE.fullmatch(node_id)
    if match:
        return f"主区 {match.group(1)} 号块导航点"
    special = {
        "mf_entry_staging": "主区入口等待点",
        "mf_exit_staging": "主区出口等待点",
        "ramp_entry_south": "南侧坡道入口点",
        "ramp_exit_north": "北侧坡道出口点",
        "ramp_entry_north": "北侧坡道入口点",
        "ramp_exit_south": "南侧坡道出口点",
    }
    return special.get(node_id, node_id)


def edge_transition_label(from_node: str, to_node: str) -> str:
    return f"{full_node_label(from_node)} -> {full_node_label(to_node)}"


def task_tag_label(task_tag: str) -> str:
    mapping = {
        "mf_grab": "主区抓取任务",
        "mf_exit": "主区离场任务",
        "mf_entry": "主区入场任务",
    }
    return mapping.get(task_tag, task_tag)


def route_tag_label(route_tag: str) -> str:
    mapping = {
        "mf_entry_default": "主区默认入场路线",
        "mf_exit_default": "主区默认离场路线",
    }
    return mapping.get(route_tag, route_tag)


def goal_value_label(goal_kind: str, goal_value: str) -> str:
    if goal_kind == "node":
        return with_raw_label(full_node_label(goal_value), goal_value)
    if goal_kind == "task":
        return with_raw_label(task_tag_label(goal_value), goal_value)
    if goal_kind == "route":
        return with_raw_label(route_tag_label(goal_value), goal_value)
    return goal_value


def task_result_reason_label(reason: str) -> str:
    return TASK_REASON_LABELS.get(reason, reason)


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping at the top level")
    return data


def round_float(value: float, digits: int = 3) -> float:
    rounded = round(float(value), digits)
    if abs(rounded) < 10 ** (-digits):
        return 0.0
    return rounded


def escape_attr(value: Any) -> str:
    return html.escape(str(value), quote=True)


def slugify(value: str) -> str:
    return "".join(ch if ch.isalnum() else "-" for ch in value).strip("-").lower() or "item"


def node_color(node_type: str) -> str:
    return NODE_COLORS.get(node_type, "#9d4edd")


def edge_color(motion_type: str) -> str:
    return EDGE_COLORS.get(motion_type, "#6c757d")


def parse_pose_text(text: str | None) -> dict[str, float]:
    values = [float(part) for part in (text or "").split()]
    while len(values) < 6:
        values.append(0.0)
    return {
        "x": values[0],
        "y": values[1],
        "z": values[2],
        "roll": values[3],
        "pitch": values[4],
        "yaw": values[5],
    }


def transform_xy(x: float, y: float, pose: dict[str, float]) -> tuple[float, float]:
    cy = math.cos(float(pose.get("yaw", 0.0)))
    sy = math.sin(float(pose.get("yaw", 0.0)))
    tx = cy * x - sy * y + float(pose.get("x", 0.0))
    ty = sy * x + cy * y + float(pose.get("y", 0.0))
    return (round_float(tx, 4), round_float(ty, 4))


def apply_matrix(matrix: list[float], point: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = point
    return (
        matrix[0] * x + matrix[1] * y + matrix[2] * z + matrix[3],
        matrix[4] * x + matrix[5] * y + matrix[6] * z + matrix[7],
        matrix[8] * x + matrix[9] * y + matrix[10] * z + matrix[11],
    )


def clamp_unit(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def diffuse_text_to_style(color_text: str) -> tuple[str, float]:
    values = [float(part) for part in color_text.split()] if color_text else [0.8, 0.8, 0.8, 1.0]
    while len(values) < 4:
        values.append(1.0)
    r, g, b, a = [clamp_unit(value) for value in values[:4]]
    color = "#{:02x}{:02x}{:02x}".format(int(round(r * 255)), int(round(g * 255)), int(round(b * 255)))
    return color, round_float(a, 3)


def polygon_area_xy(points: list[tuple[float, float]]) -> float:
    if len(points) < 3:
        return 0.0
    area = 0.0
    for left, right in zip(points, points[1:] + [points[0]]):
        area += left[0] * right[1] - right[0] * left[1]
    return abs(area) * 0.5


def polygon_centroid_xy(points: list[tuple[float, float]]) -> tuple[float, float]:
    if not points:
        return (0.0, 0.0)
    return (
        round_float(sum(point[0] for point in points) / len(points), 4),
        round_float(sum(point[1] for point in points) / len(points), 4),
    )


def polygon_z_span(points: list[tuple[float, float, float]]) -> float:
    if not points:
        return 0.0
    z_values = [point[2] for point in points]
    return round_float(max(z_values) - min(z_values), 4)


def is_marking_material(material_symbol: str) -> bool:
    return "白线" in material_symbol or "开始" in material_symbol


def world_feature_profile(
    node_name: str,
    material_symbol: str,
    area_xy: float,
    z_span: float,
    *,
    default_opacity: float,
) -> dict[str, Any] | None:
    if node_name in NOISE_WORLD_NODES:
        return None
    if area_xy < 0.004:
        return None
    if z_span > 0.05 and area_xy < 0.16:
        return None
    if node_name == "主地图" and area_xy < 0.03 and not is_marking_material(material_symbol):
        return None
    if node_name == "2-3区围栏" and area_xy < 0.02:
        return None
    if node_name in SUBTLE_WORLD_NODES and area_xy < 0.08:
        return None

    if node_name == "2-3区围栏":
        return {
            "render_class": "world-fence",
            "opacity": min(default_opacity, 0.34),
            "stroke": "rgba(33, 37, 41, 0.28)",
            "stroke_width": 1.4,
            "stroke_opacity": 0.65,
            "show_title": area_xy >= 0.18,
        }
    if node_name in {"区域三放置架", "立方体"} or "台" in material_symbol:
        return {
            "render_class": "world-platform",
            "opacity": min(default_opacity, 0.82),
            "stroke": "rgba(29, 39, 48, 0.16)",
            "stroke_width": 1.0,
            "stroke_opacity": 0.55,
            "show_title": area_xy >= 0.16,
        }
    if is_marking_material(material_symbol):
        return {
            "render_class": "world-marking",
            "opacity": min(1.0, max(default_opacity, 0.92)),
            "stroke": "none",
            "stroke_width": 0.0,
            "stroke_opacity": 0.0,
            "show_title": False,
        }

    return {
        "render_class": "world-ground",
        "opacity": min(default_opacity, 0.94),
        "stroke": "none",
        "stroke_width": 0.0,
        "stroke_opacity": 0.0,
        "show_title": False,
    }


def parse_collada_material_styles(root: ET.Element, ns: dict[str, str]) -> dict[str, dict[str, Any]]:
    effect_styles: dict[str, dict[str, Any]] = {}
    for effect in root.findall(".//c:library_effects/c:effect", ns):
        diffuse_text = effect.findtext(".//c:diffuse/c:color", default="", namespaces=ns)
        color, opacity = diffuse_text_to_style(diffuse_text)
        effect_styles[str(effect.get("id"))] = {
            "fill": color,
            "opacity": opacity,
        }

    material_styles: dict[str, dict[str, Any]] = {}
    for material in root.findall(".//c:library_materials/c:material", ns):
        instance_effect = material.find("c:instance_effect", ns)
        effect_id = instance_effect.get("url", "").lstrip("#") if instance_effect is not None else ""
        material_styles[str(material.get("id"))] = effect_styles.get(
            effect_id,
            {"fill": "#d9d9d9", "opacity": 1.0},
        )
    return material_styles


def parse_collada_geometry_primitives(root: ET.Element, ns: dict[str, str]) -> dict[str, list[dict[str, Any]]]:
    geometry_primitives: dict[str, list[dict[str, Any]]] = {}
    for geometry in root.findall(".//c:library_geometries/c:geometry", ns):
        mesh = geometry.find("c:mesh", ns)
        if mesh is None:
            continue

        source_points: dict[str, list[tuple[float, float, float]]] = {}
        for source in mesh.findall("c:source", ns):
            float_array = source.find("c:float_array", ns)
            accessor = source.find(".//c:technique_common/c:accessor", ns)
            if float_array is None or accessor is None or not float_array.text:
                continue
            stride = int(accessor.get("stride", "3"))
            values = [float(part) for part in float_array.text.split()]
            points = []
            for offset in range(0, len(values), stride):
                chunk = values[offset:offset + stride]
                if len(chunk) < 3:
                    continue
                points.append((chunk[0], chunk[1], chunk[2]))
            source_points[str(source.get("id"))] = points

        vertex_sources: dict[str, str] = {}
        for vertices in mesh.findall("c:vertices", ns):
            position_input = vertices.find("c:input[@semantic='POSITION']", ns)
            if position_input is None:
                continue
            vertex_sources[str(vertices.get("id"))] = position_input.get("source", "").lstrip("#")

        primitives: list[dict[str, Any]] = []
        for primitive in [*mesh.findall("c:polylist", ns), *mesh.findall("c:triangles", ns)]:
            inputs = primitive.findall("c:input", ns)
            if not inputs:
                continue
            stride = max(int(item.get("offset", "0")) for item in inputs) + 1
            vertex_offset = None
            position_source_id = ""
            for item in inputs:
                if item.get("semantic") != "VERTEX":
                    continue
                vertex_offset = int(item.get("offset", "0"))
                position_source_id = vertex_sources.get(item.get("source", "").lstrip("#"), "")
                break
            if vertex_offset is None or position_source_id not in source_points:
                continue

            p_text = primitive.findtext("c:p", default="", namespaces=ns)
            if not p_text:
                continue
            indices = [int(part) for part in p_text.split()]
            positions = source_points[position_source_id]
            material_symbol = str(primitive.get("material", ""))

            if primitive.tag.endswith("polylist"):
                vertex_counts = [int(part) for part in primitive.findtext("c:vcount", default="", namespaces=ns).split()]
                cursor = 0
                for vertex_count in vertex_counts:
                    polygon = []
                    for _ in range(vertex_count):
                        base = cursor * stride
                        point_index = indices[base + vertex_offset]
                        polygon.append(positions[point_index])
                        cursor += 1
                    primitives.append({"material_symbol": material_symbol, "points": polygon})
            else:
                triangle_count = int(primitive.get("count", "0"))
                cursor = 0
                for _ in range(triangle_count):
                    polygon = []
                    for _ in range(3):
                        base = cursor * stride
                        point_index = indices[base + vertex_offset]
                        polygon.append(positions[point_index])
                        cursor += 1
                    primitives.append({"material_symbol": material_symbol, "points": polygon})

        geometry_primitives[str(geometry.get("id"))] = primitives

    return geometry_primitives


def parse_collada_scene_features(dae_path: Path, model_pose: dict[str, float]) -> list[dict[str, Any]]:
    root = ET.parse(dae_path).getroot()
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    material_styles = parse_collada_material_styles(root, ns)
    geometry_primitives = parse_collada_geometry_primitives(root, ns)

    scene = root.find(".//c:library_visual_scenes/c:visual_scene", ns)
    if scene is None:
        return []

    features: list[dict[str, Any]] = []
    for index, node in enumerate(scene.findall("c:node", ns)):
        instance = node.find("c:instance_geometry", ns)
        matrix_text = node.findtext("c:matrix", default="", namespaces=ns)
        if instance is None or not matrix_text:
            continue

        node_name = str(node.get("name") or node.get("id") or f"feature-{index}")
        if node_name in HIDDEN_WORLD_NODES:
            continue

        geometry_id = instance.get("url", "").lstrip("#")
        local_primitives = geometry_primitives.get(geometry_id)
        if not local_primitives:
            continue

        matrix = [float(part) for part in matrix_text.split()]
        symbol_styles: dict[str, dict[str, Any]] = {}
        for instance_material in instance.findall(".//c:instance_material", ns):
            symbol = str(instance_material.get("symbol"))
            target = instance_material.get("target", "").lstrip("#")
            symbol_styles[symbol] = material_styles.get(target, {"fill": "#d9d9d9", "opacity": 1.0})

        for primitive_index, primitive in enumerate(local_primitives):
            projected_points_3d = []
            for local_point in primitive["points"]:
                tx, ty, tz = apply_matrix(matrix, local_point)
                world_x, world_y = transform_xy(tx, ty, model_pose)
                projected_points_3d.append((world_x, world_y, tz + float(model_pose.get("z", 0.0))))

            xy_points = [(point[0], point[1]) for point in projected_points_3d]
            area_xy = polygon_area_xy(xy_points)
            if area_xy < 1e-4:
                continue

            avg_z = sum(point[2] for point in projected_points_3d) / len(projected_points_3d)
            z_span = polygon_z_span(projected_points_3d)
            style = symbol_styles.get(primitive["material_symbol"], {"fill": WORLD_FEATURE_FILL[index % len(WORLD_FEATURE_FILL)], "opacity": 0.92})
            feature_profile = world_feature_profile(
                node_name,
                primitive["material_symbol"],
                area_xy,
                z_span,
                default_opacity=float(style["opacity"]),
            )
            if feature_profile is None:
                continue

            centroid_x, centroid_y = polygon_centroid_xy(xy_points)
            features.append(
                {
                    "id": f"{node.get('id') or 'feature'}-{primitive_index}",
                    "name": node_name,
                    "geometry_id": geometry_id,
                    "material_symbol": primitive["material_symbol"],
                    "points": [
                        {"x": round_float(point[0], 4), "y": round_float(point[1], 4), "z": round_float(point[2], 4)}
                        for point in projected_points_3d
                    ],
                    "fill": style["fill"],
                    "opacity": round_float(feature_profile["opacity"], 3),
                    "stroke": feature_profile["stroke"],
                    "stroke_width": round_float(feature_profile["stroke_width"], 3),
                    "stroke_opacity": round_float(feature_profile["stroke_opacity"], 3),
                    "render_class": feature_profile["render_class"],
                    "show_title": bool(feature_profile["show_title"]),
                    "avg_z": round_float(avg_z, 4),
                    "z_span": z_span,
                    "area_xy": round_float(area_xy, 4),
                    "centroid_x": centroid_x,
                    "centroid_y": centroid_y,
                }
            )

    features.sort(key=lambda item: (item["avg_z"], item["render_class"], item["area_xy"]))
    return features


def parse_world_context(world_path: Path, model_root: Path) -> dict[str, Any]:
    root = ET.parse(world_path).getroot()
    world = root.find("world")
    if world is None:
        return {"field_pose": parse_pose_text(None), "scene_features": [], "includes": []}

    includes = []
    field_pose = parse_pose_text(None)
    field_uri = ""
    for include in world.findall("include"):
        include_uri = include.findtext("uri", default="")
        include_name = include.findtext("name", default="")
        pose = parse_pose_text(include.findtext("pose"))
        include_item = {
            "uri": include_uri,
            "name": include_name,
            "pose": pose,
        }
        includes.append(include_item)
        if include_uri == "model://robocon2026_world":
            field_pose = pose
            field_uri = include_uri

    scene_features: list[dict[str, Any]] = []
    if field_uri:
        dae_path = model_root / "robocon2026_world" / "meshes" / "robocon2026.dae"
        if dae_path.is_file():
            scene_features = parse_collada_scene_features(dae_path, field_pose)

    return {
        "field_pose": field_pose,
        "scene_features": scene_features,
        "includes": includes,
    }


def derive_graph_alignment(document: dict[str, Any], sim_config: dict[str, Any], team: str) -> dict[str, Any]:
    team_key = team.lower()
    meilin = sim_config.get("meilin", {}).get(team_key, {})
    offsets: list[tuple[float, float]] = []
    block_targets: dict[int, dict[str, float]] = {}

    for raw_block_id, coords in meilin.items():
        block_id = int(raw_block_id)
        if not isinstance(coords, list) or len(coords) < 3:
            continue
        block_targets[block_id] = {
            "x": float(coords[0]),
            "y": float(coords[1]),
            "z": float(coords[2]),
        }

    for node in document.get("nodes", []):
        block_id = int(node.get("block_id", 0))
        if block_id <= 0 or block_id not in block_targets:
            continue
        pose = node.get("pose", {})
        offsets.append(
            (
                block_targets[block_id]["x"] - float(pose.get("x", 0.0)),
                block_targets[block_id]["y"] - float(pose.get("y", 0.0)),
            )
        )

    dx = sum(item[0] for item in offsets) / len(offsets) if offsets else 0.0
    dy = sum(item[1] for item in offsets) / len(offsets) if offsets else 0.0
    max_error = 0.0
    for current_dx, current_dy in offsets:
        max_error = max(max_error, abs(current_dx - dx), abs(current_dy - dy))

    return {
        "team": team_key,
        "dx": round_float(dx, 4),
        "dy": round_float(dy, 4),
        "max_error": round_float(max_error, 5),
        "meilin": block_targets,
    }


def map_graph_pose_to_sim(
    pose: dict[str, Any],
    *,
    block_id: int,
    alignment: dict[str, Any],
) -> dict[str, float]:
    meilin = alignment.get("meilin", {})
    if block_id > 0 and block_id in meilin:
        target = meilin[block_id]
        return {
            "x": round_float(target["x"], 4),
            "y": round_float(target["y"], 4),
            "z": round_float(float(pose.get("z", 0.0)), 4),
            "world_anchor_z": round_float(target["z"], 4),
        }

    return {
        "x": round_float(float(pose.get("x", 0.0)) + float(alignment.get("dx", 0.0)), 4),
        "y": round_float(float(pose.get("y", 0.0)) + float(alignment.get("dy", 0.0)), 4),
        "z": round_float(float(pose.get("z", 0.0)), 4),
        "world_anchor_z": None,
    }


def graph_nodes_by_id(document: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {str(node["id"]): node for node in document.get("nodes", [])}


def graph_edges_by_id(document: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return {str(edge["id"]): edge for edge in document.get("edges", [])}


def graph_adjacency(document: dict[str, Any]) -> dict[str, list[dict[str, Any]]]:
    adjacency: dict[str, list[dict[str, Any]]] = {}
    for edge in document.get("edges", []):
        adjacency.setdefault(str(edge["from"]), []).append(edge)
    return adjacency


def parse_edge_extra_cost_specs(specs: list[str]) -> dict[str, float]:
    costs: dict[str, float] = {}
    for spec in specs:
        if "=" not in spec:
            raise ValueError(f"Invalid edge extra cost spec '{spec}', expected EDGE_ID=COST")
        edge_id, raw_cost = spec.split("=", 1)
        costs[edge_id.strip()] = float(raw_cost)
    return costs


def build_overlay_state(args: argparse.Namespace, document: dict[str, Any]) -> dict[str, Any]:
    edge_extra_costs = parse_edge_extra_cost_specs(args.edge_extra_cost)
    node_overlays = {
        str(node["id"]): {
            "state": "BLOCKED" if str(node["id"]) in set(args.blocked_node) else "FREE",
        }
        for node in document.get("nodes", [])
    }
    edge_overlays = {}
    for edge in document.get("edges", []):
        edge_id = str(edge["id"])
        state = "ENABLED"
        if edge_id in set(args.blocked_edge):
            state = "BLOCKED"
        elif edge_id in set(args.slow_edge):
            state = "SLOW_ONLY"
        elif edge_id in set(args.confirm_edge):
            state = "CONFIRM_REQUIRED"
        edge_overlays[edge_id] = {
            "state": state,
            "extra_cost": round_float(edge_extra_costs.get(edge_id, 0.0), 4),
        }
    return {
        "node_overlays": node_overlays,
        "edge_overlays": edge_overlays,
        "weights": {
            "time": 1.0,
            "height_risk": 2.0,
            "confirm_required": 1.5,
            "slow_only": 2.0,
        },
    }


def edge_cost_detail(edge: dict[str, Any], overlay: dict[str, Any], weights: dict[str, float]) -> dict[str, Any]:
    if overlay["state"] == "BLOCKED":
        return {
            "blocked": True,
            "total_cost": math.inf,
            "reason": "edge_overlay_blocked",
        }

    base_term = float(edge.get("base_cost", 0.0)) * float(weights["time"])
    height_term = abs(float(edge.get("height_change", 0.0))) * float(weights["height_risk"])
    slow_term = float(weights["slow_only"]) if overlay["state"] == "SLOW_ONLY" else 0.0
    confirm_term = 0.0
    if overlay["state"] == "CONFIRM_REQUIRED" or bool(edge.get("requires_confirmation", False)):
        confirm_term = float(weights["confirm_required"])
    extra_term = float(overlay.get("extra_cost", 0.0))
    total = base_term + height_term + slow_term + confirm_term + extra_term
    return {
        "blocked": False,
        "total_cost": round_float(total, 4),
        "base_term": round_float(base_term, 4),
        "height_term": round_float(height_term, 4),
        "slow_term": round_float(slow_term, 4),
        "confirm_term": round_float(confirm_term, 4),
        "extra_term": round_float(extra_term, 4),
    }


def make_frame(
    index: int,
    message: str,
    *,
    event_type: str,
    closed: set[str],
    frontier: dict[str, float],
    active_node: str | None,
    active_edge: str | None,
    active_to: str | None,
    final_path_nodes: list[str] | None = None,
    final_path_edges: list[str] | None = None,
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    frame = {
        "index": index,
        "type": event_type,
        "message": message,
        "closed_nodes": sorted(closed),
        "frontier": {key: round_float(value, 4) for key, value in sorted(frontier.items())},
        "active_node": active_node,
        "active_edge": active_edge,
        "active_to": active_to,
        "final_path_nodes": final_path_nodes or [],
        "final_path_edges": final_path_edges or [],
        "show_final_path": bool(final_path_nodes),
    }
    if extra:
        frame.update(extra)
    return frame


def reconstruct_path(
    start_node: str,
    goal_node: str,
    prev_node: dict[str, str],
    prev_edge: dict[str, str],
) -> tuple[list[str], list[str]]:
    node_path = [goal_node]
    edge_path: list[str] = []
    cursor = goal_node
    while cursor != start_node:
        edge_path.append(prev_edge[cursor])
        cursor = prev_node[cursor]
        node_path.append(cursor)
    node_path.reverse()
    edge_path.reverse()
    return node_path, edge_path


def plan_route_trace(
    document: dict[str, Any],
    start_node: str,
    goal_node: str,
    overlay_state: dict[str, Any],
    *,
    capture_frames: bool,
) -> dict[str, Any]:
    nodes = graph_nodes_by_id(document)
    adjacency = graph_adjacency(document)
    node_overlays = overlay_state["node_overlays"]
    edge_overlays = overlay_state["edge_overlays"]
    weights = overlay_state["weights"]

    if start_node not in nodes:
        return {
            "success": False,
            "failure_reason": f"起点不存在: {with_raw_label(full_node_label(start_node), start_node)}",
            "frames": [],
            "node_path": [],
            "edge_path": [],
        }
    if goal_node not in nodes:
        return {
            "success": False,
            "failure_reason": f"目标点不存在: {with_raw_label(full_node_label(goal_node), goal_node)}",
            "frames": [],
            "node_path": [],
            "edge_path": [],
        }
    if node_overlays.get(goal_node, {}).get("state") == "BLOCKED":
        return {
            "success": False,
            "failure_reason": f"目标点已被阻塞: {with_raw_label(full_node_label(goal_node), goal_node)}",
            "frames": [],
            "node_path": [],
            "edge_path": [],
        }

    dist = {node_id: math.inf for node_id in nodes}
    dist[start_node] = 0.0
    prev_node: dict[str, str] = {}
    prev_edge: dict[str, str] = {}
    frontier: dict[str, float] = {start_node: 0.0}
    closed: set[str] = set()
    pq: list[tuple[float, str]] = [(0.0, start_node)]
    frames: list[dict[str, Any]] = []
    frame_index = 0

    if capture_frames:
        frames.append(
            make_frame(
                frame_index,
                f"初始化前沿队列，把 {with_raw_label(full_node_label(start_node), start_node)} 放入队列，累计代价为 0.0。",
                event_type="init",
                closed=closed,
                frontier=frontier,
                active_node=start_node,
                active_edge=None,
                active_to=None,
            )
        )
        frame_index += 1

    while pq:
        cost_u, node_id = heapq.heappop(pq)
        if cost_u > dist[node_id]:
            if capture_frames:
                frames.append(
                    make_frame(
                        frame_index,
                        (
                            f"跳过过期队列项 {with_raw_label(full_node_label(node_id), node_id)}: "
                            f"队列里旧代价是 {round_float(cost_u, 4)}，当前最优代价是 {round_float(dist[node_id], 4)}。"
                        ),
                        event_type="skip_stale",
                        closed=closed,
                        frontier=frontier,
                        active_node=node_id,
                        active_edge=None,
                        active_to=None,
                    )
                )
                frame_index += 1
            continue

        frontier.pop(node_id, None)
        closed.add(node_id)
        if capture_frames:
            frames.append(
                make_frame(
                    frame_index,
                    f"从优先队列取出当前最优点 {with_raw_label(full_node_label(node_id), node_id)}，当前累计代价 {round_float(cost_u, 4)}。",
                    event_type="pop",
                    closed=closed,
                    frontier=frontier,
                    active_node=node_id,
                    active_edge=None,
                    active_to=None,
                )
            )
            frame_index += 1

        if node_id == goal_node:
            break

        if node_overlays.get(node_id, {}).get("state") == "BLOCKED" and node_id != start_node:
            if capture_frames:
                frames.append(
                    make_frame(
                        frame_index,
                        f"{with_raw_label(full_node_label(node_id), node_id)} 被 overlay 标记为阻塞，因此不再扩展它的出边。",
                        event_type="blocked_node",
                        closed=closed,
                        frontier=frontier,
                        active_node=node_id,
                        active_edge=None,
                        active_to=None,
                    )
                )
                frame_index += 1
            continue

        for edge in adjacency.get(node_id, []):
            edge_id = str(edge["id"])
            detail = edge_cost_detail(edge, edge_overlays[edge_id], weights)
            if detail["blocked"]:
                if capture_frames:
                    frames.append(
                        make_frame(
                            frame_index,
                            (
                                f"拒绝边 {with_raw_label(edge_transition_label(str(edge['from']), str(edge['to'])), edge_id)}: "
                                "overlay 把这条边标记成阻塞。"
                            ),
                            event_type="edge_blocked",
                            closed=closed,
                            frontier=frontier,
                            active_node=node_id,
                            active_edge=edge_id,
                            active_to=str(edge["to"]),
                        )
                    )
                    frame_index += 1
                continue

            new_cost = dist[node_id] + float(detail["total_cost"])
            old_cost = dist[str(edge["to"])]
            if new_cost < old_cost:
                dist[str(edge["to"])] = new_cost
                prev_node[str(edge["to"])] = node_id
                prev_edge[str(edge["to"])] = edge_id
                frontier[str(edge["to"])] = new_cost
                heapq.heappush(pq, (new_cost, str(edge["to"])))
                if capture_frames:
                    frames.append(
                        make_frame(
                            frame_index,
                            (
                                f"更新更优路径 {with_raw_label(edge_transition_label(str(edge['from']), str(edge['to'])), edge_id)}: "
                                f"旧总代价为 {round_float(old_cost, 4) if not math.isinf(old_cost) else '无穷大'}，"
                                f"新总代价为 {round_float(new_cost, 4)}。"
                                f"这条边本身代价 {detail['total_cost']} = 基础 {detail['base_term']} + "
                                f"高度风险 {detail['height_term']} + 确认成本 {detail['confirm_term']} + "
                                f"慢行惩罚 {detail['slow_term']} + 额外附加 {detail['extra_term']}。"
                            ),
                            event_type="relax",
                            closed=closed,
                            frontier=frontier,
                            active_node=node_id,
                            active_edge=edge_id,
                            active_to=str(edge["to"]),
                            extra={
                                "edge_cost": detail["total_cost"],
                                "new_cost": round_float(new_cost, 4),
                                "old_cost": "inf" if math.isinf(old_cost) else round_float(old_cost, 4),
                            },
                        )
                    )
                    frame_index += 1
            elif capture_frames:
                frames.append(
                    make_frame(
                        frame_index,
                        (
                            f"保持 {with_raw_label(full_node_label(str(edge['to'])), str(edge['to']))} 的现有最优路径不变。"
                            f"如果走 {with_raw_label(edge_transition_label(str(edge['from']), str(edge['to'])), edge_id)}，"
                            f"新的总代价会是 {round_float(new_cost, 4)}，但当前最优代价已经是 {round_float(old_cost, 4)}。"
                        ),
                        event_type="keep_best",
                        closed=closed,
                        frontier=frontier,
                        active_node=node_id,
                        active_edge=edge_id,
                        active_to=str(edge["to"]),
                        extra={
                            "edge_cost": detail["total_cost"],
                            "new_cost": round_float(new_cost, 4),
                            "old_cost": round_float(old_cost, 4),
                        },
                    )
                )
                frame_index += 1

    if math.isinf(dist[goal_node]):
        if capture_frames:
            frames.append(
                make_frame(
                    frame_index,
                    (
                        f"没有找到从 {with_raw_label(full_node_label(start_node), start_node)} 到 "
                        f"{with_raw_label(full_node_label(goal_node), goal_node)} 的可行路径。"
                    ),
                    event_type="failed",
                    closed=closed,
                    frontier=frontier,
                    active_node=None,
                    active_edge=None,
                    active_to=None,
                )
            )
        return {
            "success": False,
            "failure_reason": (
                f"没有从 {with_raw_label(full_node_label(start_node), start_node)} 到 "
                f"{with_raw_label(full_node_label(goal_node), goal_node)} 的可行路径"
            ),
            "frames": frames,
            "node_path": [],
            "edge_path": [],
        }

    node_path, edge_path = reconstruct_path(start_node, goal_node, prev_node, prev_edge)
    final_segments = build_path_segments(document, edge_path, overlay_state)
    if capture_frames:
        frames.append(
            make_frame(
                frame_index,
                f"已到达目标。最终路径共 {len(edge_path)} 条边，总代价 {round_float(dist[goal_node], 4)}。",
                event_type="goal",
                closed=closed,
                frontier=frontier,
                active_node=goal_node,
                active_edge=edge_path[-1] if edge_path else None,
                active_to=None,
                final_path_nodes=node_path,
                final_path_edges=edge_path,
                extra={"total_cost": round_float(dist[goal_node], 4)},
            )
        )

    return {
        "success": True,
        "failure_reason": "",
        "frames": frames,
        "node_path": node_path,
        "edge_path": edge_path,
        "total_cost": round_float(dist[goal_node], 4),
        "final_segments": final_segments,
    }


def build_path_segments(
    document: dict[str, Any],
    edge_ids: list[str],
    overlay_state: dict[str, Any],
) -> list[dict[str, Any]]:
    edges = graph_edges_by_id(document)
    weights = overlay_state["weights"]
    edge_overlays = overlay_state["edge_overlays"]
    segments: list[dict[str, Any]] = []
    cumulative_cost = 0.0
    for edge_id in edge_ids:
        edge = edges[edge_id]
        detail = edge_cost_detail(edge, edge_overlays[edge_id], weights)
        cumulative_cost += float(detail["total_cost"])
        segments.append(
            {
                "edge_id": edge_id,
                "from": str(edge["from"]),
                "to": str(edge["to"]),
                "motion_type": str(edge.get("motion_type", "")),
                "required_mode": str(edge.get("required_mode", "")),
                "height_change": round_float(edge.get("height_change", 0.0), 4),
                "edge_cost": round_float(detail["total_cost"], 4),
                "base_cost": round_float(edge.get("base_cost", 0.0), 4),
                "cumulative_cost": round_float(cumulative_cost, 4),
                "requires_confirmation": bool(edge.get("requires_confirmation", False)),
            }
        )
    return segments


def plan_task(document: dict[str, Any], start_node: str, task_tag: str, overlay_state: dict[str, Any]) -> dict[str, Any]:
    task = None
    for item in document.get("tasks", []):
        if str(item.get("task_tag")) == task_tag:
            task = item
            break
    if task is None:
        return {
            "success": False,
            "failure_reason": f"任务不存在: {with_raw_label(task_tag_label(task_tag), task_tag)}",
            "frames": [],
            "candidate_results": [],
            "node_path": [],
            "edge_path": [],
        }

    candidate_results = []
    best_candidate = None
    best_cost = math.inf
    for candidate in task.get("candidate_nodes", []):
        candidate_id = str(candidate)
        if overlay_state["node_overlays"].get(candidate_id, {}).get("state") == "BLOCKED":
            candidate_results.append(
                {
                    "candidate": candidate_id,
                    "success": False,
                    "cost": None,
                    "reason": "candidate_blocked",
                }
            )
            continue
        result = plan_route_trace(document, start_node, candidate_id, overlay_state, capture_frames=False)
        candidate_results.append(
            {
                "candidate": candidate_id,
                "success": bool(result["success"]),
                "cost": result.get("total_cost"),
                "reason": result.get("failure_reason", ""),
            }
        )
        if result["success"] and float(result["total_cost"]) < best_cost:
            best_cost = float(result["total_cost"])
            best_candidate = candidate_id

    if best_candidate is None:
        return {
            "success": False,
            "failure_reason": f"任务 {with_raw_label(task_tag_label(task_tag), task_tag)} 没有可达候选点",
            "frames": [],
            "candidate_results": candidate_results,
            "node_path": [],
            "edge_path": [],
        }

    best_trace = plan_route_trace(document, start_node, best_candidate, overlay_state, capture_frames=True)
    best_trace["candidate_results"] = candidate_results
    best_trace["selected_candidate"] = best_candidate
    return best_trace


def plan_route_tag(document: dict[str, Any], start_node: str, route_tag: str, overlay_state: dict[str, Any]) -> dict[str, Any]:
    route = None
    for item in document.get("routes", []):
        if str(item.get("route_tag")) == route_tag:
            route = item
            break
    if route is None:
        return {
            "success": False,
            "failure_reason": f"预设路线不存在: {with_raw_label(route_tag_label(route_tag), route_tag)}",
            "frames": [],
            "node_path": [],
            "edge_path": [],
        }

    route_nodes = [str(node_id) for node_id in route.get("nodes", [])]
    if not route_nodes:
        return {
            "success": False,
            "failure_reason": f"预设路线为空: {with_raw_label(route_tag_label(route_tag), route_tag)}",
            "frames": [],
            "node_path": [],
            "edge_path": [],
        }

    prefix_trace = None
    combined_nodes: list[str] = []
    combined_edges: list[str] = []
    frames: list[dict[str, Any]] = []
    if start_node != route_nodes[0]:
        prefix_trace = plan_route_trace(document, start_node, route_nodes[0], overlay_state, capture_frames=True)
        if not prefix_trace["success"]:
            prefix_trace["failure_reason"] = (
                f"预设路线 {with_raw_label(route_tag_label(route_tag), route_tag)} 的前缀路径求解失败: "
                f"{prefix_trace['failure_reason']}"
            )
            return prefix_trace
        combined_nodes.extend(prefix_trace["node_path"])
        combined_edges.extend(prefix_trace["edge_path"])
        frames.extend(prefix_trace["frames"])
    else:
        combined_nodes.append(start_node)

    edge_lookup = {(str(edge["from"]), str(edge["to"])): str(edge["id"]) for edge in document.get("edges", [])}
    for left, right in zip(route_nodes, route_nodes[1:]):
        edge_id = edge_lookup.get((left, right))
        if edge_id is None:
            return {
                "success": False,
                "failure_reason": (
                    f"预设路线 {with_raw_label(route_tag_label(route_tag), route_tag)} 中缺少直接边: "
                    f"{with_raw_label(full_node_label(left), left)} -> {with_raw_label(full_node_label(right), right)}"
                ),
                "frames": frames,
                "node_path": [],
                "edge_path": [],
            }
        if overlay_state["node_overlays"].get(right, {}).get("state") == "BLOCKED":
            return {
                "success": False,
                "failure_reason": (
                    f"预设路线 {with_raw_label(route_tag_label(route_tag), route_tag)} 的目标点已被阻塞: "
                    f"{with_raw_label(full_node_label(right), right)}"
                ),
                "frames": frames,
                "node_path": [],
                "edge_path": [],
            }
        detail = edge_cost_detail(graph_edges_by_id(document)[edge_id], overlay_state["edge_overlays"][edge_id], overlay_state["weights"])
        if detail["blocked"]:
            return {
                "success": False,
                "failure_reason": (
                    f"预设路线 {with_raw_label(route_tag_label(route_tag), route_tag)} 的边已被阻塞: "
                    f"{with_raw_label(edge_transition_label(left, right), edge_id)}"
                ),
                "frames": frames,
                "node_path": [],
                "edge_path": [],
            }
        if not combined_nodes or combined_nodes[-1] != left:
            combined_nodes.append(left)
        combined_nodes.append(right)
        combined_edges.append(edge_id)

    total_cost = sum(segment["edge_cost"] for segment in build_path_segments(document, combined_edges, overlay_state))
    frames.append(
        make_frame(
            len(frames),
            (
                f"预设路线 {with_raw_label(route_tag_label(route_tag), route_tag)} 展开后共有 "
                f"{len(combined_edges)} 条直接边，总代价 {round_float(total_cost, 4)}。"
            ),
            event_type="route_tag",
            closed=set(),
            frontier={},
            active_node=route_nodes[-1],
            active_edge=combined_edges[-1] if combined_edges else None,
            active_to=None,
            final_path_nodes=combined_nodes,
            final_path_edges=combined_edges,
            extra={"total_cost": round_float(total_cost, 4)},
        )
    )
    return {
        "success": True,
        "failure_reason": "",
        "frames": frames,
        "node_path": combined_nodes,
        "edge_path": combined_edges,
        "total_cost": round_float(total_cost, 4),
        "final_segments": build_path_segments(document, combined_edges, overlay_state),
    }


def default_planning_request(document: dict[str, Any]) -> dict[str, str]:
    node_ids = {str(node["id"]) for node in document.get("nodes", [])}
    if "ramp_entry_south" in node_ids and "ramp_exit_north" in node_ids:
        return {"start": "ramp_entry_south", "goal_kind": "node", "goal_value": "ramp_exit_north"}
    if "mf_entry_staging" in node_ids and "mf_exit_staging" in node_ids:
        return {"start": "mf_entry_staging", "goal_kind": "node", "goal_value": "mf_exit_staging"}
    first_node = str(document.get("nodes", [{}])[0].get("id", ""))
    last_node = str(document.get("nodes", [{}])[-1].get("id", ""))
    return {"start": first_node, "goal_kind": "node", "goal_value": last_node}


def run_planning(document: dict[str, Any], args: argparse.Namespace, overlay_state: dict[str, Any]) -> dict[str, Any]:
    defaults = default_planning_request(document)
    start_node = args.start or defaults["start"]

    if args.goal_task:
        result = plan_task(document, start_node, args.goal_task, overlay_state)
        result["goal_kind"] = "task"
        result["goal_value"] = args.goal_task
    elif args.goal_route:
        result = plan_route_tag(document, start_node, args.goal_route, overlay_state)
        result["goal_kind"] = "route"
        result["goal_value"] = args.goal_route
    else:
        goal_node = args.goal_node or defaults["goal_value"]
        result = plan_route_trace(document, start_node, goal_node, overlay_state, capture_frames=True)
        result["goal_kind"] = "node"
        result["goal_value"] = goal_node

    result["start_node"] = start_node
    return result


def coordinate_mapper(scene_points: list[tuple[float, float]]):
    min_x = min(point[0] for point in scene_points)
    max_x = max(point[0] for point in scene_points)
    min_y = min(point[1] for point in scene_points)
    max_y = max(point[1] for point in scene_points)
    span_x = max(max_x - min_x, 1e-6)
    span_y = max(max_y - min_y, 1e-6)
    scale = min((SVG_WIDTH - 2 * SVG_PADDING) / span_x, (SVG_HEIGHT - 2 * SVG_PADDING) / span_y)

    def map_point(x: float, y: float) -> tuple[float, float]:
        return (
            round_float(SVG_PADDING + (x - min_x) * scale, 2),
            round_float(SVG_HEIGHT - SVG_PADDING - (y - min_y) * scale, 2),
        )

    return map_point


def collect_scene_points(
    document: dict[str, Any],
    alignment: dict[str, Any] | None,
    scene_features: list[dict[str, Any]],
    meilin_slots: list[dict[str, Any]],
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for feature in scene_features:
        points.extend((float(point["x"]), float(point["y"])) for point in feature.get("points", []))
    for slot in meilin_slots:
        points.append((float(slot["x"]), float(slot["y"])))
    if alignment is None:
        for node in document.get("nodes", []):
            pose = node.get("pose", {})
            points.append((float(pose.get("x", 0.0)), float(pose.get("y", 0.0))))
        for edge in document.get("edges", []):
            for control_point in edge.get("control_points", []):
                points.append((float(control_point.get("x", 0.0)), float(control_point.get("y", 0.0))))
        return points

    for node in document.get("nodes", []):
        sim_pose = map_graph_pose_to_sim(node.get("pose", {}), block_id=int(node.get("block_id", 0)), alignment=alignment)
        points.append((float(sim_pose["x"]), float(sim_pose["y"])))
    for edge in document.get("edges", []):
        for control_point in edge.get("control_points", []):
            sim_pose = map_graph_pose_to_sim(control_point, block_id=0, alignment=alignment)
            points.append((float(sim_pose["x"]), float(sim_pose["y"])))
    return points


def build_meilin_slots(sim_config: dict[str, Any], team: str) -> list[dict[str, Any]]:
    team_slots = sim_config.get("meilin", {}).get(team.lower(), {})
    slots = []
    for raw_block_id, coords in sorted(team_slots.items(), key=lambda item: int(item[0])):
        if not isinstance(coords, list) or len(coords) < 3:
            continue
        slots.append(
            {
                "block_id": int(raw_block_id),
                "x": round_float(coords[0], 4),
                "y": round_float(coords[1], 4),
                "z": round_float(coords[2], 4),
            }
        )
    return slots


def build_render_model(
    document: dict[str, Any],
    *,
    alignment: dict[str, Any] | None,
    scene_features: list[dict[str, Any]],
    meilin_slots: list[dict[str, Any]],
    planning: dict[str, Any],
) -> dict[str, Any]:
    points = collect_scene_points(document, alignment, scene_features, meilin_slots)
    mapper = coordinate_mapper(points)
    node_lookup = graph_nodes_by_id(document)

    rendered_features = []
    for feature in scene_features:
        mapped_points = [mapper(float(point["x"]), float(point["y"])) for point in feature.get("points", [])]
        rendered_features.append(
            {
                **feature,
                "svg_points": " ".join(f"{point[0]},{point[1]}" for point in mapped_points),
                "label_x": round_float(sum(point[0] for point in mapped_points) / len(mapped_points), 2),
                "label_y": round_float(sum(point[1] for point in mapped_points) / len(mapped_points), 2),
            }
        )

    rendered_slots = []
    half_size = MEILIN_PAD_SIZE / 2.0
    for slot in meilin_slots:
        x1, y1 = mapper(slot["x"] - half_size, slot["y"] + half_size)
        x2, y2 = mapper(slot["x"] + half_size, slot["y"] - half_size)
        rendered_slots.append(
            {
                **slot,
                "x": x1,
                "y": y1,
                "width": round_float(x2 - x1, 2),
                "height": round_float(y2 - y1, 2),
            }
        )

    rendered_nodes = []
    for node in document.get("nodes", []):
        pose = node.get("pose", {})
        node_id = str(node["id"])
        short_label = short_node_label(node_id)
        full_label = full_node_label(node_id)
        sim_pose = map_graph_pose_to_sim(pose, block_id=int(node.get("block_id", 0)), alignment=alignment) if alignment else {
            "x": float(pose.get("x", 0.0)),
            "y": float(pose.get("y", 0.0)),
            "z": float(pose.get("z", 0.0)),
            "world_anchor_z": None,
        }
        cx, cy = mapper(sim_pose["x"], sim_pose["y"])
        rendered_nodes.append(
            {
                "id": node_id,
                "type": str(node.get("type", "")),
                "short_label": short_label,
                "full_label": full_label,
                "cx": cx,
                "cy": cy,
                "sim_x": round_float(sim_pose["x"], 4),
                "sim_y": round_float(sim_pose["y"], 4),
                "semantic_z": round_float(sim_pose["z"], 4),
                "world_anchor_z": sim_pose["world_anchor_z"],
                "color": node_color(str(node.get("type", ""))),
                "title": (
                    f"{full_label}\n"
                    f"原始 ID: {node_id}\n"
                    f"节点类型: {node_type_label(str(node.get('type', '')))}\n"
                    f"图坐标 xyz: ({round_float(pose.get('x', 0.0))}, {round_float(pose.get('y', 0.0))}, {round_float(pose.get('z', 0.0))})\n"
                    f"仿真坐标 xy: ({round_float(sim_pose['x'], 4)}, {round_float(sim_pose['y'], 4)})\n"
                    f"基础代价: {round_float(node.get('base_cost', 0.0))}\n"
                    f"操作标签: {node.get('operation_tag', '') or '无'}"
                ),
            }
        )

    rendered_edges = []
    for edge in document.get("edges", []):
        from_node = node_lookup[str(edge["from"])]
        to_node = node_lookup[str(edge["to"])]
        from_pose = map_graph_pose_to_sim(from_node.get("pose", {}), block_id=int(from_node.get("block_id", 0)), alignment=alignment) if alignment else from_node.get("pose", {})
        to_pose = map_graph_pose_to_sim(to_node.get("pose", {}), block_id=int(to_node.get("block_id", 0)), alignment=alignment) if alignment else to_node.get("pose", {})

        points_world = [from_pose]
        for control_point in edge.get("control_points", []):
            points_world.append(map_graph_pose_to_sim(control_point, block_id=0, alignment=alignment) if alignment else control_point)
        points_world.append(to_pose)
        screen_points = [mapper(float(point["x"]), float(point["y"])) for point in points_world]
        path = " ".join(
            [f"M {screen_points[0][0]} {screen_points[0][1]}"]
            + [f"L {point[0]} {point[1]}" for point in screen_points[1:]]
        )
        edge_id = str(edge["id"])
        edge_label = edge_transition_label(str(edge["from"]), str(edge["to"]))
        rendered_edges.append(
            {
                "id": edge_id,
                "from": str(edge["from"]),
                "to": str(edge["to"]),
                "motion_type": str(edge.get("motion_type", "")),
                "motion_label": motion_type_label(str(edge.get("motion_type", ""))),
                "full_label": edge_label,
                "color": edge_color(str(edge.get("motion_type", ""))),
                "path": path,
                "title": (
                    f"{edge_label}\n"
                    f"原始 ID: {edge_id}\n"
                    f"运动语义: {motion_type_label(str(edge.get('motion_type', '')))}\n"
                    f"基础代价: {round_float(edge.get('base_cost', 0.0), 4)}\n"
                    f"高度变化 dZ: {round_float(edge.get('height_change', 0.0), 4)}"
                ),
            }
        )

    final_path_profile = build_path_profile(planning, node_lookup)
    return {
        "scene_features": rendered_features,
        "scene_summary": {
            "visible_surfaces": len(rendered_features),
            "ground_surfaces": sum(1 for feature in rendered_features if feature["render_class"] == "world-ground"),
            "platform_surfaces": sum(1 for feature in rendered_features if feature["render_class"] == "world-platform"),
            "fence_surfaces": sum(1 for feature in rendered_features if feature["render_class"] == "world-fence"),
            "marking_surfaces": sum(1 for feature in rendered_features if feature["render_class"] == "world-marking"),
        },
        "meilin_slots": rendered_slots,
        "nodes": rendered_nodes,
        "edges": rendered_edges,
        "path_profile": final_path_profile,
    }


def build_path_profile(planning: dict[str, Any], node_lookup: dict[str, dict[str, Any]]) -> dict[str, Any]:
    node_path = planning.get("node_path", [])
    edge_segments = planning.get("final_segments", [])
    if not node_path:
        return {"points": [], "segments": []}

    z_values = [float(node_lookup[node_id].get("pose", {}).get("z", 0.0)) for node_id in node_path]
    min_z = min(z_values)
    max_z = max(z_values)
    span_z = max(max_z - min_z, 0.05)
    width = 320.0
    height = 170.0
    x_step = width / max(len(node_path) - 1, 1)

    points = []
    for index, node_id in enumerate(node_path):
        z_value = float(node_lookup[node_id].get("pose", {}).get("z", 0.0))
        x = round_float(index * x_step, 2)
        y = round_float(height - ((z_value - min_z) / span_z) * (height - 24) - 12, 2)
        points.append({"node_id": node_id, "x": x, "y": y, "z": round_float(z_value, 4)})

    polyline = " ".join(f"{point['x']},{point['y']}" for point in points)
    return {
        "points": points,
        "polyline": polyline,
        "segments": edge_segments,
        "width": width,
        "height": height,
        "min_z": round_float(min_z, 4),
        "max_z": round_float(max_z, 4),
    }


def page_title(graph_path: Path, args: argparse.Namespace, team: str) -> str:
    if args.title:
        return args.title
    return f"RC26 拓扑导航仿真观察页 · {team_label(team)}"


def render_html(
    graph_path: Path,
    document: dict[str, Any],
    render_model: dict[str, Any],
    planning: dict[str, Any],
    overlay_state: dict[str, Any],
    alignment: dict[str, Any] | None,
    world_context: dict[str, Any] | None,
    args: argparse.Namespace,
) -> str:
    team = str(document.get("meta", {}).get("team", "unknown")).lower()
    team_display = team_label(team)
    title = page_title(graph_path, args, team)
    goal_kind = str(planning.get("goal_kind", ""))
    goal_value = str(planning.get("goal_value", ""))
    goal_display = goal_value_label(goal_kind, goal_value)
    graph_json = {
        "meta": document.get("meta", {}),
        "planning": {
            "success": bool(planning.get("success")),
            "failure_reason": planning.get("failure_reason", ""),
            "start_node": planning.get("start_node", ""),
            "start_node_label": full_node_label(str(planning.get("start_node", ""))) if planning.get("start_node") else "",
            "goal_kind": planning.get("goal_kind", ""),
            "goal_kind_label": goal_kind_label(goal_kind),
            "goal_value": planning.get("goal_value", ""),
            "goal_value_label": goal_display,
            "selected_candidate": planning.get("selected_candidate"),
            "selected_candidate_label": (
                with_raw_label(full_node_label(str(planning.get("selected_candidate"))), str(planning.get("selected_candidate")))
                if planning.get("selected_candidate")
                else ""
            ),
            "total_cost": planning.get("total_cost"),
            "frames": planning.get("frames", []),
            "candidate_results": planning.get("candidate_results", []),
            "final_segments": planning.get("final_segments", []),
        },
        "nodes": {
            item["id"]: {
                "type": item["type"],
                "type_label": node_type_label(item["type"]),
                "short_label": item["short_label"],
                "full_label": item["full_label"],
                "sim_x": item["sim_x"],
                "sim_y": item["sim_y"],
                "semantic_z": item["semantic_z"],
                "world_anchor_z": item["world_anchor_z"],
            }
            for item in render_model["nodes"]
        },
        "edges": {
            item["id"]: {
                "from": item["from"],
                "to": item["to"],
                "full_label": item["full_label"],
                "motion_type": item["motion_type"],
                "motion_label": item["motion_label"],
            }
            for item in render_model["edges"]
        },
    }
    edge_display_lookup = {
        item["id"]: with_raw_label(item["full_label"], item["id"])
        for item in render_model["edges"]
    }

    def format_node_refs(node_ids: list[str]) -> str:
        if not node_ids:
            return "无"
        return "、".join(with_raw_label(full_node_label(node_id), node_id) for node_id in node_ids)

    def format_edge_refs(edge_ids: list[str]) -> str:
        if not edge_ids:
            return "无"
        return "、".join(edge_display_lookup.get(edge_id, edge_id) for edge_id in edge_ids)

    feature_elements = []
    for feature in render_model["scene_features"]:
        title_text = (
            f"{feature['name']}\n"
            f"材质: {feature['material_symbol']}\n"
            f"分类: {world_render_class_label(feature['render_class'])}\n"
            f"平均高度: {feature['avg_z']}\n"
            f"高度跨度: {feature['z_span']}\n"
            f"投影面积: {feature['area_xy']}"
        )
        feature_elements.append(
            (
                f"<g class=\"world-feature {escape_attr(feature['render_class'])}\" id=\"feature-{escape_attr(slugify(feature['id']))}\">"
                f"<polygon points=\"{escape_attr(feature['svg_points'])}\" fill=\"{escape_attr(feature['fill'])}\" "
                f"fill-opacity=\"{escape_attr(feature['opacity'])}\" stroke=\"{escape_attr(feature['stroke'])}\" "
                f"stroke-opacity=\"{escape_attr(feature['stroke_opacity'])}\" stroke-width=\"{escape_attr(feature['stroke_width'])}\">"
                + (f"<title>{html.escape(title_text)}</title>" if feature["show_title"] else "")
                + "</polygon>"
                + "</g>"
            )
        )

    slot_elements = []
    for slot in render_model["meilin_slots"]:
        slot_elements.append(
            (
                f"<g class=\"meilin-slot\" id=\"slot-{slot['block_id']}\">"
                f"<rect x=\"{slot['x']}\" y=\"{slot['y']}\" width=\"{slot['width']}\" height=\"{slot['height']}\" rx=\"8\" ry=\"8\" />"
                f"<text class=\"meilin-slot-label\" x=\"{round_float(slot['x'] + slot['width'] / 2, 2)}\" y=\"{round_float(slot['y'] + slot['height'] / 2 + 4, 2)}\">块{slot['block_id']}</text>"
                "</g>"
            )
        )

    edge_elements = []
    for edge in render_model["edges"]:
        edge_elements.append(
            (
                f"<g class=\"edge-group\" data-edge-id=\"{escape_attr(edge['id'])}\">"
                f"<path class=\"edge\" id=\"edge-{escape_attr(edge['id'])}\" d=\"{escape_attr(edge['path'])}\" "
                f"stroke=\"{escape_attr(edge['color'])}\" "
                f"marker-end=\"url(#arrow-{escape_attr(slugify(edge['motion_type']))})\">"
                f"<title>{html.escape(edge['title'])}</title>"
                "</path>"
                "</g>"
            )
        )

    node_elements = []
    for node in render_model["nodes"]:
        node_elements.append(
            (
                f"<g class=\"node\" id=\"node-{escape_attr(node['id'])}\" data-node-id=\"{escape_attr(node['id'])}\">"
                f"<circle class=\"node-core\" cx=\"{node['cx']}\" cy=\"{node['cy']}\" r=\"11\" fill=\"{escape_attr(node['color'])}\">"
                f"<title>{html.escape(node['title'])}</title>"
                "</circle>"
                f"<text class=\"node-label\" x=\"{node['cx']}\" y=\"{round_float(node['cy'] - 18, 2)}\">{html.escape(node['short_label'])}</text>"
                f"<text class=\"node-z\" x=\"{node['cx']}\" y=\"{round_float(node['cy'] + 28, 2)}\">高={node['semantic_z']}</text>"
                f"<text class=\"node-dist\" id=\"dist-{escape_attr(node['id'])}\" x=\"{node['cx']}\" y=\"{round_float(node['cy'] + 44, 2)}\"></text>"
                "</g>"
            )
        )

    edge_legend = []
    for motion_type, color in sorted(EDGE_COLORS.items()):
        edge_legend.append(
            f"<div class=\"legend-item\"><span class=\"legend-line\" style=\"color:{escape_attr(color)}\"></span><span>{html.escape(motion_type_label(motion_type))}</span></div>"
        )

    node_legend = []
    for node_type, color in sorted(NODE_COLORS.items()):
        node_legend.append(
            f"<div class=\"legend-item\"><span class=\"legend-dot\" style=\"background:{escape_attr(color)}\"></span><span>{html.escape(node_type_label(node_type))}</span></div>"
        )

    task_result_rows = []
    for item in planning.get("candidate_results", []):
        candidate_id = str(item["candidate"])
        task_result_rows.append(
            "<tr>"
            f"<td><strong>{html.escape(full_node_label(candidate_id))}</strong><br><span class=\"table-sub\">原始 ID: {html.escape(candidate_id)}</span></td>"
            f"<td>{'可达' if item['success'] else '失败'}</td>"
            f"<td>{html.escape(str(item['cost'])) if item['cost'] is not None else '-'}</td>"
            f"<td>{html.escape(str(task_result_reason_label(str(item['reason'])) or '无'))}</td>"
            "</tr>"
        )

    profile = render_model["path_profile"]
    profile_points = []
    for point in profile["points"]:
        profile_points.append(
            f"<g><circle cx=\"{point['x']}\" cy=\"{point['y']}\" r=\"4\" /><text x=\"{point['x']}\" y=\"{round_float(point['y'] - 10, 2)}\">{html.escape(short_node_label(point['node_id']))}</text></g>"
        )

    segment_rows = []
    for segment in profile["segments"]:
        segment_label = edge_transition_label(str(segment["from"]), str(segment["to"]))
        segment_rows.append(
            "<tr>"
            f"<td><strong>{html.escape(segment_label)}</strong><br><span class=\"table-sub\">原始 ID: {html.escape(segment['edge_id'])}</span></td>"
            f"<td>{html.escape(motion_type_label(str(segment['motion_type'])))}</td>"
            f"<td>{segment['height_change']}</td>"
            f"<td>{segment['edge_cost']}</td>"
            f"<td>{segment['cumulative_cost']}</td>"
            "</tr>"
        )

    alignment_summary = ""
    if alignment is not None:
        alignment_summary = (
            f"<p class=\"hint\">拓扑图到仿真场地的自动对齐结果: x 平移 {alignment['dx']}，y 平移 {alignment['dy']}，"
            f"最大拟合误差 {alignment['max_error']}。</p>"
        )

    world_summary = ""
    if world_context is not None:
        scene_summary = render_model["scene_summary"]
        world_summary = (
            f"<p class=\"hint\">当前可见的真实场地面片共 {scene_summary['visible_surfaces']} 个，"
            f"其中地面分区 {scene_summary['ground_surfaces']} 个、标线/起始区 {scene_summary['marking_surfaces']} 个、"
            f"平台/台面 {scene_summary['platform_surfaces']} 个、围栏轮廓 {scene_summary['fence_surfaces']} 个。"
            f"细碎杆件和柱体面片已过滤，只保留更容易看懂比赛场地的真实投影。梅林块位参考框 {len(render_model['meilin_slots'])} 个。</p>"
        )

    marker_defs = []
    for motion_type, color in sorted(EDGE_COLORS.items()):
        marker_defs.append(
            (
                f"<marker id=\"arrow-{escape_attr(slugify(motion_type))}\" viewBox=\"0 0 12 12\" refX=\"10\" refY=\"6\" "
                "markerWidth=\"8\" markerHeight=\"8\" orient=\"auto\">"
                f"<path d=\"M 0 1 L 10 6 L 0 11 z\" fill=\"{escape_attr(color)}\" />"
                "</marker>"
            )
        )

    style = """
    :root {
      --bg: #f4efe8;
      --panel: rgba(255, 252, 247, 0.9);
      --panel-strong: rgba(255, 255, 255, 0.94);
      --ink: #1d2730;
      --muted: #62717f;
      --border: rgba(29, 39, 48, 0.12);
      --shadow: 0 18px 40px rgba(29, 39, 48, 0.12);
      --frontier: #4ea8de;
      --closed: #5e548e;
      --active: #e36414;
      --path: #ffb703;
      --danger: #c1121f;
      --font-sans: "IBM Plex Sans", "Noto Sans SC", "PingFang SC", sans-serif;
      --font-mono: "IBM Plex Mono", "JetBrains Mono", monospace;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: var(--font-sans);
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(255, 227, 196, 0.8), transparent 32%),
        radial-gradient(circle at bottom right, rgba(196, 224, 229, 0.62), transparent 28%),
        linear-gradient(180deg, #f7f3eb, #e9f0e5);
    }
    .page {
      min-height: 100vh;
      padding: 24px;
      display: grid;
      grid-template-columns: minmax(760px, 1fr) 420px;
      gap: 22px;
    }
    .panel {
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 22px;
      box-shadow: var(--shadow);
      backdrop-filter: blur(8px);
    }
    .main-panel {
      padding: 22px;
      display: flex;
      flex-direction: column;
      gap: 18px;
    }
    h1, h2, h3 { margin: 0; }
    p { margin: 0; }
    .header p {
      margin-top: 10px;
      color: var(--muted);
      line-height: 1.55;
      max-width: 60rem;
    }
    .meta-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(136px, 1fr));
      gap: 12px;
    }
    .meta-card {
      background: rgba(255, 255, 255, 0.62);
      border: 1px solid rgba(29, 39, 48, 0.08);
      border-radius: 18px;
      padding: 12px 14px;
    }
    .meta-card .label {
      display: block;
      font-size: 12px;
      letter-spacing: 0.04em;
      color: var(--muted);
    }
    .meta-card .value {
      display: block;
      margin-top: 6px;
      font-size: 22px;
      font-weight: 700;
    }
    .canvas {
      border-radius: 22px;
      overflow: hidden;
      border: 1px solid rgba(29, 39, 48, 0.08);
      background:
        linear-gradient(180deg, rgba(255,255,255,0.86), rgba(248,244,237,0.92)),
        linear-gradient(135deg, rgba(208, 229, 217, 0.28), rgba(255, 233, 200, 0.2));
    }
    .canvas svg {
      display: block;
      width: 100%;
      height: auto;
    }
    .world-feature polygon {
      transition: opacity 120ms ease;
    }
    .world-feature.world-ground polygon {
      shape-rendering: geometricPrecision;
    }
    .world-feature.world-marking polygon {
      shape-rendering: geometricPrecision;
    }
    .world-feature.world-platform polygon {
      filter: drop-shadow(0 2px 2px rgba(29, 39, 48, 0.06));
    }
    .world-feature.world-fence polygon {
      stroke-dasharray: 5 4;
    }
    .meilin-slot rect {
      fill: rgba(245, 187, 110, 0.22);
      stroke: rgba(193, 124, 34, 0.42);
      stroke-dasharray: 7 5;
      stroke-width: 1.8;
    }
    .meilin-slot-label {
      fill: rgba(120, 82, 22, 0.78);
      font-size: 12px;
      font-weight: 700;
      text-anchor: middle;
      pointer-events: none;
    }
    .edge {
      fill: none;
      stroke-width: 3.4;
      opacity: 0.28;
      transition: opacity 120ms ease, stroke-width 120ms ease, filter 120ms ease;
    }
    .edge.frontier {
      opacity: 0.54;
    }
    .edge.active {
      opacity: 1;
      stroke-width: 6.5;
      filter: drop-shadow(0 0 10px rgba(227, 100, 20, 0.35));
    }
    .edge.final-path {
      opacity: 1;
      stroke-width: 5.6;
      filter: drop-shadow(0 0 8px rgba(255, 183, 3, 0.35));
    }
    .node-core {
      stroke: rgba(29, 39, 48, 0.7);
      stroke-width: 2.2;
      transition: stroke-width 120ms ease, filter 120ms ease, opacity 120ms ease;
    }
    .node.closed .node-core {
      stroke-width: 4;
      filter: drop-shadow(0 0 10px rgba(94, 84, 142, 0.32));
    }
    .node.frontier .node-core {
      stroke-width: 4;
      filter: drop-shadow(0 0 10px rgba(78, 168, 222, 0.32));
    }
    .node.current .node-core {
      stroke-width: 5.5;
      filter: drop-shadow(0 0 12px rgba(227, 100, 20, 0.42));
    }
    .node.on-final-path .node-core {
      stroke-width: 4.8;
      filter: drop-shadow(0 0 10px rgba(255, 183, 3, 0.38));
    }
    .node.start .node-core { stroke: #2a9d8f; }
    .node.goal .node-core { stroke: #d62828; }
    .node-label, .node-z, .node-dist {
      text-anchor: middle;
      pointer-events: none;
    }
    .node-label {
      font-size: 12px;
      font-weight: 700;
      fill: var(--ink);
    }
    .node-z, .node-dist {
      font-size: 10px;
      fill: var(--muted);
    }
    .node-dist {
      display: none;
      font-family: var(--font-mono);
    }
    .sidebar {
      display: flex;
      flex-direction: column;
      gap: 18px;
    }
    .sidebar .panel {
      padding: 18px;
    }
    .hint {
      color: var(--muted);
      font-size: 13px;
      line-height: 1.55;
    }
    .controls {
      display: grid;
      gap: 12px;
    }
    .button-row {
      display: flex;
      gap: 10px;
      align-items: center;
    }
    button {
      border: 1px solid rgba(29, 39, 48, 0.12);
      background: var(--panel-strong);
      color: var(--ink);
      border-radius: 999px;
      padding: 9px 14px;
      cursor: pointer;
    }
    input[type="range"] {
      width: 100%;
    }
    .step-meta {
      display: flex;
      justify-content: space-between;
      align-items: center;
      color: var(--muted);
      font-size: 13px;
    }
    .details {
      margin: 0;
      white-space: pre-wrap;
      font-family: var(--font-mono);
      font-size: 13px;
      line-height: 1.55;
      background: rgba(15, 23, 29, 0.94);
      color: #edf3f6;
      border-radius: 18px;
      padding: 14px;
      min-height: 164px;
      overflow: auto;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      font-size: 13px;
    }
    th, td {
      text-align: left;
      padding: 7px 6px;
      border-bottom: 1px solid rgba(29, 39, 48, 0.08);
      vertical-align: top;
    }
    th {
      color: var(--muted);
      font-weight: 600;
    }
    .table-sub {
      color: var(--muted);
      font-size: 11px;
      font-family: var(--font-mono);
    }
    .profile {
      background: rgba(255,255,255,0.64);
      border: 1px solid rgba(29, 39, 48, 0.08);
      border-radius: 18px;
      padding: 12px;
    }
    .profile svg {
      width: 100%;
      height: auto;
      overflow: visible;
    }
    .profile polyline {
      fill: none;
      stroke: #e36414;
      stroke-width: 3;
    }
    .profile circle {
      fill: #355070;
    }
    .profile text {
      font-size: 10px;
      text-anchor: middle;
      fill: var(--muted);
    }
    .legend-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
    }
    .legend-item {
      display: flex;
      align-items: center;
      gap: 10px;
      color: var(--muted);
      font-size: 13px;
      margin-top: 8px;
    }
    .legend-dot {
      width: 14px;
      height: 14px;
      border-radius: 999px;
      border: 2px solid rgba(29, 39, 48, 0.22);
      flex: 0 0 auto;
    }
    .legend-line {
      width: 28px;
      height: 0;
      border-top: 4px solid currentColor;
      border-radius: 999px;
      flex: 0 0 auto;
    }
    .status-line {
      font-size: 13px;
      color: var(--muted);
      margin-top: 8px;
    }
    .status-line strong {
      color: var(--ink);
    }
    .danger {
      color: var(--danger);
    }
    @media (max-width: 1360px) {
      .page {
        grid-template-columns: 1fr;
      }
    }
    """

    script = f"""
    const VIEW_DATA = {json.dumps(graph_json, ensure_ascii=False, sort_keys=True)};
    const FRAME_LABELS = {json.dumps(FRAME_TYPE_LABELS, ensure_ascii=False, sort_keys=True)};

    const slider = document.getElementById("step-slider");
    const stepCountEl = document.getElementById("step-count");
    const stepTypeEl = document.getElementById("step-type");
    const detailsEl = document.getElementById("step-details");
    const playButton = document.getElementById("play-btn");
    const candidatePanel = document.getElementById("candidate-panel");
    let timer = null;

    function frameTypeLabel(frameType) {{
      return FRAME_LABELS[frameType] || frameType || "未知步骤";
    }}

    function nodeLabel(nodeId) {{
      if (!nodeId) {{
        return "无";
      }}
      const node = VIEW_DATA.nodes[nodeId];
      if (!node) {{
        return nodeId;
      }}
      return `${{node.full_label}}（${{nodeId}}）`;
    }}

    function edgeLabel(edgeId) {{
      if (!edgeId) {{
        return "无";
      }}
      const edge = VIEW_DATA.edges[edgeId];
      if (!edge) {{
        return edgeId;
      }}
      return `${{edge.full_label}}（${{edgeId}}）`;
    }}

    function frontierLabel(frontier) {{
      const entries = Object.entries(frontier || {{}});
      if (!entries.length) {{
        return "无";
      }}
      return entries
        .map(([nodeId, cost]) => `${{nodeLabel(nodeId)}}: ${{Number(cost).toFixed(2)}}`)
        .join("\\n");
    }}

    function nodeListLabel(nodeIds) {{
      if (!nodeIds || !nodeIds.length) {{
        return "无";
      }}
      return nodeIds.map((nodeId) => nodeLabel(nodeId)).join("、");
    }}

    function formatMaybe(value, suffix = "") {{
      if (value === undefined || value === null || value === "") {{
        return "无";
      }}
      return `${{value}}${{suffix}}`;
    }}

    function setNodeClasses(frame) {{
      const finalNodeSet = new Set(frame.show_final_path ? frame.final_path_nodes : []);
      const closedSet = new Set(frame.closed_nodes);
      const frontierKeys = Object.keys(frame.frontier || {{}});
      const frontierSet = new Set(frontierKeys);
      document.querySelectorAll(".node").forEach((nodeEl) => {{
        const nodeId = nodeEl.dataset.nodeId;
        nodeEl.classList.toggle("closed", closedSet.has(nodeId));
        nodeEl.classList.toggle("frontier", frontierSet.has(nodeId));
        nodeEl.classList.toggle("current", frame.active_node === nodeId);
        nodeEl.classList.toggle("on-final-path", finalNodeSet.has(nodeId));
        nodeEl.classList.toggle("start", VIEW_DATA.planning.start_node === nodeId);
        nodeEl.classList.toggle("goal", VIEW_DATA.planning.goal_kind === "node" && VIEW_DATA.planning.goal_value === nodeId);

        const distEl = document.getElementById(`dist-${{nodeId}}`);
        if (!distEl) return;
        const dist = frame.frontier[nodeId];
        const isClosed = closedSet.has(nodeId);
        const nodeInfo = VIEW_DATA.nodes[nodeId];
        if (typeof dist === "number") {{
          distEl.textContent = `代=${{dist.toFixed(2)}}`;
          distEl.style.display = "block";
        }} else if (isClosed) {{
          distEl.textContent = "已定";
          distEl.style.display = "block";
        }} else if (nodeInfo) {{
          distEl.textContent = "";
          distEl.style.display = "none";
        }}
      }});
    }}

    function setEdgeClasses(frame) {{
      const finalEdgeSet = new Set(frame.show_final_path ? frame.final_path_edges : []);
      document.querySelectorAll(".edge-group").forEach((groupEl) => {{
        const edgeId = groupEl.dataset.edgeId;
        const edgeEl = groupEl.querySelector(".edge");
        edgeEl.classList.toggle("active", frame.active_edge === edgeId);
        edgeEl.classList.toggle("final-path", finalEdgeSet.has(edgeId));
        edgeEl.classList.remove("frontier");
      }});
    }}

    function formatDetails(frame) {{
      return [
        `步骤类型: ${{frameTypeLabel(frame.type)}}`,
        `步骤说明: ${{frame.message || "无"}}`,
        `当前节点: ${{nodeLabel(frame.active_node)}}`,
        `当前边: ${{edgeLabel(frame.active_edge)}}`,
        `尝试前往: ${{nodeLabel(frame.active_to)}}`,
        `已确定节点: ${{nodeListLabel(frame.closed_nodes)}}`,
        `前沿候选:\\n${{frontierLabel(frame.frontier)}}`,
        `本次边代价: ${{formatMaybe(frame.edge_cost)}}`,
        `旧总代价: ${{formatMaybe(frame.old_cost)}}`,
        `新总代价: ${{formatMaybe(frame.new_cost)}}`,
        `当前已知总代价: ${{formatMaybe(frame.total_cost)}}`,
      ].join("\\n");
    }}

    function renderFrame(index) {{
      const frames = VIEW_DATA.planning.frames;
      if (!frames.length) {{
        stepCountEl.textContent = "0 / 0";
        stepTypeEl.textContent = VIEW_DATA.planning.success ? "无需过程回放" : "规划失败";
        detailsEl.textContent = VIEW_DATA.planning.failure_reason || "没有生成可回放的规划步骤。";
        return;
      }}
      const bounded = Math.max(0, Math.min(index, frames.length - 1));
      slider.value = String(bounded);
      const frame = frames[bounded];
      stepCountEl.textContent = `${{bounded + 1}} / ${{frames.length}}`;
      stepTypeEl.textContent = frameTypeLabel(frame.type);
      detailsEl.textContent = formatDetails(frame);
      setNodeClasses(frame);
      setEdgeClasses(frame);
    }}

    function stopPlayback() {{
      if (timer !== null) {{
        window.clearInterval(timer);
        timer = null;
      }}
      playButton.textContent = "播放";
    }}

    function togglePlayback() {{
      const frames = VIEW_DATA.planning.frames;
      if (!frames.length) {{
        return;
      }}
      if (timer !== null) {{
        stopPlayback();
        return;
      }}
      playButton.textContent = "暂停";
      timer = window.setInterval(() => {{
        const next = Number(slider.value) + 1;
        if (next >= frames.length) {{
          stopPlayback();
          return;
        }}
        renderFrame(next);
      }}, 900);
    }}

    slider.addEventListener("input", () => {{
      stopPlayback();
      renderFrame(Number(slider.value));
    }});

    document.getElementById("prev-btn").addEventListener("click", () => {{
      stopPlayback();
      renderFrame(Number(slider.value) - 1);
    }});

    document.getElementById("next-btn").addEventListener("click", () => {{
      stopPlayback();
      renderFrame(Number(slider.value) + 1);
    }});

    playButton.addEventListener("click", togglePlayback);

    if (!VIEW_DATA.planning.candidate_results.length) {{
      candidatePanel.style.display = "none";
    }}

    slider.max = String(Math.max(VIEW_DATA.planning.frames.length - 1, 0));
    renderFrame(0);
    """

    return "\n".join(
        [
            "<!DOCTYPE html>",
            "<html lang=\"zh-CN\">",
            "<head>",
            "  <meta charset=\"utf-8\" />",
            f"  <title>{html.escape(title)}</title>",
            "  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />",
            f"  <style>{style}</style>",
            "</head>",
            "<body>",
            "  <div class=\"page\">",
            "    <section class=\"panel main-panel\">",
            "      <header class=\"header\">",
            f"        <h1>{html.escape(title)}</h1>",
            "        <p>这页把 `rc26_topo_nav` 的拓扑导航图、`RC_Sim_001_github` 的 Gazebo 场地，以及离线规划过程放在同一坐标系里观察。主视图是仿真世界顶视图；路径代价仍然按当前 planner 的真实逻辑计算，所以你能同时看见场地位置、搜索顺序和高度语义。</p>",
            "      </header>",
            "      <section class=\"meta-grid\">",
            f"        <div class=\"meta-card\"><span class=\"label\">当前队伍</span><span class=\"value\">{html.escape(team_display)}</span></div>",
            f"        <div class=\"meta-card\"><span class=\"label\">导航点数量</span><span class=\"value\">{len(document.get('nodes', []))}</span></div>",
            f"        <div class=\"meta-card\"><span class=\"label\">边数量</span><span class=\"value\">{len(document.get('edges', []))}</span></div>",
            f"        <div class=\"meta-card\"><span class=\"label\">当前目标</span><span class=\"value\">{html.escape(goal_kind_label(goal_kind))}</span><span class=\"table-sub\">{html.escape(goal_display)}</span></div>",
            f"        <div class=\"meta-card\"><span class=\"label\">规划结果</span><span class=\"value\">{'成功' if planning.get('success') else '失败'}</span></div>",
            f"        <div class=\"meta-card\"><span class=\"label\">总代价</span><span class=\"value\">{html.escape(str(planning.get('total_cost', '-')))}</span></div>",
            "      </section>",
            alignment_summary,
            world_summary,
            "      <div class=\"canvas\">",
            f"        <svg viewBox=\"0 0 {SVG_WIDTH} {SVG_HEIGHT}\" aria-label=\"拓扑导航与仿真场地叠图\">",
            "          <defs>",
            *[f"            {marker}" for marker in marker_defs],
            "          </defs>",
            "          <rect x=\"16\" y=\"16\" width=\"1248\" height=\"888\" rx=\"26\" fill=\"rgba(255,255,255,0.44)\" />",
            *[f"          {feature}" for feature in feature_elements],
            *[f"          {slot}" for slot in slot_elements],
            *[f"          {edge}" for edge in edge_elements],
            *[f"          {node}" for node in node_elements],
            "        </svg>",
            "      </div>",
            "      <section class=\"panel\" style=\"padding:18px;border-radius:18px;box-shadow:none;\">",
            "        <div class=\"legend-grid\">",
            f"          <div><h3>导航点图例</h3>{''.join(node_legend)}</div>",
            f"          <div><h3>路径边图例</h3>{''.join(edge_legend)}</div>",
            "        </div>",
            "        <p class=\"status-line\"><strong>规划代价权重</strong>: 时间=1.0，高度风险=2.0，确认要求=1.5，慢行惩罚=2.0。这就是当前离线回放使用的真实代价项。</p>",
            "      </section>",
            "    </section>",
            "    <aside class=\"sidebar\">",
            "      <section class=\"panel\">",
            "        <h2>规划过程</h2>",
            f"        <p class=\"hint\">起点: <strong>{html.escape(with_raw_label(full_node_label(str(planning.get('start_node', ''))), str(planning.get('start_node', ''))))}</strong> · 目标: <strong>{html.escape(goal_display)}</strong></p>",
            (
                f"<p class=\"status-line\">最终选中的候选点: <strong>{html.escape(with_raw_label(full_node_label(str(planning.get('selected_candidate'))), str(planning.get('selected_candidate'))))}</strong></p>"
                if planning.get("selected_candidate")
                else ""
            ),
            (
                f"<p class=\"status-line danger\">失败原因: {html.escape(str(planning.get('failure_reason', '')))}</p>"
                if not planning.get("success")
                else ""
            ),
            "        <div class=\"controls\">",
            "          <div class=\"button-row\">",
            "            <button id=\"prev-btn\" type=\"button\">上一步</button>",
            "            <button id=\"play-btn\" type=\"button\">播放</button>",
            "            <button id=\"next-btn\" type=\"button\">下一步</button>",
            "          </div>",
            "          <input id=\"step-slider\" type=\"range\" min=\"0\" max=\"0\" value=\"0\" />",
            "          <div class=\"step-meta\"><span id=\"step-count\">0 / 0</span><span id=\"step-type\">初始化前沿</span></div>",
            "        </div>",
            "        <pre id=\"step-details\" class=\"details\"></pre>",
            "      </section>",
            "      <section id=\"candidate-panel\" class=\"panel\">",
            "        <h2>任务候选点</h2>",
            "        <p class=\"hint\">如果目标是任务，这里会列出每个候选点的离线求解结果，再显示最终胜出的路径回放。</p>",
            "        <table>",
            "          <thead><tr><th>候选点</th><th>状态</th><th>总代价</th><th>备注</th></tr></thead>",
            f"          <tbody>{''.join(task_result_rows)}</tbody>",
            "        </table>",
            "      </section>",
            "      <section class=\"panel\">",
            "        <h2>三维高度语义</h2>",
            "        <p class=\"hint\">顶视图负责看地图和搜索顺序，这里单独看最终路径上的高度语义变化。折线使用 graph 节点里的 `z` 值，不是 Gazebo 相机视角，所以更接近 planner 真正感知到的三维风险。</p>",
            "        <div class=\"profile\">",
            f"          <svg viewBox=\"0 0 {profile['width']} {profile['height']}\">",
            f"            <polyline points=\"{escape_attr(profile['polyline'])}\"></polyline>",
            *[f"            {point}" for point in profile_points],
            "          </svg>",
            "        </div>",
            "        <p class=\"hint\">表格说明: 高度变化 dZ 表示这一段路径从起点到终点的语义高度差；本段代价表示这一条边单独贡献的代价；累计代价表示从起点走到当前边结束位置时的总代价。</p>",
            "        <table>",
            "          <thead><tr><th>路径边段</th><th>运动语义</th><th>高度变化 dZ</th><th>本段代价</th><th>累计代价</th></tr></thead>",
            f"          <tbody>{''.join(segment_rows)}</tbody>",
            "        </table>",
            "      </section>",
            "      <section class=\"panel\">",
            "        <h2>叠加状态</h2>",
            f"        <p class=\"hint\">被阻塞的节点: {html.escape(format_node_refs(sorted(node_id for node_id, state in ((k, v['state']) for k, v in overlay_state['node_overlays'].items()) if state == 'BLOCKED')))}</p>",
            f"        <p class=\"hint\">被阻塞的边: {html.escape(format_edge_refs(sorted(edge_id for edge_id, state in ((k, v['state']) for k, v in overlay_state['edge_overlays'].items()) if state == 'BLOCKED')))}</p>",
            f"        <p class=\"hint\">慢行边: {html.escape(format_edge_refs(sorted(edge_id for edge_id, state in ((k, v['state']) for k, v in overlay_state['edge_overlays'].items()) if state == 'SLOW_ONLY')))}</p>",
            f"        <p class=\"hint\">需要确认的边: {html.escape(format_edge_refs(sorted(edge_id for edge_id, state in ((k, v['state']) for k, v in overlay_state['edge_overlays'].items()) if state == 'CONFIRM_REQUIRED')))}</p>",
            "      </section>",
            "    </aside>",
            "  </div>",
            f"  <script>{script}</script>",
            "</body>",
            "</html>",
        ]
    )


def write_html(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render rc26_topo_nav graph, RC_Sim world context, and offline planner trace into one HTML page."
    )
    parser.add_argument("--graph", type=Path, required=True, help="Input topo graph YAML")
    parser.add_argument("--out", type=Path, required=True, help="Output HTML path")
    parser.add_argument("--world", type=Path, default=None, help="Gazebo world file from RC_Sim_001_github")
    parser.add_argument("--kfs-config", type=Path, default=None, help="KFS config with meilin coordinates")
    parser.add_argument("--model-root", type=Path, default=None, help="Optional model root for resolving model://robocon2026_world")
    parser.add_argument("--start", type=str, default=None, help="Planner start node")
    goal_group = parser.add_mutually_exclusive_group()
    goal_group.add_argument("--goal-node", type=str, default=None, help="Planner goal node")
    goal_group.add_argument("--goal-task", type=str, default=None, help="Planner goal task")
    goal_group.add_argument("--goal-route", type=str, default=None, help="Planner goal route tag")
    parser.add_argument("--blocked-node", action="append", default=[], help="Repeatable blocked node id")
    parser.add_argument("--blocked-edge", action="append", default=[], help="Repeatable blocked edge id")
    parser.add_argument("--slow-edge", action="append", default=[], help="Repeatable slow-only edge id")
    parser.add_argument("--confirm-edge", action="append", default=[], help="Repeatable confirm-required edge id")
    parser.add_argument("--edge-extra-cost", action="append", default=[], help="Repeatable EDGE_ID=COST extra cost")
    parser.add_argument("--title", type=str, default=None, help="Optional page title")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    document = load_yaml(args.graph)
    team = str(document.get("meta", {}).get("team", "blue")).lower()

    sim_config = load_yaml(args.kfs_config) if args.kfs_config else {}
    alignment = derive_graph_alignment(document, sim_config, team) if sim_config else None
    meilin_slots = build_meilin_slots(sim_config, team) if sim_config else []

    world_context = None
    scene_features: list[dict[str, Any]] = []
    if args.world:
        model_root = args.model_root or (args.world.parent.parent / "models")
        world_context = parse_world_context(args.world, model_root)
        scene_features = world_context["scene_features"]

    overlay_state = build_overlay_state(args, document)
    planning = run_planning(document, args, overlay_state)
    render_model = build_render_model(
        document,
        alignment=alignment,
        scene_features=scene_features,
        meilin_slots=meilin_slots,
        planning=planning,
    )
    html_document = render_html(
        args.graph,
        document,
        render_model,
        planning,
        overlay_state,
        alignment,
        world_context,
        args,
    )
    write_html(args.out, html_document)
    print(f"[OK] Wrote topo + sim trace view: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
