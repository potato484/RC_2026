#!/usr/bin/env python3
"""Render an rc26_topo_nav graph YAML into a self-contained HTML page."""

from __future__ import annotations

import argparse
import html
import json
import math
from pathlib import Path
from typing import Any

import yaml


SVG_WIDTH = 1080
SVG_HEIGHT = 860
SVG_PADDING = 96
EDGE_OFFSET_PX = 10.0

NODE_TYPE_COLORS = {
    "mf_edge_pose": "#f2994a",
    "staging": "#2d9cdb",
    "ramp_entry": "#27ae60",
    "ramp_exit": "#eb5757",
}

EDGE_MOTION_COLORS = {
    "plane_move": "#355070",
    "ramp_up": "#3a7d44",
    "ramp_down": "#bc4749",
}


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


def slugify(value: str) -> str:
    return "".join(ch if ch.isalnum() else "-" for ch in value).strip("-").lower() or "item"


def escape_attr(value: Any) -> str:
    return html.escape(str(value), quote=True)


def graph_title(document: dict[str, Any], source_name: str, explicit_title: str | None) -> str:
    if explicit_title:
        return explicit_title
    meta = document.get("meta", {})
    team = str(meta.get("team", "unknown")).upper()
    return f"RC26 Topo Graph Viewer · {team} · {source_name}"


def compute_route_memberships(
    routes: list[dict[str, Any]],
    edge_lookup: dict[tuple[str, str], str],
) -> tuple[dict[str, list[str]], dict[str, list[str]], list[str]]:
    node_routes: dict[str, list[str]] = {}
    edge_routes: dict[str, list[str]] = {}
    warnings: list[str] = []

    for route in routes:
        tag = str(route.get("route_tag", ""))
        nodes = [str(node_id) for node_id in route.get("nodes", [])]
        for node_id in nodes:
            node_routes.setdefault(node_id, []).append(tag)
        for left, right in zip(nodes, nodes[1:]):
            edge_id = edge_lookup.get((left, right))
            if edge_id is None:
                warnings.append(f"Route '{tag}' has no direct edge for {left} -> {right}")
                continue
            edge_routes.setdefault(edge_id, []).append(tag)

    return node_routes, edge_routes, warnings


def compute_task_memberships(tasks: list[dict[str, Any]]) -> dict[str, list[str]]:
    memberships: dict[str, list[str]] = {}
    for task in tasks:
        tag = str(task.get("task_tag", ""))
        for node_id in task.get("candidate_nodes", []):
            memberships.setdefault(str(node_id), []).append(tag)
    return memberships


def coordinate_mapper(document: dict[str, Any]):
    points: list[tuple[float, float]] = []
    for node in document.get("nodes", []):
        pose = node.get("pose", {})
        points.append((float(pose.get("x", 0.0)), float(pose.get("y", 0.0))))
    for edge in document.get("edges", []):
        for point in edge.get("control_points", []):
            points.append((float(point.get("x", 0.0)), float(point.get("y", 0.0))))

    if not points:
        raise ValueError("Graph contains no points to render")

    min_x = min(point[0] for point in points)
    max_x = max(point[0] for point in points)
    min_y = min(point[1] for point in points)
    max_y = max(point[1] for point in points)

    span_x = max(max_x - min_x, 1e-6)
    span_y = max(max_y - min_y, 1e-6)
    scale = min(
        (SVG_WIDTH - 2 * SVG_PADDING) / span_x,
        (SVG_HEIGHT - 2 * SVG_PADDING) / span_y,
    )

    def map_point(point: dict[str, Any]) -> tuple[float, float]:
        x = SVG_PADDING + (float(point["x"]) - min_x) * scale
        y = SVG_HEIGHT - SVG_PADDING - (float(point["y"]) - min_y) * scale
        return (round_float(x, 2), round_float(y, 2))

    return map_point


def edge_offset_signs(edges: list[dict[str, Any]]) -> dict[str, float]:
    signs: dict[str, float] = {}
    pair_members: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for edge in edges:
        pair_key = tuple(sorted((str(edge["from"]), str(edge["to"]))))
        pair_members.setdefault(pair_key, []).append(edge)

    for pair_key, members in pair_members.items():
        if len(members) < 2:
            for edge in members:
                signs[str(edge["id"])] = 0.0
            continue
        low, high = pair_key
        for edge in members:
            direction = (str(edge["from"]), str(edge["to"]))
            if direction == (low, high):
                signs[str(edge["id"])] = 1.0
            elif direction == (high, low):
                signs[str(edge["id"])] = -1.0
            else:
                signs[str(edge["id"])] = 0.0
    return signs


def offset_screen_points(points: list[tuple[float, float]], offset_px: float) -> list[tuple[float, float]]:
    if abs(offset_px) < 1e-6 or len(points) < 2:
        return points

    dx = points[-1][0] - points[0][0]
    dy = points[-1][1] - points[0][1]
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return points

    nx = -dy / length
    ny = dx / length
    return [
        (round_float(point[0] + nx * offset_px, 2), round_float(point[1] + ny * offset_px, 2))
        for point in points
    ]


def points_to_path(points: list[tuple[float, float]]) -> str:
    if not points:
        return ""
    commands = [f"M {points[0][0]} {points[0][1]}"]
    commands.extend(f"L {point[0]} {point[1]}" for point in points[1:])
    return " ".join(commands)


def midpoint(points: list[tuple[float, float]]) -> tuple[float, float]:
    if not points:
        return (0.0, 0.0)
    if len(points) == 1:
        return points[0]

    segment_lengths = [
        math.hypot(right[0] - left[0], right[1] - left[1])
        for left, right in zip(points, points[1:])
    ]
    total_length = sum(segment_lengths)
    if total_length < 1e-6:
        return points[len(points) // 2]

    target = total_length / 2.0
    traveled = 0.0
    for index, segment_length in enumerate(segment_lengths):
        left = points[index]
        right = points[index + 1]
        if traveled + segment_length >= target:
            ratio = (target - traveled) / segment_length
            x = left[0] + (right[0] - left[0]) * ratio
            y = left[1] + (right[1] - left[1]) * ratio
            return (round_float(x, 2), round_float(y, 2))
        traveled += segment_length
    return points[-1]


def node_color(node_type: str) -> str:
    return NODE_TYPE_COLORS.get(node_type, "#9b5de5")


def edge_color(motion_type: str) -> str:
    return EDGE_MOTION_COLORS.get(motion_type, "#6c757d")


def build_render_model(document: dict[str, Any], source_name: str) -> dict[str, Any]:
    nodes = list(document.get("nodes", []))
    edges = list(document.get("edges", []))
    tasks = list(document.get("tasks", []))
    routes = list(document.get("routes", []))
    node_index = {str(node["id"]): node for node in nodes}
    edge_lookup = {(str(edge["from"]), str(edge["to"])): str(edge["id"]) for edge in edges}
    node_tasks = compute_task_memberships(tasks)
    node_routes, edge_routes, warnings = compute_route_memberships(routes, edge_lookup)
    mapper = coordinate_mapper(document)
    offset_signs = edge_offset_signs(edges)

    rendered_nodes = []
    for node in nodes:
        node_id = str(node["id"])
        pose = node.get("pose", {})
        cx, cy = mapper(pose)
        rendered_nodes.append(
            {
                "id": node_id,
                "type": str(node.get("type", "")),
                "type_slug": slugify(str(node.get("type", ""))),
                "cx": cx,
                "cy": cy,
                "z": round_float(pose.get("z", 0.0)),
                "base_cost": round_float(node.get("base_cost", 0.0)),
                "color": node_color(str(node.get("type", ""))),
                "routes": node_routes.get(node_id, []),
                "tasks": node_tasks.get(node_id, []),
                "title": (
                    f"{node_id}\n"
                    f"type: {node.get('type', '')}\n"
                    f"xyz: ({round_float(pose.get('x', 0.0))}, {round_float(pose.get('y', 0.0))}, {round_float(pose.get('z', 0.0))})\n"
                    f"yaw: {round_float(pose.get('yaw', 0.0))}\n"
                    f"base_cost: {round_float(node.get('base_cost', 0.0))}\n"
                    f"operation_tag: {node.get('operation_tag', '')}"
                ),
            }
        )

    rendered_edges = []
    for edge in edges:
        edge_id = str(edge["id"])
        from_node = node_index[str(edge["from"])]
        to_node = node_index[str(edge["to"])]
        points = [from_node["pose"], *edge.get("control_points", []), to_node["pose"]]
        screen_points = [mapper(point) for point in points]
        shifted_points = offset_screen_points(screen_points, offset_signs.get(edge_id, 0.0) * EDGE_OFFSET_PX)
        rendered_edges.append(
            {
                "id": edge_id,
                "from": str(edge["from"]),
                "to": str(edge["to"]),
                "motion_type": str(edge.get("motion_type", "")),
                "motion_slug": slugify(str(edge.get("motion_type", ""))),
                "color": edge_color(str(edge.get("motion_type", ""))),
                "path": points_to_path(shifted_points),
                "midpoint": midpoint(shifted_points),
                "routes": edge_routes.get(edge_id, []),
                "requires_confirmation": bool(edge.get("requires_confirmation", False)),
                "can_block": bool(edge.get("can_block", False)),
                "title": (
                    f"{edge_id}\n"
                    f"{edge.get('from', '')} -> {edge.get('to', '')}\n"
                    f"motion_type: {edge.get('motion_type', '')}\n"
                    f"required_mode: {edge.get('required_mode', '')}\n"
                    f"height_change: {round_float(edge.get('height_change', 0.0))}\n"
                    f"base_cost: {round_float(edge.get('base_cost', 0.0))}\n"
                    f"requires_confirmation: {bool(edge.get('requires_confirmation', False))}\n"
                    f"can_block: {bool(edge.get('can_block', False))}"
                ),
            }
        )

    route_details = []
    for route in routes:
        route_nodes = [str(node_id) for node_id in route.get("nodes", [])]
        route_edges = [
            edge_lookup[(left, right)]
            for left, right in zip(route_nodes, route_nodes[1:])
            if (left, right) in edge_lookup
        ]
        route_details.append(
            {
                "route_tag": str(route.get("route_tag", "")),
                "nodes": route_nodes,
                "edges": route_edges,
            }
        )

    task_details = []
    for task in tasks:
        task_details.append(
            {
                "task_tag": str(task.get("task_tag", "")),
                "candidate_nodes": [str(node_id) for node_id in task.get("candidate_nodes", [])],
                "selection_policy": str(task.get("selection_policy", "")),
            }
        )

    view_data = {
        "meta": document.get("meta", {}),
        "source_name": source_name,
        "warnings": warnings,
        "nodes": [
            {
                "id": str(node["id"]),
                "type": str(node.get("type", "")),
                "pose": node.get("pose", {}),
                "level": node.get("level"),
                "phase_mask": node.get("phase_mask"),
                "block_id": node.get("block_id"),
                "base_cost": node.get("base_cost"),
                "operation_tag": node.get("operation_tag"),
                "routes": node_routes.get(str(node["id"]), []),
                "tasks": node_tasks.get(str(node["id"]), []),
            }
            for node in nodes
        ],
        "edges": [
            {
                "id": str(edge["id"]),
                "from": str(edge["from"]),
                "to": str(edge["to"]),
                "motion_type": str(edge.get("motion_type", "")),
                "required_mode": edge.get("required_mode"),
                "height_change": edge.get("height_change"),
                "base_cost": edge.get("base_cost"),
                "phase_mask": edge.get("phase_mask"),
                "requires_confirmation": edge.get("requires_confirmation"),
                "can_block": edge.get("can_block"),
                "control_points": edge.get("control_points", []),
                "routes": edge_routes.get(str(edge["id"]), []),
            }
            for edge in edges
        ],
        "tasks": task_details,
        "routes": route_details,
    }

    return {
        "rendered_nodes": rendered_nodes,
        "rendered_edges": rendered_edges,
        "view_data": view_data,
        "warnings": warnings,
    }


def render_html_document(document: dict[str, Any], source_name: str, explicit_title: str | None = None) -> str:
    model = build_render_model(document, source_name)
    title = graph_title(document, source_name, explicit_title)
    view_data = model["view_data"]

    marker_defs = []
    for motion_type in sorted({edge["motion_type"] for edge in model["rendered_edges"]}):
        marker_defs.append(
            (
                f"<marker id=\"arrow-{escape_attr(slugify(motion_type))}\" "
                "viewBox=\"0 0 12 12\" refX=\"10\" refY=\"6\" markerWidth=\"7\" markerHeight=\"7\" orient=\"auto\">"
                f"<path d=\"M 0 1 L 10 6 L 0 11 z\" fill=\"{escape_attr(edge_color(motion_type))}\" />"
                "</marker>"
            )
        )

    grid_x = sorted({round_float(node["pose"]["x"], 3) for node in document.get("nodes", [])})
    grid_y = sorted({round_float(node["pose"]["y"], 3) for node in document.get("nodes", [])})
    mapper = coordinate_mapper(document)

    grid_lines = []
    for x_value in grid_x:
        top = mapper({"x": x_value, "y": min(grid_y), "z": 0.0, "yaw": 0.0})
        bottom = mapper({"x": x_value, "y": max(grid_y), "z": 0.0, "yaw": 0.0})
        grid_lines.append(
            f"<line class=\"grid-line\" x1=\"{top[0]}\" y1=\"{top[1]}\" x2=\"{bottom[0]}\" y2=\"{bottom[1]}\" />"
        )
    for y_value in grid_y:
        left = mapper({"x": min(grid_x), "y": y_value, "z": 0.0, "yaw": 0.0})
        right = mapper({"x": max(grid_x), "y": y_value, "z": 0.0, "yaw": 0.0})
        grid_lines.append(
            f"<line class=\"grid-line\" x1=\"{left[0]}\" y1=\"{left[1]}\" x2=\"{right[0]}\" y2=\"{right[1]}\" />"
        )

    edge_elements = []
    for edge in model["rendered_edges"]:
        edge_elements.append(
            (
                f"<g class=\"edge-group\" data-edge-id=\"{escape_attr(edge['id'])}\" "
                f"data-routes=\"{escape_attr(','.join(edge['routes']))}\">"
                f"<path class=\"edge edge-{escape_attr(edge['motion_slug'])}\" "
                f"id=\"edge-{escape_attr(edge['id'])}\" d=\"{escape_attr(edge['path'])}\" "
                f"stroke=\"{escape_attr(edge['color'])}\" "
                f"marker-end=\"url(#arrow-{escape_attr(edge['motion_slug'])})\" "
                f"data-confirm=\"{str(edge['requires_confirmation']).lower()}\" "
                f"data-block=\"{str(edge['can_block']).lower()}\">"
                f"<title>{html.escape(edge['title'])}</title>"
                "</path>"
                f"<path class=\"edge-hit\" d=\"{escape_attr(edge['path'])}\" data-edge-id=\"{escape_attr(edge['id'])}\" />"
                "</g>"
            )
        )

    node_elements = []
    for node in model["rendered_nodes"]:
        task_labels = ", ".join(node["tasks"]) or "none"
        route_labels = ", ".join(node["routes"]) or "none"
        node_elements.append(
            (
                f"<g class=\"node node-{escape_attr(node['type_slug'])}\" "
                f"id=\"node-{escape_attr(node['id'])}\" "
                f"data-node-id=\"{escape_attr(node['id'])}\" "
                f"data-routes=\"{escape_attr(','.join(node['routes']))}\" "
                f"data-tasks=\"{escape_attr(','.join(node['tasks']))}\">"
                f"<circle class=\"node-core\" cx=\"{node['cx']}\" cy=\"{node['cy']}\" r=\"10\" fill=\"{escape_attr(node['color'])}\">"
                f"<title>{html.escape(node['title'])}</title>"
                "</circle>"
                f"<circle class=\"node-hit\" cx=\"{node['cx']}\" cy=\"{node['cy']}\" r=\"20\" data-node-id=\"{escape_attr(node['id'])}\" />"
                f"<text class=\"node-label\" x=\"{node['cx']}\" y=\"{round_float(node['cy'] - 18, 2)}\">{html.escape(node['id'])}</text>"
                f"<text class=\"node-subtitle\" x=\"{node['cx']}\" y=\"{round_float(node['cy'] + 26, 2)}\">"
                f"z={escape_attr(node['z'])} cost={escape_attr(node['base_cost'])}"
                "</text>"
                f"<text class=\"node-membership\" x=\"{node['cx']}\" y=\"{round_float(node['cy'] + 42, 2)}\">"
                f"T:{html.escape(task_labels)} | R:{html.escape(route_labels)}"
                "</text>"
                "</g>"
            )
        )

    route_buttons = []
    for route in view_data["routes"]:
        route_buttons.append(
            (
                f"<button class=\"chip route-chip\" type=\"button\" data-route-tag=\"{escape_attr(route['route_tag'])}\">"
                f"<span>{html.escape(route['route_tag'])}</span>"
                f"<span class=\"chip-meta\">{len(route['nodes'])} nodes</span>"
                "</button>"
            )
        )

    task_buttons = []
    for task in view_data["tasks"]:
        task_buttons.append(
            (
                f"<button class=\"chip task-chip\" type=\"button\" data-task-tag=\"{escape_attr(task['task_tag'])}\">"
                f"<span>{html.escape(task['task_tag'])}</span>"
                f"<span class=\"chip-meta\">{len(task['candidate_nodes'])} candidates</span>"
                "</button>"
            )
        )

    warnings_html = ""
    if model["warnings"]:
        warnings_html = (
            "<section class=\"panel\">"
            "<h2>Warnings</h2>"
            "<ul class=\"warning-list\">"
            + "".join(f"<li>{html.escape(warning)}</li>" for warning in model["warnings"])
            + "</ul>"
            "</section>"
        )

    style = """
    :root {
      --bg-top: #f8f4ec;
      --bg-bottom: #e8efe2;
      --panel: rgba(255, 253, 247, 0.85);
      --panel-strong: rgba(255, 252, 242, 0.95);
      --ink: #182126;
      --muted: #61707d;
      --grid: rgba(77, 99, 112, 0.14);
      --card-border: rgba(24, 33, 38, 0.12);
      --shadow: 0 18px 40px rgba(44, 62, 80, 0.14);
      --route: #d97706;
      --task: #0f766e;
      --font-sans: "IBM Plex Sans", "Noto Sans SC", "PingFang SC", sans-serif;
      --font-mono: "IBM Plex Mono", "JetBrains Mono", monospace;
    }
    * {
      box-sizing: border-box;
    }
    body {
      margin: 0;
      color: var(--ink);
      font-family: var(--font-sans);
      background:
        radial-gradient(circle at top left, rgba(255, 240, 214, 0.75), transparent 32%),
        radial-gradient(circle at bottom right, rgba(168, 218, 220, 0.45), transparent 26%),
        linear-gradient(180deg, var(--bg-top), var(--bg-bottom));
    }
    .page {
      min-height: 100vh;
      padding: 28px;
      display: grid;
      grid-template-columns: minmax(720px, 1fr) 360px;
      gap: 24px;
    }
    .panel {
      background: var(--panel);
      backdrop-filter: blur(10px);
      border: 1px solid var(--card-border);
      border-radius: 22px;
      box-shadow: var(--shadow);
    }
    .main-panel {
      padding: 22px;
      display: flex;
      flex-direction: column;
      gap: 18px;
    }
    .header {
      display: flex;
      justify-content: space-between;
      gap: 16px;
      align-items: flex-start;
    }
    .header h1 {
      margin: 0;
      font-size: 28px;
      line-height: 1.2;
    }
    .header p {
      margin: 10px 0 0;
      color: var(--muted);
      max-width: 52rem;
    }
    .meta-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(120px, 1fr));
      gap: 12px;
    }
    .meta-card {
      background: rgba(255, 255, 255, 0.58);
      border: 1px solid rgba(24, 33, 38, 0.08);
      border-radius: 18px;
      padding: 12px 14px;
    }
    .meta-card .label {
      display: block;
      font-size: 12px;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      color: var(--muted);
    }
    .meta-card .value {
      display: block;
      margin-top: 6px;
      font-size: 22px;
      font-weight: 700;
    }
    .canvas {
      position: relative;
      border-radius: 20px;
      overflow: hidden;
      background:
        linear-gradient(180deg, rgba(255, 255, 255, 0.74), rgba(248, 245, 236, 0.92)),
        linear-gradient(135deg, rgba(210, 228, 236, 0.3), rgba(255, 221, 187, 0.18));
      border: 1px solid rgba(24, 33, 38, 0.08);
      min-height: 760px;
    }
    .canvas svg {
      display: block;
      width: 100%;
      height: auto;
    }
    .legend {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
      gap: 12px;
    }
    .legend-card {
      background: rgba(255, 255, 255, 0.6);
      border: 1px solid rgba(24, 33, 38, 0.08);
      border-radius: 16px;
      padding: 12px 14px;
    }
    .legend-card h2,
    .sidebar h2 {
      margin: 0 0 10px;
      font-size: 16px;
    }
    .legend-item {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      color: var(--muted);
      font-size: 13px;
      margin-top: 8px;
    }
    .legend-swatch {
      width: 18px;
      height: 18px;
      border-radius: 999px;
      border: 2px solid rgba(24, 33, 38, 0.2);
      flex: 0 0 auto;
    }
    .legend-line {
      width: 30px;
      height: 0;
      border-top: 4px solid currentColor;
      border-radius: 999px;
      flex: 0 0 auto;
    }
    .sidebar {
      display: flex;
      flex-direction: column;
      gap: 18px;
    }
    .sidebar .panel {
      padding: 18px;
    }
    .chip-list {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
    }
    .chip {
      border: 1px solid rgba(24, 33, 38, 0.12);
      background: var(--panel-strong);
      border-radius: 999px;
      color: var(--ink);
      padding: 10px 14px;
      cursor: pointer;
      display: inline-flex;
      align-items: center;
      gap: 8px;
      transition: transform 120ms ease, box-shadow 120ms ease, border-color 120ms ease;
    }
    .chip:hover {
      transform: translateY(-1px);
      box-shadow: 0 10px 18px rgba(24, 33, 38, 0.08);
    }
    .chip.active.route-chip {
      border-color: rgba(217, 119, 6, 0.48);
      box-shadow: 0 0 0 3px rgba(217, 119, 6, 0.15);
    }
    .chip.active.task-chip {
      border-color: rgba(15, 118, 110, 0.48);
      box-shadow: 0 0 0 3px rgba(15, 118, 110, 0.15);
    }
    .chip-meta {
      color: var(--muted);
      font-size: 12px;
    }
    .hint {
      margin: 0;
      color: var(--muted);
      font-size: 13px;
      line-height: 1.55;
    }
    .details {
      margin: 0;
      background: rgba(20, 29, 34, 0.94);
      color: #eff4f7;
      border-radius: 18px;
      padding: 16px;
      font-size: 13px;
      line-height: 1.55;
      font-family: var(--font-mono);
      white-space: pre-wrap;
      overflow: auto;
      min-height: 200px;
    }
    .warning-list {
      padding-left: 20px;
      margin: 0;
      color: #92400e;
    }
    .grid-line {
      stroke: var(--grid);
      stroke-width: 1;
      stroke-dasharray: 6 8;
    }
    .edge {
      fill: none;
      stroke-width: 3.5;
      opacity: 0.72;
      transition: opacity 120ms ease, stroke-width 120ms ease, filter 120ms ease;
    }
    .edge[data-confirm="true"] {
      stroke-dasharray: 9 7;
    }
    .edge[data-block="false"] {
      opacity: 0.58;
    }
    .edge.is-muted {
      opacity: 0.12;
    }
    .edge.is-route-active {
      opacity: 1;
      stroke-width: 6.5;
      filter: drop-shadow(0 0 10px rgba(217, 119, 6, 0.32));
    }
    .edge-hit {
      fill: none;
      stroke: transparent;
      stroke-width: 16;
      cursor: pointer;
    }
    .node {
      transition: opacity 120ms ease, transform 120ms ease, filter 120ms ease;
    }
    .node.is-muted {
      opacity: 0.16;
    }
    .node.is-task-active .node-core {
      stroke-width: 4;
      filter: drop-shadow(0 0 10px rgba(15, 118, 110, 0.45));
    }
    .node.is-route-active .node-core {
      stroke-width: 4;
      filter: drop-shadow(0 0 12px rgba(217, 119, 6, 0.4));
    }
    .node-core {
      stroke: rgba(24, 33, 38, 0.72);
      stroke-width: 2.5;
    }
    .node-hit {
      fill: transparent;
      cursor: pointer;
    }
    .node-label,
    .node-subtitle,
    .node-membership {
      fill: var(--ink);
      text-anchor: middle;
      pointer-events: none;
    }
    .node-label {
      font-size: 12px;
      font-weight: 700;
    }
    .node-subtitle,
    .node-membership {
      font-size: 10px;
      fill: var(--muted);
    }
    .footer-note {
      margin: 0;
      color: var(--muted);
      font-size: 13px;
      text-align: right;
    }
    @media (max-width: 1280px) {
      .page {
        grid-template-columns: 1fr;
      }
      .canvas {
        min-height: 640px;
      }
    }
    """

    script = f"""
    const GRAPH_DATA = {json.dumps(view_data, ensure_ascii=False, sort_keys=True)};

    const state = {{
      routeTag: null,
      taskTag: null,
      selection: null,
    }};

    const detailsEl = document.getElementById("details");
    const routeButtons = Array.from(document.querySelectorAll("[data-route-tag]"));
    const taskButtons = Array.from(document.querySelectorAll("[data-task-tag]"));
    const nodeEls = Array.from(document.querySelectorAll(".node"));
    const edgeEls = Array.from(document.querySelectorAll(".edge"));

    const nodeById = Object.fromEntries(GRAPH_DATA.nodes.map((node) => [node.id, node]));
    const edgeById = Object.fromEntries(GRAPH_DATA.edges.map((edge) => [edge.id, edge]));
    const routeByTag = Object.fromEntries(GRAPH_DATA.routes.map((route) => [route.route_tag, route]));
    const taskByTag = Object.fromEntries(GRAPH_DATA.tasks.map((task) => [task.task_tag, task]));

    function formatDetails(label, data) {{
      return `${{label}}\\n${{JSON.stringify(data, null, 2)}}`;
    }}

    function setSelection(kind, value) {{
      state.selection = {{ kind, value }};
      if (kind === "node") {{
        detailsEl.textContent = formatDetails(`Node · ${{value}}`, nodeById[value]);
      }} else {{
        detailsEl.textContent = formatDetails(`Edge · ${{value}}`, edgeById[value]);
      }}
    }}

    function updateHighlights() {{
      routeButtons.forEach((button) => {{
        button.classList.toggle("active", state.routeTag === button.dataset.routeTag);
      }});
      taskButtons.forEach((button) => {{
        button.classList.toggle("active", state.taskTag === button.dataset.taskTag);
      }});

      nodeEls.forEach((nodeEl) => {{
        const routeTags = (nodeEl.dataset.routes || "").split(",").filter(Boolean);
        const taskTags = (nodeEl.dataset.tasks || "").split(",").filter(Boolean);
        const routeMatch = !state.routeTag || routeTags.includes(state.routeTag);
        const taskMatch = !state.taskTag || taskTags.includes(state.taskTag);
        nodeEl.classList.toggle("is-route-active", !!state.routeTag && routeMatch);
        nodeEl.classList.toggle("is-task-active", !!state.taskTag && taskMatch);
        nodeEl.classList.toggle("is-muted", !!state.routeTag && !routeMatch);
      }});

      edgeEls.forEach((edgeEl) => {{
        const routeTags = (edgeEl.parentElement.dataset.routes || "").split(",").filter(Boolean);
        const routeMatch = !state.routeTag || routeTags.includes(state.routeTag);
        edgeEl.classList.toggle("is-route-active", !!state.routeTag && routeMatch);
        edgeEl.classList.toggle("is-muted", !!state.routeTag && !routeMatch);
      }});
    }}

    function toggleRoute(routeTag) {{
      state.routeTag = state.routeTag === routeTag ? null : routeTag;
      if (state.routeTag) {{
        detailsEl.textContent = formatDetails(`Route · ${{state.routeTag}}`, routeByTag[state.routeTag]);
      }}
      updateHighlights();
    }}

    function toggleTask(taskTag) {{
      state.taskTag = state.taskTag === taskTag ? null : taskTag;
      if (state.taskTag) {{
        detailsEl.textContent = formatDetails(`Task · ${{state.taskTag}}`, taskByTag[state.taskTag]);
      }}
      updateHighlights();
    }}

    routeButtons.forEach((button) => {{
      button.addEventListener("click", () => toggleRoute(button.dataset.routeTag));
    }});

    taskButtons.forEach((button) => {{
      button.addEventListener("click", () => toggleTask(button.dataset.taskTag));
    }});

    document.querySelectorAll(".node-hit").forEach((target) => {{
      target.addEventListener("click", () => setSelection("node", target.dataset.nodeId));
    }});

    document.querySelectorAll(".edge-hit").forEach((target) => {{
      target.addEventListener("click", () => setSelection("edge", target.dataset.edgeId));
    }});

    detailsEl.textContent = formatDetails("Graph", {{
      source_name: GRAPH_DATA.source_name,
      meta: GRAPH_DATA.meta,
      warnings: GRAPH_DATA.warnings,
      node_count: GRAPH_DATA.nodes.length,
      edge_count: GRAPH_DATA.edges.length,
      task_count: GRAPH_DATA.tasks.length,
      route_count: GRAPH_DATA.routes.length,
    }});
    updateHighlights();
    """

    meta = document.get("meta", {})
    html_parts = [
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
        "        <div>",
        f"          <h1>{html.escape(title)}</h1>",
        "          <p>这是对 `graph_file` 的静态离线渲染，不依赖 ROS 运行态。边颜色表示运动语义，节点颜色表示点类型；点击侧栏 route/task 或图上的节点、边，可以直接检查 topo 算法当前吃到的图结构。</p>",
        "        </div>",
        "      </header>",
        "      <section class=\"meta-grid\">",
        f"        <div class=\"meta-card\"><span class=\"label\">Team</span><span class=\"value\">{html.escape(str(meta.get('team', 'unknown')).upper())}</span></div>",
        f"        <div class=\"meta-card\"><span class=\"label\">Nodes</span><span class=\"value\">{len(document.get('nodes', []))}</span></div>",
        f"        <div class=\"meta-card\"><span class=\"label\">Edges</span><span class=\"value\">{len(document.get('edges', []))}</span></div>",
        f"        <div class=\"meta-card\"><span class=\"label\">Tasks</span><span class=\"value\">{len(document.get('tasks', []))}</span></div>",
        f"        <div class=\"meta-card\"><span class=\"label\">Routes</span><span class=\"value\">{len(document.get('routes', []))}</span></div>",
        f"        <div class=\"meta-card\"><span class=\"label\">Grid</span><span class=\"value\">{html.escape(str(meta.get('grid_spacing_m', 'n/a')))} m</span></div>",
        "      </section>",
        "      <div class=\"canvas\">",
        f"        <svg viewBox=\"0 0 {SVG_WIDTH} {SVG_HEIGHT}\" aria-label=\"Topo graph\">",
        "          <defs>",
        "            <filter id=\"paper-shadow\" x=\"-20%\" y=\"-20%\" width=\"140%\" height=\"140%\">",
        "              <feDropShadow dx=\"0\" dy=\"8\" stdDeviation=\"14\" flood-color=\"rgba(20, 29, 34, 0.12)\" />",
        "            </filter>",
        *[f"            {marker}" for marker in marker_defs],
        "          </defs>",
        "          <rect x=\"28\" y=\"28\" width=\"1024\" height=\"804\" rx=\"28\" fill=\"rgba(255,255,255,0.58)\" filter=\"url(#paper-shadow)\" />",
        *[f"          {line}" for line in grid_lines],
        *[f"          {edge}" for edge in edge_elements],
        *[f"          {node}" for node in node_elements],
        "        </svg>",
        "      </div>",
        "      <section class=\"legend\">",
        "        <div class=\"legend-card\">",
        "          <h2>Node Legend</h2>",
        *[
            (
                f"          <div class=\"legend-item\"><span style=\"display:flex;align-items:center;gap:10px;\">"
                f"<span class=\"legend-swatch\" style=\"background:{escape_attr(color)}\"></span>{html.escape(node_type)}"
                "</span></div>"
            )
            for node_type, color in sorted(NODE_TYPE_COLORS.items())
        ],
        "        </div>",
        "        <div class=\"legend-card\">",
        "          <h2>Edge Legend</h2>",
        *[
            (
                f"          <div class=\"legend-item\" style=\"color:{escape_attr(color)}\"><span style=\"display:flex;align-items:center;gap:10px;\">"
                "<span class=\"legend-line\"></span>"
                f"{html.escape(motion_type)}</span></div>"
            )
            for motion_type, color in sorted(EDGE_MOTION_COLORS.items())
        ],
        "        </div>",
        "        <div class=\"legend-card\">",
        "          <h2>Encoding</h2>",
        "          <div class=\"legend-item\"><span>Dashed edge</span><span>requires_confirmation</span></div>",
        "          <div class=\"legend-item\"><span>Bidirectional offset</span><span>看清 A→B / B→A</span></div>",
        "          <div class=\"legend-item\"><span>Node subtitle</span><span>`z` 与 `base_cost`</span></div>",
        "        </div>",
        "      </section>",
        f"      <p class=\"footer-note\">Source graph: {html.escape(source_name)}</p>",
        "    </section>",
        "    <aside class=\"sidebar\">",
        "      <section class=\"panel\">",
        "        <h2>Routes</h2>",
        "        <p class=\"hint\">点一下 route，会高亮这条预设链路经过的节点和有向边。</p>",
        "        <div class=\"chip-list\">",
        *[f"          {button}" for button in route_buttons],
        "        </div>",
        "      </section>",
        "      <section class=\"panel\">",
        "        <h2>Tasks</h2>",
        "        <p class=\"hint\">点一下 task，会高亮这个任务当前允许的候选节点集合。</p>",
        "        <div class=\"chip-list\">",
        *[f"          {button}" for button in task_buttons],
        "        </div>",
        "      </section>",
        "      <section class=\"panel\">",
        "        <h2>Inspector</h2>",
        "        <p class=\"hint\">点击图上的节点或边，可以查看该元素完整字段。默认显示整张图的摘要。</p>",
        "        <pre id=\"details\" class=\"details\"></pre>",
        "      </section>",
        warnings_html,
        "    </aside>",
        "  </div>",
        f"  <script>{script}</script>",
        "</body>",
        "</html>",
    ]

    return "\n".join(part for part in html_parts if part)


def write_html(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render an rc26_topo_nav graph YAML into a static HTML visualizer."
    )
    parser.add_argument("--graph", type=Path, required=True, help="Input graph YAML path")
    parser.add_argument("--out", type=Path, required=True, help="Output HTML path")
    parser.add_argument("--title", type=str, default=None, help="Optional page title override")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    document = load_yaml(args.graph)
    html_document = render_html_document(document, args.graph.name, explicit_title=args.title)
    write_html(args.out, html_document)
    print(f"[OK] Wrote static graph view: {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
