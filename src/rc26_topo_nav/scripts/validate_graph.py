#!/usr/bin/env python3
"""Offline graph YAML validator for rc26_topo_nav."""

import sys
import yaml
import math
from pathlib import Path


def validate_graph(data: dict) -> list[str]:
    errors = []

    nodes = {n["id"]: n for n in data.get("nodes", [])}
    edges = data.get("edges", [])
    tasks = data.get("tasks", [])
    routes = data.get("routes", [])
    grid_spacing = data.get("meta", {}).get("grid_spacing_m", 1.2)
    tol = grid_spacing * 0.1

    # Edge endpoint existence
    for e in edges:
        if e["from"] not in nodes:
            errors.append(f"Edge '{e['id']}' references missing from-node '{e['from']}'")
        if e["to"] not in nodes:
            errors.append(f"Edge '{e['id']}' references missing to-node '{e['to']}'")

    for node_id, node in nodes.items():
        if node.get("type") != "surface_point":
            continue
        if "center_clearance_m" not in node:
            errors.append(f"Surface node '{node_id}' missing center_clearance_m")
        if "surface_pitch_deg" not in node:
            errors.append(f"Surface node '{node_id}' missing surface_pitch_deg")

    for edge in edges:
        from_node = nodes.get(edge["from"])
        to_node = nodes.get(edge["to"])
        if not from_node or not to_node:
            continue
        if from_node.get("type") != "surface_point" or to_node.get("type") != "surface_point":
            continue
        for key in ("horizontal_length_m", "slope_deg", "center_clearance_m", "nominal_yaw", "same_surface"):
            if key not in edge:
                errors.append(f"Surface edge '{edge['id']}' missing {key}")

    # MF edges: only manhattan-adjacent (no diagonal)
    for e in edges:
        fn = nodes.get(e["from"])
        tn = nodes.get(e["to"])
        if not fn or not tn:
            continue
        if fn.get("block_id", 0) > 0 and tn.get("block_id", 0) > 0:
            dx = abs(fn["pose"]["x"] - tn["pose"]["x"])
            dy = abs(fn["pose"]["y"] - tn["pose"]["y"])
            is_x = abs(dx - grid_spacing) < tol and dy < tol
            is_y = abs(dy - grid_spacing) < tol and dx < tol
            if not is_x and not is_y:
                errors.append(
                    f"Edge '{e['id']}' between MF blocks is diagonal or non-adjacent"
                )

    # Task candidate nodes exist
    for t in tasks:
        if not t.get("candidate_nodes"):
            errors.append(f"Task '{t['task_tag']}' has no candidate nodes")
        for cn in t.get("candidate_nodes", []):
            if cn not in nodes:
                errors.append(f"Task '{t['task_tag']}' references missing node '{cn}'")

    edge_pairs = {(e["from"], e["to"]) for e in edges if e.get("from") in nodes and e.get("to") in nodes}
    for route in routes:
        route_tag = route.get("route_tag", "<missing>")
        route_nodes = route.get("nodes", [])
        if not route_nodes:
            errors.append(f"Route '{route_tag}' has no nodes")
            continue
        for node_id in route_nodes:
            if node_id not in nodes:
                errors.append(f"Route '{route_tag}' references missing node '{node_id}'")
        for idx in range(1, len(route_nodes)):
            pair = (route_nodes[idx - 1], route_nodes[idx])
            if pair not in edge_pairs:
                errors.append(
                    f"Route '{route_tag}' has no direct edge from '{pair[0]}' to '{pair[1]}'"
                )

    return errors


def graph_summary(data: dict) -> dict[str, object]:
    nodes = list(data.get("nodes", []))
    edges = list(data.get("edges", []))
    surface_nodes = [node for node in nodes if node.get("type") == "surface_point"]
    node_types = {node["id"]: node.get("type") for node in nodes if "id" in node}
    surface_edges = [
        edge
        for edge in edges
        if node_types.get(edge.get("from")) == "surface_point"
        and node_types.get(edge.get("to")) == "surface_point"
    ]
    return {
        "team": str(data.get("meta", {}).get("team", "")),
        "schema_version": str(data.get("meta", {}).get("schema_version", "")),
        "node_count": len(nodes),
        "edge_count": len(edges),
        "surface_node_count": len(surface_nodes),
        "surface_edge_count": len(surface_edges),
    }


def format_graph_summary(summary: dict[str, object]) -> str:
    return (
        f"team={summary['team']} "
        f"schema={summary['schema_version']} "
        f"nodes={summary['node_count']} "
        f"edges={summary['edge_count']} "
        f"surface_nodes={summary['surface_node_count']} "
        f"surface_edges={summary['surface_edge_count']}"
    )


def validate_symmetry(blue: dict, red: dict) -> list[str]:
    errors = []
    b_nodes = {n["id"] for n in blue.get("nodes", [])}
    r_nodes = {n["id"] for n in red.get("nodes", [])}
    if b_nodes != r_nodes:
        errors.append("Node ID sets differ between blue and red graphs")

    b_edges = {e["id"] for e in blue.get("edges", [])}
    r_edges = {e["id"] for e in red.get("edges", [])}
    if b_edges != r_edges:
        errors.append("Edge ID sets differ between blue and red graphs")

    b_tasks = {t["task_tag"] for t in blue.get("tasks", [])}
    r_tasks = {t["task_tag"] for t in red.get("tasks", [])}
    if b_tasks != r_tasks:
        errors.append("Task tag sets differ between blue and red graphs")

    b_routes = {r["route_tag"] for r in blue.get("routes", [])}
    r_routes = {r["route_tag"] for r in red.get("routes", [])}
    if b_routes != r_routes:
        errors.append("Route tag sets differ between blue and red graphs")

    return errors


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <blue.yaml> [red.yaml]")
        sys.exit(1)

    blue_path = Path(sys.argv[1])
    with open(blue_path) as f:
        blue = yaml.safe_load(f)

    errors = validate_graph(blue)
    for e in errors:
        print(f"[BLUE ERROR] {e}")

    if len(sys.argv) >= 3:
        red_path = Path(sys.argv[2])
        with open(red_path) as f:
            red = yaml.safe_load(f)
        errors += validate_graph(red)
        for e in validate_graph(red):
            print(f"[RED ERROR] {e}")
        sym_errors = validate_symmetry(blue, red)
        errors += sym_errors
        for e in sym_errors:
            print(f"[SYMMETRY ERROR] {e}")

    if errors:
        print(f"\nValidation FAILED: {len(errors)} error(s)")
        sys.exit(1)
    else:
        print(f"Validation PASSED: {format_graph_summary(graph_summary(blue))}")
        if len(sys.argv) >= 3:
            print(f"Validation PASSED (peer): {format_graph_summary(graph_summary(red))}")
        sys.exit(0)


if __name__ == "__main__":
    main()
