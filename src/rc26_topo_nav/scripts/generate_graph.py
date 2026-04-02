#!/usr/bin/env python3
"""Generate topo graph YAMLs from shared MF world geometry and semantic overlay."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import yaml


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping at the top level")
    return data


def round_float(value: float, digits: int = 4) -> float:
    rounded = round(float(value), digits)
    if abs(rounded) < 10 ** (-digits):
        return 0.0
    return rounded


def block_height(block: dict[str, Any]) -> float:
    if "expected_height_m" in block and block["expected_height_m"] is not None:
        return float(block["expected_height_m"])
    return float(block.get("depth", 0)) * 0.2


def normalize_pose(pose: dict[str, Any]) -> dict[str, float]:
    return {
        "x": round_float(pose["x"]),
        "y": round_float(pose["y"]),
        "z": round_float(pose["z"]),
        "yaw": round_float(pose["yaw"]),
    }


def build_generated_block_node(
    block: dict[str, Any],
    node_defaults: dict[str, Any],
) -> dict[str, Any]:
    height_m = block_height(block)
    z_scale = float(node_defaults.get("z_from_expected_height_scale", 0.5))
    node_id = f"mf_b{int(block['id'])}"
    return {
        "id": node_id,
        "type": node_defaults["type"],
        "pose": {
            "x": round_float(block["x"]),
            "y": round_float(block["y"]),
            "z": round_float(height_m * z_scale),
            "yaw": round_float(node_defaults["yaw"]),
        },
        "level": int(node_defaults["level"]),
        "phase_mask": int(node_defaults["phase_mask"]),
        "block_id": int(block["id"]),
        "base_cost": round_float(float(node_defaults.get("base_cost_default", 1.0))),
        "operation_tag": node_defaults["operation_tag"],
    }


def row_key(block: dict[str, Any], digits: int = 6) -> float:
    return round(float(block["y"]), digits)


def build_grid_edge(
    from_node: dict[str, Any],
    to_node: dict[str, Any],
    edge_defaults: dict[str, Any],
) -> dict[str, Any]:
    from_block_id = int(from_node["block_id"])
    to_block_id = int(to_node["block_id"])
    return {
        "id": f"e_b{from_block_id}_b{to_block_id}",
        "from": from_node["id"],
        "to": to_node["id"],
        "motion_type": edge_defaults["motion_type"],
        "height_change": round_float(abs(to_node["pose"]["z"] - from_node["pose"]["z"])),
        "required_mode": edge_defaults["required_mode"],
        "requires_confirmation": bool(edge_defaults["requires_confirmation"]),
        "can_block": bool(edge_defaults["can_block"]),
        "phase_mask": int(edge_defaults["phase_mask"]),
        "base_cost": round_float(edge_defaults["base_cost"]),
    }


def clone_yaml_value(value: Any) -> Any:
    return yaml.safe_load(yaml.safe_dump(value, sort_keys=False))


def build_generated_grid_edges(
    generated_nodes: list[dict[str, Any]],
    grid_spacing_m: float,
    edge_defaults: dict[str, Any],
    edge_overrides: dict[str, dict[str, Any]],
) -> list[dict[str, Any]]:
    tol = grid_spacing_m * 0.1
    rows: list[list[dict[str, Any]]] = []
    current_row: list[dict[str, Any]] = []
    current_key: float | None = None
    for node in sorted(generated_nodes, key=lambda item: (item["pose"]["y"], item["pose"]["x"], item["id"])):
        key = row_key(node["pose"])
        if current_key is None or abs(key - current_key) <= 1e-6:
            current_row.append(node)
            current_key = key
            continue
        rows.append(current_row)
        current_row = [node]
        current_key = key
    if current_row:
        rows.append(current_row)

    edges: list[dict[str, Any]] = []

    def append_edge(from_node: dict[str, Any], to_node: dict[str, Any]) -> None:
        edge = build_grid_edge(from_node, to_node, edge_defaults)
        override = edge_overrides.get(edge["id"])
        if override:
            edge.update(clone_yaml_value(override))
        edges.append(edge)

    for row_index, row in enumerate(rows):
        row.sort(key=lambda item: (item["pose"]["x"], item["id"]))
        for left, right in zip(row, row[1:]):
            dx = abs(float(right["pose"]["x"]) - float(left["pose"]["x"]))
            dy = abs(float(right["pose"]["y"]) - float(left["pose"]["y"]))
            if abs(dx - grid_spacing_m) <= tol and dy <= tol:
                append_edge(left, right)
                append_edge(right, left)

        if row_index + 1 >= len(rows):
            continue
        next_row = sorted(rows[row_index + 1], key=lambda item: (item["pose"]["x"], item["id"]))
        for upper in row:
            matches = [
                lower for lower in next_row
                if abs(float(lower["pose"]["x"]) - float(upper["pose"]["x"])) <= tol
                and abs(float(lower["pose"]["y"]) - float(upper["pose"]["y"]) - grid_spacing_m) <= tol
            ]
            for lower in matches:
                append_edge(upper, lower)
                append_edge(lower, upper)

    return edges


def build_graph_document(
    world_layout: dict[str, Any],
    overlay: dict[str, Any],
    *,
    team: str,
    world_layout_name: str,
    overlay_name: str,
) -> dict[str, Any]:
    world_meta = world_layout.get("meta", {})
    overlay_meta = overlay.get("meta", {})
    node_defaults = overlay["mf_block_node"]
    edge_defaults = overlay["mf_grid_edge"]
    grid_spacing_m = float(world_meta.get("grid_spacing_m", 1.2))

    blocks = list(world_layout.get("blocks", []))
    generated_nodes = [build_generated_block_node(block, node_defaults) for block in blocks]
    node_overrides = {
        item["id"]: {k: clone_yaml_value(v) for k, v in item.items() if k != "id"}
        for item in overlay.get("node_overrides", [])
    }
    generated_node_ids = {node["id"] for node in generated_nodes}
    unknown_node_override_ids = sorted(set(node_overrides) - generated_node_ids)
    if unknown_node_override_ids:
        raise ValueError(
            "Node override references unknown generated node(s): "
            + ", ".join(unknown_node_override_ids)
        )
    for node in generated_nodes:
        override = node_overrides.get(node["id"])
        if override:
            node.update(clone_yaml_value(override))

    edge_overrides = {
        item["id"]: {k: clone_yaml_value(v) for k, v in item.items() if k != "id"}
        for item in overlay.get("edge_overrides", [])
    }
    generated_edges = build_generated_grid_edges(
        generated_nodes,
        grid_spacing_m,
        edge_defaults,
        edge_overrides,
    )

    nodes = [
        *clone_yaml_value(overlay.get("nodes_prepend", [])),
        *generated_nodes,
        *clone_yaml_value(overlay.get("nodes_append", [])),
    ]
    node_ids = {node["id"] for node in nodes}
    if len(node_ids) != len(nodes):
        raise ValueError("Duplicate node IDs detected while assembling graph")

    edges = [
        *clone_yaml_value(overlay.get("edges_prepend", [])),
        *generated_edges,
        *clone_yaml_value(overlay.get("edges_append", [])),
    ]
    generated_edge_ids = {edge["id"] for edge in generated_edges}
    edge_ids = [edge["id"] for edge in edges]
    if len(set(edge_ids)) != len(edge_ids):
        raise ValueError("Duplicate edge IDs detected while assembling graph")
    unknown_edge_override_ids = sorted(set(edge_overrides) - generated_edge_ids)
    if unknown_edge_override_ids:
        raise ValueError(
            "Edge override references unknown generated edge(s): "
            + ", ".join(unknown_edge_override_ids)
        )

    for edge in edges:
        if edge["from"] not in node_ids or edge["to"] not in node_ids:
            raise ValueError(
                f"Edge '{edge['id']}' references missing node(s): {edge['from']} -> {edge['to']}"
            )

    return {
        "meta": {
            "team": team.lower(),
            "schema_version": overlay_meta.get("schema_version", "1.0"),
            "source": world_meta.get("layout_version", world_meta.get("source", "")),
            "grid_spacing_m": round_float(grid_spacing_m),
            "coordinate_frame": overlay_meta.get("coordinate_frame", "map"),
            "generated": True,
            "world_layout_file": world_layout_name,
            "overlay_file": overlay_name,
        },
        "nodes": nodes,
        "edges": edges,
        "tasks": clone_yaml_value(overlay.get("tasks", [])),
        "routes": clone_yaml_value(overlay.get("routes", [])),
    }


def canonicalize_graph_document(document: dict[str, Any]) -> dict[str, Any]:
    canonical = clone_yaml_value(document)
    canonical["nodes"] = sorted(canonical.get("nodes", []), key=lambda item: item["id"])
    canonical["edges"] = sorted(canonical.get("edges", []), key=lambda item: item["id"])
    canonical["tasks"] = sorted(canonical.get("tasks", []), key=lambda item: item["task_tag"])
    canonical["routes"] = sorted(canonical.get("routes", []), key=lambda item: item["route_tag"])
    return canonical


def dump_yaml(path: Path, document: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(document, f, sort_keys=False, allow_unicode=False)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate rc26_topo_nav graph YAML from shared world layout and semantic overlay."
    )
    parser.add_argument("--world-layout", required=True, type=Path, help="Path to r2_mf_world.yaml")
    parser.add_argument("--overlay", required=True, type=Path, help="Path to topo graph overlay YAML")
    parser.add_argument("--team", required=True, choices=["blue", "red"], help="Target competition side")
    parser.add_argument("--out", required=True, type=Path, help="Output graph YAML path")
    parser.add_argument(
        "--check-existing",
        action="store_true",
        help="Do not overwrite output; fail if the generated graph differs from the current file",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    world_layout = load_yaml(args.world_layout)
    overlay = load_yaml(args.overlay)
    document = build_graph_document(
        world_layout,
        overlay,
        team=args.team,
        world_layout_name=args.world_layout.name,
        overlay_name=args.overlay.name,
    )

    if args.check_existing:
        if not args.out.is_file():
            print(f"[ERROR] Output file does not exist for --check-existing: {args.out}", file=sys.stderr)
            return 1
        existing = load_yaml(args.out)
        if canonicalize_graph_document(existing) != canonicalize_graph_document(document):
            print(f"[ERROR] Generated graph differs from existing file: {args.out}", file=sys.stderr)
            return 1
        print(f"[OK] Graph matches existing file: {args.out}")
        return 0

    args.out.parent.mkdir(parents=True, exist_ok=True)
    dump_yaml(args.out, document)
    print(f"[OK] Generated {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
