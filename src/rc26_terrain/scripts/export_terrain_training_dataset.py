#!/usr/bin/env python3
"""Export per-cell terrain features from rosbag2 to CSV for risk-model calibration."""

from __future__ import annotations

import argparse
import csv
import os
import sys
from typing import Dict, Optional, Tuple

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except Exception as exc:  # pragma: no cover - runtime dependency
    print(f"[ERROR] rosbag2_py/rclpy not available: {exc}", file=sys.stderr)
    sys.exit(2)


def detect_storage_id(bag_path: str) -> str:
    if os.path.isfile(bag_path) and bag_path.endswith(".mcap"):
        return "mcap"
    metadata_path = os.path.join(bag_path, "metadata.yaml")
    if not os.path.exists(metadata_path):
        return "sqlite3"
    with open(metadata_path, "r", encoding="utf-8") as stream:
        text = stream.read()
    if "storage_identifier: mcap" in text:
        return "mcap"
    return "sqlite3"


def load_labels(labels_csv: str) -> Dict[Tuple[int, int], Tuple[Optional[float], Optional[float]]]:
    labels: Dict[Tuple[int, int], Tuple[Optional[float], Optional[float]]] = {}
    with open(labels_csv, "r", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {"stamp_ns", "cell_index"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"labels csv missing columns: {sorted(missing)}")
        for row in reader:
            stamp_ns = int(row["stamp_ns"])
            cell_index = int(row["cell_index"])
            obstacle = row.get("obstacle_label", "")
            drop = row.get("drop_label", "")
            obstacle_val = float(obstacle) if obstacle != "" else None
            drop_val = float(drop) if drop != "" else None
            labels[(stamp_ns, cell_index)] = (obstacle_val, drop_val)
    return labels


def main() -> int:
    parser = argparse.ArgumentParser(description="Export terrain feature dataset from rosbag2.")
    parser.add_argument("bag", help="rosbag2 directory path or mcap file")
    parser.add_argument("--topic", default="/terrain_features",
                        help="TerrainFeatureGrid topic name (default: /terrain_features)")
    parser.add_argument("--out", required=True, help="Output CSV path")
    parser.add_argument("--labels-csv", default="",
                        help="Optional labels CSV with columns: stamp_ns,cell_index,obstacle_label,drop_label")
    parser.add_argument("--fresh-only", action="store_true",
                        help="Only export cells with fresh=1")
    args = parser.parse_args()

    labels: Dict[Tuple[int, int], Tuple[Optional[float], Optional[float]]] = {}
    if args.labels_csv:
        labels = load_labels(args.labels_csv)
        print(f"[INFO] loaded labels: {len(labels)}")

    storage_id = detect_storage_id(args.bag)
    storage_options = rosbag2_py.StorageOptions(uri=args.bag, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="", output_serialization_format=""
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}

    topic_name = args.topic
    if topic_name not in type_map and topic_name.startswith("/"):
        topic_name = topic_name[1:]
    if topic_name not in type_map and ("/" + topic_name) in type_map:
        topic_name = "/" + topic_name
    if topic_name not in type_map:
        print(f"[ERROR] topic not found in bag: {args.topic}", file=sys.stderr)
        print(f"[INFO] available topics: {sorted(type_map)}", file=sys.stderr)
        return 1

    msg_type = get_message(type_map[topic_name])
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)

    fieldnames = [
        "stamp_ns",
        "msg_index",
        "cell_index",
        "ix",
        "iy",
        "fresh",
        "in_radius",
        "density",
        "h_ground",
        "sigma_h",
        "h_top",
        "slope_x",
        "slope_y",
        "slope_abs",
        "roughness",
        "step_up",
        "p_climbable",
        "p_obstacle_proxy",
        "p_drop_proxy",
        "height_span",
        "obstacle_label",
        "drop_label",
    ]

    rows = 0
    msg_index = 0
    with open(args.out, "w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()

        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic != topic_name:
                continue
            msg = deserialize_message(data, msg_type)
            stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
            width = int(msg.width)
            height = int(msg.height)
            cells = width * height
            if cells <= 0:
                msg_index += 1
                continue

            for idx in range(cells):
                in_radius = int(msg.in_radius[idx]) if idx < len(msg.in_radius) else 0
                if in_radius == 0:
                    continue
                fresh = int(msg.fresh[idx]) if idx < len(msg.fresh) else 0
                if args.fresh_only and fresh == 0:
                    continue
                ix = idx // width
                iy = idx % width
                slope_x = float(msg.slope_x[idx]) if idx < len(msg.slope_x) else 0.0
                slope_y = float(msg.slope_y[idx]) if idx < len(msg.slope_y) else 0.0
                h_ground = float(msg.h_ground[idx]) if idx < len(msg.h_ground) else 0.0
                h_top = float(msg.h_top[idx]) if idx < len(msg.h_top) else 0.0
                obstacle_label = ""
                drop_label = ""
                if labels:
                    found = labels.get((stamp_ns, idx))
                    if found is not None:
                        if found[0] is not None:
                            obstacle_label = str(found[0])
                        if found[1] is not None:
                            drop_label = str(found[1])

                writer.writerow({
                    "stamp_ns": stamp_ns,
                    "msg_index": msg_index,
                    "cell_index": idx,
                    "ix": ix,
                    "iy": iy,
                    "fresh": fresh,
                    "in_radius": in_radius,
                    "density": int(msg.density[idx]) if idx < len(msg.density) else 0,
                    "h_ground": h_ground,
                    "sigma_h": float(msg.sigma_h[idx]) if idx < len(msg.sigma_h) else 0.0,
                    "h_top": h_top,
                    "slope_x": slope_x,
                    "slope_y": slope_y,
                    "slope_abs": max(abs(slope_x), abs(slope_y)),
                    "roughness": float(msg.roughness[idx]) if idx < len(msg.roughness) else 0.0,
                    "step_up": float(msg.step_up[idx]) if idx < len(msg.step_up) else 0.0,
                    "p_climbable": float(msg.p_climbable[idx]) if idx < len(msg.p_climbable) else 0.0,
                    "p_obstacle_proxy": float(msg.p_obstacle[idx]) if idx < len(msg.p_obstacle) else 0.0,
                    "p_drop_proxy": float(msg.p_drop[idx]) if idx < len(msg.p_drop) else 0.0,
                    "height_span": max(0.0, h_top - h_ground),
                    "obstacle_label": obstacle_label,
                    "drop_label": drop_label,
                })
                rows += 1
            msg_index += 1

    print(f"[INFO] export done: rows={rows}, output={args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
