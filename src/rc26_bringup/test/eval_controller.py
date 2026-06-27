#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import os
import sys
from typing import Dict, List, Optional, Sequence, Tuple

try:
    import numpy as np
except ImportError:  # noqa: F401
    np = None

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:
    print(f"[ERROR] Missing ROS Python dependency: {exc}", file=sys.stderr)
    print("[HINT] Please source ROS2 env and install rosbag2_py/rclpy.", file=sys.stderr)
    sys.exit(2)


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def percentile(values: Sequence[float], q: float) -> Optional[float]:
    if not values:
        return None
    if np is not None:
        return float(np.percentile(np.asarray(values, dtype=float), q))

    sorted_vals = sorted(values)
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    pos = (len(sorted_vals) - 1) * q / 100.0
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return sorted_vals[lo]
    ratio = pos - lo
    return sorted_vals[lo] * (1.0 - ratio) + sorted_vals[hi] * ratio


def mean_std(values: Sequence[float]) -> Tuple[Optional[float], Optional[float]]:
    if not values:
        return None, None
    mean_v = float(sum(values) / len(values))
    if len(values) == 1:
        return mean_v, 0.0
    var = float(sum((x - mean_v) ** 2 for x in values) / len(values))
    return mean_v, math.sqrt(var)


def detect_storage_id(bag_path: str) -> str:
    if os.path.isfile(bag_path) and bag_path.endswith(".mcap"):
        return "mcap"

    metadata_path = os.path.join(bag_path, "metadata.yaml")
    if not os.path.exists(metadata_path):
        return "sqlite3"

    with open(metadata_path, "r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.strip()
            if line.startswith("storage_identifier:"):
                parts = line.split(":", 1)
                if len(parts) == 2:
                    return parts[1].strip().strip("'").strip('"')
    return "sqlite3"


def load_bag_messages(
    bag_path: str,
    odom_topic: str,
    plan_topic: Optional[str],
    real_dt_topic: str,
    compute_time_topic: str,
    cmd_vel_topic: str,
):
    storage_id = detect_storage_id(bag_path)
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format="", output_serialization_format="")

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topics_info = reader.get_all_topics_and_types()
    topic_type_map: Dict[str, str] = {x.name: x.type for x in topics_info}

    required_topics = {odom_topic, real_dt_topic, compute_time_topic, cmd_vel_topic}
    if plan_topic:
        required_topics.add(plan_topic)

    topic_msg_type = {}
    for topic in required_topics:
        if topic in topic_type_map:
            topic_msg_type[topic] = get_message(topic_type_map[topic])

    odom_samples: List[Tuple[float, float, float, float]] = []
    plan_samples: List[Tuple[float, List[Tuple[float, float]]]] = []
    cmd_vel_samples: List[Tuple[float, float]] = []
    real_dt_samples: List[float] = []
    compute_time_samples: List[float] = []

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in topic_msg_type:
            continue

        msg = deserialize_message(data, topic_msg_type[topic])
        bag_time = float(t_ns) * 1e-9

        if topic == odom_topic:
            t = stamp_to_sec(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else bag_time
            px = float(msg.pose.pose.position.x)
            py = float(msg.pose.pose.position.y)
            v = math.hypot(float(msg.twist.twist.linear.x), float(msg.twist.twist.linear.y))
            odom_samples.append((t, px, py, v))
        elif plan_topic and topic == plan_topic:
            t = stamp_to_sec(msg.header.stamp) if msg.header.stamp.sec or msg.header.stamp.nanosec else bag_time
            pts = [(float(p.pose.position.x), float(p.pose.position.y)) for p in msg.poses]
            plan_samples.append((t, pts))
        elif topic == cmd_vel_topic:
            t = bag_time
            if hasattr(msg, "twist") and hasattr(msg.twist, "linear"):
                tw = msg.twist
            else:
                tw = msg
            cmd_speed = math.hypot(float(tw.linear.x), float(tw.linear.y))
            cmd_vel_samples.append((t, cmd_speed))
        elif topic == real_dt_topic:
            real_dt_samples.append(float(msg.data))
        elif topic == compute_time_topic:
            compute_time_samples.append(float(msg.data))

    odom_samples.sort(key=lambda x: x[0])
    plan_samples.sort(key=lambda x: x[0])
    cmd_vel_samples.sort(key=lambda x: x[0])

    return odom_samples, plan_samples, cmd_vel_samples, real_dt_samples, compute_time_samples


def compute_path_rms_error(
    odom_samples: Sequence[Tuple[float, float, float, float]],
    plan_samples: Sequence[Tuple[float, List[Tuple[float, float]]]],
) -> Optional[float]:
    if np is None:
        return None
    if not odom_samples or not plan_samples:
        return None

    plan_idx = 0
    errors = []
    for t, x, y, _ in odom_samples:
        while plan_idx + 1 < len(plan_samples) and plan_samples[plan_idx + 1][0] <= t:
            plan_idx += 1
        if plan_samples[plan_idx][0] > t:
            continue
        points = plan_samples[plan_idx][1]
        if not points:
            continue
        arr = np.asarray(points, dtype=float)
        dist2 = (arr[:, 0] - x) ** 2 + (arr[:, 1] - y) ** 2
        errors.append(float(math.sqrt(float(np.min(dist2)))))

    if not errors:
        return None
    return float(math.sqrt(float(np.mean(np.asarray(errors, dtype=float) ** 2))))


def compute_overshoot_percent(
    cmd_vel_samples: Sequence[Tuple[float, float]],
    odom_samples: Sequence[Tuple[float, float, float, float]],
) -> Optional[float]:
    if np is None:
        return None
    if len(cmd_vel_samples) < 2 or len(odom_samples) < 2:
        return None

    step_idx = None
    for i in range(1, len(cmd_vel_samples)):
        if cmd_vel_samples[i - 1][1] < 0.05 and cmd_vel_samples[i][1] > 0.2:
            step_idx = i
            break
    if step_idx is None:
        return None

    t_step = cmd_vel_samples[step_idx][0]
    cmd_window = [v for t, v in cmd_vel_samples if t_step <= t <= t_step + 2.0]
    if not cmd_window:
        return None
    setpoint = max(cmd_window)
    if setpoint <= 1e-6:
        return None

    odom_window = [v for t, _, _, v in odom_samples if t_step <= t <= t_step + 3.0]
    if not odom_window:
        return None
    peak = max(odom_window)
    return max(0.0, (peak - setpoint) / setpoint * 100.0)


def format_or_na(value: Optional[float], precision: int = 6) -> str:
    if value is None:
        return "N/A"
    return f"{value:.{precision}f}"


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate Nav2 controller/runtime metrics from rosbag2.")
    parser.add_argument("bag", help="rosbag2 directory path or .mcap file")
    parser.add_argument("--odom-topic", default="/odometry", help="Odometry topic")
    parser.add_argument("--plan-topic", default=None, help="Plan topic, e.g. /plan or /local_plan")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel", help="Command velocity topic")
    parser.add_argument("--real-dt-topic", default="/real_dt", help="Debug real dt topic")
    parser.add_argument("--compute-time-topic", default="/compute_time_ms", help="Debug compute time topic (ms)")
    args = parser.parse_args()

    odom_samples, plan_samples, cmd_vel_samples, real_dt_samples, compute_time_samples = load_bag_messages(
        bag_path=args.bag,
        odom_topic=args.odom_topic,
        plan_topic=args.plan_topic,
        real_dt_topic=args.real_dt_topic,
        compute_time_topic=args.compute_time_topic,
        cmd_vel_topic=args.cmd_vel_topic,
    )

    if not odom_samples:
        print(f"[ERROR] No odometry data found on topic: {args.odom_topic}", file=sys.stderr)
        return 3

    if real_dt_samples:
        dt_values = [x for x in real_dt_samples if math.isfinite(x) and x >= 0.0]
        dt_source = args.real_dt_topic
    else:
        dt_values = []
        for i in range(1, len(odom_samples)):
            dt = odom_samples[i][0] - odom_samples[i - 1][0]
            if math.isfinite(dt) and dt > 0.0:
                dt_values.append(dt)
        dt_source = f"{args.odom_topic} delta_t (fallback)"

    dt_mean, dt_std = mean_std(dt_values)
    dt_p99 = percentile(dt_values, 99.0)

    compute_values = [x for x in compute_time_samples if math.isfinite(x) and x >= 0.0]
    compute_p99 = percentile(compute_values, 99.0)

    if np is None:
        e_xy_rms = None
        overshoot_pct = None
    else:
        e_xy_rms = compute_path_rms_error(odom_samples, plan_samples) if args.plan_topic else None
        overshoot_pct = compute_overshoot_percent(cmd_vel_samples, odom_samples)

    print("=== Nav2 controller runtime bag eval ===")
    print(f"bag: {args.bag}")
    print(f"odom samples: {len(odom_samples)}")
    print(f"plan samples: {len(plan_samples)}")
    print(f"cmd_vel samples: {len(cmd_vel_samples)}")
    print(f"numpy: {'yes' if np is not None else 'no'}")
    print("")
    print(f"e_xy RMS (m): {format_or_na(e_xy_rms, 4)}")
    print(f"real_dt source: {dt_source}")
    print(f"real_dt mean (s): {format_or_na(dt_mean, 6)}")
    print(f"real_dt std (s): {format_or_na(dt_std, 6)}")
    print(f"real_dt P99 (s): {format_or_na(dt_p99, 6)}")
    print(f"compute_time_ms P99: {format_or_na(compute_p99, 3)}")
    print(f"overshoot (%): {format_or_na(overshoot_pct, 2)}")

    if np is None:
        print("\n[INFO] numpy not found, e_xy RMS and overshoot are reported as N/A.")
    elif args.plan_topic and e_xy_rms is None:
        print("\n[INFO] plan/odom alignment unavailable, e_xy RMS is N/A.")
    elif not args.plan_topic:
        print("\n[INFO] --plan-topic not provided, e_xy RMS is N/A.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
