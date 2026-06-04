#!/usr/bin/env python3
"""Publish minimal localization input: registered_scan only."""

import argparse
import math
import sys
import time
from typing import List, Tuple

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

try:
    from sensor_msgs_py import point_cloud2
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("缺少 sensor_msgs_py，请先 source ROS 环境再运行") from exc


def build_base_cloud() -> List[Tuple[float, float, float]]:
    points: List[Tuple[float, float, float]] = []
    for x_i in range(-12, 13):
        for y_i in range(-12, 13):
            x = x_i * 0.25
            y = y_i * 0.25
            if abs(x_i) in (12, 11) or abs(y_i) in (12, 11) or (x_i + y_i) % 7 == 0:
                z = 0.15 * math.sin(0.7 * x) * math.cos(0.5 * y)
                points.append((x, y, z))
    return points


class SyntheticLocalizationInputNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("synthetic_localization_input")
        self.scan_topic = args.scan_topic
        self.odom_frame = args.odom_frame
        self.start_time = self.get_clock().now()
        self.base_points = build_base_cloud()

        self.scan_pub = self.create_publisher(PointCloud2, self.scan_topic, 10)
        period = max(0.02, 1.0 / max(1.0, float(args.rate)))
        self.timer = self.create_timer(period, self.on_timer)
        self.get_logger().info(
            f"synthetic localization input started: scan={self.scan_topic}, points={len(self.base_points)}"
        )

    def _header(self, frame_id: str) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header

    def _make_cloud(self, t: float) -> PointCloud2:
        dx = 0.03 * math.sin(0.6 * t)
        dy = 0.02 * math.cos(0.4 * t)
        points = [(x + dx, y + dy, z) for (x, y, z) in self.base_points]
        return point_cloud2.create_cloud_xyz32(
            header=self._header(self.odom_frame),
            points=points,
        )

    def on_timer(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        self.scan_pub.publish(self._make_cloud(elapsed))


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish synthetic registered_scan inputs")
    parser.add_argument("--scan-topic", default="registered_scan")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--rate", type=float, default=8.0)
    parser.add_argument("--duration", type=float, default=60.0)
    return parser.parse_args(argv)


def main(argv: List[str]) -> int:
    args = parse_args(argv)
    rclpy.init()
    node = SyntheticLocalizationInputNode(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    end_time = time.monotonic() + max(0.1, float(args.duration))
    try:
        while rclpy.ok() and time.monotonic() < end_time:
            executor.spin_once(timeout_sec=0.1)
        node.get_logger().info("synthetic localization input finished")
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
