#!/usr/bin/env python3
"""发布最小化定位输入（registered_scan + local_plan + control_degraded）。"""

import argparse
import math
import sys
import time
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Header

try:
    from sensor_msgs_py import point_cloud2
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("缺少 sensor_msgs_py，请先 source ROS 环境再运行") from exc


def build_base_cloud() -> List[Tuple[float, float, float]]:
    points: List[Tuple[float, float, float]] = []
    for x in range(-3, 4):
        points.append((float(x), -3.0, 0.0))
    for y in range(-3, 4):
        points.append((-3.0, float(y), 0.0))
    for x in range(-2, 3):
        points.append((float(x), 2.0, 0.2))
    for y in range(-2, 2):
        points.append((2.0, float(y), 0.0))
    return points


class SyntheticLocalizationInputNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("synthetic_localization_input")
        self.scan_topic = args.scan_topic
        self.plan_topic = args.plan_topic
        self.control_degraded_topic = args.control_degraded_topic
        self.candidate_topic = args.candidate_topic
        self.publish_candidates = args.publish_candidates
        self.odom_frame = args.odom_frame
        self.plan_frame = args.plan_frame
        self.start_time = self.get_clock().now()
        self.base_points = build_base_cloud()

        self.scan_pub = self.create_publisher(PointCloud2, self.scan_topic, 10)
        self.plan_pub = self.create_publisher(Path, self.plan_topic, 10)
        self.degraded_pub = self.create_publisher(Bool, self.control_degraded_topic, 10)
        self.candidate_pub = self.create_publisher(PoseArray, self.candidate_topic, 10) if self.publish_candidates else None

        period = max(0.02, 1.0 / max(1.0, float(args.rate)))
        self.timer = self.create_timer(period, self.on_timer)
        self.get_logger().info(
            f"synthetic input started: scan={self.scan_topic}, plan={self.plan_topic}, "
            f"control_degraded={self.control_degraded_topic}, "
            f"candidates={self.candidate_topic if self.publish_candidates else 'disabled'}"
        )

    def _make_cloud(self, t: float) -> PointCloud2:
        dx = 0.05 * math.sin(0.6 * t)
        dy = 0.03 * math.cos(0.4 * t)
        points = [(x + dx, y + dy, z) for (x, y, z) in self.base_points]
        return point_cloud2.create_cloud_xyz32(
            header=self._header(self.odom_frame),
            points=points,
        )

    def _make_path(self) -> Path:
        path = Path()
        path.header = self._header(self.plan_frame)
        for i in range(15):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = 0.4 * i
            pose.pose.position.y = 0.0
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def _header(self, frame_id: str):
        header = Header()
        now = self.get_clock().now().to_msg()
        header.stamp = now
        header.frame_id = frame_id
        return header

    def on_timer(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        self.scan_pub.publish(self._make_cloud(elapsed))
        self.plan_pub.publish(self._make_path())
        self.degraded_pub.publish(Bool(data=False))
        if self.candidate_pub is not None:
            candidate_msg = PoseArray()
            candidate_msg.header = self._header("map")
            pose = Pose()
            pose.position.x = 0.2 * math.sin(0.2 * elapsed)
            pose.position.y = 0.2 * math.cos(0.2 * elapsed)
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            candidate_msg.poses.append(pose)
            self.candidate_pub.publish(candidate_msg)


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish synthetic localization inputs")
    parser.add_argument("--scan-topic", default="registered_scan")
    parser.add_argument("--plan-topic", default="local_plan")
    parser.add_argument("--control-degraded-topic", default="/control_degraded")
    parser.add_argument("--candidate-topic", default="/localization/p4/learned_candidates")
    parser.add_argument("--publish-candidates", action="store_true")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--plan-frame", default="odom")
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
        node.get_logger().info("synthetic input finished")
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
