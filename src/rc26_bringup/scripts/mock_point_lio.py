#!/usr/bin/env python3

import argparse
import math
from typing import List, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def parse_bool(value: str) -> bool:
    lowered = value.strip().lower()
    if lowered in ("1", "true", "t", "yes", "y", "on"):
        return True
    if lowered in ("0", "false", "f", "no", "n", "off"):
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {value}")


class MockPointLioNode(Node):
    def __init__(
        self,
        rate_hz: float,
        obstacle_enable: bool,
        ground_enable: bool,
        body_frame: str,
        stationary_warmup_sec: float,
    ) -> None:
        super().__init__("mock_point_lio")

        self._rate_hz = rate_hz if rate_hz > 0.0 else 10.0
        self._obstacle_enable = obstacle_enable
        self._ground_enable = ground_enable
        self._body_frame = body_frame.strip() if body_frame.strip() else "point_lio_body"
        self._stationary_warmup_sec = max(0.0, stationary_warmup_sec)

        self._odom_pub = self.create_publisher(Odometry, "state_estimation", 10)
        self._cloud_pub = self.create_publisher(PointCloud2, "cloud_registered", 10)

        self._start_sec = self.get_clock().now().nanoseconds * 1e-9
        self._ground_points = self._make_ground_points()
        self._obstacle_points = self._make_obstacle_points()

        self.create_timer(1.0 / self._rate_hz, self._on_timer)
        self.get_logger().info(
            f"mock_point_lio started: rate_hz={self._rate_hz:.2f} "
            f"obstacle_enable={self._obstacle_enable} "
            f"ground_enable={self._ground_enable} "
            f"body_frame={self._body_frame} "
            f"stationary_warmup_sec={self._stationary_warmup_sec:.2f}"
        )

    @staticmethod
    def _frange(start: float, stop: float, step: float):
        value = start
        while value <= stop + 1e-9:
            yield value
            value += step

    @staticmethod
    def _make_ground_points() -> List[Tuple[float, float, float]]:
        points: List[Tuple[float, float, float]] = []
        radius = 1.5
        step = 0.05
        max_index = int(math.ceil(radius / step))
        for ix in range(-max_index, max_index + 1):
            x = ix * step
            for iy in range(-max_index, max_index + 1):
                y = iy * step
                if x * x + y * y <= radius * radius:
                    points.append((x, y, 0.0))
        return points

    @staticmethod
    def _make_obstacle_points() -> List[Tuple[float, float, float]]:
        points: List[Tuple[float, float, float]] = []
        for x in MockPointLioNode._frange(0.85, 1.15, 0.03):
            for y in MockPointLioNode._frange(-0.25, 0.25, 0.03):
                for z in MockPointLioNode._frange(0.42, 0.62, 0.03):
                    points.append((round(x, 3), round(y, 3), round(z, 3)))
        return points

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        stamp = now.to_msg()
        elapsed_sec = now.nanoseconds * 1e-9 - self._start_sec
        in_stationary_warmup = elapsed_sec < self._stationary_warmup_sec
        t = max(0.0, elapsed_sec - self._stationary_warmup_sec)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = self._body_frame
        odom.pose.pose.position.x = 0.0 if in_stationary_warmup else 0.5 * math.sin(t)
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = 0.0 if in_stationary_warmup else 0.5 * math.cos(t)
        odom.twist.twist.linear.y = 0.0 if in_stationary_warmup else 0.1 * math.sin(0.5 * t)
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.z = 0.0 if in_stationary_warmup else 0.2 * math.cos(0.3 * t)
        self._odom_pub.publish(odom)

        points: List[Tuple[float, float, float]] = []
        if self._ground_enable:
            points.extend(self._ground_points)
        if self._obstacle_enable:
            points.extend(self._obstacle_points)
        if not points:
            points.append((0.0, 0.0, 0.0))

        cloud_header = Header()
        cloud_header.stamp = stamp
        cloud_header.frame_id = "odom"
        cloud_msg = point_cloud2.create_cloud_xyz32(cloud_header, points)
        self._cloud_pub.publish(cloud_msg)


def main() -> None:
    parser = argparse.ArgumentParser(description="Mock Point-LIO publisher for odometry/terrain integration testing.")
    parser.add_argument("--rate_hz", type=float, default=10.0, help="publish rate in Hz")
    parser.add_argument(
        "--obstacle_enable",
        type=parse_bool,
        default=True,
        help="whether to publish obstacle points (true/false)",
    )
    parser.add_argument(
        "--ground_enable",
        type=parse_bool,
        default=True,
        help="whether to publish ground points (true/false)",
    )
    parser.add_argument(
        "--body_frame",
        default="point_lio_body",
        help="internal Point-LIO body frame name used by state_estimation.child_frame_id",
    )
    parser.add_argument(
        "--stationary_warmup_sec",
        type=float,
        default=1.5,
        help="initial stationary duration to satisfy odom_interface zero-origin warmup",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = MockPointLioNode(
        args.rate_hz,
        args.obstacle_enable,
        args.ground_enable,
        args.body_frame,
        args.stationary_warmup_sec,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
