#!/usr/bin/env python3

import argparse
import math
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


def parse_bool(value: str) -> bool:
    lowered = value.strip().lower()
    if lowered in ("1", "true", "t", "yes", "y", "on"):
        return True
    if lowered in ("0", "false", "f", "no", "n", "off"):
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {value}")


class MockOdomScanNode(Node):
    def __init__(
        self,
        rate_hz: float,
        obstacle_enable: bool,
        ground_enable: bool,
        ground_tilt_x: float,
        ground_tilt_y: float,
        ground_step: float,
    ) -> None:
        super().__init__("mock_odom_scan")

        self._rate_hz = rate_hz if rate_hz > 0.0 else 10.0
        self._obstacle_enable = obstacle_enable
        self._ground_enable = ground_enable
        self._ground_tilt_x = ground_tilt_x
        self._ground_tilt_y = ground_tilt_y
        self._ground_step = max(0.01, ground_step)

        self._odom_pub = self.create_publisher(Odometry, "/odom", 20)
        self._scan_pub = self.create_publisher(PointCloud2, "/registered_scan", 20)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._start_sec = self.get_clock().now().nanoseconds * 1e-9
        self._ground_points = self._make_ground_points(
            ground_tilt_x,
            ground_tilt_y,
            self._ground_step,
        )
        self._obstacle_points = self._make_obstacle_points()

        self.create_timer(1.0 / self._rate_hz, self._on_timer)
        self.get_logger().info(
            "mock_odom_scan started: "
            f"rate_hz={self._rate_hz:.2f} "
            f"obstacle_enable={self._obstacle_enable} "
            f"ground_enable={self._ground_enable} "
            f"ground_tilt_x={self._ground_tilt_x:.3f} "
            f"ground_tilt_y={self._ground_tilt_y:.3f} "
            f"ground_step={self._ground_step:.3f}"
        )

    @staticmethod
    def _frange(start: float, stop: float, step: float):
        value = start
        while value <= stop + 1e-9:
            yield value
            value += step

    @staticmethod
    def _make_ground_points(
        tilt_x: float,
        tilt_y: float,
        ground_step: float,
    ) -> List[Tuple[float, float, float]]:
        points: List[Tuple[float, float, float]] = []
        radius = 1.5
        step = max(0.01, ground_step)
        max_index = int(math.ceil(radius / step))
        for ix in range(-max_index, max_index + 1):
            x = ix * step
            for iy in range(-max_index, max_index + 1):
                y = iy * step
                if x * x + y * y <= radius * radius:
                    # livox 在 base_link 上方 0.13m，地面在雷达坐标系近似 z=-0.13
                    z = -0.13 + tilt_x * x + tilt_y * y
                    points.append((x, y, z))
        return points

    @staticmethod
    def _make_obstacle_points() -> List[Tuple[float, float, float]]:
        points: List[Tuple[float, float, float]] = []
        for x in MockOdomScanNode._frange(0.85, 1.15, 0.03):
            for y in MockOdomScanNode._frange(-0.24, 0.24, 0.03):
                for z in MockOdomScanNode._frange(0.30, 0.54, 0.03):
                    points.append((round(x, 3), round(y, 3), round(z, 3)))
        return points

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        stamp = now.to_msg()
        t = now.nanoseconds * 1e-9 - self._start_sec

        base_x = 0.35 * math.sin(0.35 * t)
        base_y = 0.0
        base_yaw = 0.0

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = base_x
        odom.pose.pose.position.y = base_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = 0.35 * 0.35 * math.cos(0.35 * t)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.z = 0.0
        self._odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = base_x
        tf.transform.translation.y = base_y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = math.sin(base_yaw * 0.5)
        tf.transform.rotation.w = math.cos(base_yaw * 0.5)
        self._tf_broadcaster.sendTransform(tf)

        points: List[Tuple[float, float, float]] = []
        if self._ground_enable:
            points.extend(self._ground_points)
        if self._obstacle_enable:
            points.extend(self._obstacle_points)
        if not points:
            points.append((0.0, 0.0, -0.13))

        cloud_header = Header()
        cloud_header.stamp = stamp
        cloud_header.frame_id = "livox_frame"
        cloud_msg = point_cloud2.create_cloud_xyz32(cloud_header, points)
        self._scan_pub.publish(cloud_msg)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Mock odom + registered_scan publisher for rc26_terrain launch tests."
    )
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
        "--ground_tilt_x",
        type=float,
        default=0.0,
        help="ground plane tilt coefficient along x in livox frame",
    )
    parser.add_argument(
        "--ground_tilt_y",
        type=float,
        default=0.0,
        help="ground plane tilt coefficient along y in livox frame",
    )
    parser.add_argument(
        "--ground_step",
        type=float,
        default=0.06,
        help="ground point spacing in meters",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = MockOdomScanNode(
        args.rate_hz,
        args.obstacle_enable,
        args.ground_enable,
        args.ground_tilt_x,
        args.ground_tilt_y,
        args.ground_step,
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
