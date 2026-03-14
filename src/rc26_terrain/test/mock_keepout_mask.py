#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class MockKeepoutMaskNode(Node):
    def __init__(self) -> None:
        super().__init__("mock_keepout_mask")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._pub = self.create_publisher(OccupancyGrid, "/kfs_filter_mask", qos)
        self._timer = self.create_timer(0.5, self._on_timer)
        self.get_logger().info("mock_keepout_mask started")

    def _on_timer(self) -> None:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = 0.6
        msg.info.width = 8
        msg.info.height = 8
        msg.info.origin.position.x = -2.4
        msg.info.origin.position.y = -2.4
        msg.info.origin.orientation.w = 1.0
        msg.data = [0] * (msg.info.width * msg.info.height)
        center = int((msg.info.height // 2) * msg.info.width + (msg.info.width // 2))
        msg.data[center] = 100
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MockKeepoutMaskNode()
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
