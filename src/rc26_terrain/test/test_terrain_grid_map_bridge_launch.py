import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import launch_testing.actions
from grid_map_msgs.msg import GridMap
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def generate_test_description():
    terrain_dir = get_package_share_directory("rc26_terrain")
    test_dir = os.path.dirname(os.path.realpath(__file__))
    mock_odom_scan_script = os.path.join(test_dir, "mock_odom_scan.py")
    mock_keepout_script = os.path.join(test_dir, "mock_keepout_mask.py")

    static_tf_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_to_odom",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "map",
            "--child-frame-id", "odom",
        ],
    )

    static_tf_base_to_livox = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_livox",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0.13",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "livox_frame",
        ],
    )

    mock_odom_scan = ExecuteProcess(
        cmd=[
            "python3",
            mock_odom_scan_script,
            "--rate_hz",
            "10.0",
            "--obstacle_enable",
            "true",
            "--ground_enable",
            "true",
        ],
        output="screen",
    )

    mock_keepout = ExecuteProcess(
        cmd=["python3", mock_keepout_script],
        output="screen",
    )

    terrain_node = Node(
        package="rc26_terrain",
        executable="rc26_terrain_node",
        name="terrain_semantic",
        output="screen",
        parameters=[
            os.path.join(terrain_dir, "config", "terrain_semantic.yaml"),
            {
                "use_sim_time": False,
                "enable_fail_safe": False,
                "enable_terrain_features_pub": True,
            },
        ],
    )

    terrain_grid_map_bridge_node = Node(
        package="rc26_terrain",
        executable="terrain_grid_map_bridge_node",
        name="terrain_grid_map_bridge",
        output="screen",
        parameters=[
            os.path.join(terrain_dir, "config", "terrain_grid_map_bridge.yaml"),
            {"use_sim_time": False},
        ],
    )

    return (
        launch.LaunchDescription(
            [
                static_tf_map_to_odom,
                static_tf_base_to_livox,
                mock_odom_scan,
                mock_keepout,
                terrain_node,
                terrain_grid_map_bridge_node,
                TimerAction(period=3.0, actions=[launch_testing.actions.ReadyToTest()]),
            ]
        ),
        {},
    )


class TestTerrainGridMapBridgeLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_bridge_output_and_main_chain_alive(self):
        node = rclpy.create_node("test_terrain_grid_map_bridge_launch_probe")
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        grid_qos = QoSProfile(depth=1)
        grid_qos.reliability = ReliabilityPolicy.RELIABLE
        grid_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        terrain_qos = QoSProfile(depth=10)
        terrain_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        terrain_qos.durability = DurabilityPolicy.VOLATILE

        grid_msgs = []
        obstacle_msgs = []
        drop_msgs = []

        sub_grid = node.create_subscription(
            GridMap,
            "/terrain_grid_map",
            lambda msg: grid_msgs.append(msg),
            grid_qos,
        )
        sub_obstacle = node.create_subscription(
            PointCloud2,
            "/terrain_obstacles",
            lambda msg: obstacle_msgs.append(msg),
            terrain_qos,
        )
        sub_drop = node.create_subscription(
            PointCloud2,
            "/terrain_drop",
            lambda msg: drop_msgs.append(msg),
            terrain_qos,
        )

        deadline = time.time() + 40.0
        try:
            while time.time() < deadline:
                executor.spin_once(timeout_sec=0.2)
                if len(grid_msgs) >= 2 and len(obstacle_msgs) >= 2 and len(drop_msgs) >= 2:
                    break

            self.assertGreaterEqual(len(grid_msgs), 1, "/terrain_grid_map 未收到消息")
            latest = grid_msgs[-1]
            self.assertEqual(latest.header.frame_id, "map")

            required_layers = {
                "elevation_abs",
                "elevation_top_abs",
                "kfs_keepout",
                "block_id",
                "expected_height",
                "traversability",
            }
            self.assertTrue(
                required_layers.issubset(set(latest.layers)),
                f"图层缺失: expect={required_layers}, actual={set(latest.layers)}",
            )

            self.assertGreater(len(obstacle_msgs), 0, "/terrain_obstacles 未收到消息")
            self.assertGreater(len(drop_msgs), 0, "/terrain_drop 未收到消息")
        finally:
            node.destroy_subscription(sub_grid)
            node.destroy_subscription(sub_obstacle)
            node.destroy_subscription(sub_drop)
            executor.remove_node(node)
            node.destroy_node()
