import os
import time
import unittest
import math

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import launch_testing.actions
from grid_map_msgs.msg import GridMap
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _cloud_point_count(msg: PointCloud2) -> int:
    return int(msg.width) * int(msg.height)


def _rate_hz(stamps) -> float:
    if len(stamps) < 2:
        return 0.0
    duration = stamps[-1] - stamps[0]
    if duration <= 0.0:
        return 0.0
    return (len(stamps) - 1) / duration


def _max_nearest_skew_sec(ref_stamps, probe_stamps) -> float:
    if not ref_stamps or not probe_stamps:
        return float("inf")

    j = 0
    max_skew = 0.0
    for stamp in ref_stamps:
        while j + 1 < len(probe_stamps):
            cur = abs(probe_stamps[j] - stamp)
            nxt = abs(probe_stamps[j + 1] - stamp)
            if nxt <= cur:
                j += 1
            else:
                break
        max_skew = max(max_skew, abs(probe_stamps[j] - stamp))
    return max_skew


def _drain_executor(executor: SingleThreadedExecutor, budget_sec: float) -> None:
    deadline = time.monotonic() + budget_sec
    first_spin = True
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02 if first_spin else 0.0)
        first_spin = False


def _grid_map_layer_values(msg: GridMap, layer_name: str):
    try:
        layer_index = msg.layers.index(layer_name)
    except ValueError:
        return []

    if layer_index >= len(msg.data):
        return []
    return list(msg.data[layer_index].data)


def _grid_map_stats(msg: GridMap):
    fresh_values = _grid_map_layer_values(msg, "fresh")
    traversability_values = _grid_map_layer_values(msg, "traversability")

    fresh_cells = sum(1 for value in fresh_values if value >= 0.5)
    valid_traversability = [value for value in traversability_values if math.isfinite(value)]
    valid_fresh_traversability = [
        traversability
        for traversability, fresh in zip(traversability_values, fresh_values)
        if math.isfinite(traversability) and fresh >= 0.5
    ]

    min_traversability = min(valid_fresh_traversability, default=1.0)
    max_penalty = max((1.0 - value) for value in valid_fresh_traversability) if valid_fresh_traversability else 0.0

    return {
        "fresh_cells": fresh_cells,
        "valid_traversability_cells": len(valid_traversability),
        "valid_fresh_traversability_cells": len(valid_fresh_traversability),
        "min_traversability": min_traversability,
        "max_penalty": max_penalty,
    }


def _shutdown_lifecycle_manager(node, executor, timeout_sec: float = 5.0) -> bool:
    client = node.create_client(
        ManageLifecycleNodes,
        "/lifecycle_manager_local_costmap/manage_nodes",
    )
    try:
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if client.wait_for_service(timeout_sec=0.2):
                break
            executor.spin_once(timeout_sec=0.05)

        if not client.service_is_ready():
            node.get_logger().warning("lifecycle manager shutdown service unavailable during teardown")
            return False

        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.SHUTDOWN
        future = client.call_async(request)

        while time.time() < deadline and not future.done():
            executor.spin_once(timeout_sec=0.05)

        if not future.done():
            node.get_logger().warning("lifecycle manager shutdown request timed out")
            return False
        if future.exception() is not None:
            node.get_logger().warning(f"lifecycle manager shutdown failed: {future.exception()}")
            return False

        response = future.result()
        return response is not None and response.success
    finally:
        node.destroy_client(client)


def generate_test_description():
    terrain_dir = get_package_share_directory("rc26_terrain")
    mock_odom_scan_script = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        "mock_odom_scan.py",
    )

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
            "false",
            "--ground_enable",
            "true",
            "--ground_tilt_x",
            "0.20",
            "--ground_step",
            "0.03",
        ],
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
                "max_rel_z_m": 1.0,
                "unknown_policy": "aggressive",
                "unknown_output": "drop",
                "enable_fail_safe": False,
            },
        ],
    )

    terrain_grid_map_bridge = Node(
        package="rc26_terrain",
        executable="terrain_grid_map_bridge_node",
        name="terrain_grid_map_bridge",
        output="screen",
        parameters=[
            os.path.join(terrain_dir, "config", "terrain_grid_map_bridge.yaml"),
            os.path.join(terrain_dir, "config", "terrain_filter_chain.yaml"),
            {
                "use_sim_time": False,
                "enable_mf_semantics": False,
                "fusion_enable": False,
                "fusion_publish_raw": False,
                "publish_marker_array": False,
            },
        ],
    )

    local_costmap_overrides = {
        "use_sim_time": False,
        "global_frame": "odom",
        "robot_base_frame": "base_link",
        "rolling_window": True,
        "width": 7,
        "height": 7,
        "resolution": 0.1,
        "publish_frequency": 10.0,
        "update_frequency": 10.0,
        "always_send_full_costmap": True,
        "footprint": "[[0.4, 0.4], [0.4, -0.4], [-0.4, -0.4], [-0.4, 0.4]]",
        "plugins": ["obstacle_layer", "drop_layer", "terrain_traversability_layer", "inflation_layer"],
        "obstacle_layer": {
            "plugin": "nav2_costmap_2d::ObstacleLayer",
            "enabled": True,
            "observation_sources": "terrain_obstacles",
            "terrain_obstacles": {
                "topic": "/terrain_obstacles",
                "data_type": "PointCloud2",
                "marking": True,
                "clearing": False,
                "max_obstacle_height": 2.0,
                "min_obstacle_height": -1.0,
                "obstacle_max_range": 4.0,
                "obstacle_min_range": 0.0,
                "raytrace_max_range": 5.0,
                "raytrace_min_range": 0.0,
            },
        },
        "drop_layer": {
            "plugin": "nav2_costmap_2d::ObstacleLayer",
            "enabled": True,
            "observation_sources": "terrain_drop",
            "terrain_drop": {
                "topic": "/terrain_drop",
                "data_type": "PointCloud2",
                "marking": True,
                "clearing": False,
                "max_obstacle_height": 2.0,
                "min_obstacle_height": -1.0,
                "obstacle_max_range": 4.0,
                "obstacle_min_range": 0.0,
                "raytrace_max_range": 5.0,
                "raytrace_min_range": 0.0,
            },
        },
        "terrain_traversability_layer": {
            "plugin": "rc26_terrain_nav2::TerrainTraversabilityLayer",
            "enabled": True,
            "terrain_grid_topic": "/terrain_grid_map_local",
            "traversability_layer": "traversability",
            "fresh_layer": "fresh",
            "drop_layer": "drop_prob",
            "climbable_layer": "climbable_prob",
            "lethal_threshold": 0.25,
            "inscribed_threshold": 0.45,
            "drop_lethal_threshold": 0.8,
            "climbable_soft_cost_max": 80.0,
            "stale_timeout_sec": 0.6,
            "unknown_policy": "keep",
        },
        "inflation_layer": {
            "plugin": "nav2_costmap_2d::InflationLayer",
            "cost_scaling_factor": 10.0,
            "inflation_radius": 0.55,
        },
        # Compatibility keys for nav2 parameter parsers that don't fully expand nested dicts.
        "obstacle_layer.observation_sources": "terrain_obstacles",
        "obstacle_layer.terrain_obstacles.topic": "/terrain_obstacles",
        "obstacle_layer.terrain_obstacles.data_type": "PointCloud2",
        "obstacle_layer.terrain_obstacles.marking": True,
        "obstacle_layer.terrain_obstacles.clearing": False,
        "obstacle_layer.terrain_obstacles.max_obstacle_height": 2.0,
        "obstacle_layer.terrain_obstacles.min_obstacle_height": -1.0,
        "obstacle_layer.terrain_obstacles.obstacle_max_range": 4.0,
        "obstacle_layer.terrain_obstacles.obstacle_min_range": 0.0,
        "obstacle_layer.terrain_obstacles.raytrace_max_range": 5.0,
        "obstacle_layer.terrain_obstacles.raytrace_min_range": 0.0,
        "drop_layer.observation_sources": "terrain_drop",
        "drop_layer.terrain_drop.topic": "/terrain_drop",
        "drop_layer.terrain_drop.data_type": "PointCloud2",
        "drop_layer.terrain_drop.marking": True,
        "drop_layer.terrain_drop.clearing": False,
        "drop_layer.terrain_drop.max_obstacle_height": 2.0,
        "drop_layer.terrain_drop.min_obstacle_height": -1.0,
        "drop_layer.terrain_drop.obstacle_max_range": 4.0,
        "drop_layer.terrain_drop.obstacle_min_range": 0.0,
        "drop_layer.terrain_drop.raytrace_max_range": 5.0,
        "drop_layer.terrain_drop.raytrace_min_range": 0.0,
        "terrain_traversability_layer.plugin": "rc26_terrain_nav2::TerrainTraversabilityLayer",
        "terrain_traversability_layer.enabled": True,
        "terrain_traversability_layer.terrain_grid_topic": "/terrain_grid_map_local",
        "terrain_traversability_layer.traversability_layer": "traversability",
        "terrain_traversability_layer.fresh_layer": "fresh",
        "terrain_traversability_layer.drop_layer": "drop_prob",
        "terrain_traversability_layer.climbable_layer": "climbable_prob",
        "terrain_traversability_layer.lethal_threshold": 0.25,
        "terrain_traversability_layer.inscribed_threshold": 0.45,
        "terrain_traversability_layer.drop_lethal_threshold": 0.8,
        "terrain_traversability_layer.climbable_soft_cost_max": 80.0,
        "terrain_traversability_layer.stale_timeout_sec": 0.6,
        "terrain_traversability_layer.unknown_policy": "keep",
    }

    local_costmap_node = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        namespace="costmap",
        name="costmap",
        output="screen",
        parameters=[local_costmap_overrides],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_local_costmap",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["costmap/costmap"],
                "bond_timeout": 0.0,
            }
        ],
    )

    return (
        launch.LaunchDescription(
            [
                static_tf_map_to_odom,
                static_tf_base_to_livox,
                mock_odom_scan,
                terrain_node,
                terrain_grid_map_bridge,
                local_costmap_node,
                lifecycle_manager,
                TimerAction(period=2.0, actions=[launch_testing.actions.ReadyToTest()]),
            ]
        ),
        {},
    )


class TestCostmapIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_full_data_chain_reasonable(self):
        node = rclpy.create_node("test_costmap_integration_probe")
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        odom_msgs = []
        registered_scan_msgs = []
        terrain_obstacles_msgs = []
        terrain_grid_msgs = []
        terrain_qos = QoSProfile(depth=10)
        terrain_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        terrain_qos.durability = DurabilityPolicy.VOLATILE
        scan_qos = QoSProfile(depth=50)
        scan_qos.reliability = ReliabilityPolicy.RELIABLE
        scan_qos.durability = DurabilityPolicy.VOLATILE

        def on_odom(msg: Odometry):
            odom_msgs.append(msg)

        def on_registered_scan(msg: PointCloud2):
            registered_scan_msgs.append(msg)

        def on_terrain_obstacles(msg: PointCloud2):
            terrain_obstacles_msgs.append(msg)

        def on_terrain_grid(msg: GridMap):
            terrain_grid_msgs.append(msg)

        sub_odom = node.create_subscription(Odometry, "/odom", on_odom, 20)
        sub_registered_scan = node.create_subscription(
            PointCloud2,
            "/registered_scan",
            on_registered_scan,
            scan_qos,
        )
        sub_terrain_obstacles = node.create_subscription(
            PointCloud2,
            "/terrain_obstacles",
            on_terrain_obstacles,
            terrain_qos,
        )
        terrain_grid_qos = QoSProfile(depth=1)
        terrain_grid_qos.reliability = ReliabilityPolicy.RELIABLE
        terrain_grid_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        sub_terrain_grid = node.create_subscription(
            GridMap,
            "/terrain_grid_map_local",
            on_terrain_grid,
            terrain_grid_qos,
        )

        costmap_client = node.create_client(GetCostmap, "/costmap/get_costmap")
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node, spin_thread=False)

        response_count = 0
        max_cost = 0
        max_intermediate_cost_cells = 0
        max_grid_fresh_cells = 0
        max_grid_valid_traversability_cells = 0
        max_grid_valid_fresh_traversability_cells = 0
        min_grid_traversability = 1.0
        max_grid_penalty = 0.0
        failed_calls = 0
        have_map_to_base = False
        have_map_to_livox = False
        have_base_to_livox = False

        deadline = time.time() + 35.0
        next_costmap_probe = 0.0
        try:
            while time.time() < deadline:
                _drain_executor(executor, 0.2)

                try:
                    tf_buffer.lookup_transform("base_link", "livox_frame", rclpy.time.Time())
                    have_base_to_livox = True
                except TransformException:
                    pass

                try:
                    tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
                    have_map_to_base = True
                except TransformException:
                    pass

                try:
                    tf_buffer.lookup_transform("map", "livox_frame", rclpy.time.Time())
                    have_map_to_livox = True
                except TransformException:
                    pass

                now_wall = time.time()
                if now_wall >= next_costmap_probe:
                    next_costmap_probe = now_wall + 0.4
                    if costmap_client.wait_for_service(timeout_sec=0.1):
                        future = costmap_client.call_async(GetCostmap.Request())
                        call_deadline = time.time() + 1.0
                        while time.time() < call_deadline and not future.done():
                            _drain_executor(executor, 0.05)

                        if not future.done():
                            failed_calls += 1
                        elif future.exception() is not None:
                            failed_calls += 1
                        else:
                            response = future.result()
                            if response is not None and response.map.data:
                                response_count += 1
                                max_cost = max(max_cost, max(response.map.data))
                                intermediate_cells = sum(
                                    1 for value in response.map.data if 0 < value < 253
                                )
                                max_intermediate_cost_cells = max(
                                    max_intermediate_cost_cells,
                                    intermediate_cells,
                                )
                            else:
                                failed_calls += 1

                have_traversability_grid = any(
                    "traversability" in msg.layers and "fresh" in msg.layers
                    for msg in terrain_grid_msgs
                )
                if terrain_grid_msgs:
                    latest_grid_stats = _grid_map_stats(terrain_grid_msgs[-1])
                    max_grid_fresh_cells = max(
                        max_grid_fresh_cells,
                        latest_grid_stats["fresh_cells"],
                    )
                    max_grid_valid_traversability_cells = max(
                        max_grid_valid_traversability_cells,
                        latest_grid_stats["valid_traversability_cells"],
                    )
                    max_grid_valid_fresh_traversability_cells = max(
                        max_grid_valid_fresh_traversability_cells,
                        latest_grid_stats["valid_fresh_traversability_cells"],
                    )
                    min_grid_traversability = min(
                        min_grid_traversability,
                        latest_grid_stats["min_traversability"],
                    )
                    max_grid_penalty = max(
                        max_grid_penalty,
                        latest_grid_stats["max_penalty"],
                    )
                if (
                    len(odom_msgs) >= 25
                    and len(registered_scan_msgs) >= 25
                    and have_traversability_grid
                    and response_count >= 3
                    and max_intermediate_cost_cells > 0
                    and max_grid_valid_fresh_traversability_cells > 0
                    and max_grid_penalty > 0.05
                    and have_base_to_livox
                    and have_map_to_base
                    and have_map_to_livox
                ):
                    break

            costmap_services = [
                f"{name}:{','.join(types)}"
                for name, types in node.get_service_names_and_types()
                if "costmap" in name
            ]
            have_traversability_grid = any(
                "traversability" in msg.layers and "fresh" in msg.layers
                for msg in terrain_grid_msgs
            )

            self.assertGreaterEqual(
                response_count,
                3,
                "35 秒内未成功调用至少 3 次 /costmap/get_costmap "
                f"(responses={response_count}, failed_calls={failed_calls}, "
                f"terrain_grid_msgs={len(terrain_grid_msgs)}, "
                f"services={costmap_services})",
            )

            self.assertGreaterEqual(
                len(odom_msgs),
                25,
                f"odom 消息不足: {len(odom_msgs)}",
            )
            self.assertGreaterEqual(
                len(registered_scan_msgs), 25, f"registered_scan 消息不足: {len(registered_scan_msgs)}",
            )
            self.assertGreaterEqual(
                len(terrain_grid_msgs),
                1,
                "未观察到 /terrain_grid_map_local 消息",
            )
            self.assertTrue(
                have_traversability_grid,
                "terrain grid 缺少 traversability/fresh layer",
            )
            self.assertGreater(
                max_grid_fresh_cells,
                0,
                "terrain grid 未产生 fresh cell",
            )
            self.assertGreater(
                max_grid_valid_fresh_traversability_cells,
                0,
                "terrain grid 未产生有效的 fresh traversability cell",
            )
            self.assertGreater(
                max_grid_penalty,
                0.05,
                "terrain grid 中 traversability 未出现可观测衰减 "
                f"(fresh_cells={max_grid_fresh_cells}, "
                f"valid_traversability_cells={max_grid_valid_traversability_cells}, "
                f"valid_fresh_traversability_cells={max_grid_valid_fresh_traversability_cells}, "
                f"min_traversability={min_grid_traversability:.3f}, "
                f"max_penalty={max_grid_penalty:.3f})",
            )

            odom_stamps = [_stamp_to_sec(msg.header.stamp) for msg in odom_msgs]
            scan_stamps = [_stamp_to_sec(msg.header.stamp) for msg in registered_scan_msgs]
            odom_rate = _rate_hz(odom_stamps)
            scan_rate = _rate_hz(scan_stamps)
            self.assertGreaterEqual(odom_rate, 9.0, f"/odom 频率过低: {odom_rate:.3f} Hz")
            self.assertGreaterEqual(scan_rate, 9.0, f"/registered_scan 频率过低: {scan_rate:.3f} Hz")

            max_skew = _max_nearest_skew_sec(odom_stamps, scan_stamps)
            self.assertLessEqual(
                max_skew,
                0.1,
                f"/odom 与 /registered_scan 时戳最大偏差过大: {max_skew:.4f} s",
            )

            non_monotonic_count = 0
            max_jump_speed = 0.0
            for prev, curr in zip(odom_msgs, odom_msgs[1:]):
                prev_t = _stamp_to_sec(prev.header.stamp)
                curr_t = _stamp_to_sec(curr.header.stamp)
                dt = curr_t - prev_t
                if dt <= 0.0:
                    non_monotonic_count += 1
                    continue

                dx = curr.pose.pose.position.x - prev.pose.pose.position.x
                dy = curr.pose.pose.position.y - prev.pose.pose.position.y
                dz = curr.pose.pose.position.z - prev.pose.pose.position.z
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                max_jump_speed = max(max_jump_speed, dist / dt)

            self.assertEqual(non_monotonic_count, 0, "odom 时间戳存在非递增帧")
            self.assertLess(
                max_jump_speed,
                2.0,
                f"odom 位姿跳变过大: max_speed={max_jump_speed:.3f} m/s",
            )

            self.assertTrue(have_base_to_livox, "TF 缺失: base_link -> livox_frame")
            self.assertTrue(have_map_to_base, "TF 缺失: map -> base_link")
            self.assertTrue(have_map_to_livox, "TF 缺失: map -> livox_frame")

            tf_base_to_livox = tf_buffer.lookup_transform("base_link", "livox_frame", rclpy.time.Time())
            tx = tf_base_to_livox.transform.translation.x
            ty = tf_base_to_livox.transform.translation.y
            tz = tf_base_to_livox.transform.translation.z
            qx = tf_base_to_livox.transform.rotation.x
            qy = tf_base_to_livox.transform.rotation.y
            qz = tf_base_to_livox.transform.rotation.z
            qw = tf_base_to_livox.transform.rotation.w

            self.assertAlmostEqual(tx, 0.0, delta=1e-3)
            self.assertAlmostEqual(ty, 0.0, delta=1e-3)
            self.assertAlmostEqual(tz, 0.13, delta=1e-3)
            self.assertAlmostEqual(qx, 0.0, delta=1e-3)
            self.assertAlmostEqual(qy, 0.0, delta=1e-3)
            self.assertAlmostEqual(qz, 0.0, delta=1e-3)
            self.assertAlmostEqual(qw, 1.0, delta=1e-3)

            frames_yaml = tf_buffer.all_frames_as_yaml()
            self.assertIn("map", frames_yaml)
            self.assertIn("odom", frames_yaml)
            self.assertIn("base_link", frames_yaml)
            self.assertIn("livox_frame", frames_yaml)
            self.assertNotIn("laser_link", frames_yaml)

            self.assertGreater(
                max_cost,
                0,
                "get_costmap 返回数据未出现非零代价值 "
                f"(fresh_cells={max_grid_fresh_cells}, "
                f"valid_fresh_traversability_cells={max_grid_valid_fresh_traversability_cells}, "
                f"min_traversability={min_grid_traversability:.3f}, "
                f"max_penalty={max_grid_penalty:.3f})",
            )
            self.assertGreater(
                max_intermediate_cost_cells,
                0,
                "costmap 内未观察到 terrain_traversability_layer 写入的连续代价值 "
                f"(max_cost={max_cost}, fresh_cells={max_grid_fresh_cells}, "
                f"valid_fresh_traversability_cells={max_grid_valid_fresh_traversability_cells}, "
                f"min_traversability={min_grid_traversability:.3f}, "
                f"max_penalty={max_grid_penalty:.3f})",
            )

            # Keep TF listener alive until teardown to avoid intermittent gc cleanup.
            self.assertIsNotNone(tf_listener)
        finally:
            _shutdown_lifecycle_manager(node, executor)
            node.destroy_subscription(sub_odom)
            node.destroy_subscription(sub_registered_scan)
            node.destroy_subscription(sub_terrain_obstacles)
            node.destroy_subscription(sub_terrain_grid)
            node.destroy_client(costmap_client)
            executor.remove_node(node)
            node.destroy_node()
