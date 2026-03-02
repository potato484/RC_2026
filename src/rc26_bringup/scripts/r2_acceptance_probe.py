#!/usr/bin/env python3

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException, TransformListener


def parse_bool(value: str) -> bool:
    lowered = value.strip().lower()
    if lowered in ("1", "true", "t", "yes", "y", "on"):
        return True
    if lowered in ("0", "false", "f", "no", "n", "off"):
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {value}")


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def cloud_points(msg: PointCloud2) -> int:
    return int(msg.width) * int(msg.height)


def calc_rate_hz(stamps: List[float]) -> float:
    if len(stamps) < 2:
        return 0.0
    duration = stamps[-1] - stamps[0]
    if duration <= 0.0:
        return 0.0
    return (len(stamps) - 1) / duration


def max_nearest_skew_sec(ref_stamps: List[float], probe_stamps: List[float]) -> float:
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


def quaternion_to_euler_rpy(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


@dataclass
class Check:
    name: str
    ok: bool
    detail: str


class AcceptanceProbe:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("r2_acceptance_probe")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=False)

        sensor_qos = QoSProfile(depth=50)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        sensor_qos.durability = DurabilityPolicy.VOLATILE

        reliable_qos = QoSProfile(depth=50)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        reliable_qos.durability = DurabilityPolicy.VOLATILE

        self.odom_stamps: List[float] = []
        self.scan_stamps: List[float] = []
        self.odom_positions: List[Tuple[float, float, float]] = []
        self.odom_non_monotonic = 0
        self.max_jump_speed = 0.0

        self.odom_frame_mismatch = 0
        self.odom_child_mismatch = 0
        self.scan_frame_mismatch = 0

        self.terrain_obstacles_total = 0
        self.terrain_obstacles_non_empty = 0
        self.terrain_obstacles_max_points = 0
        self.terrain_drop_total = 0
        self.terrain_drop_non_empty = 0

        self.tf_msg_count = 0
        self.tf_msg_stamps: List[float] = []

        self.diag_terrain_total = 0
        self.diag_terrain_error = 0
        self.diag_last_values: Dict[str, str] = {}
        self.diag_last_message = ""
        self.diag_last_level = 0

        self.costmap_service_candidates = [self.args.costmap_service]
        for item in self.args.costmap_service_fallbacks.split(","):
            name = item.strip()
            if name and name not in self.costmap_service_candidates:
                self.costmap_service_candidates.append(name)

        self.costmap_clients = {
            service: self.node.create_client(GetCostmap, service)
            for service in self.costmap_service_candidates
        }
        self.active_costmap_service = self.args.costmap_service
        self.costmap_response_count = 0
        self.costmap_failed_calls = 0
        self.costmap_max_cost = 0

        self.tf_seen_map_odom = False
        self.tf_seen_odom_base = False
        self.tf_seen_map_base = False
        self.tf_seen_base_livox = False
        self.tf_seen_map_livox = False
        self.tf_base_livox: Optional[Tuple[float, float, float, float, float, float, float]] = None

        self.sub_odom = self.node.create_subscription(Odometry, "/odom", self._on_odom, 50)
        self.sub_registered_scan = self.node.create_subscription(
            PointCloud2, "/registered_scan", self._on_registered_scan, sensor_qos
        )
        self.sub_terrain_obstacles = self.node.create_subscription(
            PointCloud2, "/terrain_obstacles", self._on_terrain_obstacles, sensor_qos
        )
        self.sub_terrain_drop = self.node.create_subscription(
            PointCloud2, "/terrain_drop", self._on_terrain_drop, sensor_qos
        )
        self.sub_tf = self.node.create_subscription(TFMessage, "/tf", self._on_tf, sensor_qos)
        self.sub_diag = self.node.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diagnostics, reliable_qos
        )

    def destroy(self):
        self.node.destroy_subscription(self.sub_odom)
        self.node.destroy_subscription(self.sub_registered_scan)
        self.node.destroy_subscription(self.sub_terrain_obstacles)
        self.node.destroy_subscription(self.sub_terrain_drop)
        self.node.destroy_subscription(self.sub_tf)
        self.node.destroy_subscription(self.sub_diag)
        self.executor.remove_node(self.node)
        self.node.destroy_node()

    def _on_odom(self, msg: Odometry):
        stamp = stamp_to_sec(msg.header.stamp)
        if self.odom_stamps and stamp <= self.odom_stamps[-1]:
            self.odom_non_monotonic += 1

        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        if self.odom_stamps:
            dt = stamp - self.odom_stamps[-1]
            if dt > 0.0:
                prev = self.odom_positions[-1]
                dist = math.sqrt(
                    (pos[0] - prev[0]) ** 2 + (pos[1] - prev[1]) ** 2 + (pos[2] - prev[2]) ** 2
                )
                self.max_jump_speed = max(self.max_jump_speed, dist / dt)

        self.odom_stamps.append(stamp)
        self.odom_positions.append(pos)

        if msg.header.frame_id != self.args.expected_odom_frame:
            self.odom_frame_mismatch += 1
        if msg.child_frame_id != self.args.expected_base_frame:
            self.odom_child_mismatch += 1

    def _on_registered_scan(self, msg: PointCloud2):
        self.scan_stamps.append(stamp_to_sec(msg.header.stamp))
        if msg.header.frame_id != self.args.expected_scan_frame:
            self.scan_frame_mismatch += 1

    def _on_terrain_obstacles(self, msg: PointCloud2):
        self.terrain_obstacles_total += 1
        points = cloud_points(msg)
        if points > 0:
            self.terrain_obstacles_non_empty += 1
        self.terrain_obstacles_max_points = max(self.terrain_obstacles_max_points, points)

    def _on_terrain_drop(self, msg: PointCloud2):
        self.terrain_drop_total += 1
        points = cloud_points(msg)
        if points > 0:
            self.terrain_drop_non_empty += 1

    def _on_tf(self, msg: TFMessage):
        now = self.node.get_clock().now().nanoseconds * 1e-9
        self.tf_msg_stamps.append(now)
        self.tf_msg_count += len(msg.transforms)

    def _on_diagnostics(self, msg: DiagnosticArray):
        for status in msg.status:
            if self.args.diag_name_keyword not in status.name:
                continue
            self.diag_terrain_total += 1
            level_raw = status.level
            if isinstance(level_raw, (bytes, bytearray)):
                self.diag_last_level = int.from_bytes(level_raw, byteorder="little")
            else:
                self.diag_last_level = int(level_raw)
            self.diag_last_message = status.message
            self.diag_last_values = {item.key: item.value for item in status.values}
            if self.diag_last_level >= 2:
                self.diag_terrain_error += 1

    def _probe_tf(self):
        try:
            self.tf_buffer.lookup_transform(
                self.args.map_frame, self.args.odom_frame, rclpy.time.Time()
            )
            self.tf_seen_map_odom = True
        except TransformException:
            pass

        try:
            self.tf_buffer.lookup_transform(
                self.args.odom_frame, self.args.base_frame, rclpy.time.Time()
            )
            self.tf_seen_odom_base = True
        except TransformException:
            pass

        try:
            self.tf_buffer.lookup_transform(
                self.args.map_frame, self.args.base_frame, rclpy.time.Time()
            )
            self.tf_seen_map_base = True
        except TransformException:
            pass

        try:
            tf_base_livox = self.tf_buffer.lookup_transform(
                self.args.base_frame, self.args.lidar_frame, rclpy.time.Time()
            )
            self.tf_seen_base_livox = True
            t = tf_base_livox.transform.translation
            q = tf_base_livox.transform.rotation
            self.tf_base_livox = (t.x, t.y, t.z, q.x, q.y, q.z, q.w)
        except TransformException:
            pass

        try:
            self.tf_buffer.lookup_transform(
                self.args.map_frame, self.args.lidar_frame, rclpy.time.Time()
            )
            self.tf_seen_map_livox = True
        except TransformException:
            pass

    def _call_costmap_once(self):
        if not self.args.require_costmap:
            return

        selected_service = None
        selected_client = None
        for service in self.costmap_service_candidates:
            client = self.costmap_clients[service]
            if client.wait_for_service(timeout_sec=0.05):
                selected_service = service
                selected_client = client
                break

        if selected_client is None:
            self.costmap_failed_calls += 1
            return
        self.active_costmap_service = selected_service or self.active_costmap_service

        future = selected_client.call_async(GetCostmap.Request())
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline and not future.done():
            self.executor.spin_once(timeout_sec=0.05)

        if not future.done():
            self.costmap_failed_calls += 1
            return
        if future.exception() is not None:
            self.costmap_failed_calls += 1
            return

        response = future.result()
        if response is None or not response.map.data:
            self.costmap_failed_calls += 1
            return

        self.costmap_response_count += 1
        self.costmap_max_cost = max(self.costmap_max_cost, max(response.map.data))

    def run(self) -> List[Check]:
        start = time.monotonic()
        end = start + self.args.duration_sec
        next_costmap_probe = start + self.args.warmup_sec

        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.1)
            self._probe_tf()

            now = time.monotonic()
            if self.args.require_costmap and now >= next_costmap_probe:
                self._call_costmap_once()
                next_costmap_probe = now + self.args.costmap_probe_interval_sec

        checks = self._build_checks()
        return checks

    def _build_checks(self) -> List[Check]:
        checks: List[Check] = []

        odom_rate = calc_rate_hz(self.odom_stamps)
        scan_rate = calc_rate_hz(self.scan_stamps)
        tf_rate = calc_rate_hz(self.tf_msg_stamps)
        skew = max_nearest_skew_sec(self.odom_stamps, self.scan_stamps)

        checks.append(
            Check(
                name="TopicRate/odom",
                ok=odom_rate >= self.args.min_topic_hz,
                detail=f"rate={odom_rate:.3f}Hz threshold>={self.args.min_topic_hz:.3f}",
            )
        )
        checks.append(
            Check(
                name="TopicRate/registered_scan",
                ok=scan_rate >= self.args.min_topic_hz,
                detail=f"rate={scan_rate:.3f}Hz threshold>={self.args.min_topic_hz:.3f}",
            )
        )
        checks.append(
            Check(
                name="TopicRate/tf",
                ok=tf_rate >= self.args.min_tf_hz,
                detail=f"rate={tf_rate:.3f}Hz threshold>={self.args.min_tf_hz:.3f}",
            )
        )
        checks.append(
            Check(
                name="StampSkew/odom_scan",
                ok=skew <= self.args.max_stamp_skew_sec,
                detail=f"max_skew={skew:.4f}s threshold<={self.args.max_stamp_skew_sec:.4f}s",
            )
        )
        checks.append(
            Check(
                name="OdomContinuity/monotonic",
                ok=self.odom_non_monotonic == 0,
                detail=f"non_monotonic_frames={self.odom_non_monotonic}",
            )
        )
        checks.append(
            Check(
                name="OdomContinuity/jump_speed",
                ok=self.max_jump_speed <= self.args.max_odom_jump_speed_mps,
                detail=(
                    f"max_jump_speed={self.max_jump_speed:.3f}mps "
                    f"threshold<={self.args.max_odom_jump_speed_mps:.3f}mps"
                ),
            )
        )
        checks.append(
            Check(
                name="FrameID/odom",
                ok=self.odom_frame_mismatch == 0,
                detail=f"mismatch_count={self.odom_frame_mismatch} expected={self.args.expected_odom_frame}",
            )
        )
        checks.append(
            Check(
                name="FrameID/odom_child",
                ok=self.odom_child_mismatch == 0,
                detail=f"mismatch_count={self.odom_child_mismatch} expected={self.args.expected_base_frame}",
            )
        )
        checks.append(
            Check(
                name="FrameID/registered_scan",
                ok=self.scan_frame_mismatch == 0,
                detail=f"mismatch_count={self.scan_frame_mismatch} expected={self.args.expected_scan_frame}",
            )
        )

        checks.append(
            Check(
                name="TF/map->odom",
                ok=(self.tf_seen_map_odom if self.args.require_map_chain else True),
                detail=f"seen={self.tf_seen_map_odom} require_map_chain={self.args.require_map_chain}",
            )
        )
        checks.append(
            Check(
                name="TF/odom->base_link",
                ok=self.tf_seen_odom_base,
                detail=f"seen={self.tf_seen_odom_base}",
            )
        )
        checks.append(
            Check(
                name="TF/base_link->livox_frame",
                ok=self.tf_seen_base_livox,
                detail=f"seen={self.tf_seen_base_livox}",
            )
        )
        checks.append(
            Check(
                name="TF/map->base_link",
                ok=(self.tf_seen_map_base if self.args.require_map_chain else True),
                detail=f"seen={self.tf_seen_map_base} require_map_chain={self.args.require_map_chain}",
            )
        )
        checks.append(
            Check(
                name="TF/map->livox_frame",
                ok=(self.tf_seen_map_livox if self.args.require_map_chain else True),
                detail=f"seen={self.tf_seen_map_livox} require_map_chain={self.args.require_map_chain}",
            )
        )

        if self.tf_base_livox is None:
            checks.append(
                Check(
                    name="TFExtrinsic/base_livox_numeric",
                    ok=False,
                    detail="base_link->livox_frame transform not available",
                )
            )
        else:
            tx, ty, tz, qx, qy, qz, qw = self.tf_base_livox
            roll, pitch, yaw = quaternion_to_euler_rpy(qx, qy, qz, qw)
            ok = (
                abs(tx - self.args.expected_lidar_x) <= self.args.expected_lidar_xyz_tol
                and abs(ty - self.args.expected_lidar_y) <= self.args.expected_lidar_xyz_tol
                and abs(tz - self.args.expected_lidar_z) <= self.args.expected_lidar_xyz_tol
                and abs(roll) <= self.args.expected_lidar_rpy_tol_rad
                and abs(pitch) <= self.args.expected_lidar_rpy_tol_rad
                and abs(yaw) <= self.args.expected_lidar_rpy_tol_rad
            )
            checks.append(
                Check(
                    name="TFExtrinsic/base_livox_numeric",
                    ok=ok,
                    detail=(
                        f"xyz=({tx:.4f},{ty:.4f},{tz:.4f}) rpy=({roll:.4f},{pitch:.4f},{yaw:.4f}) "
                        f"expected_z={self.args.expected_lidar_z:.4f}"
                    ),
                )
            )

        if self.args.expect_obstacle:
            checks.append(
                Check(
                    name="Terrain/obstacles_non_empty",
                    ok=self.terrain_obstacles_non_empty >= self.args.min_non_empty_obstacle_msgs,
                    detail=(
                        f"non_empty={self.terrain_obstacles_non_empty} "
                        f"threshold>={self.args.min_non_empty_obstacle_msgs} "
                        f"total={self.terrain_obstacles_total} max_points={self.terrain_obstacles_max_points}"
                    ),
                )
            )
            if self.args.require_costmap:
                checks.append(
                    Check(
                        name="Costmap/non_zero_cost",
                        ok=self.costmap_max_cost > 0,
                        detail=(
                            f"max_cost={self.costmap_max_cost} responses={self.costmap_response_count} "
                            f"failed_calls={self.costmap_failed_calls} service={self.active_costmap_service}"
                        ),
                    )
                )
            else:
                checks.append(
                    Check(
                        name="Costmap/non_zero_cost",
                        ok=True,
                        detail="SKIPPED (require_costmap=false)",
                    )
                )
        else:
            checks.append(
                Check(
                    name="Terrain/topic_alive",
                    ok=self.terrain_obstacles_total > 0,
                    detail=f"obstacles_total={self.terrain_obstacles_total}",
                )
            )
            checks.append(
                Check(
                    name="Costmap/service_alive",
                    ok=(self.costmap_response_count > 0 if self.args.require_costmap else True),
                    detail=(
                        f"responses={self.costmap_response_count} failed_calls={self.costmap_failed_calls} "
                        f"service={self.active_costmap_service}"
                    ),
                )
            )

        if self.args.require_diagnostics:
            checks.append(
                Check(
                    name="Diagnostics/terrain_present",
                    ok=self.diag_terrain_total > 0,
                    detail=f"diag_count={self.diag_terrain_total} keyword={self.args.diag_name_keyword}",
                )
            )
            checks.append(
                Check(
                    name="Diagnostics/terrain_no_error",
                    ok=self.diag_terrain_error == 0,
                    detail=f"error_count={self.diag_terrain_error} last_level={self.diag_last_level}",
                )
            )
            checks.extend(self._build_diagnostic_age_checks())

        return checks

    def _build_diagnostic_age_checks(self) -> List[Check]:
        checks: List[Check] = []
        for key in ("cloud_age_sec", "odom_age_sec", "tf_age_sec"):
            raw = self.diag_last_values.get(key)
            if raw is None:
                checks.append(Check(name=f"Diagnostics/{key}", ok=False, detail="missing"))
                continue
            try:
                val = float(raw)
            except ValueError:
                checks.append(Check(name=f"Diagnostics/{key}", ok=False, detail=f"invalid={raw}"))
                continue
            ok = math.isfinite(val) and val <= self.args.max_diag_age_sec
            checks.append(
                Check(
                    name=f"Diagnostics/{key}",
                    ok=ok,
                    detail=f"value={val:.4f} threshold<={self.args.max_diag_age_sec:.4f}",
                )
            )
        return checks


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="R2 全链路验收探针（TF/时戳/频率/costmap）")
    parser.add_argument("--duration_sec", type=float, default=60.0, help="采样总时长")
    parser.add_argument("--warmup_sec", type=float, default=3.0, help="启动后预热时长")

    parser.add_argument("--min_topic_hz", type=float, default=9.0, help="/odom /registered_scan 最低频率")
    parser.add_argument("--min_tf_hz", type=float, default=8.0, help="/tf 最低频率")
    parser.add_argument("--max_stamp_skew_sec", type=float, default=0.1, help="odom 与 scan 最大时戳偏差")
    parser.add_argument("--max_odom_jump_speed_mps", type=float, default=2.0, help="odom 最大位姿跳变速度")

    parser.add_argument("--require_map_chain", type=parse_bool, default=True, help="是否强制要求 map 链路")
    parser.add_argument("--map_frame", default="map")
    parser.add_argument("--odom_frame", default="odom")
    parser.add_argument("--base_frame", default="base_link")
    parser.add_argument("--lidar_frame", default="livox_frame")

    parser.add_argument("--expected_odom_frame", default="odom")
    parser.add_argument("--expected_base_frame", default="base_link")
    parser.add_argument("--expected_scan_frame", default="odom")

    parser.add_argument("--expected_lidar_x", type=float, default=0.0)
    parser.add_argument("--expected_lidar_y", type=float, default=0.0)
    parser.add_argument("--expected_lidar_z", type=float, default=0.13)
    parser.add_argument("--expected_lidar_xyz_tol", type=float, default=0.02)
    parser.add_argument("--expected_lidar_rpy_tol_rad", type=float, default=0.05)

    parser.add_argument("--require_costmap", type=parse_bool, default=True, help="是否检查 costmap 服务")
    parser.add_argument("--costmap_service", default="/local_costmap/get_costmap")
    parser.add_argument(
        "--costmap_service_fallbacks",
        default="/local_costmap/local_costmap/get_costmap,/costmap/get_costmap",
        help="逗号分隔的 costmap 服务回退列表",
    )
    parser.add_argument("--costmap_probe_interval_sec", type=float, default=0.5)

    parser.add_argument("--expect_obstacle", type=parse_bool, default=False, help="是否要求出现障碍非零代价")
    parser.add_argument(
        "--min_non_empty_obstacle_msgs",
        type=int,
        default=3,
        help="expect_obstacle=true 时障碍点云最小非空消息数",
    )

    parser.add_argument("--require_diagnostics", type=parse_bool, default=True, help="是否检查 terrain diagnostics")
    parser.add_argument("--diag_name_keyword", default="terrain_semantic")
    parser.add_argument("--max_diag_age_sec", type=float, default=1.2)
    return parser


def print_report(checks: List[Check], args):
    print("=== R2 Acceptance Probe Report ===")
    print(
        "config: "
        f"duration={args.duration_sec:.1f}s "
        f"require_map_chain={args.require_map_chain} "
        f"require_costmap={args.require_costmap} "
        f"expect_obstacle={args.expect_obstacle}"
    )
    for check in checks:
        status = "PASS" if check.ok else "FAIL"
        print(f"[{status}] {check.name}: {check.detail}")


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    rclpy.init()
    probe = AcceptanceProbe(args)
    try:
        checks = probe.run()
        print_report(checks, args)
        failed = [c for c in checks if not c.ok]
        if failed:
            print(f"RESULT: FAIL ({len(failed)} checks failed)")
            return 2
        print("RESULT: PASS")
        return 0
    finally:
        probe.destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
