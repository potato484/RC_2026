#!/usr/bin/env python3

import argparse
import datetime as dt
import math
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


QUIT_WORDS = {"q", "quit", "exit"}


@dataclass
class NavPoint:
    name: str
    x: float
    y: float
    z: float
    yaw_rad: float
    yaw_deg: float
    stamp_sec: float


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-9:
        raise ValueError("四元数长度为 0，无法计算 yaw")

    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def default_output_path() -> Path:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path.cwd() / f"nav_points_{stamp}.txt"


def normalize_name(raw_name: str, fallback: str) -> str:
    name = raw_name.strip() or fallback
    name = re.sub(r"\s+", "_", name)
    return name


def xml_comment_safe(value: str) -> str:
    return value.replace("--", "- -").replace("<", "(").replace(">", ")")


def format_float(value: float, precision: int) -> str:
    return f"{value:.{precision}f}"


def render_points(points: List[NavPoint], frame_id: str, base_frame: str, precision: int) -> str:
    generated_at = dt.datetime.now().isoformat(timespec="seconds")
    lines = [
        "# RC26 Nav2 导航点采集结果",
        f"# 坐标系: {frame_id}",
        f"# 机器人基座: {base_frame}",
        f"# 生成时间: {generated_at}",
        "# 用法: 将下面的 NavToPose XML 行复制到 rc26_decision 的行为树中。",
        "",
        "# 点位表",
        "# 名称 x y yaw_rad yaw_deg z stamp_sec",
    ]

    for point in points:
        lines.append(
            " ".join(
                [
                    point.name,
                    format_float(point.x, precision),
                    format_float(point.y, precision),
                    format_float(point.yaw_rad, precision),
                    f"{point.yaw_deg:.2f}",
                    format_float(point.z, precision),
                    f"{point.stamp_sec:.4f}",
                ]
            )
        )

    lines.extend(["", "# BehaviorTree.CPP NavToPose XML"])
    for point in points:
        safe_name = xml_comment_safe(point.name)
        lines.append(
            f"<!-- {safe_name}，朝向={point.yaw_deg:.2f}deg，采集时间={point.stamp_sec:.4f} -->"
        )
        lines.append(
            f'<NavToPose x="{format_float(point.x, precision)}" '
            f'y="{format_float(point.y, precision)}" '
            f'yaw="{format_float(point.yaw_rad, precision)}" '
            f'frame_id="{frame_id}"/>'
        )

    lines.append("")
    return "\n".join(lines)


def write_points(output_path: Path, points: List[NavPoint], frame_id: str, base_frame: str, precision: int) -> None:
    output_path.write_text(render_points(points, frame_id, base_frame, precision), encoding="utf-8")


def wait_for_transform(tf_buffer: Buffer, frame_id: str, base_frame: str) -> bool:
    print(f"等待 TF 就绪: {frame_id} -> {base_frame}")
    last_report = 0.0
    while rclpy.ok():
        if tf_buffer.can_transform(frame_id, base_frame, Time(), timeout=Duration(seconds=0.2)):
            print("TF 已就绪，可以开始采点。")
            return True

        now = time.monotonic()
        if now - last_report >= 2.0:
            print("还在等待 TF；请确认定位链和 odom_interface 已启动。")
            last_report = now
    return False


def capture_point(tf_buffer: Buffer, frame_id: str, base_frame: str, timeout_sec: float, name: str) -> NavPoint:
    transform = tf_buffer.lookup_transform(
        frame_id,
        base_frame,
        Time(),
        timeout=Duration(seconds=timeout_sec),
    )
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    yaw = quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
    return NavPoint(
        name=name,
        x=translation.x,
        y=translation.y,
        z=translation.z,
        yaw_rad=yaw,
        yaw_deg=math.degrees(yaw),
        stamp_sec=stamp_to_sec(transform.header.stamp),
    )


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("必须大于 0")
    return parsed


def non_negative_int(value: str) -> int:
    parsed = int(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("不能小于 0")
    return parsed


def parse_args():
    parser = argparse.ArgumentParser(
        description="采集当前 map -> base_footprint 位姿，生成可复制到 NavToPose 的导航点 txt。"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=default_output_path(),
        help="输出 .txt 文件路径；默认在当前目录生成 nav_points_时间戳.txt",
    )
    parser.add_argument("--frame-id", default="map", help="导航目标所在坐标系，默认 map")
    parser.add_argument("--base-frame", default="base_footprint", help="机器人导航基座，默认 base_footprint")
    parser.add_argument("--name-prefix", default="wp", help="自动点名前缀，默认 wp")
    parser.add_argument("--precision", type=non_negative_int, default=4, help="x/y/yaw 小数位数，默认 4")
    parser.add_argument(
        "--lookup-timeout-sec",
        type=positive_float,
        default=1.0,
        help="每次采点等待 TF 的最长时间，默认 1.0 秒",
    )
    parser.add_argument("--overwrite", action="store_true", help="允许覆盖已有输出文件")
    return parser.parse_known_args()


def main() -> int:
    args, ros_args = parse_args()
    output_path = args.output.expanduser().resolve()

    if output_path.exists() and not args.overwrite:
        print(f"输出文件已存在，未覆盖: {output_path}", file=sys.stderr)
        print("如需覆盖，请追加 --overwrite。", file=sys.stderr)
        return 2
    if not output_path.parent.exists():
        print(f"输出目录不存在: {output_path.parent}", file=sys.stderr)
        return 2

    rclpy.init(args=ros_args)
    node = rclpy.create_node("capture_nav_points")
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node, spin_thread=True)
    points: List[NavPoint] = []

    try:
        if not wait_for_transform(tf_buffer, args.frame_id, args.base_frame):
            return 1

        print(f"输出文件: {output_path}")
        print("操作: 直接按 Enter 自动命名并采点；输入点名后按 Enter 使用该名称；输入 q/quit/exit 结束。")

        while rclpy.ok():
            try:
                raw_name = input("> ")
            except EOFError:
                print("\n收到 EOF，结束采点。")
                break

            if raw_name.strip().lower() in QUIT_WORDS:
                print("结束采点。")
                break

            fallback_name = f"{args.name_prefix}{len(points) + 1:03d}"
            point_name = normalize_name(raw_name, fallback_name)

            try:
                point = capture_point(
                    tf_buffer,
                    args.frame_id,
                    args.base_frame,
                    args.lookup_timeout_sec,
                    point_name,
                )
            except (TransformException, ValueError) as exc:
                print(f"本次未采点: 无法读取 {args.frame_id} -> {args.base_frame}: {exc}")
                continue

            points.append(point)
            write_points(output_path, points, args.frame_id, args.base_frame, args.precision)
            print(
                f"已保存 {point.name}: "
                f"x={format_float(point.x, args.precision)}, "
                f"y={format_float(point.y, args.precision)}, "
                f"yaw={format_float(point.yaw_rad, args.precision)}rad "
                f"({point.yaw_deg:.2f}deg)"
            )

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，结束采点。")
    finally:
        del tf_listener
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if points:
        print(f"共采集 {len(points)} 个导航点，结果已写入: {output_path}")
    else:
        print("未采集到导航点，未生成结果内容。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
