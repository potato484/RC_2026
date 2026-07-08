#!/usr/bin/env python3
"""Listen for upstream manual external-limit 3 feedback 0x13 and toggle r2_active_side.yaml."""

from __future__ import annotations

import argparse
import os
import re
import tempfile
from pathlib import Path
from typing import Any

import yaml


DEFAULT_FEEDBACK_TOPIC = "/mechanism/command_feedback"
DEFAULT_SWITCH_FEEDBACK_ID = 0x13


def parse_feedback_id(text: str) -> int:
    value = int(str(text).strip(), 0)
    if value < 0 or value > 0xFF:
        raise argparse.ArgumentTypeError(
            f"switch feedback id must be in 0..255, got: {text}"
        )
    return value


def load_selector(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return data


def atomic_write_selector(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            yaml.safe_dump(
                data,
                f,
                allow_unicode=True,
                sort_keys=False,
                default_flow_style=False,
            )
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp_name, path)
    except Exception:
        try:
            os.unlink(tmp_name)
        except FileNotFoundError:
            pass
        raise


def atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp_name, path)
    except Exception:
        try:
            os.unlink(tmp_name)
        except FileNotFoundError:
            pass
        raise


def toggle_active_side_file(path: Path) -> tuple[str, str]:
    data = load_selector(path)
    current = str(data.get("active_side", "")).strip().lower()
    if current == "red":
        next_side = "blue"
    elif current == "blue":
        next_side = "red"
    else:
        raise ValueError(
            f"{path} active_side must be red or blue before toggling, got: {current!r}"
        )
    text = path.read_text(encoding="utf-8")
    pattern = re.compile(
        r"^(?P<prefix>\s*active_side\s*:\s*)"
        r"(?P<value>[^#\s]+)"
        r"(?P<suffix>\s*(?:#.*)?)$",
        re.MULTILINE,
    )
    updated_text, count = pattern.subn(
        lambda m: f"{m.group('prefix')}{next_side}{m.group('suffix')}",
        text,
        count=1,
    )
    if count != 1:
        data["active_side"] = next_side
        atomic_write_selector(path, data)
    else:
        atomic_write_text(path, updated_text)
    return current, next_side


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Toggle r2_active_side.yaml when upstream manual external-limit 3 "
            "feedback 0x13 is received."
        )
    )
    parser.add_argument(
        "--side-config-file",
        required=True,
        help="Path to r2_active_side.yaml to update.",
    )
    parser.add_argument(
        "--feedback-topic",
        default=DEFAULT_FEEDBACK_TOPIC,
        help=f"Mechanism feedback topic, default: {DEFAULT_FEEDBACK_TOPIC}",
    )
    parser.add_argument(
        "--switch-feedback-id",
        type=parse_feedback_id,
        default=DEFAULT_SWITCH_FEEDBACK_ID,
        help=(
            "Upstream manual external-limit feedback id that toggles active_side, "
            "default: 0x13."
        ),
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Exit after the first successful toggle; useful for tests.",
    )
    return parser


class ActiveSideSwitchListener:
    def __init__(self, *, node: Any, args: argparse.Namespace) -> None:
        from rc26_interfaces.msg import MechanismTransportFeedback

        self.node = node
        self.side_config_file = Path(args.side_config_file).expanduser().resolve()
        self.switch_feedback_id = int(args.switch_feedback_id) & 0xFF
        self.once = bool(args.once)
        self.subscription = node.create_subscription(
            MechanismTransportFeedback,
            args.feedback_topic,
            self.handle_feedback,
            32,
        )
        node.get_logger().info(
            "0x13 manual external-limit active-side switch listener started: topic=%s feedback=0x%02X file=%s"
            % (args.feedback_topic, self.switch_feedback_id, self.side_config_file)
        )

    def handle_feedback(self, msg: Any) -> None:
        feedback_id = int(getattr(msg, "feedback_id", -1)) & 0xFF
        if feedback_id != self.switch_feedback_id:
            return
        seq = int(getattr(msg, "seq", 0)) & 0xFF
        try:
            old_side, new_side = toggle_active_side_file(self.side_config_file)
        except Exception as exc:
            self.node.get_logger().error(
                "failed to toggle active_side after manual external-limit feedback=0x%02X seq=%u: %s"
                % (feedback_id, seq, exc)
            )
            return
        self.node.get_logger().warn(
            "received manual external-limit feedback=0x%02X seq=%u, active_side switched: %s -> %s"
            % (feedback_id, seq, old_side, new_side)
        )
        if self.once:
            import rclpy

            rclpy.shutdown()


def main() -> None:
    args = build_parser().parse_args()
    import rclpy

    rclpy.init()
    node = rclpy.create_node("active_side_switch_listener")
    ActiveSideSwitchListener(node=node, args=args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
