#!/usr/bin/env python3
"""Listen for upstream manual external-limit 3 feedback 0x13 and toggle r2_active_side.yaml."""

from __future__ import annotations

import argparse
import os
import re
import stat
import tempfile
from pathlib import Path
from typing import Any

import yaml


DEFAULT_FEEDBACK_TOPIC = "/mechanism/command_feedback"
DEFAULT_SWITCH_FEEDBACK_ID = 0x13


def _selector_side(data: dict[str, Any], *, source: Path) -> str:
    side = str(data.get("active_side", "")).strip().lower()
    if side not in {"red", "blue"}:
        raise ValueError(
            f"{source} active_side must be red or blue, got: {side!r}"
        )
    return side


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


def _fsync_directory(path: Path) -> None:
    flags = os.O_RDONLY
    if hasattr(os, "O_DIRECTORY"):
        flags |= os.O_DIRECTORY
    fd = os.open(path, flags)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _pending_path(path: Path) -> Path:
    return path.with_name(f".{path.name}.pending")


def _target_mode(path: Path) -> int:
    try:
        return stat.S_IMODE(path.stat().st_mode)
    except FileNotFoundError:
        return 0o644


def _write_staged_text(path: Path, text: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    staged_path = Path(tmp_name)
    try:
        os.fchmod(fd, _target_mode(path))
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            fd = -1
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        _fsync_directory(path.parent)
        return staged_path
    except Exception:
        if fd >= 0:
            os.close(fd)
        try:
            staged_path.unlink()
            _fsync_directory(path.parent)
        except FileNotFoundError:
            pass
        raise


def _prepare_pending_text(path: Path, text: str) -> Path:
    staged_path = _write_staged_text(path, text)
    pending_path = _pending_path(path)
    try:
        os.replace(staged_path, pending_path)
    except Exception:
        try:
            staged_path.unlink()
            _fsync_directory(path.parent)
        except FileNotFoundError:
            pass
        raise
    _fsync_directory(path.parent)
    return pending_path


def _install_pending_text(path: Path, pending_path: Path) -> None:
    staged_path = _write_staged_text(
        path, pending_path.read_text(encoding="utf-8")
    )
    try:
        os.replace(staged_path, path)
        _fsync_directory(path.parent)
    except Exception:
        try:
            staged_path.unlink()
            _fsync_directory(path.parent)
        except FileNotFoundError:
            pass
        raise

    pending_path.unlink()
    _fsync_directory(path.parent)


def _toggle_text(path: Path, text: str, current: str) -> tuple[str, str]:
    next_side = "blue" if current == "red" else "red"
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
        data = load_selector(path)
        data["active_side"] = next_side
        updated_text = yaml.safe_dump(
            data,
            allow_unicode=True,
            sort_keys=False,
            default_flow_style=False,
        )
    return next_side, updated_text


def _matches_current_or_one_toggle(
    current_data: dict[str, Any],
    candidate_data: dict[str, Any],
    *,
    source: Path,
) -> bool:
    current_side = _selector_side(current_data, source=source)
    candidate_side = _selector_side(candidate_data, source=source)
    if candidate_side == current_side:
        return candidate_data == current_data
    expected = dict(current_data)
    expected["active_side"] = "blue" if current_side == "red" else "red"
    return candidate_data == expected


def recover_interrupted_selector_write(path: Path) -> tuple[str, str] | None:
    path = path.expanduser().resolve()
    current_data = load_selector(path)
    current_side = _selector_side(current_data, source=path)
    pending_path = _pending_path(path)

    if pending_path.exists():
        pending_data = load_selector(pending_path)
        pending_side = _selector_side(pending_data, source=pending_path)
        if not _matches_current_or_one_toggle(
            current_data, pending_data, source=pending_path
        ):
            raise ValueError(
                f"{pending_path} does not match the current selector or one toggle"
            )
        _install_pending_text(path, pending_path)
        return current_side, pending_side

    target_mtime_ns = path.stat().st_mtime_ns
    staged_paths = sorted(
        path.parent.glob(f".{path.name}.*.tmp"),
        key=lambda candidate: candidate.stat().st_mtime_ns,
        reverse=True,
    )
    newer_staged_paths = [
        candidate
        for candidate in staged_paths
        if candidate.stat().st_mtime_ns >= target_mtime_ns
    ]
    if not newer_staged_paths:
        return None

    newest_staged = newer_staged_paths[0]
    try:
        staged_data = load_selector(newest_staged)
        staged_matches = _matches_current_or_one_toggle(
            current_data, staged_data, source=newest_staged
        )
    except (OSError, UnicodeError, yaml.YAMLError, ValueError):
        staged_data = None
        staged_matches = False

    if staged_data is not None and staged_matches:
        recovered_text = newest_staged.read_text(encoding="utf-8")
        recovered_side = _selector_side(staged_data, source=newest_staged)
    else:
        recovered_side, recovered_text = _toggle_text(
            path, path.read_text(encoding="utf-8"), current_side
        )

    pending_path = _prepare_pending_text(path, recovered_text)
    _install_pending_text(path, pending_path)

    removed = False
    for staged_path in staged_paths:
        try:
            staged_path.unlink()
            removed = True
        except FileNotFoundError:
            pass
    if removed:
        _fsync_directory(path.parent)
    return current_side, recovered_side


def atomic_write_selector(path: Path, data: dict[str, Any]) -> None:
    atomic_write_text(
        path,
        yaml.safe_dump(
            data,
            allow_unicode=True,
            sort_keys=False,
            default_flow_style=False,
        ),
    )


def atomic_write_text(path: Path, text: str) -> None:
    pending_path = _prepare_pending_text(path, text)
    _install_pending_text(path, pending_path)


def toggle_active_side_file(path: Path) -> tuple[str, str]:
    recover_interrupted_selector_write(path)
    data = load_selector(path)
    current = _selector_side(data, source=path)
    text = path.read_text(encoding="utf-8")
    next_side, updated_text = _toggle_text(path, text, current)
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
    parser.add_argument(
        "--recover-only",
        action="store_true",
        help="Recover an interrupted selector replacement and exit.",
    )
    return parser


class ActiveSideSwitchListener:
    def __init__(self, *, node: Any, args: argparse.Namespace) -> None:
        from rc26_interfaces.msg import MechanismTransportFeedback

        self.node = node
        self.side_config_file = Path(args.side_config_file).expanduser().resolve()
        self.switch_feedback_id = int(args.switch_feedback_id) & 0xFF
        self.once = bool(args.once)
        recovered = recover_interrupted_selector_write(self.side_config_file)
        if recovered is not None:
            old_side, new_side = recovered
            node.get_logger().warn(
                "recovered interrupted active-side replacement: %s -> %s"
                % (old_side, new_side)
            )
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
    if args.recover_only:
        path = Path(args.side_config_file).expanduser().resolve()
        recovered = recover_interrupted_selector_write(path)
        if recovered is not None:
            old_side, new_side = recovered
            print(
                "Recovered interrupted active-side replacement: "
                f"{old_side} -> {new_side}"
            )
        return

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
