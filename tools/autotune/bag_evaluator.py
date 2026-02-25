#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import re
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent

import sys

if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import artifacts  # noqa: E402
import ros_utils  # noqa: E402


def _percentile95(values: List[float]) -> float:
    if not values:
        return 0.0
    data = sorted(values)
    idx = 0.95 * (len(data) - 1)
    i0 = int(math.floor(idx))
    i1 = int(math.ceil(idx))
    w = idx - i0
    return data[i0] * (1.0 - w) + data[i1] * w


def _to_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _load_params_yaml(path: Optional[Path]) -> Dict[str, Any]:
    if not path or not path.exists():
        return {}
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if isinstance(raw, dict) and isinstance(raw.get("params"), dict):
        return dict(raw["params"])
    if isinstance(raw, dict):
        return {k: v for k, v in raw.items() if isinstance(k, str)}
    return {}


def _extract_rmse(text: str) -> Optional[float]:
    patterns = [
        r"rmse\s+([0-9eE+\-.]+)",
        r"RMSE[:\s]+([0-9eE+\-.]+)",
    ]
    for pat in patterns:
        m = re.search(pat, text)
        if m:
            try:
                return float(m.group(1))
            except ValueError:
                pass
    return None


def _run_evo_ape(ref_tum: Path, est_tum: Path) -> Optional[float]:
    code, out, err = ros_utils.run_command(
        ["evo_ape", "tum", str(ref_tum), str(est_tum), "-a", "--rmse"],
        timeout_sec=60.0,
    )
    if code != 0:
        return None
    return _extract_rmse(out + "\n" + err)


def _run_evo_rpe(ref_tum: Path, est_tum: Path) -> Optional[float]:
    code, out, err = ros_utils.run_command(
        ["evo_rpe", "tum", str(ref_tum), str(est_tum), "-a", "--rmse"],
        timeout_sec=60.0,
    )
    if code != 0:
        return None
    return _extract_rmse(out + "\n" + err)


def _bag_topics(bag_path: Path) -> List[str]:
    code, out, _ = ros_utils.run_command(["ros2", "bag", "info", str(bag_path)], timeout_sec=20.0)
    if code != 0:
        return []
    topics = []
    for line in out.splitlines():
        m = re.search(r"Topic:\s+(\S+)", line)
        if m:
            topics.append(m.group(1))
    return sorted(set(topics))


def _build_bag_play_cmd(bag_path: Path) -> List[str]:
    cmd = ["ros2", "bag", "play", str(bag_path), "--clock", "--read-ahead-queue-size", "2000"]
    topics = _bag_topics(bag_path)
    if "/tf" in topics:
        kept = [t for t in topics if t != "/tf"]
        if kept:
            cmd.extend(["--topics", *kept])
    return cmd


def _compute_jerk_rms(cmd_samples: List[Tuple[float, float, float, float]]) -> Optional[float]:
    if len(cmd_samples) < 4:
        return None
    v = []
    for t, vx, vy, wz in cmd_samples:
        v.append((t, math.sqrt(vx * vx + vy * vy + wz * wz)))

    accel: List[Tuple[float, float]] = []
    for i in range(1, len(v)):
        dt = v[i][0] - v[i - 1][0]
        if dt <= 1e-6:
            continue
        a = (v[i][1] - v[i - 1][1]) / dt
        accel.append((v[i][0], a))
    if len(accel) < 3:
        return None

    jerks = []
    for i in range(1, len(accel)):
        dt = accel[i][0] - accel[i - 1][0]
        if dt <= 1e-6:
            continue
        j = (accel[i][1] - accel[i - 1][1]) / dt
        jerks.append(j)
    if not jerks:
        return None
    return math.sqrt(sum(x * x for x in jerks) / len(jerks))


def _aggregate_health(cards: List[Dict[str, Any]]) -> Dict[str, Any]:
    if not cards:
        return {}

    def p95(key: str) -> float:
        return _percentile95([_to_float(c.get(key), 0.0) for c in cards])

    def avg(key: str) -> float:
        vals = [_to_float(c.get(key), 0.0) for c in cards]
        return (sum(vals) / len(vals)) if vals else 0.0

    latest = cards[-1]
    return {
        "stamp_sec": _to_float(latest.get("stamp_sec"), 0.0),
        "norm_err_p95": p95("norm_err_p95"),
        "dt_ms_p95": p95("dt_ms_p95"),
        "inliers_min": int(min((_to_float(c.get("inliers_min"), 0) for c in cards), default=0)),
        "tracking_ratio": max(0.0, min(1.0, avg("tracking_ratio"))),
        "s1_spike_per_sec": int(round(p95("s1_spike_per_sec"))),
        "s2_reject_count": int(round(p95("s2_reject_count"))),
        "s2_eig_min": p95("s2_eig_min"),
        "s2_consec_max": int(round(p95("s2_consec_max"))),
        "reloc_total": int(round(max((_to_float(c.get("reloc_total"), 0) for c in cards), default=0))),
        "reloc_success_rate": max(0.0, min(1.0, avg("reloc_success_rate"))),
        "ghosting_score": p95("ghosting_score"),
        "map_entropy": p95("map_entropy"),
        "z_drift_slope": p95("z_drift_slope"),
        "wall_thickness_sigma": p95("wall_thickness_sigma"),
        "any_guardrail_triggered": any(bool(c.get("any_guardrail_triggered", False)) for c in cards),
        "shadow_timeout": any(bool(c.get("shadow_timeout", False)) for c in cards),
    }


def _write_tum(path: Path, tf_samples: List[Tuple[float, float, float, float, float, float, float, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    for s in tf_samples:
        t, x, y, z, qx, qy, qz, qw = s
        lines.append(f"{t:.9f} {x:.9f} {y:.9f} {z:.9f} {qx:.9f} {qy:.9f} {qz:.9f} {qw:.9f}")
    path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")


@dataclass
class CollectorRuntime:
    stop_event: threading.Event
    thread: threading.Thread
    executor: Any
    node: Any
    rclpy_module: Any


def _start_collector(base_frame: str) -> Tuple[Optional[CollectorRuntime], str]:
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
    except Exception as ex:
        return None, f"collector import failed: {ex}"

    class RuntimeCollector(Node):
        def __init__(self, frame: str) -> None:
            super().__init__(f"autotune_bag_collector_{int(time.time())}")
            self.base_frame = frame
            self.health_cards: List[Dict[str, Any]] = []
            self.cmd_samples: List[Tuple[float, float, float, float]] = []
            self.tf_samples: List[Tuple[float, float, float, float, float, float, float, float]] = []
            self.latest_tf: Optional[Tuple[float, float, float, float, float, float, float, float]] = None
            self.last_tf_emit: float = -1.0
            self._lock = threading.Lock()

            self.create_subscription(String, "//health_card", self._on_health, 50)
            self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 200)
            self.create_subscription(TFMessage, "/tf", self._on_tf, 200)
            self.create_timer(0.1, self._on_timer)

        def _on_health(self, msg: Any) -> None:
            try:
                payload = json.loads(msg.data)
                if isinstance(payload, dict):
                    with self._lock:
                        self.health_cards.append(payload)
            except Exception:
                return

        def _on_cmd(self, msg: Any) -> None:
            now_sec = time.time()
            with self._lock:
                self.cmd_samples.append((now_sec, float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)))

        def _on_tf(self, msg: Any) -> None:
            for tf in msg.transforms:
                if tf.header.frame_id != "map":
                    continue
                if tf.child_frame_id != self.base_frame:
                    continue
                t = float(tf.header.stamp.sec) + float(tf.header.stamp.nanosec) * 1e-9
                tr = tf.transform.translation
                rq = tf.transform.rotation
                sample = (t, float(tr.x), float(tr.y), float(tr.z), float(rq.x), float(rq.y), float(rq.z), float(rq.w))
                with self._lock:
                    self.latest_tf = sample

        def _on_timer(self) -> None:
            with self._lock:
                if self.latest_tf is None:
                    return
                if self.latest_tf[0] <= self.last_tf_emit + 1e-6:
                    return
                self.tf_samples.append(self.latest_tf)
                self.last_tf_emit = self.latest_tf[0]

    rclpy.init(args=None)
    node = RuntimeCollector(base_frame)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    stop_event = threading.Event()

    def _spin() -> None:
        while rclpy.ok() and not stop_event.is_set():
            executor.spin_once(timeout_sec=0.1)

    thread = threading.Thread(target=_spin, daemon=True)
    thread.start()
    return CollectorRuntime(stop_event=stop_event, thread=thread, executor=executor, node=node, rclpy_module=rclpy), ""


def _stop_collector(runtime: Optional[CollectorRuntime]) -> None:
    if runtime is None:
        return
    runtime.stop_event.set()
    runtime.thread.join(timeout=2.0)
    try:
        runtime.executor.remove_node(runtime.node)
    except Exception:
        pass
    try:
        runtime.node.destroy_node()
    except Exception:
        pass
    try:
        runtime.rclpy_module.shutdown()
    except Exception:
        pass


def _apply_trial_params(
    trial_params: Dict[str, Any],
    constraints: Dict[str, Dict[str, Any]],
) -> Tuple[bool, List[Dict[str, str]]]:
    failures = []
    for name, value in trial_params.items():
        cfg = constraints.get(name)
        if not cfg:
            failures.append({"param": name, "reason": "constraint_missing"})
            continue
        node_name = str(cfg.get("node", "")).strip()
        if not node_name:
            failures.append({"param": name, "reason": "constraint_node_empty"})
            continue
        target_node = node_name if node_name.startswith("/") else f"/{node_name}"
        ok, msg = ros_utils.ros2_param_set(target_node, name, value, timeout_sec=4.0)
        if not ok:
            failures.append({"param": name, "reason": msg})
    return len(failures) == 0, failures


def evaluate_once(
    bag_path: Path,
    output_dir: Path,
    trial_id: str,
    constraints: Dict[str, Dict[str, Any]],
    trial_params: Dict[str, Any],
    localization_params: Path,
    agent_config: Path,
    constraints_path: Path,
    base_frame: str,
    baseline_ref_tum: Optional[Path],
    play_timeout_sec: float,
    set_nav_safe: bool,
) -> Dict[str, Any]:
    trial_dir = output_dir / trial_id
    traj_dir = output_dir / "trajs"
    trial_dir.mkdir(parents=True, exist_ok=True)
    traj_dir.mkdir(parents=True, exist_ok=True)
    metrics_path = trial_dir / "metrics.json"
    tum_path = traj_dir / f"{trial_id}.tum"

    metrics: Dict[str, Any] = {
        "ok": False,
        "reason": "",
        "trial_id": trial_id,
        "generated_at": time.time(),
    }

    if set_nav_safe:
        ok, msg = ros_utils.call_set_nav_mode(profile="safe", timeout=0.0, reason="autotune_trial")
        if not ok:
            metrics["reason"] = "set_nav_mode_failed"
            metrics["events"] = [{"type": "set_nav_mode", "ok": False, "detail": msg}]
            artifacts.write_json(metrics_path, metrics)
            return metrics

    collector_runtime: Optional[CollectorRuntime] = None
    localization_proc: Optional[ros_utils.ManagedProcess] = None
    agent_proc: Optional[ros_utils.ManagedProcess] = None
    bag_proc: Optional[ros_utils.ManagedProcess] = None

    try:
        localization_cmd = [
            "ros2",
            "run",
            "rc26_localization",
            "rc26_localization_node",
            "--ros-args",
            "--params-file",
            str(localization_params),
            "-p",
            "use_sim_time:=true",
        ]
        agent_cmd = [
            "ros2",
            "run",
            "",
            "_node",
            "--ros-args",
            "--params-file",
            str(agent_config),
            "-p",
            "competition_mode_lock:=true",
            "-p",
            f"param_constraints_file:={constraints_path}",
            "-p",
            "use_sim_time:=true",
        ]

        localization_proc = ros_utils.ManagedProcess(localization_cmd, name="localization")
        localization_proc.start()
        agent_proc = ros_utils.ManagedProcess(agent_cmd, name="")
        agent_proc.start()

        ros_utils.wait_for_topic("//health_card", timeout_sec=15.0)
        ros_utils.wait_for_topic("/tf", timeout_sec=15.0)

        if trial_params:
            ok, failures = _apply_trial_params(trial_params, constraints)
            if not ok:
                metrics["reason"] = "trial_param_set_failed"
                metrics["events"] = [{"type": "param_set", "ok": False, "failures": failures}]
                artifacts.write_json(metrics_path, metrics)
                return metrics

        collector_runtime, collector_error = _start_collector(base_frame)
        if collector_runtime is None:
            metrics["reason"] = collector_error
            artifacts.write_json(metrics_path, metrics)
            return metrics

        bag_cmd = _build_bag_play_cmd(bag_path)
        bag_proc = ros_utils.ManagedProcess(bag_cmd, name="bag_play")
        bag_proc.start()
        exit_code = bag_proc.wait(timeout_sec=play_timeout_sec)
        if exit_code is None:
            bag_proc.terminate()
            metrics["reason"] = "bag_play_timeout"
            artifacts.write_json(metrics_path, metrics)
            return metrics

        time.sleep(1.0)
        health_cards = list(collector_runtime.node.health_cards)
        cmd_samples = list(collector_runtime.node.cmd_samples)
        tf_samples = list(collector_runtime.node.tf_samples)

        _write_tum(tum_path, tf_samples)
        agg = _aggregate_health(health_cards)
        if not agg:
            metrics["reason"] = "no_health_card_samples"
            metrics["events"] = [{"type": "collector", "ok": False}]
            artifacts.write_json(metrics_path, metrics)
            return metrics

        metrics.update(agg)
        metrics["ok"] = True
        metrics["reason"] = ""
        metrics["traj_tum"] = str(tum_path)
        metrics["nav"] = {
            "success": None,
            "t_goal_sec": None,
            "jerk_rms": _compute_jerk_rms(cmd_samples),
        }
        metrics["slam"] = {"ape_rmse": None, "rpe_rmse": None}
        metrics["events"] = [{"type": "bag_eval", "ok": True, "health_samples": len(health_cards)}]

        if baseline_ref_tum and baseline_ref_tum.exists() and tum_path.exists():
            metrics["slam"]["ape_rmse"] = _run_evo_ape(baseline_ref_tum, tum_path)
            metrics["slam"]["rpe_rmse"] = _run_evo_rpe(baseline_ref_tum, tum_path)

        artifacts.write_json(metrics_path, metrics)
        return metrics
    finally:
        _stop_collector(collector_runtime)
        if bag_proc:
            bag_proc.terminate()
        if agent_proc:
            agent_proc.terminate()
        if localization_proc:
            localization_proc.terminate()


def _build_baseline(args: argparse.Namespace, constraints: Dict[str, Dict[str, Any]]) -> int:
    output_dir = Path(args.output_dir)
    baseline_round_dir = output_dir / "baseline_rounds"
    baseline_round_dir.mkdir(parents=True, exist_ok=True)

    tum_paths: List[Path] = []
    metric_paths: List[Path] = []
    for i in range(args.trials):
        tid = f"baseline_{i}"
        result = evaluate_once(
            bag_path=Path(args.bag_path),
            output_dir=baseline_round_dir,
            trial_id=tid,
            constraints=constraints,
            trial_params={},
            localization_params=Path(args.localization_params),
            agent_config=Path(args.agent_config),
            constraints_path=Path(args.constraints),
            base_frame=args.base_frame,
            baseline_ref_tum=None,
            play_timeout_sec=args.play_timeout_sec,
            set_nav_safe=not args.skip_set_nav_safe,
        )
        metric_paths.append(baseline_round_dir / tid / "metrics.json")
        tum_paths.append(baseline_round_dir / "trajs" / f"{tid}.tum")
        if not result.get("ok", False):
            break

    metrics = [artifacts.read_json(p) for p in metric_paths if p.exists()]
    ok_metrics = [m for m in metrics if m.get("ok", False)]
    if len(ok_metrics) < 1:
        baseline = {"ok": False, "reason": "all_baseline_rounds_failed"}
        artifacts.write_json(output_dir / "baseline_metrics.json", baseline)
        return 1

    valid_indices = [i for i, m in enumerate(metrics) if m.get("ok", False) and tum_paths[i].exists()]
    if not valid_indices:
        baseline = {"ok": False, "reason": "baseline_tum_missing"}
        artifacts.write_json(output_dir / "baseline_metrics.json", baseline)
        return 1

    # Representative baseline: pick trajectory with smallest mean pairwise APE.
    means = {}
    for i in valid_indices:
        dists = []
        for j in valid_indices:
            if i == j:
                continue
            rmse = _run_evo_ape(tum_paths[j], tum_paths[i])
            if rmse is not None:
                dists.append(rmse)
        means[i] = (sum(dists) / len(dists)) if dists else float(i)

    best_idx = min(means.keys(), key=lambda k: means[k])
    best_metrics = metrics[best_idx]
    best_tum = tum_paths[best_idx]

    baseline_ref = output_dir / "baseline_ref.tum"
    baseline_ref.write_text(best_tum.read_text(encoding="utf-8"), encoding="utf-8")
    artifacts.write_json(output_dir / "baseline_metrics.json", best_metrics)
    artifacts.write_yaml(output_dir / "baseline_params.yaml", {})
    artifacts.write_json(
        output_dir / "baseline_selection.json",
        {"candidate_indices": valid_indices, "mean_pairwise_ape": means, "selected": best_idx},
    )
    return 0


def _main() -> int:
    parser = argparse.ArgumentParser(description="Offline bag evaluator for SLAM autotune")
    parser.add_argument("--mode", choices=["trial", "baseline_build"], default="trial")
    parser.add_argument("--bag_path", required=True)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--trial_id", default="trial")
    parser.add_argument("--params_yaml", default="")
    parser.add_argument("--constraints", default="src//config/param_constraints.yaml")
    parser.add_argument("--localization_params", default="src/rc26_bringup/config/localization.yaml")
    parser.add_argument("--agent_config", default="src//config/agent_config.yaml")
    parser.add_argument("--base_frame", default="base_link")
    parser.add_argument("--baseline_ref", default="")
    parser.add_argument("--play_timeout_sec", type=float, default=300.0)
    parser.add_argument("--trials", type=int, default=3, help="baseline mode only")
    parser.add_argument("--skip_set_nav_safe", action="store_true")
    args = parser.parse_args()

    constraints = {}
    try:
        constraints = yaml.safe_load(Path(args.constraints).read_text(encoding="utf-8")).get("params", {})
    except Exception:
        constraints = {}

    if args.mode == "baseline_build":
        return _build_baseline(args, constraints)

    trial_params = _load_params_yaml(Path(args.params_yaml)) if args.params_yaml else {}
    result = evaluate_once(
        bag_path=Path(args.bag_path),
        output_dir=Path(args.output_dir),
        trial_id=args.trial_id,
        constraints=constraints,
        trial_params=trial_params,
        localization_params=Path(args.localization_params),
        agent_config=Path(args.agent_config),
        constraints_path=Path(args.constraints),
        base_frame=args.base_frame,
        baseline_ref_tum=Path(args.baseline_ref) if args.baseline_ref else None,
        play_timeout_sec=args.play_timeout_sec,
        set_nav_safe=not args.skip_set_nav_safe,
    )
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result.get("ok", False) else 1


if __name__ == "__main__":
    raise SystemExit(_main())
