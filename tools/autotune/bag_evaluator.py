#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = SCRIPT_DIR.parent.parent

import sys

if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import artifacts  # noqa: E402


def _load_offline_pipeline() -> Tuple[Any, Any]:
    package_root = WORKSPACE_ROOT / "src" / ""
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))
    from offline.artifact_builder import ArtifactBuilder
    from offline.bag_reader import read_mcap

    return ArtifactBuilder, read_mcap


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


def _read_jsonl(path: Path) -> List[Dict[str, Any]]:
    if not path.exists():
        return []
    rows: List[Dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        text = line.strip()
        if not text:
            continue
        try:
            payload = json.loads(text)
        except Exception:
            continue
        if isinstance(payload, dict):
            rows.append(payload)
    return rows


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


def evaluate_once(
    bag_path: Path,
    output_dir: Path,
    trial_id: str,
    constraints: Dict[str, Dict[str, Any]],
    trial_params: Dict[str, Any],
) -> Dict[str, Any]:
    _ = constraints

    trial_dir = output_dir / trial_id
    trial_dir.mkdir(parents=True, exist_ok=True)
    metrics_path = trial_dir / "metrics.json"
    extract_run_dir = trial_dir / "offline_extract"

    metrics: Dict[str, Any] = {
        "ok": False,
        "reason": "",
        "trial_id": trial_id,
        "generated_at": time.time(),
    }

    if not bag_path.exists():
        metrics["reason"] = f"bag_not_found:{bag_path}"
        artifacts.write_json(metrics_path, metrics)
        return metrics

    try:
        ArtifactBuilder, read_mcap = _load_offline_pipeline()
        event_stream = read_mcap(bag_path)
        summary = ArtifactBuilder().extract(event_stream=event_stream, run_dir=extract_run_dir)
    except Exception as ex:
        metrics["reason"] = f"offline_extract_failed:{ex}"
        artifacts.write_json(metrics_path, metrics)
        return metrics

    cards = _read_jsonl(extract_run_dir / "health_card.jsonl")
    agg = _aggregate_health(cards)
    if not agg:
        metrics["reason"] = "no_health_card_samples"
        metrics["events"] = [{"type": "offline_extract", "ok": False}]
        artifacts.write_json(metrics_path, metrics)
        return metrics

    metrics.update(agg)
    metrics["ok"] = True
    metrics["reason"] = ""
    metrics["card_count"] = int(summary.card_count)
    metrics["bundle_count"] = int(summary.bundle_count)
    metrics["run_dir"] = str(extract_run_dir)
    metrics["evidence_dir"] = str(extract_run_dir / "bundles")
    metrics["params"] = trial_params
    metrics["nav"] = {"success": None, "t_goal_sec": None, "jerk_rms": None}
    metrics["slam"] = {"ape_rmse": None, "rpe_rmse": None}
    metrics["events"] = [{"type": "offline_extract", "ok": True, "health_samples": len(cards)}]
    artifacts.write_json(metrics_path, metrics)
    return metrics


def _build_baseline(args: argparse.Namespace, constraints: Dict[str, Dict[str, Any]]) -> int:
    output_dir = Path(args.output_dir)
    baseline_round_dir = output_dir / "baseline_rounds"
    baseline_round_dir.mkdir(parents=True, exist_ok=True)

    selected_metrics: Optional[Dict[str, Any]] = None
    selected_index: Optional[int] = None
    max_rounds = max(1, int(args.trials))
    for index in range(max_rounds):
        trial_id = f"baseline_{index}"
        result = evaluate_once(
            bag_path=Path(args.bag_path),
            output_dir=baseline_round_dir,
            trial_id=trial_id,
            constraints=constraints,
            trial_params={},
        )
        if result.get("ok", False):
            selected_metrics = result
            selected_index = index
            break

    if not selected_metrics:
        baseline = {"ok": False, "reason": "all_baseline_rounds_failed"}
        artifacts.write_json(output_dir / "baseline_metrics.json", baseline)
        return 1

    baseline_ref = output_dir / "baseline_ref.tum"
    baseline_ref.write_text("# offline evaluator does not produce trajectory\n", encoding="utf-8")
    artifacts.write_json(output_dir / "baseline_metrics.json", selected_metrics)
    artifacts.write_yaml(output_dir / "baseline_params.yaml", {})
    artifacts.write_json(
        output_dir / "baseline_selection.json",
        {
            "selected": selected_index,
            "mode": "offline_extract",
        },
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
    parser.add_argument("--trials", type=int, default=3, help="baseline mode only")
    args = parser.parse_args()

    constraints = {}
    try:
        loaded = yaml.safe_load(Path(args.constraints).read_text(encoding="utf-8")) or {}
        if isinstance(loaded, dict):
            params = loaded.get("params", {})
            if isinstance(params, dict):
                constraints = params
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
    )
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result.get("ok", False) else 1


if __name__ == "__main__":
    raise SystemExit(_main())
