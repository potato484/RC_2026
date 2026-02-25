#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict


def _load_json(path: Path) -> Dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _clip(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _to_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _to_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "y", "on"}
    return False


def _ref_from_baseline(baseline: Dict[str, Any], key: str, fallback: float) -> float:
    ref = abs(_to_float(baseline.get(key), fallback))
    return ref if ref > 0 else fallback


def _compute_nav(metrics: Dict[str, Any], baseline: Dict[str, Any]) -> float:
    nav = metrics.get("nav")
    if not isinstance(nav, dict):
        return 0.0

    has_signal = any(nav.get(k) is not None for k in ("success", "t_goal_sec", "jerk_rms"))
    if not has_signal:
        return 0.0

    baseline_nav = baseline.get("nav") if isinstance(baseline.get("nav"), dict) else {}
    t_ref = _ref_from_baseline(baseline_nav, "t_goal_sec", 60.0)
    jerk_ref = _ref_from_baseline(baseline_nav, "jerk_rms", 1.0)

    score = 0.0
    if nav.get("success") is not None:
        score -= 0.40 * (0.0 if _to_bool(nav.get("success")) else 1.0)
    if nav.get("t_goal_sec") is not None:
        score -= 0.35 * _clip(_to_float(nav.get("t_goal_sec"), t_ref) / t_ref, 0.0, 3.0)
    if nav.get("jerk_rms") is not None:
        score -= 0.25 * _clip(_to_float(nav.get("jerk_rms"), jerk_ref) / jerk_ref, 0.0, 3.0)
    return score


def compute(metrics: Dict[str, Any], baseline: Dict[str, Any], vlm_beta: float = 0.10) -> Dict[str, float]:
    w1 = 0.35
    w2 = 0.15
    w3 = 0.20
    w4 = 0.20
    w5 = 0.10

    norm_ref = _ref_from_baseline(baseline, "norm_err_p95", 1e-3)
    dt_ref = _ref_from_baseline(baseline, "dt_ms_p95", 1.0)
    ghost_ref = _ref_from_baseline(baseline, "ghosting_score", 0.05)

    norm_err = _to_float(metrics.get("norm_err_p95"), 0.0)
    dt_ms = _to_float(metrics.get("dt_ms_p95"), 0.0)
    tracking = _clip(_to_float(metrics.get("tracking_ratio"), 0.0), 0.0, 1.0)
    ghosting = _to_float(metrics.get("ghosting_score"), 0.0)
    reloc = _clip(_to_float(metrics.get("reloc_success_rate"), 1.0), 0.0, 1.0)

    f_slam = -(
        w1 * _clip(norm_err / norm_ref, 0.0, 3.0)
        + w2 * _clip(dt_ms / dt_ref, 0.0, 3.0)
        + w3 * (1.0 - tracking)
        + w4 * _clip(ghosting / ghost_ref, 0.0, 3.0)
        + w5 * (1.0 - reloc)
    )

    penalty = 0.0
    if _to_bool(metrics.get("collision_detected")):
        penalty += 10000.0
    if _to_float(metrics.get("reloc_success_rate"), 1.0) < 0.3:
        penalty += 5000.0
    if _to_bool(metrics.get("shadow_timeout")):
        penalty += 2000.0
    if _to_bool(metrics.get("any_guardrail_triggered")):
        penalty += 500.0

    f_nav = _compute_nav(metrics, baseline)
    total = f_slam + f_nav - penalty

    vlm = metrics.get("vlm")
    if isinstance(vlm, dict) and _to_bool(vlm.get("ok", True)):
        label = str(vlm.get("label", "")).strip().upper()
        score = _clip(_to_float(vlm.get("score"), 0.0), 0.0, 1.0)
        beta = _clip(vlm_beta, 0.0, 0.10)
        if label and label != "NORMAL":
            total -= beta * score

    return {
        "fitness": float(total),
        "f_slam": float(f_slam),
        "f_nav": float(f_nav),
        "penalty": float(penalty),
    }


def _main() -> int:
    parser = argparse.ArgumentParser(description="Compute scalar fitness from metrics and baseline JSON.")
    parser.add_argument("--metrics", required=True, help="Path to metrics.json")
    parser.add_argument("--baseline", required=True, help="Path to baseline_metrics.json")
    parser.add_argument("--vlm_beta", type=float, default=0.10, help="VLM penalty weight upper-bounded at 0.10")
    parser.add_argument("--details", action="store_true", help="Print full score breakdown JSON")
    args = parser.parse_args()

    metrics = _load_json(Path(args.metrics))
    baseline = _load_json(Path(args.baseline))
    result = compute(metrics, baseline, vlm_beta=args.vlm_beta)

    if args.details:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print(f"{result['fitness']:.8f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
