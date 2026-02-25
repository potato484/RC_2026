from __future__ import annotations

from typing import Any, Dict, Tuple


def _is_int_like(value: float, eps: float = 1e-9) -> bool:
    return abs(value - round(value)) <= eps


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _quantize(value: float, lo: float, hi: float, step: float) -> float:
    if step <= 0:
        return _clamp(value, lo, hi)
    snapped = lo + round((value - lo) / step) * step
    return _clamp(snapped, lo, hi)


def clamp(params: Dict[str, float], constraints: Dict[str, Dict[str, Any]]) -> Tuple[Dict[str, float], Dict[str, Any]]:
    out: Dict[str, float] = {}
    clamped_keys = []
    skipped = []

    for name, value in params.items():
        cfg = constraints.get(name)
        if not cfg:
            skipped.append({"param": name, "reason": "constraint_missing"})
            continue
        if bool(cfg.get("blacklist", False)):
            skipped.append({"param": name, "reason": "blacklist"})
            continue

        lo = float(cfg["min"])
        hi = float(cfg["max"])
        step = float(cfg.get("step", 0.0))
        raw = float(value)
        bounded = _quantize(_clamp(raw, lo, hi), lo, hi, step)

        int_mode = _is_int_like(lo) and _is_int_like(hi) and (step <= 0 or _is_int_like(step))
        if int_mode:
            bounded = float(int(round(bounded)))

        if abs(bounded - raw) > 1e-9:
            clamped_keys.append(name)
        out[name] = bounded

    meta = {
        "any_clamped": len(clamped_keys) > 0,
        "clamped_params": sorted(clamped_keys),
        "skipped": skipped,
    }
    return out, meta
