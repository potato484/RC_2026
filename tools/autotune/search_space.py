from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Optional

import yaml


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float))


def _is_int_like(value: float, eps: float = 1e-9) -> bool:
    return abs(value - round(value)) <= eps


def load_constraints(path: Path) -> Dict[str, Dict[str, Any]]:
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    params = raw.get("params", {})
    if not isinstance(params, dict):
        return {}
    out: Dict[str, Dict[str, Any]] = {}
    for name, cfg in params.items():
        if not isinstance(cfg, dict):
            continue
        if not all(k in cfg for k in ("min", "max", "step", "node")):
            continue
        out[str(name)] = cfg
    return out


def normalize_proposal(raw: Dict[str, Any], constraints: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    if not isinstance(raw, dict):
        return {"ok": False, "range": {}, "freeze": []}

    payload = raw.get("proposal") if isinstance(raw.get("proposal"), dict) else raw
    if not isinstance(payload, dict):
        return {"ok": False, "range": {}, "freeze": []}
    if payload.get("ok") is False:
        return {"ok": False, "range": {}, "freeze": []}

    normalized_range: Dict[str, Dict[str, float]] = {}
    raw_range = payload.get("range", {})
    if isinstance(raw_range, dict):
        for name, window in raw_range.items():
            if name not in constraints or not isinstance(window, dict):
                continue
            c = constraints[name]
            cmin = float(c["min"])
            cmax = float(c["max"])
            wmin = float(window.get("min", cmin)) if _is_number(window.get("min", cmin)) else cmin
            wmax = float(window.get("max", cmax)) if _is_number(window.get("max", cmax)) else cmax
            lo = max(cmin, min(wmin, wmax))
            hi = min(cmax, max(wmin, wmax))
            if lo <= hi:
                normalized_range[name] = {"min": lo, "max": hi}

    freeze = payload.get("freeze", [])
    normalized_freeze = sorted({str(x) for x in freeze if isinstance(x, str) and x in constraints})
    return {"ok": True, "range": normalized_range, "freeze": normalized_freeze}


def load_proposal(path: Path, constraints: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    if not path.exists():
        return {"ok": False, "range": {}, "freeze": []}
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {"ok": False, "range": {}, "freeze": []}
    return normalize_proposal(raw, constraints)


def _effective_domain(name: str, cfg: Dict[str, Any], proposal: Optional[Dict[str, Any]]) -> Optional[Dict[str, float]]:
    if proposal and name in set(proposal.get("freeze", [])):
        return None

    lo = float(cfg["min"])
    hi = float(cfg["max"])
    step = float(cfg["step"])
    if lo > hi:
        lo, hi = hi, lo

    if proposal:
        shrink = proposal.get("range", {}).get(name, {})
        if isinstance(shrink, dict):
            s_lo = float(shrink.get("min", lo)) if _is_number(shrink.get("min", lo)) else lo
            s_hi = float(shrink.get("max", hi)) if _is_number(shrink.get("max", hi)) else hi
            lo = max(lo, min(s_lo, s_hi))
            hi = min(hi, max(s_lo, s_hi))
            if lo > hi:
                return None

    return {"min": lo, "max": hi, "step": step}


def suggest_params(trial: Any, constraints: Dict[str, Dict[str, Any]], proposal: Optional[Dict[str, Any]] = None) -> Dict[str, float]:
    suggested: Dict[str, float] = {}
    for name in sorted(constraints.keys()):
        domain = _effective_domain(name, constraints[name], proposal)
        if domain is None:
            continue
        lo = float(domain["min"])
        hi = float(domain["max"])
        step = float(domain["step"])

        if abs(hi - lo) <= 1e-12:
            suggested[name] = lo
            continue

        int_mode = _is_int_like(lo) and _is_int_like(hi) and (step <= 0 or _is_int_like(step))
        if int_mode:
            i_lo = int(round(lo))
            i_hi = int(round(hi))
            i_step = int(round(step)) if step > 0 else 1
            if i_step <= 0:
                i_step = 1
            suggested[name] = float(trial.suggest_int(name, i_lo, i_hi, step=i_step))
        else:
            if step > 0:
                suggested[name] = float(trial.suggest_float(name, lo, hi, step=step))
            else:
                suggested[name] = float(trial.suggest_float(name, lo, hi))
    return suggested
