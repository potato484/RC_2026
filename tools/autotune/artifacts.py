from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict

import yaml


def ensure_run_dirs(runs_root: Path, run_id: str) -> Dict[str, Path]:
    run_dir = runs_root / run_id
    trials_dir = run_dir / "trials"
    best_dir = run_dir / "best"
    llm_dir = run_dir / "llm"
    for path in (run_dir, trials_dir, best_dir, llm_dir):
        path.mkdir(parents=True, exist_ok=True)
    return {
        "run_dir": run_dir,
        "trials_dir": trials_dir,
        "best_dir": best_dir,
        "llm_dir": llm_dir,
    }


def write_json(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def read_json(path: Path) -> Dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def write_yaml(path: Path, payload: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(payload, allow_unicode=True, sort_keys=True), encoding="utf-8")


def read_yaml(path: Path) -> Dict[str, Any]:
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except Exception:
        return {}


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def write_resolved_config(path: Path, payload: Dict[str, Any]) -> None:
    write_yaml(path, payload)


def write_diff_md(path: Path, baseline: Dict[str, Any], best: Dict[str, Any]) -> None:
    keys = sorted(set(baseline.keys()) | set(best.keys()))
    lines = [
        "# Parameter Diff",
        "",
        "| param | baseline | best |",
        "|---|---:|---:|",
    ]
    changed = 0
    for key in keys:
        b = baseline.get(key, "")
        n = best.get(key, "")
        if b != n:
            changed += 1
        lines.append(f"| {key} | {b} | {n} |")
    lines.append("")
    lines.append(f"changed: {changed}")
    write_text(path, "\n".join(lines) + "\n")
