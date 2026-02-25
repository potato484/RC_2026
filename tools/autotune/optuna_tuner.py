#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path
from typing import Any, Dict

SCRIPT_DIR = Path(__file__).resolve().parent

import sys

if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import artifacts  # noqa: E402
import fitness_evaluator  # noqa: E402
import ros_utils  # noqa: E402
import safety_guard  # noqa: E402
import search_space  # noqa: E402

try:
    import optuna
except Exception as ex:  # pragma: no cover
    print(json.dumps({"ok": False, "reason": f"optuna import failed: {ex}"}))
    raise SystemExit(1)


def _run_python(cmd: list[str], timeout_sec: float = 3600.0) -> tuple[int, str, str]:
    return ros_utils.run_command(["python3", *cmd], timeout_sec=timeout_sec)


def _ensure_baseline(
    run_dir: Path,
    bag_path: Path,
    constraints: Path,
    localization_params: Path,
    agent_config: Path,
    base_frame: str,
    play_timeout_sec: float,
) -> Dict[str, Any]:
    baseline_metrics_path = run_dir / "baseline_metrics.json"
    baseline_ref_path = run_dir / "baseline_ref.tum"
    baseline_params_path = run_dir / "baseline_params.yaml"
    if baseline_metrics_path.exists() and baseline_ref_path.exists():
        baseline = artifacts.read_json(baseline_metrics_path)
        if baseline:
            if not baseline_params_path.exists():
                artifacts.write_yaml(baseline_params_path, {})
            return baseline

    cmd = [
        str(SCRIPT_DIR / "bag_evaluator.py"),
        "--mode",
        "baseline_build",
        "--bag_path",
        str(bag_path),
        "--output_dir",
        str(run_dir),
        "--constraints",
        str(constraints),
        "--localization_params",
        str(localization_params),
        "--agent_config",
        str(agent_config),
        "--base_frame",
        base_frame,
        "--play_timeout_sec",
        str(play_timeout_sec),
        "--trials",
        "3",
    ]
    code, out, err = _run_python(cmd, timeout_sec=max(600.0, 3.0 * play_timeout_sec))
    if code != 0:
        print(out)
        print(err)
    baseline = artifacts.read_json(baseline_metrics_path)
    if not baseline:
        raise RuntimeError("failed to build baseline metrics")
    if not baseline_ref_path.exists():
        raise RuntimeError("failed to build baseline_ref.tum")
    if not baseline_params_path.exists():
        artifacts.write_yaml(baseline_params_path, {})
    return baseline


def _call_propose_space(
    run_dir: Path,
    constraints: Path,
    baseline_metrics: Path,
    reasoning_client: Path,
) -> Path:
    out_path = run_dir / "llm" / "SearchSpaceProposal.json"
    if not reasoning_client.exists():
        artifacts.write_json(out_path, {"ok": False, "reason": f"reasoning_client not found: {reasoning_client}"})
        return out_path

    cmd = [
        str(reasoning_client),
        "--mode",
        "propose_space",
        "--constraints",
        str(constraints),
        "--baseline_metrics",
        str(baseline_metrics),
        "--trials_glob",
        str(run_dir / "trials" / "*" / "metrics.json"),
        "--out",
        str(out_path),
    ]
    ros_utils.run_command(["python3", *cmd], timeout_sec=120.0)
    if not out_path.exists():
        artifacts.write_json(out_path, {"ok": False, "reason": "propose_space did not generate output"})
    return out_path


def _load_latest_health(evidence_base_dir: Path) -> Dict[str, Any]:
    latest = evidence_base_dir / "latest_health_card.json"
    if not latest.exists():
        return {"ok": False, "reason": f"missing {latest}"}
    try:
        data = json.loads(latest.read_text(encoding="utf-8"))
    except Exception as ex:
        return {"ok": False, "reason": f"read latest health card failed: {ex}"}
    if not isinstance(data, dict):
        return {"ok": False, "reason": "latest_health_card is not JSON object"}
    data["ok"] = True
    data.setdefault("events", [])
    data.setdefault("nav", {"success": None, "t_goal_sec": None, "jerk_rms": None})
    data.setdefault("slam", {"ape_rmse": None, "rpe_rmse": None})
    return data


def _main() -> int:
    parser = argparse.ArgumentParser(description="Optuna-based autotuner for ")
    parser.add_argument("--constraints", required=True)
    parser.add_argument("--bag_path", required=True)
    parser.add_argument("--run_id", required=True)
    parser.add_argument("--n_trials", type=int, default=15)
    parser.add_argument("--runs_root", default="runs")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--eval_mode", choices=["offline", "online"], default="offline")
    parser.add_argument("--require_unlock", action="store_true")
    parser.add_argument("--safe_profile", default="safe")
    parser.add_argument("--reasoning_client", default="src//scripts/reasoning_client.py")
    parser.add_argument("--localization_params", default="src/rc26_bringup/config/localization.yaml")
    parser.add_argument("--agent_config", default="src//config/agent_config.yaml")
    parser.add_argument("--base_frame", default="base_link")
    parser.add_argument("--play_timeout_sec", type=float, default=300.0)
    parser.add_argument("--trial_timeout_sec", type=float, default=1800.0)
    parser.add_argument("--vlm_enable", action="store_true")
    parser.add_argument("--vlm_beta", type=float, default=0.10)
    parser.add_argument("--evidence_base_dir", default="/tmp/_evidence")
    args = parser.parse_args()

    constraints_path = Path(args.constraints)
    bag_path = Path(args.bag_path)
    runs_root = Path(args.runs_root)
    dirs = artifacts.ensure_run_dirs(runs_root, args.run_id)
    run_dir = dirs["run_dir"]
    trials_dir = dirs["trials_dir"]
    best_dir = dirs["best_dir"]

    if args.require_unlock:
        found, lock_value = ros_utils.read_bool_param(
            ["/", "/monitor_node", "", "monitor_node"],
            "competition_mode_lock",
        )
        if not found:
            print(json.dumps({"ok": False, "reason": "require_unlock=true but competition_mode_lock is unreadable"}))
            return 2
        if lock_value:
            print(json.dumps({"ok": False, "reason": "require_unlock=true and competition_mode_lock=true"}))
            return 2

    constraints = search_space.load_constraints(constraints_path)
    if not constraints:
        print(json.dumps({"ok": False, "reason": f"no constraints loaded from {constraints_path}"}))
        return 2

    resolved_config = {
        "constraints": str(constraints_path),
        "bag_path": str(bag_path),
        "run_id": args.run_id,
        "n_trials": int(args.n_trials),
        "seed": int(args.seed),
        "eval_mode": args.eval_mode,
        "safe_profile": args.safe_profile,
        "vlm_enable": bool(args.vlm_enable),
        "vlm_beta": min(max(float(args.vlm_beta), 0.0), 0.10),
        "created_at": time.time(),
    }
    artifacts.write_resolved_config(run_dir / "resolved_config.yaml", resolved_config)

    baseline = _ensure_baseline(
        run_dir=run_dir,
        bag_path=bag_path,
        constraints=constraints_path,
        localization_params=Path(args.localization_params),
        agent_config=Path(args.agent_config),
        base_frame=args.base_frame,
        play_timeout_sec=args.play_timeout_sec,
    )
    baseline_metrics_path = run_dir / "baseline_metrics.json"
    baseline_ref_path = run_dir / "baseline_ref.tum"
    baseline_params = artifacts.read_yaml(run_dir / "baseline_params.yaml")

    proposal_path = _call_propose_space(
        run_dir=run_dir,
        constraints=constraints_path,
        baseline_metrics=baseline_metrics_path,
        reasoning_client=Path(args.reasoning_client),
    )
    proposal = search_space.load_proposal(proposal_path, constraints)

    storage = f"sqlite:///{run_dir / 'optimizer.db'}"
    study = optuna.create_study(
        study_name=args.run_id,
        storage=storage,
        load_if_exists=True,
        direction="maximize",
        sampler=optuna.samplers.TPESampler(seed=args.seed),
        pruner=optuna.pruners.MedianPruner(),
    )

    def objective(trial: "optuna.Trial") -> float:
        trial_id = f"trial_{trial.number:04d}"
        trial_dir = trials_dir / trial_id
        trial_dir.mkdir(parents=True, exist_ok=True)

        proposed = search_space.suggest_params(trial, constraints, proposal if proposal.get("ok") else None)
        clamped, clamp_meta = safety_guard.clamp(proposed, constraints)
        params_path = trial_dir / "params.yaml"
        metrics_path = trial_dir / "metrics.json"
        events_path = trial_dir / "events.json"
        artifacts.write_yaml(params_path, {"params": clamped})

        events: Dict[str, Any] = {
            "trial_id": trial_id,
            "started_at": time.time(),
            "proposed_count": len(proposed),
            "clamp": clamp_meta,
            "safe_profile": args.safe_profile,
        }

        metrics: Dict[str, Any]
        if args.eval_mode == "offline":
            cmd = [
                str(SCRIPT_DIR / "bag_evaluator.py"),
                "--mode",
                "trial",
                "--bag_path",
                str(bag_path),
                "--output_dir",
                str(trials_dir),
                "--trial_id",
                trial_id,
                "--params_yaml",
                str(params_path),
                "--constraints",
                str(constraints_path),
                "--localization_params",
                str(args.localization_params),
                "--agent_config",
                str(args.agent_config),
                "--base_frame",
                args.base_frame,
                "--baseline_ref",
                str(baseline_ref_path),
                "--play_timeout_sec",
                str(args.play_timeout_sec),
            ]
            code, out, err = _run_python(cmd, timeout_sec=max(args.trial_timeout_sec, args.play_timeout_sec + 120.0))
            events["bag_eval_exit_code"] = code
            events["bag_eval_stdout_tail"] = out[-2000:]
            events["bag_eval_stderr_tail"] = err[-2000:]
            metrics = artifacts.read_json(metrics_path)
            if not metrics:
                metrics = {"ok": False, "reason": "missing metrics.json after bag_evaluator"}
        else:
            safe_ok, safe_msg = ros_utils.call_set_nav_mode(
                profile=args.safe_profile,
                timeout=0.0,
                reason="autotune_trial",
            )
            events["set_nav_mode"] = {"ok": safe_ok, "detail": safe_msg}
            if not safe_ok:
                metrics = {"ok": False, "reason": "set_nav_mode_failed"}
            else:
                metrics = _load_latest_health(Path(args.evidence_base_dir))
                artifacts.write_json(metrics_path, metrics)

        if args.vlm_enable:
            vlm_out = trial_dir / "vlm.json"
            code, _, _ = _run_python(
                [
                    str(SCRIPT_DIR / "vlm_behavior_scorer.py"),
                    "--evidence_dir",
                    args.evidence_base_dir,
                    "--constraints",
                    str(constraints_path),
                    "--metrics",
                    str(metrics_path),
                    "--out",
                    str(vlm_out),
                ],
                timeout_sec=60.0,
            )
            events["vlm_exit_code"] = code
            vlm_payload = artifacts.read_json(vlm_out)
            if isinstance(vlm_payload, dict):
                metrics["vlm"] = vlm_payload

        score = fitness_evaluator.compute(metrics, baseline, vlm_beta=args.vlm_beta)
        objective_value = float(score["fitness"])
        if not bool(metrics.get("ok", False)):
            objective_value = -1e9
            score["fitness"] = objective_value
        metrics.update(score)
        metrics["trial_id"] = trial_id
        metrics["params"] = clamped
        metrics["ok"] = bool(metrics.get("ok", False))
        artifacts.write_json(metrics_path, metrics)

        events["finished_at"] = time.time()
        events["fitness"] = objective_value
        artifacts.write_json(events_path, events)

        trial.set_user_attr("trial_id", trial_id)
        trial.set_user_attr("params", clamped)
        trial.set_user_attr("metrics_path", str(metrics_path))
        return objective_value

    try:
        study.optimize(objective, n_trials=args.n_trials)
    except KeyboardInterrupt:
        print("interrupted, resume by rerunning the same command")

    completed = [t for t in study.trials if t.value is not None]
    if not completed:
        print(json.dumps({"ok": False, "reason": "no completed trials"}))
        return 1

    best = study.best_trial
    best_params = best.user_attrs.get("params", best.params)
    artifacts.write_yaml(best_dir / "params.yaml", {"params": best_params})
    artifacts.write_diff_md(
        best_dir / "diff.md",
        baseline_params.get("params", baseline_params) if isinstance(baseline_params, dict) else {},
        best_params,
    )
    summary = {
        "ok": True,
        "run_dir": str(run_dir),
        "best_trial": best.number,
        "best_value": best.value,
        "best_params_path": str(best_dir / "params.yaml"),
    }
    artifacts.write_json(run_dir / "summary.json", summary)
    print(json.dumps(summary, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
