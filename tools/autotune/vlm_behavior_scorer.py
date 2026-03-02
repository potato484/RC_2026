#!/usr/bin/env python3
from __future__ import annotations

import argparse
import base64
import json
import os
import re
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent

import sys

if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import artifacts  # noqa: E402
import ros_utils  # noqa: E402


VALID_LABELS = {"DRIFT_VISUAL", "GHOSTING", "SCAN_DEGENERATE", "NORMAL"}
PNG_NAMES = ["bev_density.png", "bev_temporal_overlay.png", "height_slices.png", "xz_section.png"]


def _load_json(path: Path) -> Dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _resolve_chat_url(endpoint: str) -> str:
    endpoint = endpoint.strip().rstrip("/")
    if endpoint.endswith("/chat/completions"):
        return endpoint
    if endpoint.endswith("/v1"):
        return endpoint + "/chat/completions"
    return endpoint + "/v1/chat/completions"


def _extract_json_object(text: str) -> Dict[str, Any]:
    text = text.strip()
    if not text:
        return {}
    try:
        return json.loads(text)
    except Exception:
        pass
    m = re.search(r"\{.*\}", text, flags=re.DOTALL)
    if not m:
        return {}
    try:
        return json.loads(m.group(0))
    except Exception:
        return {}


def _call_model(endpoint: str, model: str, api_key: str, prompt: str, timeout_sec: float = 8.0) -> Dict[str, Any]:
    image_mode = os.getenv("SLAM_AGENT_VLM_IMAGE_MODE", "path").strip().lower()
    if image_mode not in {"path", "data_url"}:
        image_mode = "path"

    # Default: keep backward-compatible "path mode" where the prompt contains local image paths.
    user_content: Any = prompt
    if image_mode == "data_url":
        # OpenAI-compatible multimodal message:
        # The actual images are attached as data URLs, and the prompt stays as the text part.
        bundle_dir = os.getenv("SLAM_AGENT_VLM_BUNDLE_DIR", "").strip()
        image_paths: list[str] = []
        if bundle_dir:
            for name in PNG_NAMES:
                p = Path(bundle_dir) / name
                if p.exists():
                    image_paths.append(str(p))
        content_parts: list[dict[str, Any]] = [{"type": "text", "text": prompt}]
        for p in image_paths:
            data = Path(p).read_bytes()
            b64 = base64.b64encode(data).decode("ascii")
            content_parts.append({"type": "image_url", "image_url": {"url": "data:image/png;base64," + b64}})
        user_content = content_parts

    payload = {
        "model": model,
        "temperature": 0.0,
        "messages": [
            {"role": "system", "content": "你是SLAM行为诊断器。必须只输出JSON对象。"},
            {"role": "user", "content": user_content},
        ],
    }
    data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    req = urllib.request.Request(_resolve_chat_url(endpoint), data=data, method="POST")
    req.add_header("Content-Type", "application/json")
    if api_key:
        req.add_header("Authorization", f"Bearer {api_key}")
    with urllib.request.urlopen(req, timeout=max(1.0, timeout_sec)) as resp:
        return json.loads(resp.read().decode("utf-8", errors="replace"))


def _load_guard_thresholds(constraints_path: Path) -> Dict[str, float]:
    try:
        raw = yaml.safe_load(constraints_path.read_text(encoding="utf-8")) or {}
    except Exception:
        return {}
    guard = raw.get("guard", {})
    return guard if isinstance(guard, dict) else {}


def _pick_bundle(bundle_dir: Optional[Path], evidence_dir: Path) -> Optional[Path]:
    if bundle_dir and bundle_dir.exists():
        return bundle_dir
    return ros_utils.find_latest_bundle(evidence_dir)


def _main() -> int:
    parser = argparse.ArgumentParser(description="VLM behavior scorer for SLAM evidence bundle")
    parser.add_argument("--constraints", required=True)
    parser.add_argument("--metrics", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--bundle_dir", default="")
    parser.add_argument("--evidence_dir", default="")
    parser.add_argument("--timeout_sec", type=float, default=8.0)
    parser.add_argument("--force", action="store_true", help="Force VLM call even if metrics are below trigger threshold")
    args = parser.parse_args()

    out_path = Path(args.out)
    metrics = _load_json(Path(args.metrics))
    guard = _load_guard_thresholds(Path(args.constraints))

    norm_max = float(guard.get("norm_err_p95_max", 0.25))
    ghost_max = float(guard.get("ghosting_score_max", 0.25))
    norm = float(metrics.get("norm_err_p95", 0.0))
    ghost = float(metrics.get("ghosting_score", 0.0))

    need_vlm = bool(args.force) or (norm > norm_max * 1.5 or ghost > ghost_max * 2.0)
    if not need_vlm:
        artifacts.write_json(
            out_path,
            {"ok": True, "label": "NORMAL", "score": 0.0, "reason": "below_trigger_threshold", "forced": False},
        )
        return 0

    evidence_dir = Path(args.evidence_dir) if args.evidence_dir else Path(".")
    chosen_bundle = _pick_bundle(Path(args.bundle_dir) if args.bundle_dir else None, evidence_dir)
    if chosen_bundle is None:
        artifacts.write_json(out_path, {"ok": False, "reason": "no evidence bundle found"})
        return 0

    existing_pngs = [str(chosen_bundle / name) for name in PNG_NAMES if (chosen_bundle / name).exists()]
    if not existing_pngs:
        artifacts.write_json(out_path, {"ok": False, "reason": "bundle has no required png evidence"})
        return 0

    endpoint = os.getenv("SLAM_AGENT_AI_ENDPOINT", "").strip()
    model = os.getenv("SLAM_AGENT_AI_MODEL", "").strip()
    api_key = os.getenv("SLAM_AGENT_API_KEY", "").strip()
    if not api_key:
        artifacts.write_json(out_path, {"ok": False, "reason": "SLAM_AGENT_API_KEY missing"})
        return 0
    if not endpoint or not model:
        artifacts.write_json(out_path, {"ok": False, "reason": "SLAM_AGENT_AI_ENDPOINT/MODEL missing"})
        return 0

    image_mode = os.getenv("SLAM_AGENT_VLM_IMAGE_MODE", "path").strip().lower()
    if image_mode not in {"path", "data_url"}:
        image_mode = "path"

    # For "data_url" mode, avoid leaking absolute paths into the model prompt.
    prompt_images: list[str] = existing_pngs if image_mode == "path" else [Path(p).name for p in existing_pngs]

    # In "data_url" mode, `_call_model` needs the bundle dir to load images and attach them.
    if image_mode == "data_url":
        os.environ["SLAM_AGENT_VLM_BUNDLE_DIR"] = str(chosen_bundle)

    prompt = json.dumps(
        {
            "task": "根据SLAM指标和BEV图像路径给出行为标签与置信度。",
            "allowed_labels": sorted(VALID_LABELS),
            "metrics": {"norm_err_p95": norm, "ghosting_score": ghost},
            "images": prompt_images,
            "output_schema": {"label": "string", "score": "number[0,1]"},
        },
        ensure_ascii=False,
    )

    try:
        raw_resp = _call_model(endpoint, model, api_key, prompt, timeout_sec=args.timeout_sec)
        content = ""
        choices = raw_resp.get("choices", [])
        if isinstance(choices, list) and choices:
            msg = choices[0].get("message", {}) if isinstance(choices[0], dict) else {}
            if isinstance(msg, dict):
                content = str(msg.get("content", ""))
        if not content:
            content = str(raw_resp.get("output_text", ""))
        payload = _extract_json_object(content)
        label = str(payload.get("label", "NORMAL")).strip().upper()
        if label not in VALID_LABELS:
            label = "NORMAL"
        try:
            score = float(payload.get("score", 0.0))
        except (TypeError, ValueError):
            score = 0.0
        score = max(0.0, min(1.0, score))
        artifacts.write_json(
            out_path,
            {
                "ok": True,
                "label": label,
                "score": score,
                "bundle_dir": str(chosen_bundle),
                "images": existing_pngs,
                "forced": bool(args.force),
                "image_mode": image_mode,
                "model": model,
                "endpoint": _resolve_chat_url(endpoint),
            },
        )
        return 0
    except urllib.error.HTTPError as ex:
        artifacts.write_json(out_path, {"ok": False, "reason": f"http error: {ex.code}"})
        return 0
    except Exception as ex:
        artifacts.write_json(out_path, {"ok": False, "reason": f"vlm scoring failed: {ex}"})
        return 0


if __name__ == "__main__":
    raise SystemExit(_main())
