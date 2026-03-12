#!/usr/bin/env python3
"""Fit lightweight logistic models for obstacle/drop probability calibration."""

from __future__ import annotations

import argparse
import csv
import math
import os
from typing import Dict, List, Sequence, Tuple

import numpy as np
import yaml


def sigmoid(x: np.ndarray) -> np.ndarray:
    x = np.clip(x, -30.0, 30.0)
    return 1.0 / (1.0 + np.exp(-x))


def auc_score(y_true: np.ndarray, y_prob: np.ndarray) -> float:
    if y_true.size == 0:
        return float("nan")
    positives = y_true > 0.5
    negatives = ~positives
    n_pos = int(np.sum(positives))
    n_neg = int(np.sum(negatives))
    if n_pos == 0 or n_neg == 0:
        return float("nan")
    order = np.argsort(y_prob)
    ranks = np.empty_like(order, dtype=np.float64)
    ranks[order] = np.arange(1, y_true.size + 1, dtype=np.float64)
    pos_rank_sum = float(np.sum(ranks[positives]))
    return (pos_rank_sum - n_pos * (n_pos + 1) * 0.5) / (n_pos * n_neg)


def train_logistic(
    x: np.ndarray,
    y: np.ndarray,
    max_iter: int,
    lr: float,
    l2: float,
) -> Tuple[np.ndarray, float]:
    if x.size == 0 or y.size == 0:
        raise ValueError("empty training set")
    n_samples, n_features = x.shape
    weights = np.zeros(n_features, dtype=np.float64)
    y_mean = float(np.clip(np.mean(y), 1e-4, 1.0 - 1e-4))
    bias = math.log(y_mean / (1.0 - y_mean))
    for _ in range(max_iter):
        logits = x.dot(weights) + bias
        probs = sigmoid(logits)
        err = probs - y
        grad_w = x.T.dot(err) / n_samples + l2 * weights
        grad_b = float(np.mean(err))
        weights -= lr * grad_w
        bias -= lr * grad_b
    return weights, bias


def read_csv_rows(path: str) -> List[Dict[str, str]]:
    with open(path, "r", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
    if not rows:
        raise ValueError("input csv is empty")
    return rows


def build_dataset(
    rows: Sequence[Dict[str, str]],
    feature_cols: Sequence[str],
    label_col: str,
    proxy_col: str,
    proxy_threshold: float,
) -> Tuple[np.ndarray, np.ndarray]:
    x_data: List[List[float]] = []
    y_data: List[float] = []
    for row in rows:
        label_text = row.get(label_col, "")
        if label_text == "":
            proxy_text = row.get(proxy_col, "")
            if proxy_text == "":
                continue
            label_value = 1.0 if float(proxy_text) >= proxy_threshold else 0.0
        else:
            label_value = float(label_text)

        features = []
        valid = True
        for name in feature_cols:
            value_text = row.get(name, "")
            if value_text == "":
                valid = False
                break
            value = float(value_text)
            if not math.isfinite(value):
                valid = False
                break
            features.append(value)
        if not valid:
            continue
        x_data.append(features)
        y_data.append(1.0 if label_value >= 0.5 else 0.0)

    if not x_data:
        return np.empty((0, len(feature_cols)), dtype=np.float64), np.empty((0,), dtype=np.float64)
    return np.asarray(x_data, dtype=np.float64), np.asarray(y_data, dtype=np.float64)


def main() -> int:
    parser = argparse.ArgumentParser(description="Fit logistic terrain risk model from exported CSV.")
    parser.add_argument("input_csv", help="Dataset CSV from export_terrain_training_dataset.py")
    parser.add_argument("--output-yaml", required=True, help="Output YAML file")
    parser.add_argument(
        "--features",
        default="slope_abs,roughness,sigma_h,step_up,height_span,p_climbable,p_obstacle_proxy,p_drop_proxy",
        help="Comma-separated feature columns",
    )
    parser.add_argument("--obstacle-label-col", default="obstacle_label")
    parser.add_argument("--drop-label-col", default="drop_label")
    parser.add_argument("--obstacle-proxy-col", default="p_obstacle_proxy")
    parser.add_argument("--drop-proxy-col", default="p_drop_proxy")
    parser.add_argument("--proxy-threshold", type=float, default=0.6)
    parser.add_argument("--max-iter", type=int, default=600)
    parser.add_argument("--learning-rate", type=float, default=0.05)
    parser.add_argument("--l2", type=float, default=1e-4)
    args = parser.parse_args()

    feature_cols = [item.strip() for item in args.features.split(",") if item.strip()]
    if not feature_cols:
        raise ValueError("feature list is empty")

    rows = read_csv_rows(args.input_csv)
    x_obs, y_obs = build_dataset(
        rows, feature_cols, args.obstacle_label_col, args.obstacle_proxy_col, args.proxy_threshold
    )
    x_drop, y_drop = build_dataset(
        rows, feature_cols, args.drop_label_col, args.drop_proxy_col, args.proxy_threshold
    )
    if x_obs.shape[0] < 20 or x_drop.shape[0] < 20:
        raise ValueError(
            f"not enough samples: obstacle={x_obs.shape[0]}, drop={x_drop.shape[0]} (need >=20 each)"
        )

    obs_w, obs_b = train_logistic(x_obs, y_obs, args.max_iter, args.learning_rate, args.l2)
    drop_w, drop_b = train_logistic(x_drop, y_drop, args.max_iter, args.learning_rate, args.l2)

    obs_prob = sigmoid(x_obs.dot(obs_w) + obs_b)
    drop_prob = sigmoid(x_drop.dot(drop_w) + drop_b)
    obs_auc = auc_score(y_obs, obs_prob)
    drop_auc = auc_score(y_drop, drop_prob)
    obs_brier = float(np.mean((obs_prob - y_obs) ** 2))
    drop_brier = float(np.mean((drop_prob - y_drop) ** 2))

    output: Dict[str, object] = {
        "terrain_risk_model": {
            "enabled": True,
            "obstacle": {
                "enabled": True,
                "intercept": float(obs_b),
                "coefficients": {feature_cols[i]: float(obs_w[i]) for i in range(len(feature_cols))},
            },
            "drop": {
                "enabled": True,
                "intercept": float(drop_b),
                "coefficients": {feature_cols[i]: float(drop_w[i]) for i in range(len(feature_cols))},
            },
            "metrics": {
                "obstacle_auc": float(obs_auc),
                "drop_auc": float(drop_auc),
                "obstacle_brier": obs_brier,
                "drop_brier": drop_brier,
                "samples_obstacle": int(x_obs.shape[0]),
                "samples_drop": int(x_drop.shape[0]),
            },
        }
    }

    os.makedirs(os.path.dirname(os.path.abspath(args.output_yaml)), exist_ok=True)
    with open(args.output_yaml, "w", encoding="utf-8") as stream:
        yaml.safe_dump(output, stream, allow_unicode=False, sort_keys=False)

    print("[INFO] model fitted")
    print(
        f"[INFO] obstacle: samples={x_obs.shape[0]} auc={obs_auc:.4f} brier={obs_brier:.4f}"
    )
    print(f"[INFO] drop: samples={x_drop.shape[0]} auc={drop_auc:.4f} brier={drop_brier:.4f}")
    print(f"[INFO] output: {args.output_yaml}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
