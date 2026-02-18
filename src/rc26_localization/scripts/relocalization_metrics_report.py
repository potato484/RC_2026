#!/usr/bin/env python3
"""Parse rc26_localization RELOC_METRIC logs and report latency/quality stats."""

import argparse
import math
from collections import Counter


def percentile(values, p):
    if not values:
        return math.nan
    vals = sorted(values)
    if len(vals) == 1:
        return vals[0]
    rank = (len(vals) - 1) * p
    low = int(math.floor(rank))
    high = int(math.ceil(rank))
    if low == high:
        return vals[low]
    weight = rank - low
    return vals[low] * (1.0 - weight) + vals[high] * weight


def parse_metric_line(line):
    marker = "RELOC_METRIC,"
    idx = line.find(marker)
    if idx < 0:
        return None
    payload = line[idx + len(marker):].strip()
    result = {}
    for part in payload.split(','):
        if '=' not in part:
            continue
        k, v = part.split('=', 1)
        result[k.strip()] = v.strip()
    return result


def to_float(d, key):
    try:
        return float(d.get(key, "nan"))
    except ValueError:
        return math.nan


def to_int(d, key):
    try:
        return int(float(d.get(key, "0")))
    except ValueError:
        return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log_file", help="Path to ros2 log text file")
    args = parser.parse_args()

    rows = []
    with open(args.log_file, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = parse_metric_line(line)
            if m:
                rows.append(m)

    if not rows:
        print("No RELOC_METRIC lines found.")
        return 1

    t_total = [to_float(r, "t_total_ms") for r in rows if not math.isnan(to_float(r, "t_total_ms"))]
    t_l1 = [to_float(r, "t_l1_ms") for r in rows if not math.isnan(to_float(r, "t_l1_ms"))]
    t_l2 = [to_float(r, "t_l2_ms") for r in rows if not math.isnan(to_float(r, "t_l2_ms"))]
    fitness = [to_float(r, "best_fitness") for r in rows if not math.isnan(to_float(r, "best_fitness"))]
    score_j = [to_float(r, "best_J") for r in rows if not math.isnan(to_float(r, "best_J"))]
    accepted = [to_int(r, "accepted") for r in rows]
    paths = Counter(r.get("path_used", "unknown") for r in rows)
    reasons = Counter(r.get("trigger_reason", "unknown") for r in rows)

    print(f"samples: {len(rows)}")
    print(f"accepted: {sum(accepted)}/{len(accepted)} ({100.0 * sum(accepted) / max(1, len(accepted)):.1f}%)")
    print(f"t_total_ms: p50={percentile(t_total, 0.50):.2f}, p95={percentile(t_total, 0.95):.2f}")
    print(f"t_l1_ms:    p50={percentile(t_l1, 0.50):.2f}, p95={percentile(t_l1, 0.95):.2f}")
    print(f"t_l2_ms:    p50={percentile(t_l2, 0.50):.2f}, p95={percentile(t_l2, 0.95):.2f}")
    print(f"best_fitness: p50={percentile(fitness, 0.50):.4f}, p95={percentile(fitness, 0.95):.4f}")
    print(f"best_J:       p50={percentile(score_j, 0.50):.4f}, p95={percentile(score_j, 0.95):.4f}")

    print("path_used breakdown:")
    for k, v in sorted(paths.items(), key=lambda kv: (-kv[1], kv[0])):
        print(f"  {k}: {v}")

    print("trigger_reason breakdown:")
    for k, v in sorted(reasons.items(), key=lambda kv: (-kv[1], kv[0])):
        print(f"  {k}: {v}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
