#!/usr/bin/env bash
set -euo pipefail
NODE=${1:-rc26_localization_node}
DURATION=${2:-30}
PID=$(pgrep -f "$NODE" | head -1)
[[ -z "$PID" ]] && { echo "未找到节点进程: $NODE"; exit 1; }
OUT_DIR=${3:-/tmp/localization_profile_${PID}_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$OUT_DIR"

echo "=== 话题频率采样 ==="
(timeout "$DURATION" ros2 topic hz /state_estimation >"$OUT_DIR/state_estimation_hz.log" 2>&1 || true) &
HZ_PID=$!

echo "=== CPU 亲和性快照 ==="
ps -eLo pid,tid,psr,comm | awk -v pid="$PID" '$1==pid' | tee "$OUT_DIR/psr_snapshot.log"

echo "=== PERF_METRIC 日志采样 (${DURATION}s) ==="
(timeout "$DURATION" ros2 topic echo /rosout 2>/dev/null | grep PERF_METRIC >"$OUT_DIR/perf_metric.log" || true) &
PERF_PID=$!

echo "=== /proc/<pid>/task/*/stat 线程采样 (${DURATION}s) ==="
END_SEC=$((SECONDS + DURATION))
while (( SECONDS < END_SEC )); do
    sample_ts=$(date +%s%3N)
    for task_dir in /proc/"$PID"/task/*; do
        [[ -r "$task_dir/stat" ]] || continue
        tid=$(basename "$task_dir")
        comm=$(cat "$task_dir/comm" 2>/dev/null || echo "unknown")
        psr=$(awk '{print $39}' "$task_dir/stat" 2>/dev/null || echo "-")
        echo "${sample_ts},tid=${tid},psr=${psr},comm=${comm}" >> "$OUT_DIR/task_stat_samples.log"
    done
    sleep 1
done

wait "$HZ_PID" || true
wait "$PERF_PID" || true

echo "性能采样完成，日志目录: $OUT_DIR"
