#!/usr/bin/env bash
set -euo pipefail
NODE=${1:-rc26_localization_node}
PID=$(pgrep -f "$NODE" | head -1)
[[ -z "$PID" ]] && { echo "未找到节点进程: $NODE"; exit 1; }

taskset -cp 0-4 "$PID"

for TID in /proc/"$PID"/task/*/; do
    tid=$(basename "$TID")
    comm=$(cat "$TID/comm" 2>/dev/null || true)
    if [[ "$comm" =~ gicp|omp|registration|lio ]]; then
        taskset -cp 1-4 "$tid" 2>/dev/null || true
        chrt -f -p 60 "$tid" 2>/dev/null || true
    else
        taskset -cp 5-7 "$tid" 2>/dev/null || true
    fi
done
echo "实时优先级设置完成: $NODE (pid=$PID)"
