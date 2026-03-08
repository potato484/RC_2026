#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
一键执行 rc26_localization 验收（编译 + 启动 + 可选 rosbag 回放 + 指标汇总）

用法:
  run_localization_acceptance.sh --map /abs/path/to/prior.pcd [--bag /abs/path/to/test.mcap]
                                 [--duration 180] [--workspace ${RC26_WS:-$HOME/RC_2026}]
                                 [--use-sim-time auto|true|false] [--output-dir /tmp/xxx]
                                 [--skip-build]

示例:
  export RC26_WS=${RC26_WS:-$HOME/RC_2026}
  ./src/rc26_localization/scripts/run_localization_acceptance.sh \
    --map ${RC26_WS}/src/rc26_bringup/pcd/my_map.pcd \
    --bag /data/loc_long_corridor.mcap \
    --duration 240
EOF
}

expand_path() {
    local path="$1"
    path="${path/#\~/$HOME}"
    printf '%s' "${path}"
}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
DEFAULT_WORKSPACE=$(cd "${RC26_WS:-${SCRIPT_DIR}/../../..}" && pwd)

WORKSPACE="${DEFAULT_WORKSPACE}"
MAP_FILE=""
BAG_FILE=""
DURATION=120
USE_SIM_TIME="auto"
OUTPUT_DIR=""
SKIP_BUILD=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --map)
            MAP_FILE="${2:-}"
            shift 2
            ;;
        --bag)
            BAG_FILE="${2:-}"
            shift 2
            ;;
        --duration)
            DURATION="${2:-}"
            shift 2
            ;;
        --workspace)
            WORKSPACE="${2:-}"
            shift 2
            ;;
        --use-sim-time)
            USE_SIM_TIME="${2:-}"
            shift 2
            ;;
        --output-dir)
            OUTPUT_DIR="${2:-}"
            shift 2
            ;;
        --skip-build)
            SKIP_BUILD=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[ERROR] 未知参数: $1" >&2
            usage
            exit 1
            ;;
    esac
done

WORKSPACE=$(expand_path "${WORKSPACE}")
MAP_FILE=$(expand_path "${MAP_FILE}")
if [[ -n "${BAG_FILE}" ]]; then
    BAG_FILE=$(expand_path "${BAG_FILE}")
fi
if [[ -n "${OUTPUT_DIR}" ]]; then
    OUTPUT_DIR=$(expand_path "${OUTPUT_DIR}")
fi

if [[ -z "${MAP_FILE}" ]]; then
    echo "[ERROR] 必须提供 --map /abs/path/to/prior.pcd" >&2
    exit 1
fi
if [[ ! -f "${MAP_FILE}" ]]; then
    echo "[ERROR] 地图文件不存在: ${MAP_FILE}" >&2
    exit 1
fi
if [[ -n "${BAG_FILE}" && ! -f "${BAG_FILE}" ]]; then
    echo "[ERROR] rosbag 文件不存在: ${BAG_FILE}" >&2
    exit 1
fi
if [[ "${USE_SIM_TIME}" != "auto" && "${USE_SIM_TIME}" != "true" && "${USE_SIM_TIME}" != "false" ]]; then
    echo "[ERROR] --use-sim-time 仅支持 auto|true|false" >&2
    exit 1
fi
if ! [[ "${DURATION}" =~ ^[0-9]+$ ]] || [[ "${DURATION}" -le 0 ]]; then
    echo "[ERROR] --duration 需为正整数秒" >&2
    exit 1
fi

if [[ "${USE_SIM_TIME}" == "auto" ]]; then
    if [[ -n "${BAG_FILE}" ]]; then
        USE_SIM_TIME="true"
    else
        USE_SIM_TIME="false"
    fi
fi

if [[ -z "${OUTPUT_DIR}" ]]; then
    OUTPUT_DIR="/tmp/localization_acceptance_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "${OUTPUT_DIR}"
mkdir -p "${OUTPUT_DIR}/raw"

LAUNCH_PID=""
BAG_PID=""
ROSOUT_PID=""
POSE_HZ_PID=""
DIAG_HZ_PID=""
HEALTH_HZ_PID=""
BACKEND_HZ_PID=""
ROUTE_HZ_PID=""

cleanup() {
    for pid in "${BAG_PID}" "${ROSOUT_PID}" "${POSE_HZ_PID}" "${DIAG_HZ_PID}" "${HEALTH_HZ_PID}" "${BACKEND_HZ_PID}" "${ROUTE_HZ_PID}" "${LAUNCH_PID}"; do
        if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
            kill "${pid}" 2>/dev/null || true
        fi
    done
}
trap cleanup EXIT INT TERM

echo "[INFO] 工作空间: ${WORKSPACE}"
echo "[INFO] 输出目录: ${OUTPUT_DIR}"
echo "[INFO] use_sim_time=${USE_SIM_TIME}, duration=${DURATION}s"

if [[ "${SKIP_BUILD}" -eq 0 ]]; then
    echo "[INFO] 编译 rc26_localization ..."
    (
        cd "${WORKSPACE}"
        colcon build --symlink-install --parallel-workers 3 --packages-select rc26_interfaces rc26_localization rc26_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
    ) | tee "${OUTPUT_DIR}/raw/build.log"
else
    echo "[INFO] 跳过编译 (--skip-build)"
fi

source "${WORKSPACE}/install/setup.bash"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_log}"
mkdir -p "${ROS_LOG_DIR}"

echo "[INFO] 启动 localization.launch.py ..."
(
    cd "${WORKSPACE}"
    ros2 launch rc26_bringup localization.launch.py \
        use_sim_time:="${USE_SIM_TIME}" \
        prior_pcd_file:="${MAP_FILE}"
) > "${OUTPUT_DIR}/raw/localization.launch.log" 2>&1 &
LAUNCH_PID=$!

echo "[INFO] 等待 localization 节点上线 ..."
READY=0
for _ in $(seq 1 30); do
    if ros2 node list 2>/dev/null | grep -Eq '(^|/)localization$'; then
        READY=1
        break
    fi
    sleep 1
done
if [[ "${READY}" -ne 1 ]]; then
    echo "[WARN] 30 秒内未检测到 localization 节点，请检查 ${OUTPUT_DIR}/raw/localization.launch.log"
fi

timeout 10 ros2 topic info /localization/pose_with_cov > "${OUTPUT_DIR}/raw/pose_cov_info.log" 2>&1 || true
timeout 10 ros2 topic info /localization/diagnostics > "${OUTPUT_DIR}/raw/diagnostics_info.log" 2>&1 || true
timeout 10 ros2 topic info /localization/health > "${OUTPUT_DIR}/raw/health_info.log" 2>&1 || true
timeout 10 ros2 topic info /localization/backend_status > "${OUTPUT_DIR}/raw/backend_status_info.log" 2>&1 || true
timeout 10 ros2 topic info /localization/route_observability > "${OUTPUT_DIR}/raw/route_observability_info.log" 2>&1 || true

(timeout "${DURATION}" ros2 topic hz /localization/pose_with_cov > "${OUTPUT_DIR}/raw/pose_cov_hz.log" 2>&1 || true) &
POSE_HZ_PID=$!
(timeout "${DURATION}" ros2 topic hz /localization/diagnostics > "${OUTPUT_DIR}/raw/diagnostics_hz.log" 2>&1 || true) &
DIAG_HZ_PID=$!
(timeout "${DURATION}" ros2 topic hz /localization/health > "${OUTPUT_DIR}/raw/health_hz.log" 2>&1 || true) &
HEALTH_HZ_PID=$!
(timeout "${DURATION}" ros2 topic hz /localization/backend_status > "${OUTPUT_DIR}/raw/backend_status_hz.log" 2>&1 || true) &
BACKEND_HZ_PID=$!
(timeout "${DURATION}" ros2 topic hz /localization/route_observability > "${OUTPUT_DIR}/raw/route_observability_hz.log" 2>&1 || true) &
ROUTE_HZ_PID=$!
(timeout "${DURATION}" ros2 topic echo /rosout 2>/dev/null | grep -E "PERF_METRIC|RELOC_METRIC|定位状态切换|GLOBAL_RECOVERY" > "${OUTPUT_DIR}/raw/metrics.log" || true) &
ROSOUT_PID=$!

if [[ -n "${BAG_FILE}" ]]; then
    echo "[INFO] 回放 rosbag: ${BAG_FILE}"
    ros2 bag play "${BAG_FILE}" --clock > "${OUTPUT_DIR}/raw/bag_play.log" 2>&1 &
    BAG_PID=$!
    wait "${BAG_PID}" || true
else
    echo "[INFO] 未提供 rosbag，采集 ${DURATION}s 在线数据"
    sleep "${DURATION}"
fi

wait "${ROSOUT_PID}" || true
wait "${POSE_HZ_PID}" || true
wait "${DIAG_HZ_PID}" || true
wait "${HEALTH_HZ_PID}" || true
wait "${BACKEND_HZ_PID}" || true
wait "${ROUTE_HZ_PID}" || true

python3 - "${OUTPUT_DIR}/raw/metrics.log" "${OUTPUT_DIR}/acceptance_summary.md" "${MAP_FILE}" "${BAG_FILE}" <<'PY'
import pathlib
import re
import sys
from collections import Counter

metrics_path = pathlib.Path(sys.argv[1])
summary_path = pathlib.Path(sys.argv[2])
map_file = sys.argv[3]
bag_file = sys.argv[4]

lines = []
if metrics_path.exists():
    lines = [line.strip() for line in metrics_path.read_text(encoding="utf-8", errors="ignore").splitlines() if line.strip()]

perf_local = sum("PERF_METRIC phase=LOCAL" in line for line in lines)
perf_l0 = sum("PERF_METRIC phase=L0" in line for line in lines)
perf_l1 = sum("PERF_METRIC phase=L1" in line for line in lines)
perf_l2 = sum("PERF_METRIC phase=L2" in line for line in lines)
global_recovery = sum("GLOBAL_RECOVERY" in line for line in lines)

reloc_total = 0
reloc_accepted = 0
winner_counter = Counter()
path_counter = Counter()

for line in lines:
    if "RELOC_METRIC" not in line:
        continue
    reloc_total += 1
    payload = line.split("RELOC_METRIC", 1)[1].lstrip(" ,")
    kv = {}
    for part in payload.split(","):
        if "=" not in part:
            continue
        k, v = part.split("=", 1)
        kv[k.strip()] = v.strip()
    if kv.get("accepted") == "1":
        reloc_accepted += 1
    winner_counter[kv.get("winner_channel", "unknown")] += 1
    path_counter[kv.get("path_used", "unknown")] += 1

accept_rate = 0.0 if reloc_total == 0 else (reloc_accepted / reloc_total) * 100.0

def dict_to_lines(title, data):
    if not data:
        return [f"- {title}: 无"]
    return [f"- {title}: " + ", ".join(f"{k}={v}" for k, v in sorted(data.items()))]

out = []
out.append("# rc26_localization 验收摘要")
out.append("")
out.append("## 输入")
out.append(f"- map: `{map_file}`")
out.append(f"- bag: `{bag_file if bag_file else '(未提供，在线采样)'}`")
out.append("")
out.append("## 核心指标")
out.append(f"- `PERF_METRIC phase=LOCAL` 条数: **{perf_local}**")
out.append(f"- `GLOBAL_RECOVERY` 相关日志条数: **{global_recovery}**")
out.append(f"- `RELOC_METRIC` 总次数: **{reloc_total}**")
out.append(f"- `RELOC_METRIC accepted=1` 次数: **{reloc_accepted}**")
out.append(f"- 重定位接受率: **{accept_rate:.1f}%**")
out.append(f"- `PERF_METRIC` 通道样本数: L0={perf_l0}, L1={perf_l1}, L2={perf_l2}")
out.extend(dict_to_lines("winner_channel 分布", winner_counter))
out.extend(dict_to_lines("path_used 分布", path_counter))
out.append("")
out.append("## 验收判读建议")
out.append("- 长廊退化可用性：`GLOBAL_RECOVERY` 越少越好，建议与改造前同 bag 对比。")
out.append("- 重定位成功率：重点看 `accepted=1` 和 `winner_channel` 分布是否符合预期。")
out.append("- TF 连续性：结合 `raw/pose_cov_hz.log` 与 `raw/localization.launch.log` 检查是否有明显中断或跳变告警。")
out.append("- 可观测性：确认 `raw/pose_cov_info.log` 与 `raw/diagnostics_info.log` 显示 topic 存在且类型正确。")
out.append("- P0 话题链：确认 `raw/health_info.log` 与 `raw/backend_status_info.log` 均可读。")
out.append("- P2 话题链：确认 `raw/route_observability_info.log` 与 `raw/route_observability_hz.log` 正常输出。")
out.append("")
out.append("## 原始文件")
out.append("- `raw/build.log`")
out.append("- `raw/localization.launch.log`")
out.append("- `raw/metrics.log`")
out.append("- `raw/pose_cov_hz.log`")
out.append("- `raw/diagnostics_hz.log`")
out.append("- `raw/health_hz.log`")
out.append("- `raw/backend_status_hz.log`")
out.append("- `raw/route_observability_hz.log`")

summary_path.write_text("\n".join(out) + "\n", encoding="utf-8")
print("\n".join(out))
PY

echo "[INFO] 验收完成，摘要: ${OUTPUT_DIR}/acceptance_summary.md"
