#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
一键执行 rc26_localization 最小链路验收（编译 + 启动 + 可选 rosbag/synthetic 输入 + 摘要）

用法:
  run_localization_acceptance.sh --map /abs/path/to/prior.pcd [--bag /abs/path/to/test_bag_dir_or_file]
                                 [--duration 60] [--workspace ${RC26_WS:-$HOME/RC_2026}]
                                 [--use-sim-time auto|true|false] [--output-dir /tmp/xxx]
                                 [--params-file /abs/path/to/localization.yaml]
                                 [--synthetic-input] [--skip-build]

示例:
  ./src/rc26_localization/scripts/run_localization_acceptance.sh \
    --map ${RC26_WS:-$HOME/RC_2026}/src/rc26_bringup/pcd/default.pcd \
    --synthetic-input --duration 10
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
BAG_PLAY_URI=""
DURATION=60
USE_SIM_TIME="auto"
OUTPUT_DIR=""
PARAMS_FILE=""
SYNTHETIC_INPUT=0
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
        --params-file)
            PARAMS_FILE="${2:-}"
            shift 2
            ;;
        --synthetic-input)
            SYNTHETIC_INPUT=1
            shift
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
if [[ -n "${PARAMS_FILE}" ]]; then
    PARAMS_FILE=$(expand_path "${PARAMS_FILE}")
fi

if [[ -z "${MAP_FILE}" ]]; then
    MAP_FILE="${WORKSPACE}/src/rc26_bringup/pcd/default.pcd"
    echo "[WARN] 未指定 --map，自动使用默认地图: ${MAP_FILE}"
fi
if [[ ! -f "${MAP_FILE}" ]]; then
    echo "[ERROR] 地图文件不存在: ${MAP_FILE}" >&2
    exit 1
fi
if [[ -n "${BAG_FILE}" ]]; then
    if [[ -d "${BAG_FILE}" ]]; then
        [[ -f "${BAG_FILE}/metadata.yaml" ]] || {
            echo "[ERROR] rosbag 目录缺少 metadata.yaml: ${BAG_FILE}" >&2
            exit 1
        }
        BAG_PLAY_URI="${BAG_FILE}"
    elif [[ -f "${BAG_FILE}" ]]; then
        BAG_PLAY_URI="${BAG_FILE}"
        case "${BAG_FILE}" in
            *.db3|*.mcap)
                BAG_DIR=$(dirname "${BAG_FILE}")
                [[ -f "${BAG_DIR}/metadata.yaml" ]] && BAG_PLAY_URI="${BAG_DIR}"
                ;;
        esac
    else
        echo "[ERROR] rosbag 路径不存在: ${BAG_FILE}" >&2
        exit 1
    fi
fi
if [[ "${USE_SIM_TIME}" != "auto" && "${USE_SIM_TIME}" != "true" && "${USE_SIM_TIME}" != "false" ]]; then
    echo "[ERROR] --use-sim-time 仅支持 auto|true|false" >&2
    exit 1
fi
if ! [[ "${DURATION}" =~ ^[0-9]+$ ]] || [[ "${DURATION}" -le 0 ]]; then
    echo "[ERROR] --duration 需为正整数秒" >&2
    exit 1
fi

if [[ -z "${PARAMS_FILE}" ]]; then
    PARAMS_FILE="${WORKSPACE}/src/rc26_bringup/config/localization.yaml"
fi
if [[ ! -f "${PARAMS_FILE}" ]]; then
    echo "[ERROR] 参数文件不存在: ${PARAMS_FILE}" >&2
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
mkdir -p "${OUTPUT_DIR}/raw"

LAUNCH_PID=""
BAG_PID=""
POSE_HZ_PID=""
DIAG_HZ_PID=""
SYNTH_PID=""
STATIC_TF_PID=""

cleanup() {
    for pid in "${SYNTH_PID}" "${STATIC_TF_PID}" "${BAG_PID}" "${POSE_HZ_PID}" "${DIAG_HZ_PID}" "${LAUNCH_PID}"; do
        if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
            kill "${pid}" 2>/dev/null || true
        fi
    done
}
trap cleanup EXIT INT TERM

wait_for_topic_publisher() {
    local topic="$1"
    local log_file="$2"
    local ready=0

    for _ in $(seq 1 20); do
        if timeout 3 ros2 topic info "${topic}" > "${log_file}" 2>&1 &&
            grep -Eq 'Publisher count: [1-9][0-9]*' "${log_file}"; then
            ready=1
            break
        fi
        sleep 1
    done

    if [[ "${ready}" -ne 1 ]]; then
        echo "[ERROR] 未检测到话题 publisher: ${topic}" >&2
        echo "[ERROR] 请检查: ${log_file}" >&2
        echo "[ERROR] 请检查: ${OUTPUT_DIR}/raw/localization.launch.log" >&2
        exit 1
    fi
}

assert_hz_sampled() {
    local topic="$1"
    local log_file="$2"

    if ! grep -Eq '^average rate:' "${log_file}"; then
        echo "[ERROR] 未采集到 ${topic} 的有效频率" >&2
        echo "[ERROR] 请检查: ${log_file}" >&2
        exit 1
    fi
}

echo "[INFO] 工作空间: ${WORKSPACE}"
echo "[INFO] 输出目录: ${OUTPUT_DIR}"
echo "[INFO] use_sim_time=${USE_SIM_TIME}, 验收时长=${DURATION}s"
echo "[INFO] 参数文件: ${PARAMS_FILE}"
echo "[INFO] 地图文件: ${MAP_FILE}, 合成输入=${SYNTHETIC_INPUT}"

if [[ "${SKIP_BUILD}" -eq 0 ]]; then
    echo "[INFO] 编译最小定位链 ..."
    (
        cd "${WORKSPACE}"
        MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
          --packages-select rc26_interfaces rc26_small_gicp rc26_localization rc26_bringup
    ) | tee "${OUTPUT_DIR}/raw/build.log"
else
    echo "[INFO] 跳过编译 (--skip-build)"
    echo "已按 --skip-build 跳过编译" > "${OUTPUT_DIR}/raw/build.log"
fi

set +u
source "${WORKSPACE}/install/setup.bash"
set -u
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_log}"
mkdir -p "${ROS_LOG_DIR}"

echo "[INFO] 启动 localization.launch.py ..."
(
    cd "${WORKSPACE}"
    ros2 launch rc26_bringup localization.launch.py \
        use_sim_time:="${USE_SIM_TIME}" \
        prior_pcd_file:="${MAP_FILE}" \
        localization_params_file:="${PARAMS_FILE}"
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
    echo "[ERROR] 30 秒内未检测到 localization 节点，请检查 ${OUTPUT_DIR}/raw/localization.launch.log" >&2
    exit 1
fi

wait_for_topic_publisher /localization/pose_with_cov "${OUTPUT_DIR}/raw/pose_cov_info.log"
wait_for_topic_publisher /localization/diagnostics "${OUTPUT_DIR}/raw/diagnostics_info.log"
(timeout "${DURATION}" ros2 topic hz /localization/pose_with_cov > "${OUTPUT_DIR}/raw/pose_cov_hz.log" 2>&1 || true) &
POSE_HZ_PID=$!
(timeout "${DURATION}" ros2 topic hz /localization/diagnostics > "${OUTPUT_DIR}/raw/diagnostics_hz.log" 2>&1 || true) &
DIAG_HZ_PID=$!

if [[ "${SYNTHETIC_INPUT}" -eq 1 ]]; then
    echo "[INFO] 启动合成 registered_scan 输入"
    ros2 run tf2_ros static_transform_publisher \
        --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
        --frame-id odom --child-frame-id base_link \
        > "${OUTPUT_DIR}/raw/static_tf.log" 2>&1 &
    STATIC_TF_PID=$!

    python3 "${WORKSPACE}/src/rc26_localization/scripts/publish_synthetic_loc_inputs.py" \
        --duration "${DURATION}" \
        --scan-topic registered_scan \
        --odom-frame odom \
        > "${OUTPUT_DIR}/raw/synthetic_input.log" 2>&1 &
    SYNTH_PID=$!
fi

if [[ -n "${BAG_FILE}" ]]; then
    echo "[INFO] 回放 rosbag: ${BAG_PLAY_URI}"
    ros2 bag play "${BAG_PLAY_URI}" --clock > "${OUTPUT_DIR}/raw/bag_play.log" 2>&1 &
    BAG_PID=$!
    wait "${BAG_PID}" || true
elif [[ "${SYNTHETIC_INPUT}" -eq 1 ]]; then
    wait "${SYNTH_PID}" || {
        echo "[ERROR] 合成输入进程异常退出，请检查 ${OUTPUT_DIR}/raw/synthetic_input.log" >&2
        exit 1
    }
else
    echo "[INFO] 未提供 rosbag，采集 ${DURATION}s 在线数据"
    sleep "${DURATION}"
fi

wait "${POSE_HZ_PID}" || true
wait "${DIAG_HZ_PID}" || true
assert_hz_sampled /localization/pose_with_cov "${OUTPUT_DIR}/raw/pose_cov_hz.log"
assert_hz_sampled /localization/diagnostics "${OUTPUT_DIR}/raw/diagnostics_hz.log"

python3 - "${OUTPUT_DIR}/acceptance_summary.md" "${MAP_FILE}" "${BAG_FILE}" "${PARAMS_FILE}" <<'PY'
import pathlib
import sys

summary_path = pathlib.Path(sys.argv[1])
map_file = sys.argv[2]
bag_file = sys.argv[3]
params_file = sys.argv[4]

out = [
    "# rc26_localization 最小链路验收摘要",
    "",
    "## 输入",
    f"- 地图: `{map_file}`",
    f"- rosbag: `{bag_file if bag_file else '(未提供)'}`",
    f"- 参数文件: `{params_file}`",
    "",
    "## 验收判读",
    "- `raw/pose_cov_info.log` 应显示 `/localization/pose_with_cov` 存在。",
    "- `raw/diagnostics_info.log` 应显示 `/localization/diagnostics` 存在。",
    "- `raw/pose_cov_hz.log` 应能看到 20Hz 左右的位姿输出。",
    "- `raw/diagnostics_hz.log` 应能看到 2Hz 左右的诊断输出，诊断中包含开局重定位状态。",
    "- `raw/localization.launch.log` 中不应出现 graph/P4/backend/route/health 相关日志。",
    "",
    "## 原始文件",
    "- `raw/build.log`",
    "- `raw/localization.launch.log`",
    "- `raw/pose_cov_info.log`",
    "- `raw/diagnostics_info.log`",
    "- `raw/pose_cov_hz.log`",
    "- `raw/diagnostics_hz.log`",
]

summary_path.write_text("\n".join(out) + "\n", encoding="utf-8")
print("\n".join(out))
PY

echo "[INFO] 验收完成，摘要: ${OUTPUT_DIR}/acceptance_summary.md"
