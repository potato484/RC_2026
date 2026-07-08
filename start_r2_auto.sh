#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./start_r2_auto.sh [options]

Options:
  --side-config-file <path>          红蓝方选择 YAML，默认：src/rc26_bringup/config/r2_active_side.yaml
  --runtime-config-file <path>       直接覆盖完整运行配置 YAML，透传给 bringup
  --no-realsense                     关闭 RealSense D455，默认会启动 RealSense
  --use-rviz                         启动 RViz2 观察界面，默认关闭
  --recover-mid360-stream            启动前尝试恢复 Mid-360 数据流
  --mcu-port <dev>                   目标 MCU 串口，透传给 rc26_mcu_transport
  --mcu-baudrate <baud>              目标 MCU 串口波特率，透传给 rc26_mcu_transport
  --startup-delay-decision-sec <sec> decision_node 错峰启动延时
  --startup-delay-realsense-sec <sec> RealSense 错峰启动延时
  --extra-launch-arg <name:=value>   追加透传 bringup launch 参数，可重复
  --dry-run                          只打印命令，不实际启动
  -h, --help                         显示本帮助

Notes:
  本脚本是 R2 自动决策/比赛链路快捷入口。
  红蓝方路线与行为树仍由 rc26_bringup 根据 r2_active_side.yaml 选择的运行配置加载。
  默认启动 RealSense D455，匹配完整 MC + MF 预选链路。
  运行前请确认 start_r2_teleop.sh 或其它 /cmd_vel 发布者没有同时运行。

Examples:
  ./start_r2_auto.sh
  ./start_r2_auto.sh --dry-run
  ./start_r2_auto.sh --no-realsense --use-rviz
EOF
}

print_cmd() {
  printf '+ '
  printf '%q ' "$@"
  printf '\n'
}

source_with_relaxed_nounset() {
  local nounset_was_enabled="false"
  case "$-" in
    *u*)
      nounset_was_enabled="true"
      set +u
      ;;
  esac

  # setup.bash 路径由运行时工作区决定，静态检查无法解析。
  source "$1"

  if [[ "${nounset_was_enabled}" == "true" ]]; then
    set -u
  fi
}

abs_path() {
  local path="$1"
  if [[ "${path}" = /* ]]; then
    printf '%s\n' "${path}"
  else
    printf '%s\n' "$(pwd)/${path}"
  fi
}

yaml_scalar_value() {
  local file="$1"
  local key="$2"
  python3 - "$file" "$key" <<'PY'
import sys
import yaml

path, key = sys.argv[1], sys.argv[2]
with open(path, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
value = data.get(key, "")
if value is None:
    value = ""
print(str(value).strip())
PY
}

yaml_runtime_config_value() {
  local file="$1"
  local side="$2"
  python3 - "$file" "$side" <<'PY'
import sys
import yaml

path, side = sys.argv[1], sys.argv[2]
with open(path, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f) or {}
runtime_configs = data.get("runtime_configs") or {}
value = runtime_configs.get(side, "")
if value is None:
    value = ""
print(str(value).strip())
PY
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="${RC26_WS:-${script_dir}}"
setup_file="${workspace_dir}/install/setup.bash"
side_config_file="${workspace_dir}/src/rc26_bringup/config/r2_active_side.yaml"
runtime_config_file=""
use_realsense="true"
use_rviz="false"
recover_mid360_stream="false"
mcu_port=""
mcu_baudrate=""
startup_delay_decision_sec=""
startup_delay_realsense_sec=""
dry_run="false"
extra_launch_args=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --side-config-file)
      side_config_file="${2:-}"
      shift 2
      ;;
    --runtime-config-file)
      runtime_config_file="${2:-}"
      shift 2
      ;;
    --no-realsense)
      use_realsense="false"
      shift
      ;;
    --use-rviz)
      use_rviz="true"
      shift
      ;;
    --recover-mid360-stream)
      recover_mid360_stream="true"
      shift
      ;;
    --mcu-port)
      mcu_port="${2:-}"
      shift 2
      ;;
    --mcu-baudrate)
      mcu_baudrate="${2:-}"
      shift 2
      ;;
    --startup-delay-decision-sec)
      startup_delay_decision_sec="${2:-}"
      shift 2
      ;;
    --startup-delay-realsense-sec)
      startup_delay_realsense_sec="${2:-}"
      shift 2
      ;;
    --extra-launch-arg)
      extra_launch_args+=("${2:-}")
      shift 2
      ;;
    --dry-run)
      dry_run="true"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

side_config_file="$(abs_path "${side_config_file}")"
if [[ -n "${runtime_config_file}" ]]; then
  runtime_config_file="$(abs_path "${runtime_config_file}")"
fi

if [[ ! -f "${side_config_file}" ]]; then
  echo "side config file not found: ${side_config_file}" >&2
  exit 1
fi

active_side="$(yaml_scalar_value "${side_config_file}" "active_side" | tr '[:upper:]' '[:lower:]')"
preselection_mode="$(yaml_scalar_value "${side_config_file}" "preselection_mode" | tr '[:upper:]' '[:lower:]')"
first_repeat_enable="$(yaml_scalar_value "${side_config_file}" "first_preselection_mc_repeat_enable")"
first_repeat_max_count="$(yaml_scalar_value "${side_config_file}" "first_preselection_mc_repeat_max_count")"
first_repeat_base_forward="$(yaml_scalar_value "${side_config_file}" "first_preselection_mc_repeat_base_forward_x_m")"
first_repeat_forward_step="$(yaml_scalar_value "${side_config_file}" "first_preselection_mc_repeat_forward_x_step_m")"
case "${active_side}" in
  red|blue)
    ;;
  *)
    echo "Invalid active_side in ${side_config_file}: ${active_side}. Expected red or blue." >&2
    exit 1
    ;;
esac

selected_runtime_config="$(yaml_runtime_config_value "${side_config_file}" "${active_side}")"
if [[ -z "${selected_runtime_config}" ]]; then
  echo "Missing runtime_configs.${active_side} in ${side_config_file}" >&2
  exit 1
fi
if [[ "${selected_runtime_config}" != /* ]]; then
  selected_runtime_config="$(cd "$(dirname "${side_config_file}")" && pwd)/${selected_runtime_config}"
fi
if [[ ! -f "${selected_runtime_config}" ]]; then
  echo "selected runtime config not found: ${selected_runtime_config}" >&2
  exit 1
fi

if [[ -n "${runtime_config_file}" && ! -f "${runtime_config_file}" ]]; then
  echo "runtime config file not found: ${runtime_config_file}" >&2
  exit 1
fi

for arg in "${extra_launch_args[@]}"; do
  if [[ "${arg}" != *:=* ]]; then
    echo "Invalid --extra-launch-arg: ${arg}. Expected name:=value." >&2
    exit 1
  fi
done

launch_cmd=(
  ros2 launch rc26_bringup bringup.launch.py
  run_mode:=navigation
  "side_config_file:=${side_config_file}"
  "use_realsense:=${use_realsense}"
  "use_rviz:=${use_rviz}"
  "recover_mid360_stream:=${recover_mid360_stream}"
)

if [[ -n "${runtime_config_file}" ]]; then
  launch_cmd+=("runtime_config_file:=${runtime_config_file}")
fi
if [[ -n "${mcu_port}" ]]; then
  launch_cmd+=("mcu_transport_target_serial_port:=${mcu_port}")
fi
if [[ -n "${mcu_baudrate}" ]]; then
  launch_cmd+=("mcu_transport_target_baudrate:=${mcu_baudrate}")
fi
if [[ -n "${startup_delay_decision_sec}" ]]; then
  launch_cmd+=("startup_delay_decision_sec:=${startup_delay_decision_sec}")
fi
if [[ -n "${startup_delay_realsense_sec}" ]]; then
  launch_cmd+=("startup_delay_realsense_sec:=${startup_delay_realsense_sec}")
fi
launch_cmd+=("${extra_launch_args[@]}")

active_side_switch_listener_cmd=(
  python3
  "${workspace_dir}/src/rc26_bringup/scripts/active_side_switch_listener.py"
  "--side-config-file"
  "${side_config_file}"
  "--feedback-topic"
  "/mechanism/command_feedback"
  "--switch-feedback-id"
  "0x13"
)

print_summary() {
  echo "Workspace: ${workspace_dir}"
  echo "Side config: ${side_config_file}"
  echo "Active side: ${active_side}"
  echo "Preselection mode: ${preselection_mode:-first}"
  echo "First MC repeat: enable=${first_repeat_enable:-true}, max_count=${first_repeat_max_count:-1}, base_forward_x_m=${first_repeat_base_forward:-0.2}, forward_step_m=${first_repeat_forward_step:-0.2}"
  echo "0x13 active-side switch listener: enabled by start_r2_auto.sh only"
  echo "Selected runtime config: ${selected_runtime_config}"
  if [[ -n "${runtime_config_file}" ]]; then
    echo "Runtime config override: ${runtime_config_file}"
  fi
  echo "RealSense D455: ${use_realsense}"
  echo "RViz2: ${use_rviz}"
  echo "Recover Mid-360 stream: ${recover_mid360_stream}"
}

if [[ "${dry_run}" == "true" ]]; then
  print_summary
  print_cmd source "${setup_file}"
  print_cmd "${active_side_switch_listener_cmd[@]}"
  print_cmd "${launch_cmd[@]}"
  exit 0
fi

if [[ ! -f "${setup_file}" ]]; then
  echo "setup.bash not found: ${setup_file}" >&2
  exit 1
fi

source_with_relaxed_nounset "${setup_file}"

print_summary
echo "Press Ctrl+C to stop the R2 auto stack."
print_cmd "${active_side_switch_listener_cmd[@]}"
print_cmd "${launch_cmd[@]}"

"${active_side_switch_listener_cmd[@]}" &
active_side_switch_listener_pid="$!"

"${launch_cmd[@]}" &
launch_pid="$!"

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  if [[ -n "${active_side_switch_listener_pid:-}" ]]; then
    kill "${active_side_switch_listener_pid}" 2>/dev/null || true
    wait "${active_side_switch_listener_pid}" 2>/dev/null || true
  fi
  if [[ -n "${launch_pid:-}" ]]; then
    echo
    echo "Stopping R2 auto stack..."
    kill "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
  exit "${exit_code}"
}

trap cleanup EXIT INT TERM

wait "${launch_pid}"
