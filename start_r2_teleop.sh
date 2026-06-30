#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./start_r2_teleop.sh [options]

Options:
  --mode <stick|dpad>         控制模式，默认：dpad
                              stick = 左摇杆控制 vx/vy，右摇杆左右控制 wz
                              dpad  = 十字键控制 vx/vy，X/B 控制 wz
  --v-linear <m/s>            最大线速度，默认：2.0
  --v-angular <rad/s>         最大角速度，默认：2.0
  --cmd-vel-topic <topic>     遥控输出话题，默认：cmd_vel
  --device-name <name>        手柄设备名，默认：Xbox 360 Controller
  --joy-node-deadzone <val>   joy_node 死区，默认：0.02
  --autorepeat-rate <hz>      joy_node 自动重复发布频率，默认：50.0
  --joy-deadzone <val>        Stick 模式死区，默认：0.35
  --deadzone-hyst <val>       Stick 模式死区滞回宽度，默认：0.02
  --joy-timeout-s <sec>       遥控看门狗超时，默认：0.3
  --max-accel <m/s^2>         最大线加速度，默认：1.5
  --max-alpha <rad/s^2>       最大角加速度，默认：3.0
  --stop-repeat-n <count>     零速指令重复帧数，默认：10
  --require-deadman           要求持续按住 deadman 安全键
  --deadman-button <index>    deadman 安全键编号，默认：4
  --mcu-port <dev>            目标 MCU 串口，默认：/dev/ttyUSB0
  --mcu-baudrate <baud>       目标 MCU 串口波特率，默认：1000000
  --mcu-open-retry-ms <ms>    目标 MCU 串口初始打开重试周期，默认：1000
  --mcu-diagnostics-ms <ms>   MCU transport diagnostics 周期，默认：1000
  --dry-run                   只打印命令，不实际启动
  -h, --help                  显示本帮助

Notes:
  本脚本启动 rc26_mcu_transport、joy_node、telecontrol 和前/后推杆 sidecar。
  /mechanism/send_command 与 /mechanism/command_feedback 由 rc26_mcu_transport 提供。
  /cmd_vel 的底盘硬件消费方由 rc26_mcu_transport 默认提供。

Examples:
  ./start_r2_teleop.sh
  ./start_r2_teleop.sh --mode stick
  ./start_r2_teleop.sh --cmd-vel-topic cmd_vel_external
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

run_and_track() {
  print_cmd "$@"
  "$@" &
  pids+=("$!")
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="${RC26_WS:-${script_dir}}"
setup_file="${workspace_dir}/install/setup.bash"

mode="dpad"
v_linear="0.5"
v_angular="0.5"
cmd_vel_topic="cmd_vel"
device_name="Xbox 360 Controller"
joy_node_deadzone="0.02"
autorepeat_rate="50.0"
joy_deadzone="0.35"
deadzone_hyst="0.02"
joy_timeout_s="0.3"
max_accel="1.5"
max_alpha="3.0"
stop_repeat_n="10"
require_deadman="false"
deadman_button="4"
mcu_port="/dev/ttyUSB0"
mcu_baudrate="1000000"
mcu_open_retry_ms="1000"
mcu_diagnostics_ms="1000"
dry_run="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      mode="${2:-}"
      shift 2
      ;;
    --v-linear)
      v_linear="${2:-}"
      shift 2
      ;;
    --v-angular)
      v_angular="${2:-}"
      shift 2
      ;;
    --cmd-vel-topic)
      cmd_vel_topic="${2:-}"
      shift 2
      ;;
    --device-name)
      device_name="${2:-}"
      shift 2
      ;;
    --joy-node-deadzone)
      joy_node_deadzone="${2:-}"
      shift 2
      ;;
    --autorepeat-rate)
      autorepeat_rate="${2:-}"
      shift 2
      ;;
    --joy-deadzone)
      joy_deadzone="${2:-}"
      shift 2
      ;;
    --deadzone-hyst)
      deadzone_hyst="${2:-}"
      shift 2
      ;;
    --joy-timeout-s)
      joy_timeout_s="${2:-}"
      shift 2
      ;;
    --max-accel)
      max_accel="${2:-}"
      shift 2
      ;;
    --max-alpha)
      max_alpha="${2:-}"
      shift 2
      ;;
    --stop-repeat-n)
      stop_repeat_n="${2:-}"
      shift 2
      ;;
    --require-deadman)
      require_deadman="true"
      shift
      ;;
    --deadman-button)
      deadman_button="${2:-}"
      shift 2
      ;;
    --mcu-port)
      mcu_port="${2:-}"
      shift 2
      ;;
    --mcu-baudrate)
      mcu_baudrate="${2:-}"
      shift 2
      ;;
    --mcu-open-retry-ms)
      mcu_open_retry_ms="${2:-}"
      shift 2
      ;;
    --mcu-diagnostics-ms)
      mcu_diagnostics_ms="${2:-}"
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

case "${mode}" in
  stick|joystick)
    mode="stick"
    teleop_executable="rc26_telecontrol"
    ;;
  dpad|hat)
    mode="dpad"
    teleop_executable="rc26_telecontrol_dpad"
    ;;
  *)
    echo "Invalid --mode: ${mode}. Expected stick or dpad." >&2
    usage
    exit 1
    ;;
esac

mcu_transport_cmd=(
  ros2 run rc26_mcu_transport mcu_transport_node
  --ros-args
  -p "target_serial_port:=${mcu_port}"
  -p "target_baudrate:=${mcu_baudrate}"
  -p "open_retry_period_ms:=${mcu_open_retry_ms}"
  -p "diagnostics_period_ms:=${mcu_diagnostics_ms}"
  -p "enable_chassis_cmd_vel_consumer:=true"
  -p "chassis_cmd_vel_topic:=${cmd_vel_topic}"
  -p "chassis_target_send_rate_hz:=50"
  -p "chassis_cmd_vel_timeout_ms:=200"
  -p "chassis_v_max_mps:=2.0"
  -p "chassis_w_max_radps:=2.0"
  -p "chassis_stop_repeat_n:=10"
)

joy_cmd=(
  ros2 run joy joy_node
  --ros-args
  -p "device_name:=${device_name}"
  -p "deadzone:=${joy_node_deadzone}"
  -p "autorepeat_rate:=${autorepeat_rate}"
)

teleop_cmd=(
  ros2 run rc26_telecontrol "${teleop_executable}"
  --ros-args
  -p "cmd_vel_topic:=${cmd_vel_topic}"
  -p "v_linear:=${v_linear}"
  -p "v_angular:=${v_angular}"
  -p "joy_timeout_s:=${joy_timeout_s}"
  -p "max_accel:=${max_accel}"
  -p "max_alpha:=${max_alpha}"
  -p "stop_repeat_n:=${stop_repeat_n}"
  -p "require_deadman:=${require_deadman}"
  -p "deadman_button:=${deadman_button}"
)

if [[ "${mode}" == "stick" ]]; then
  teleop_cmd+=(
    -p "joy_deadzone:=${joy_deadzone}"
    -p "deadzone_hyst:=${deadzone_hyst}"
  )
fi

front_pushrod_cmd=(
  ros2 run rc26_telecontrol rc26_telecontrol_front_pushrod_buttons
)

rear_pushrod_cmd=(
  ros2 run rc26_telecontrol rc26_telecontrol_rear_pushrod_buttons
)

print_summary() {
  echo "Workspace: ${workspace_dir}"
  echo "Mode: ${mode}"
  echo "cmd_vel topic: ${cmd_vel_topic}"
  echo "Linear speed limit: ${v_linear} m/s"
  echo "Angular speed limit: ${v_angular} rad/s"
  echo "MCU transport: ${mcu_port} @ ${mcu_baudrate}"
  echo "cmd_vel consumer: rc26_mcu_transport (${cmd_vel_topic}, 2.0 m/s, 2.0 rad/s)"
}

if [[ "${dry_run}" == "true" ]]; then
  print_summary
  print_cmd source "${setup_file}"
  print_cmd "${mcu_transport_cmd[@]}"
  print_cmd "${joy_cmd[@]}"
  print_cmd "${teleop_cmd[@]}"
  print_cmd "${front_pushrod_cmd[@]}"
  print_cmd "${rear_pushrod_cmd[@]}"
  exit 0
fi

if [[ ! -f "${setup_file}" ]]; then
  echo "setup.bash not found: ${setup_file}" >&2
  exit 1
fi

source_with_relaxed_nounset "${setup_file}"

print_summary
echo "Press Ctrl+C to stop all nodes."

pids=()

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  if (( ${#pids[@]} > 0 )); then
    echo
    echo "Stopping R2 teleop stack..."
    kill "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
  fi
  exit "${exit_code}"
}

trap cleanup EXIT INT TERM

run_and_track "${mcu_transport_cmd[@]}"
run_and_track "${joy_cmd[@]}"
run_and_track "${teleop_cmd[@]}"
run_and_track "${front_pushrod_cmd[@]}"
run_and_track "${rear_pushrod_cmd[@]}"

wait -n "${pids[@]}"
