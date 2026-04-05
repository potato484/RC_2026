#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./start_r2_mcu_teleop.sh [options]

Purpose:
  Start the minimum teleop stack for directly sending joystick target velocity
  to the MCU target serial port.

Options:
  --mode <stick|dpad>                 Control mode, default: dpad
  --feedback-serial-port <device>     Optional MCU feedback serial port, default: /dev/ttyUSB0
  --target-serial-port <device>       MCU target serial port, default: /dev/ttyUSB1
  --baudrate <int>                    Serial baudrate, default: 1000000
  --cmd-vel-topic <topic>             Teleop output topic, default: cmd_vel
  --device-name <name>                Joystick device name, default: Xbox 360 Controller
  --joy-node-deadzone <val>           joy_node deadzone, default: 0.01
  --autorepeat-rate <hz>              joy_node autorepeat rate, default: 50.0
  --joy-deadzone <val>                Stick mode deadzone, default: 0.15
  --deadzone-hyst <val>               Stick mode hysteresis width, default: 0.02
  --joy-timeout-s <sec>               Teleop watchdog timeout, default: 0.3
  --v-linear <m/s>                    Max linear speed, default: 0.2
  --v-angular <rad/s>                 Max angular speed, default: 0.5
  --max-accel <m/s^2>                 Max linear acceleration, default: 1.5
  --max-alpha <rad/s^2>               Max angular acceleration, default: 3.0
  --stop-repeat-n <count>             Repeated zero-twist frames, default: 10
  --require-deadman                   Require deadman button hold
  --deadman-button <index>            Deadman button index, default: 4
  --stats-log                         Enable PoseSender 1s stats logs
  --dry-run                           Print commands only
  -h, --help                          Show this help

Examples:
  ./start_r2_mcu_teleop.sh
  ./start_r2_mcu_teleop.sh --mode stick
  ./start_r2_mcu_teleop.sh --target-serial-port /dev/ttyUSB3
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

  # shellcheck disable=SC1090
  source "$1"

  if [[ "${nounset_was_enabled}" == "true" ]]; then
    set -u
  fi
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="${RC26_WS:-${script_dir}}"
setup_file="${workspace_dir}/install/setup.bash"

mode="dpad"
feedback_serial_port="/dev/ttyUSB0"
target_serial_port="/dev/ttyUSB1"
baudrate="1000000"
cmd_vel_topic="cmd_vel"
device_name="Xbox 360 Controller"
joy_node_deadzone="0.01"
autorepeat_rate="50.0"
joy_deadzone="0.15"
deadzone_hyst="0.02"
joy_timeout_s="0.3"
v_linear="0.2"
v_angular="0.5"
max_accel="1.5"
max_alpha="3.0"
stop_repeat_n="10"
require_deadman="false"
deadman_button="4"
stats_log_enable="false"
dry_run="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      mode="${2:-}"
      shift 2
      ;;
    --feedback-serial-port)
      feedback_serial_port="${2:-}"
      shift 2
      ;;
    --target-serial-port)
      target_serial_port="${2:-}"
      shift 2
      ;;
    --baudrate)
      baudrate="${2:-}"
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
    --v-linear)
      v_linear="${2:-}"
      shift 2
      ;;
    --v-angular)
      v_angular="${2:-}"
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
    --stats-log)
      stats_log_enable="true"
      shift
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

if [[ ! -f "${setup_file}" ]]; then
  echo "setup.bash not found: ${setup_file}" >&2
  exit 1
fi

if [[ -z "${target_serial_port}" ]]; then
  echo "Target serial port must not be empty." >&2
  exit 1
fi

if [[ "${target_serial_port}" == "/dev/ttyUSB1" && ! -e "${target_serial_port}" && -e "/dev/ttyUSB0" ]]; then
  target_serial_port="/dev/ttyUSB0"
fi

feedback_port_notice=""
if [[ -z "${feedback_serial_port}" || "${feedback_serial_port}" == "${target_serial_port}" || ! -e "${feedback_serial_port}" ]]; then
  feedback_serial_port="__disabled__"
  feedback_port_notice="feedback serial disabled for this run"
fi

pose_sender_cmd=(
  ros2 run rc26_merge_odom pose_sender_node
  --ros-args
  -p "chassis_model:=tracked_diff"
  -p "feedback_serial_port:=${feedback_serial_port}"
  -p "target_serial_port:=${target_serial_port}"
  -p "baudrate:=${baudrate}"
  -p "cmd_vel_topic:=${cmd_vel_topic}"
  -p "odom_topic:=wheel_odom"
  -p "imu_gate_enable:=false"
  -p "latency_comp_enable:=false"
  -p "stats_log_enable:=${stats_log_enable}"
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
  -p "chassis_model:=tracked_diff"
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

if [[ "${dry_run}" == "true" ]]; then
  echo "Workspace: ${workspace_dir}"
  echo "Mode: ${mode}"
  echo "Feedback serial: ${feedback_serial_port}"
  echo "Target serial: ${target_serial_port}"
  echo "cmd_vel topic: ${cmd_vel_topic}"
  if [[ -n "${feedback_port_notice}" ]]; then
    echo "Notice: ${feedback_port_notice}"
  fi
  print_cmd source "${setup_file}"
  print_cmd "${pose_sender_cmd[@]}"
  print_cmd "${joy_cmd[@]}"
  print_cmd "${teleop_cmd[@]}"
  exit 0
fi

if [[ ! -e "${target_serial_port}" ]]; then
  echo "Target serial device not found: ${target_serial_port}" >&2
  exit 1
fi

source_with_relaxed_nounset "${setup_file}"

echo "Workspace: ${workspace_dir}"
echo "Mode: ${mode}"
echo "Feedback serial: ${feedback_serial_port}"
echo "Target serial: ${target_serial_port}"
echo "cmd_vel topic: ${cmd_vel_topic}"
echo "Linear speed limit: ${v_linear} m/s"
echo "Angular speed limit: ${v_angular} rad/s"
if [[ -n "${feedback_port_notice}" ]]; then
  echo "Notice: ${feedback_port_notice}"
fi
echo "Press Ctrl+C to stop all nodes."

pids=()

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  if (( ${#pids[@]} > 0 )); then
    echo
    echo "Stopping R2 MCU teleop stack..."
    kill "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
  fi
  exit "${exit_code}"
}

trap cleanup EXIT INT TERM

print_cmd "${pose_sender_cmd[@]}"
"${pose_sender_cmd[@]}" &
pids+=("$!")

print_cmd "${joy_cmd[@]}"
"${joy_cmd[@]}" &
pids+=("$!")

print_cmd "${teleop_cmd[@]}"
"${teleop_cmd[@]}" &
pids+=("$!")

wait -n "${pids[@]}"
