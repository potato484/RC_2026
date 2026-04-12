#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./start_r2_teleop.sh [options]

Options:
  --stack <full|minimal-mcu>  Startup stack, default: full
                              full        = merge_odom + joy + telecontrol + pushrod-dpad + front-track
                              minimal-mcu = pose_sender + joy + telecontrol + pushrod-dpad
  --mode <stick|dpad>         Control mode, default: dpad
                              stick = 履带模式，左摇杆前后 + 右摇杆旋转
                              dpad  = 履带模式，十字键前后 + X右旋/B左旋，左/右单次触发推杆
  --pose-mode <imu|no-imu|wheel-only>
                              Only valid for --stack full
                              imu        = EKF uses IMU
                              no-imu     = EKF ignores IMU, but dm_imu_node and PoseSender IMU protection stay on
                              wheel-only = Do not start/read IMU; EKF uses wheel odom only
  --feedback-serial-port <device>
                              Serial port for ODOM_DATA / POSE_FEEDBACK, default: /dev/ttyUSB0
  --target-serial-port <device>
                              Serial port for POSE_TARGET / mechanism transport, default: /dev/ttyUSB1
  --baudrate <int>            Serial baudrate, default: 1000000
  --v-linear <m/s>            Max linear speed, default: 0.2
  --v-angular <rad/s>         Max angular speed, default: 0.5
  --cmd-vel-topic <topic>     Teleop output topic, default: cmd_vel
  --device-name <name>        Joystick device name, default: Xbox 360 Controller
  --joy-node-deadzone <val>   joy_node deadzone, default: 0.01
  --autorepeat-rate <hz>      joy_node autorepeat rate, default: 50.0
  --joy-deadzone <val>        Stick mode deadzone, default: 0.15
  --deadzone-hyst <val>       Stick mode hysteresis width, default: 0.02
  --joy-timeout-s <sec>       Teleop watchdog timeout, default: 0.3
  --max-accel <m/s^2>         Max linear acceleration, default: 1.5
  --max-alpha <rad/s^2>       Max angular acceleration, default: 3.0
  --stop-repeat-n <count>     Repeated zero-twist frames, default: 10
  --require-deadman           Require deadman button hold
  --deadman-button <index>    Deadman button index, default: 4
  --use-can-odom              Enable CAN odom in merge_odom (full stack only)
  --start-ekf                 Enable EKF in merge_odom (full stack only)
  --stats-log                 Enable PoseSender 1s stats logs
  --dry-run                   Print commands only
  -h, --help                  Show this help

Examples:
  ./start_r2_teleop.sh
  ./start_r2_teleop.sh --pose-mode imu
  ./start_r2_teleop.sh --pose-mode no-imu
  ./start_r2_teleop.sh --pose-mode wheel-only
  ./start_r2_teleop.sh --stack minimal-mcu
  ./start_r2_teleop.sh --stack minimal-mcu --target-serial-port /dev/ttyUSB3
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

run_and_track() {
  print_cmd "$@"
  "$@" &
  pids+=("$!")
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="${RC26_WS:-${script_dir}}"
setup_file="${workspace_dir}/install/setup.bash"

stack_mode="full"
mode="dpad"
pose_mode=""
chassis_model="tracked_diff"
feedback_serial_port="/dev/ttyUSB0"
target_serial_port="/dev/ttyUSB1"
baudrate="1000000"
v_linear="0.2"
v_angular="0.5"
cmd_vel_topic="cmd_vel"
device_name="Xbox 360 Controller"
joy_node_deadzone="0.01"
autorepeat_rate="50.0"
joy_deadzone="0.15"
deadzone_hyst="0.02"
joy_timeout_s="0.3"
max_accel="1.5"
max_alpha="3.0"
stop_repeat_n="10"
require_deadman="false"
deadman_button="4"
use_can_odom="false"
start_ekf="false"
use_imu_for_ekf="true"
start_imu="true"
stats_log_enable="false"
dry_run="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --stack)
      stack_mode="${2:-}"
      shift 2
      ;;
    --mode)
      mode="${2:-}"
      shift 2
      ;;
    --pose-mode)
      pose_mode="${2:-}"
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
    --use-can-odom)
      use_can_odom="true"
      shift
      ;;
    --start-ekf)
      start_ekf="true"
      shift
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

case "${stack_mode}" in
  full|minimal-mcu)
    ;;
  *)
    echo "Invalid --stack: ${stack_mode}. Expected full or minimal-mcu." >&2
    usage
    exit 1
    ;;
esac

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

if [[ "${stack_mode}" == "full" ]]; then
  case "${pose_mode}" in
    "")
      ;;
    imu)
      start_ekf="true"
      use_imu_for_ekf="true"
      start_imu="true"
      ;;
    no-imu)
      start_ekf="true"
      use_imu_for_ekf="false"
      start_imu="true"
      ;;
    wheel-only)
      start_ekf="true"
      use_imu_for_ekf="false"
      start_imu="false"
      use_can_odom="false"
      ;;
    *)
      echo "Invalid --pose-mode: ${pose_mode}. Expected imu, no-imu, or wheel-only." >&2
      usage
      exit 1
      ;;
  esac
else
  if [[ -n "${pose_mode}" ]]; then
    echo "--pose-mode only applies to --stack full." >&2
    exit 1
  fi
  if [[ "${use_can_odom}" == "true" || "${start_ekf}" == "true" ]]; then
    echo "--use-can-odom and --start-ekf only apply to --stack full." >&2
    exit 1
  fi
fi

if [[ ! -f "${setup_file}" ]]; then
  echo "setup.bash not found: ${setup_file}" >&2
  exit 1
fi

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
  -p "chassis_model:=${chassis_model}"
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

pushrod_cmd=(
  ros2 run rc26_telecontrol rc26_telecontrol_pushrod_dpad
)

feedback_port_notice=""
if [[ "${stack_mode}" == "full" ]]; then
  if [[ -z "${target_serial_port}" ]]; then
    echo "Target serial port must not be empty." >&2
    exit 1
  fi

  if [[ "${target_serial_port}" == "/dev/ttyUSB1" && ! -e "${target_serial_port}" && -e "/dev/ttyUSB0" ]]; then
    target_serial_port="/dev/ttyUSB0"
  fi

  if [[ -z "${feedback_serial_port}" || "${feedback_serial_port}" == "${target_serial_port}" || ! -e "${feedback_serial_port}" ]]; then
    feedback_serial_port="__disabled__"
    feedback_port_notice="feedback serial disabled for this run"
  fi

  merge_odom_cmd=(
    ros2 launch rc26_merge_odom merge_odom.launch.py
    "use_can_odom:=${use_can_odom}"
    "start_ekf:=${start_ekf}"
    "use_imu_for_ekf:=${use_imu_for_ekf}"
    "start_imu:=${start_imu}"
    "feedback_serial_port:=${feedback_serial_port}"
    "target_serial_port:=${target_serial_port}"
    "baudrate:=${baudrate}"
    "stats_log_enable:=${stats_log_enable}"
    "chassis_model:=${chassis_model}"
  )

  button_test_cmd=(
    ros2 run rc26_telecontrol rc26_telecontrol_front_track_test
  )
else
  if [[ -z "${target_serial_port}" ]]; then
    echo "Target serial port must not be empty." >&2
    exit 1
  fi

  if [[ "${target_serial_port}" == "/dev/ttyUSB1" && ! -e "${target_serial_port}" && -e "/dev/ttyUSB0" ]]; then
    target_serial_port="/dev/ttyUSB0"
  fi

  if [[ -z "${feedback_serial_port}" || "${feedback_serial_port}" == "${target_serial_port}" || ! -e "${feedback_serial_port}" ]]; then
    feedback_serial_port="__disabled__"
    feedback_port_notice="feedback serial disabled for this run"
  fi

  pose_sender_cmd=(
    ros2 run rc26_merge_odom pose_sender_node
    --ros-args
    -p "chassis_model:=${chassis_model}"
    -p "feedback_serial_port:=${feedback_serial_port}"
    -p "target_serial_port:=${target_serial_port}"
    -p "baudrate:=${baudrate}"
    -p "cmd_vel_topic:=${cmd_vel_topic}"
    -p "odom_topic:=wheel_odom"
    -p "imu_gate_enable:=false"
    -p "latency_comp_enable:=false"
    -p "stats_log_enable:=${stats_log_enable}"
  )
fi

if [[ "${dry_run}" == "true" ]]; then
  echo "Workspace: ${workspace_dir}"
  echo "Stack: ${stack_mode}"
  echo "Mode: ${mode}"
  echo "Chassis model: ${chassis_model}"
  echo "cmd_vel topic: ${cmd_vel_topic}"
  if [[ "${stack_mode}" == "full" ]]; then
    if [[ -n "${pose_mode}" ]]; then
      echo "Pose mode: ${pose_mode}"
    else
      echo "Pose mode: disabled"
    fi
    echo "Feedback serial: ${feedback_serial_port}"
    echo "Target serial: ${target_serial_port}"
    if [[ -n "${feedback_port_notice}" ]]; then
      echo "Notice: ${feedback_port_notice}"
    fi
    echo "IMU input: $([[ "${start_imu}" == "true" ]] && echo enabled || echo disabled)"
    print_cmd source "${setup_file}"
    print_cmd "${merge_odom_cmd[@]}"
    print_cmd "${joy_cmd[@]}"
    print_cmd "${teleop_cmd[@]}"
    print_cmd "${pushrod_cmd[@]}"
    print_cmd "${button_test_cmd[@]}"
  else
    echo "Feedback serial: ${feedback_serial_port}"
    echo "Target serial: ${target_serial_port}"
    if [[ -n "${feedback_port_notice}" ]]; then
      echo "Notice: ${feedback_port_notice}"
    fi
    print_cmd source "${setup_file}"
    print_cmd "${pose_sender_cmd[@]}"
    print_cmd "${joy_cmd[@]}"
    print_cmd "${teleop_cmd[@]}"
    print_cmd "${pushrod_cmd[@]}"
  fi
  exit 0
fi

if [[ "${stack_mode}" == "minimal-mcu" && ! -e "${target_serial_port}" ]]; then
  echo "Target serial device not found: ${target_serial_port}" >&2
  exit 1
fi

source_with_relaxed_nounset "${setup_file}"

echo "Workspace: ${workspace_dir}"
echo "Stack: ${stack_mode}"
echo "Mode: ${mode}"
echo "Chassis model: ${chassis_model}"
echo "cmd_vel topic: ${cmd_vel_topic}"
echo "Linear speed limit: ${v_linear} m/s"
echo "Angular speed limit: ${v_angular} rad/s"
echo "Feedback serial: ${feedback_serial_port}"
echo "Target serial: ${target_serial_port}"
if [[ -n "${feedback_port_notice}" ]]; then
  echo "Notice: ${feedback_port_notice}"
fi
if [[ "${stack_mode}" == "full" ]]; then
  echo "IMU input: $([[ "${start_imu}" == "true" ]] && echo enabled || echo disabled)"
fi
echo "Press Ctrl+C to stop all nodes."

pids=()

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  if (( ${#pids[@]} > 0 )); then
    echo
    if [[ "${stack_mode}" == "full" ]]; then
      echo "Stopping R2 teleop stack..."
    else
      echo "Stopping R2 minimal MCU teleop stack..."
    fi
    kill "${pids[@]}" 2>/dev/null || true
    wait "${pids[@]}" 2>/dev/null || true
  fi
  exit "${exit_code}"
}

trap cleanup EXIT INT TERM

if [[ "${stack_mode}" == "full" ]]; then
  run_and_track "${merge_odom_cmd[@]}"
  run_and_track "${joy_cmd[@]}"
  run_and_track "${teleop_cmd[@]}"
  run_and_track "${pushrod_cmd[@]}"
  run_and_track "${button_test_cmd[@]}"
else
  run_and_track "${pose_sender_cmd[@]}"
  run_and_track "${joy_cmd[@]}"
  run_and_track "${teleop_cmd[@]}"
  run_and_track "${pushrod_cmd[@]}"
fi

wait -n "${pids[@]}"
