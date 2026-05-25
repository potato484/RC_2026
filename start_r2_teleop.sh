#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./start_r2_teleop.sh [options]

Options:
  --stack <full|minimal-mcu>  启动栈，默认：full
                              full        = merge_odom + joy + telecontrol + front-pushrod-sidecar + rear-pushrod-sidecar
                              minimal-mcu = pose_sender + joy + telecontrol + front-pushrod-sidecar + rear-pushrod-sidecar
  --mode <stick|dpad>         控制模式，默认：dpad
                              stick = 左摇杆控制 vx/vy，右摇杆左右控制 wz
                              dpad  = 十字键控制 vx/vy，X/B 控制 wz
  --pose-mode <imu|no-imu|wheel-only>
                              仅对 --stack full 有效
                              imu        = EKF 使用 IMU
                              no-imu     = EKF 不使用 IMU，但保留 dm_imu_node 和 PoseSender IMU 保护链
                              wheel-only = 不启动也不读取 IMU；EKF 仅使用轮速里程计
  --feedback-serial-port <device>
                              ODOM_DATA / POSE_FEEDBACK 反馈串口，默认：/dev/ttyUSB0
  --target-serial-port <device>
                              POSE_TARGET / mechanism transport 目标串口，默认：/dev/ttyUSB1
  --baudrate <int>            串口波特率，默认：1000000
  --v-linear <m/s>            最大线速度，默认：0.2
  --v-angular <rad/s>         最大角速度，默认：0.5
  --cmd-vel-topic <topic>     遥控输出话题，默认：cmd_vel
  --device-name <name>        手柄设备名，默认：Xbox 360 Controller
  --joy-node-deadzone <val>   joy_node 死区，默认：0.01
  --autorepeat-rate <hz>      joy_node 自动重复发布频率，默认：50.0
  --joy-deadzone <val>        Stick 模式死区，默认：0.15
  --deadzone-hyst <val>       Stick 模式死区滞回宽度，默认：0.02
  --joy-timeout-s <sec>       遥控看门狗超时，默认：0.3
  --max-accel <m/s^2>         最大线加速度，默认：1.5
  --max-alpha <rad/s^2>       最大角加速度，默认：3.0
  --stop-repeat-n <count>     零速指令重复帧数，默认：10
  --require-deadman           要求持续按住 deadman 安全键
  --deadman-button <index>    deadman 安全键编号，默认：4
  --use-can-odom              在 merge_odom 中启用 CAN 里程计（仅 full 栈）
  --start-ekf                 在 merge_odom 中启用 EKF（仅 full 栈）
  --stats-log                 启用 PoseSender 1 秒统计日志
  --dry-run                   只打印命令，不实际启动
  -h, --help                  显示本帮助

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

  # shellcheck：setup.bash 路径由运行时工作区决定，静态检查无法解析。
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

stack_mode="minimal-mcu"
mode="dpad"
pose_mode=""
feedback_serial_port="/dev/ttyUSB0"
target_serial_port="/dev/ttyUSB1"
baudrate="1000000"
v_linear="0.1"
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
    print_cmd "${front_pushrod_cmd[@]}"
    print_cmd "${rear_pushrod_cmd[@]}"
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
    print_cmd "${front_pushrod_cmd[@]}"
    print_cmd "${rear_pushrod_cmd[@]}"
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
  run_and_track "${front_pushrod_cmd[@]}"
  run_and_track "${rear_pushrod_cmd[@]}"
else
  run_and_track "${pose_sender_cmd[@]}"
  run_and_track "${joy_cmd[@]}"
  run_and_track "${teleop_cmd[@]}"
  run_and_track "${front_pushrod_cmd[@]}"
  run_and_track "${rear_pushrod_cmd[@]}"
fi

wait -n "${pids[@]}"
