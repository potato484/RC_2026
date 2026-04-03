#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./start_r2_teleop.sh [options]

Options:
  --mode <stick|dpad>         Control mode, default: stick
                              stick = 履带模式，左摇杆前后 + 右摇杆旋转
                              dpad  = 履带模式，十字键前后 + X右旋/B左旋
  --v-linear <m/s>            Max linear speed, default: 0.2
  --v-angular <rad/s>         Max angular speed, default: 0.5
  --cmd-vel-topic <topic>     Teleop output topic, default: cmd_vel
  --terrain-speed-limit-topic <topic>
                              PoseSender terrain speed limit topic, default: disabled for manual teleop test
  --enable-terrain-speed-limit
                              Re-enable terrain_speed_limit topic (default topic: terrain_speed_limit)
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
  --use-can-odom              Enable CAN odom in merge_odom
  --start-ekf                 Enable EKF in merge_odom
  --dry-run                   Print commands only
  -h, --help                  Show this help

Examples:
  ./start_r2_teleop.sh
  ./start_r2_teleop.sh --mode dpad
  ./start_r2_teleop.sh --mode stick --v-linear 0.4 --v-angular 0.8
  ./start_r2_teleop.sh --mode dpad --cmd-vel-topic cmd_vel
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
terrain_speed_limit_disabled_sentinel="__disabled__"

mode="stick"
chassis_model="tracked_diff"
v_linear="0.2"
v_angular="0.5"
cmd_vel_topic="cmd_vel"
terrain_speed_limit_topic="${terrain_speed_limit_disabled_sentinel}"
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
    --terrain-speed-limit-topic)
      terrain_speed_limit_topic="${2:-}"
      if [[ -z "${terrain_speed_limit_topic}" ]]; then
        terrain_speed_limit_topic="${terrain_speed_limit_disabled_sentinel}"
      fi
      shift 2
      ;;
    --enable-terrain-speed-limit)
      terrain_speed_limit_topic="terrain_speed_limit"
      shift
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

merge_odom_cmd=(
  ros2 launch rc26_merge_odom merge_odom.launch.py
  "use_can_odom:=${use_can_odom}"
  "start_ekf:=${start_ekf}"
  "terrain_speed_limit_topic:=${terrain_speed_limit_topic}"
  "chassis_model:=${chassis_model}"
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

if [[ "${dry_run}" == "true" ]]; then
  echo "Workspace: ${workspace_dir}"
  echo "Mode: ${mode}"
  print_cmd source "${setup_file}"
  print_cmd "${merge_odom_cmd[@]}"
  print_cmd "${joy_cmd[@]}"
  print_cmd "${teleop_cmd[@]}"
  exit 0
fi

source_with_relaxed_nounset "${setup_file}"

echo "Workspace: ${workspace_dir}"
echo "Mode: ${mode}"
echo "Chassis model: ${chassis_model}"
echo "cmd_vel topic: ${cmd_vel_topic}"
if [[ "${terrain_speed_limit_topic}" != "${terrain_speed_limit_disabled_sentinel}" ]]; then
  echo "Terrain speed limit topic: ${terrain_speed_limit_topic}"
else
  echo "Terrain speed limit topic: disabled (manual teleop test mode)"
fi
echo "Linear speed limit: ${v_linear} m/s"
echo "Angular speed limit: ${v_angular} rad/s"
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

print_cmd "${merge_odom_cmd[@]}"
"${merge_odom_cmd[@]}" &
pids+=("$!")

print_cmd "${joy_cmd[@]}"
"${joy_cmd[@]}" &
pids+=("$!")

print_cmd "${teleop_cmd[@]}"
"${teleop_cmd[@]}" &
pids+=("$!")

wait -n "${pids[@]}"
