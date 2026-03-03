#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  tools/record_controller_bag.sh --route A|B|C --phase before|after [options]

Options:
  --route <A|B|C>            Route label
  --phase <before|after>     Baseline or modified run
  --run <id>                 Run id, default: 01
  --output-dir <path>        Output directory, default: log/bags
  --duration <sec>           Optional timeout seconds
  --enable-debug             Set FollowPath.publish_debug=true before recording
  --controller-node <name>   Controller node name, default: /controller_server
  --dry-run                  Print command only
  -h, --help                 Show this help

Examples:
  tools/record_controller_bag.sh --route B --phase after --run 01
  tools/record_controller_bag.sh --route C --phase before --run 02 --duration 120
  tools/record_controller_bag.sh --route A --phase after --run 03 --enable-debug
EOF
}

route=""
phase=""
run_id="01"
output_dir="log/bags"
duration=""
enable_debug="false"
controller_node="/controller_server"
dry_run="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --route)
      route="${2:-}"
      shift 2
      ;;
    --phase)
      phase="${2:-}"
      shift 2
      ;;
    --run)
      run_id="${2:-}"
      shift 2
      ;;
    --output-dir)
      output_dir="${2:-}"
      shift 2
      ;;
    --duration)
      duration="${2:-}"
      shift 2
      ;;
    --enable-debug)
      enable_debug="true"
      shift
      ;;
    --controller-node)
      controller_node="${2:-}"
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

if [[ ! "${route}" =~ ^[ABC]$ ]]; then
  echo "Invalid --route: ${route}" >&2
  usage
  exit 1
fi

if [[ ! "${phase}" =~ ^(before|after)$ ]]; then
  echo "Invalid --phase: ${phase}" >&2
  usage
  exit 1
fi

if [[ -n "${duration}" ]] && [[ ! "${duration}" =~ ^[0-9]+$ ]]; then
  echo "Invalid --duration: ${duration}" >&2
  usage
  exit 1
fi

bag_name="${phase}_route${route}_run${run_id}"
bag_path="${output_dir%/}/${bag_name}"

topics=(
  /odometry
  /tf
  /tf_static
  /cmd_vel
  /compute_time_ms
  /collision_check_outside_map_count
  /collision_check_ms
  /collision_d_min
  /v_safe
  /DM_IMU
)

mkdir -p "${output_dir}"

if [[ "${enable_debug}" == "true" ]]; then
  if [[ "${dry_run}" == "true" ]]; then
    echo "ros2 param set ${controller_node} FollowPath.publish_debug true"
  else
    ros2 param set "${controller_node}" FollowPath.publish_debug true
    ros2 param get "${controller_node}" FollowPath.publish_debug
  fi
fi

if [[ "${dry_run}" == "true" ]]; then
  if [[ -n "${duration}" ]]; then
    echo "timeout ${duration}s ros2 bag record -o ${bag_path} ${topics[*]}"
  else
    echo "ros2 bag record -o ${bag_path} ${topics[*]}"
  fi
  exit 0
fi

echo "Recording to: ${bag_path}"
if [[ -n "${duration}" ]]; then
  timeout "${duration}s" ros2 bag record -o "${bag_path}" "${topics[@]}"
else
  ros2 bag record -o "${bag_path}" "${topics[@]}"
fi
