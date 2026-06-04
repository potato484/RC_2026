#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

DEFAULT_BUILD_PACKAGES=(
  rc26_interfaces
  rc26_mid360_driver
  rc26_odom_interface
  rc26_point_lio
  rc26_robot_geometry
  rc26_sensor_extrinsics
  rc26_sensor_scan
  rc26_serial
  rc26_telecontrol
  rc26_localization
  rc26_vision
  rc26_decision
  rc26_bringup
)

DEFAULT_TEST_PACKAGES=(
  rc26_interfaces
  rc26_robot_geometry
  rc26_serial
  rc26_telecontrol
  rc26_vision
  rc26_decision
  rc26_bringup
)

if [[ "$#" -gt 0 ]]; then
  BUILD_PACKAGES=("$@")
  TEST_PACKAGES=("$@")
else
  BUILD_PACKAGES=("${DEFAULT_BUILD_PACKAGES[@]}")
  TEST_PACKAGES=("${DEFAULT_TEST_PACKAGES[@]}")
fi

ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[ERROR] Missing ROS environment: ${ROS_SETUP}" >&2
  exit 1
fi

BUILD_BASE_REL="${RC26_CI_BUILD_BASE:-build/ros2_workspace_ci}"
INSTALL_BASE_REL="${RC26_CI_INSTALL_BASE:-install/ros2_workspace_ci}"
TEST_RESULT_BASE_REL="${RC26_CI_TEST_RESULT_BASE:-${BUILD_BASE_REL}/test_results_smoke}"

cd "${ROOT_DIR}"

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u

export MAKEFLAGS="${MAKEFLAGS:--j2 -l2}"

echo "[INFO] ROS_DISTRO=${ROS_DISTRO_NAME}"
echo "[INFO] Build packages: ${BUILD_PACKAGES[*]}"
echo "[INFO] Test packages: ${TEST_PACKAGES[*]}"
echo "[INFO] build-base=${BUILD_BASE_REL}"
echo "[INFO] install-base=${INSTALL_BASE_REL}"
echo "[INFO] test-result-base=${TEST_RESULT_BASE_REL}"

colcon build \
  --build-base "${BUILD_BASE_REL}" \
  --install-base "${INSTALL_BASE_REL}" \
  --executor sequential \
  --parallel-workers 1 \
  --packages-select "${BUILD_PACKAGES[@]}"

set +u
# shellcheck disable=SC1091
source "${ROOT_DIR}/${INSTALL_BASE_REL}/setup.bash"
set -u

if [[ -z "${RC26_CI_TEST_RESULT_BASE:-}" ]]; then
  rm -rf "${TEST_RESULT_BASE_REL}"
fi

colcon test \
  --build-base "${BUILD_BASE_REL}" \
  --install-base "${INSTALL_BASE_REL}" \
  --executor sequential \
  --parallel-workers 1 \
  --test-result-base "${TEST_RESULT_BASE_REL}" \
  --packages-select "${TEST_PACKAGES[@]}" \
  --event-handlers console_direct+

colcon test-result \
  --test-result-base "${TEST_RESULT_BASE_REL}" \
  --all \
  --verbose
