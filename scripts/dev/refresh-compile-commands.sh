#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
BUILD_BASE_REL="${RC26_IDE_BUILD_BASE:-build}"
INSTALL_BASE_REL="${RC26_IDE_INSTALL_BASE:-install}"
OUTPUT_PATH="${RC26_COMPILE_COMMANDS_OUTPUT:-${ROOT_DIR}/compile_commands.json}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "[ERROR] Missing ROS environment: ${ROS_SETUP}" >&2
  exit 1
fi

cd "${ROOT_DIR}"

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
set -u

export MAKEFLAGS="${MAKEFLAGS:--j2 -l2}"

echo "[INFO] ROS_DISTRO=${ROS_DISTRO_NAME}"
echo "[INFO] build-base=${BUILD_BASE_REL}"
echo "[INFO] install-base=${INSTALL_BASE_REL}"
echo "[INFO] output=${OUTPUT_PATH}"
if [[ "$#" -gt 0 ]]; then
  echo "[INFO] colcon args: $*"
else
  echo "[INFO] colcon args: <workspace default>"
fi

set +e
colcon build \
  --build-base "${BUILD_BASE_REL}" \
  --install-base "${INSTALL_BASE_REL}" \
  --continue-on-error \
  --executor sequential \
  --parallel-workers 1 \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  "$@"
build_exit_code=$?
set -e

python3 - "${ROOT_DIR}" "${BUILD_BASE_REL}" "${OUTPUT_PATH}" <<'PY'
import json
import sys
from pathlib import Path

root_dir = Path(sys.argv[1])
build_base = root_dir / sys.argv[2]
output_path = Path(sys.argv[3])
source_root = root_dir / "src"

compile_db_files = sorted(build_base.glob("*/compile_commands.json"))
entries_by_file = {}

for compile_db in compile_db_files:
    with compile_db.open("r", encoding="utf-8") as handle:
        entries = json.load(handle)
    for entry in entries:
        file_path = entry.get("file")
        if not file_path:
            continue
        try:
            path_obj = Path(file_path).resolve()
        except OSError:
            continue
        # Keep only real workspace source files so VS Code/clangd do not
        # associate headers with generated build/* translation units.
        if source_root not in path_obj.parents:
            continue
        entries_by_file[file_path] = entry

merged_entries = [entries_by_file[key] for key in sorted(entries_by_file)]
output_path.write_text(
    json.dumps(merged_entries, indent=2, ensure_ascii=False) + "\n",
    encoding="utf-8",
)

print(f"[INFO] merged {len(compile_db_files)} package databases into {output_path}")
print(f"[INFO] total translation units: {len(merged_entries)}")
PY

if [[ "${build_exit_code}" -ne 0 ]]; then
  echo "[WARN] colcon build reported failures, but compile_commands.json was still refreshed from the available package databases." >&2
fi

exit "${build_exit_code}"
