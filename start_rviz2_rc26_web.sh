#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./start_rviz2_rc26_web.sh [options]

Options:
  --host <host>                 Bind host for rviz2_rc26_server.py, default: 127.0.0.1
  --port <port>                 Preferred bind port, default: 8796
  --skip-build                  Skip the default incremental rviz2 + rc26_topo_nav build
  --skip-frontend-build         Skip viewer build, even if sources changed
  --rebuild                     Force rebuild of rviz2, rc26_topo_nav and viewer
  --no-browser                  Do not open a browser automatically
  --dry-run                     Print planned actions only
  -h, --help                    Show this help

Examples:
  ./start_rviz2_rc26_web.sh
  ./start_rviz2_rc26_web.sh --rebuild
  ./start_rviz2_rc26_web.sh --host 0.0.0.0 --port 8800 --no-browser
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

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

check_python_modules() {
  python3 - <<'PY'
import importlib.util
import sys

required = ("fastapi", "uvicorn", "pydantic")
missing = [name for name in required if importlib.util.find_spec(name) is None]
if missing:
    print("Missing Python modules: " + ", ".join(missing), file=sys.stderr)
    raise SystemExit(1)
PY
}

check_newer_than() {
  local candidate="$1"
  local target="$2"

  if [[ ! -e "${candidate}" ]]; then
    return 1
  fi

  if [[ -d "${candidate}" ]]; then
    find "${candidate}" -type f -newer "${target}" -print -quit 2>/dev/null | grep -q .
    return $?
  fi

  [[ "${candidate}" -nt "${target}" ]]
}

any_newer_than() {
  local target="$1"
  shift

  local candidate
  for candidate in "$@"; do
    if check_newer_than "${candidate}" "${target}"; then
      return 0
    fi
  done
  return 1
}

frontend_needs_build() {
  if [[ ! -f "${frontend_dist}/index.html" ]]; then
    return 0
  fi

  any_newer_than "${frontend_dist}/index.html" \
    "${frontend_dir}/index.html" \
    "${frontend_dir}/package.json" \
    "${frontend_dir}/package-lock.json" \
    "${frontend_dir}/tsconfig.json" \
    "${frontend_dir}/vite.config.ts" \
    "${frontend_dir}/src"
}

port_is_listening() {
  local host="$1"
  local port="$2"

  if command -v lsof >/dev/null 2>&1; then
    lsof -nP -iTCP:"${port}" -sTCP:LISTEN >/dev/null 2>&1
    return $?
  fi

  python3 - "$host" "$port" <<'PY'
import socket
import sys

host = sys.argv[1]
port = int(sys.argv[2])

try:
    with socket.create_connection((host, port), timeout=0.75):
        pass
except OSError:
    raise SystemExit(1)
else:
    raise SystemExit(0)
PY
}

find_available_port() {
  local host="$1"
  local preferred_port="$2"
  local label="$3"
  local port="${preferred_port}"

  for _ in {1..50}; do
    if ! port_is_listening "${host}" "${port}"; then
      printf '%s\n' "${port}"
      return 0
    fi
    port=$((port + 1))
  done

  echo "Could not find a free port for ${label}, starting from ${preferred_port}" >&2
  exit 1
}

resolve_check_host() {
  local host="$1"

  case "${host}" in
    ""|localhost|0.0.0.0)
      printf '%s\n' "127.0.0.1"
      ;;
    "::"|"::1")
      printf '%s\n' "::1"
      ;;
    *)
      printf '%s\n' "${host}"
      ;;
  esac
}

wait_for_http() {
  local url="$1"
  local label="$2"
  local pid="$3"

  for _ in {1..120}; do
    if curl -sf "${url}" >/dev/null 2>&1; then
      return 0
    fi

    if ! kill -0 "${pid}" 2>/dev/null; then
      echo "${label} exited before becoming ready: ${url}" >&2
      return 1
    fi

    sleep 0.5
  done

  echo "${label} timed out: ${url}" >&2
  return 1
}

stop_pid() {
  local pid="${1:-}"

  if [[ -z "${pid}" ]] || ! kill -0 "${pid}" 2>/dev/null; then
    return 0
  fi

  kill "${pid}" 2>/dev/null || true

  for _ in {1..20}; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      return 0
    fi
    sleep 0.25
  done

  kill -9 "${pid}" 2>/dev/null || true
}

open_browser_url() {
  local url="$1"

  if [[ "${open_browser}" != "true" ]]; then
    return 0
  fi

  if command -v xdg-open >/dev/null 2>&1; then
    xdg-open "${url}" >/dev/null 2>&1 &
    return 0
  fi

  if command -v gio >/dev/null 2>&1; then
    gio open "${url}" >/dev/null 2>&1 &
    return 0
  fi

  if command -v sensible-browser >/dev/null 2>&1; then
    sensible-browser "${url}" >/dev/null 2>&1 &
    return 0
  fi

  python3 -m webbrowser "${url}" >/dev/null 2>&1 || true
}

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  stop_pid "${backend_pid:-}"
  exit "${exit_code}"
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root_dir="${RC26_WS:-${script_dir}}"
pkg_root="${root_dir}/src/rc26_xhu_viewer/rviz2"
frontend_dir="${pkg_root}/viewer"
frontend_dist="${frontend_dir}/dist"
backend_script="${pkg_root}/scripts/rviz2_rc26_server.py"
setup_file="${root_dir}/install/setup.bash"
planner_trace_cli="${root_dir}/install/rc26_topo_nav/lib/rc26_topo_nav/planner_trace_cli"

host="127.0.0.1"
preferred_port="8796"
backend_build_mode="build"
frontend_build_mode="auto"
open_browser="true"
dry_run="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host)
      host="${2:-}"
      shift 2
      ;;
    --port)
      preferred_port="${2:-}"
      shift 2
      ;;
    --skip-build)
      backend_build_mode="skip"
      shift
      ;;
    --skip-frontend-build)
      frontend_build_mode="skip"
      shift
      ;;
    --rebuild)
      backend_build_mode="force"
      frontend_build_mode="force"
      shift
      ;;
    --no-browser)
      open_browser="false"
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

if [[ ! -d "${pkg_root}" ]]; then
  echo "rviz2 package directory not found: ${pkg_root}" >&2
  exit 1
fi

if [[ ! -f "${backend_script}" ]]; then
  echo "rviz2_rc26_server.py not found: ${backend_script}" >&2
  exit 1
fi

require_command python3
require_command curl
check_python_modules

browser_host="$(resolve_check_host "${host}")"
requested_port="${preferred_port}"
port="$(find_available_port "${browser_host}" "${preferred_port}" "rviz2 RC26 web server")"
viewer_url="http://${browser_host}:${port}/"
health_url="http://${browser_host}:${port}/api/health"

if [[ "${dry_run}" == "true" ]]; then
  echo "Workspace: ${root_dir}"
  echo "Viewer URL: ${viewer_url}"

  if [[ "${backend_build_mode}" != "skip" ]]; then
    print_cmd env MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 --allow-overriding rc26_topo_nav rviz2 --packages-select rviz2 rc26_topo_nav
  fi

  if [[ "${frontend_build_mode}" != "skip" ]]; then
    print_cmd npm run build
  fi

  print_cmd source "${setup_file}"
  print_cmd python3 "${backend_script}" --host "${host}" --port "${port}"
  exit 0
fi

if [[ "${backend_build_mode}" != "skip" ]]; then
  require_command colcon
  echo "==> Building rviz2 + rc26_topo_nav"
  (
    cd "${root_dir}"
    env MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 --allow-overriding rc26_topo_nav rviz2 --packages-select rviz2 rc26_topo_nav
  )
elif [[ ! -f "${setup_file}" || ! -x "${planner_trace_cli}" ]]; then
  echo "Missing install/setup.bash or planner_trace_cli; rerun without --skip-build." >&2
  exit 1
fi

if [[ "${frontend_build_mode}" == "force" ]] || ([[ "${frontend_build_mode}" == "auto" ]] && frontend_needs_build); then
  require_command npm
  echo "==> Building rviz2_rc26 web frontend"
  (
    cd "${frontend_dir}"
    if [[ ! -d node_modules ]]; then
      npm ci
    fi
    npm run build
  )
elif [[ ! -f "${frontend_dist}/index.html" ]]; then
  echo "Missing viewer/dist/index.html; rerun without --skip-frontend-build." >&2
  exit 1
else
  echo "==> Reusing existing viewer/dist"
fi

if [[ ! -f "${setup_file}" ]]; then
  echo "setup.bash not found after build: ${setup_file}" >&2
  exit 1
fi

if [[ "${port}" != "${requested_port}" ]]; then
  echo "==> Port ${requested_port} is busy, using ${port} instead"
fi

source_with_relaxed_nounset "${setup_file}"

trap cleanup EXIT INT TERM

echo "==> Starting rviz2_rc26 server"
print_cmd python3 "${backend_script}" --host "${host}" --port "${port}"
python3 "${backend_script}" --host "${host}" --port "${port}" &
backend_pid=$!

wait_for_http "${health_url}" "rviz2_rc26 server" "${backend_pid}"

echo "==> Viewer ready: ${viewer_url}"
open_browser_url "${viewer_url}"
echo "==> Press Ctrl+C to stop the rviz2_rc26 server"

wait "${backend_pid}"
