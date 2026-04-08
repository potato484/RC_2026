#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
FRONTEND_DIR="$ROOT_DIR/src/rc26_xhu_viewer/rc26_xhu_viewer/viewer"
FRONTEND_DIST="$FRONTEND_DIR/dist-e2e"
ARTIFACT_DIR="${E2E_ARTIFACT_DIR:-$ROOT_DIR/artifacts/e2e}"
BACKEND_BIND_HOST="${E2E_BACKEND_HOST:-127.0.0.1}"
FRONTEND_BIND_HOST="${E2E_FRONTEND_HOST:-127.0.0.1}"
BACKEND_PORT="${E2E_BACKEND_PORT:-8877}"
FRONTEND_PORT="${E2E_FRONTEND_PORT:-4173}"
BACKEND_LOG="$ARTIFACT_DIR/backend-e2e.log"
FRONTEND_LOG="$ARTIFACT_DIR/frontend-e2e.log"

mkdir -p "$ARTIFACT_DIR"

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "缺少必要命令: $1" >&2
    exit 1
  fi
}

check_python_modules() {
  python3 - <<'PY'
import importlib.util
import sys

required = ("fastapi", "uvicorn")
missing = [name for name in required if importlib.util.find_spec(name) is None]
if missing:
    print("缺少 Python 模块: " + ", ".join(missing), file=sys.stderr)
    raise SystemExit(1)
PY
}

can_connect_tcp() {
  local host="$1"
  local port="$2"

  node -e '
    const net = require("node:net");
    const host = process.argv[1];
    const port = Number(process.argv[2]);
    const socket = net.connect({ host, port });

    const finish = (code) => {
      socket.destroy();
      process.exit(code);
    };

    socket.setTimeout(1000);
    socket.once("connect", () => finish(0));
    socket.once("timeout", () => finish(1));
    socket.once("error", () => finish(1));
  ' "$host" "$port"
}

port_is_listening() {
  local host="$1"
  local port="$2"

  if command -v lsof >/dev/null 2>&1; then
    lsof -nP -iTCP:"$port" -sTCP:LISTEN >/dev/null 2>&1
    return $?
  fi

  can_connect_tcp "$host" "$port"
}

find_available_port() {
  local host="$1"
  local preferred_port="$2"
  local label="$3"
  local port="$preferred_port"

  for _ in {1..50}; do
    if ! port_is_listening "$host" "$port"; then
      printf '%s\n' "$port"
      return 0
    fi
    port=$((port + 1))
  done

  echo "无法为 ${label} 找到可用端口，起始端口: ${preferred_port}" >&2
  return 1
}

resolve_check_host() {
  local host="$1"

  case "$host" in
    ""|localhost|0.0.0.0)
      printf '%s\n' "127.0.0.1"
      ;;
    "::"|"::1")
      printf '%s\n' "::1"
      ;;
    *)
      printf '%s\n' "$host"
      ;;
  esac
}

print_log_tail() {
  local log_file="${1:-}"
  if [[ -n "$log_file" && -f "$log_file" ]]; then
    echo "---- 最近日志: $log_file ----" >&2
    tail -n 80 "$log_file" >&2 || true
    echo "---- 日志结束 ----" >&2
  fi
}

wait_for_http() {
  local url="$1"
  local label="$2"
  local pid="$3"
  local log_file="${4:-}"

  for _ in {1..60}; do
    if curl -sf "$url" >/dev/null 2>&1; then
      return 0
    fi

    if ! kill -0 "$pid" 2>/dev/null; then
      echo "$label 启动失败。" >&2
      print_log_tail "$log_file"
      return 1
    fi

    sleep 0.5
  done

  echo "$label 启动超时: $url" >&2
  print_log_tail "$log_file"
  return 1
}

stop_pid() {
  local pid="${1:-}"

  if [[ -z "$pid" ]] || ! kill -0 "$pid" 2>/dev/null; then
    return
  fi

  kill "$pid" 2>/dev/null || true

  for _ in {1..20}; do
    if ! kill -0 "$pid" 2>/dev/null; then
      return
    fi
    sleep 0.25
  done

  kill -9 "$pid" 2>/dev/null || true
}

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  stop_pid "${FRONTEND_PID:-}"
  stop_pid "${BACKEND_PID:-}"
  exit "$exit_code"
}

require_command npm
require_command node
require_command curl
require_command python3
check_python_modules
bash "$ROOT_DIR/docs/test/e2e/ensure-playwright-ready.sh"

trap cleanup EXIT INT TERM

BACKEND_CHECK_HOST="$(resolve_check_host "$BACKEND_BIND_HOST")"
FRONTEND_CHECK_HOST="$(resolve_check_host "$FRONTEND_BIND_HOST")"
REQUESTED_BACKEND_PORT="$BACKEND_PORT"
REQUESTED_FRONTEND_PORT="$FRONTEND_PORT"

BACKEND_PORT="$(find_available_port "$BACKEND_CHECK_HOST" "$BACKEND_PORT" "rc26_xhu_viewer E2E Stub 后端")"
FRONTEND_PORT="$(find_available_port "$FRONTEND_CHECK_HOST" "$FRONTEND_PORT" "rc26_xhu_viewer 前端预览服务")"

if [[ "$BACKEND_PORT" != "$REQUESTED_BACKEND_PORT" ]]; then
  echo "==> rc26_xhu_viewer E2E Stub 后端端口 ${REQUESTED_BACKEND_PORT} 已被占用，改用 ${BACKEND_PORT}"
fi

if [[ "$FRONTEND_PORT" != "$REQUESTED_FRONTEND_PORT" ]]; then
  echo "==> rc26_xhu_viewer 前端预览端口 ${REQUESTED_FRONTEND_PORT} 已被占用，改用 ${FRONTEND_PORT}"
fi

FRONTEND_ORIGIN="http://${FRONTEND_CHECK_HOST}:${FRONTEND_PORT}"
BACKEND_BASE_URL="http://${BACKEND_CHECK_HOST}:${BACKEND_PORT}"
BACKEND_WS_BASE_URL="ws://${BACKEND_CHECK_HOST}:${BACKEND_PORT}"

echo "==> 启动 rc26_xhu_viewer E2E Stub 后端"
(
  cd "$ROOT_DIR"
  exec python3 docs/test/e2e/xhu_viewer_stub_server.py --host "$BACKEND_BIND_HOST" --port "$BACKEND_PORT"
) >"$BACKEND_LOG" 2>&1 &
BACKEND_PID=$!

wait_for_http "${BACKEND_BASE_URL}/health" "rc26_xhu_viewer E2E Stub 后端" "$BACKEND_PID" "$BACKEND_LOG"

rm -rf "$FRONTEND_DIST"

echo "==> 使用 E2E Stub API 构建 rc26_xhu_viewer web"
(
  cd "$FRONTEND_DIR"
  exec env \
    VITE_API_BASE_URL="$BACKEND_BASE_URL" \
    VITE_WS_BASE_URL="$BACKEND_WS_BASE_URL" \
    npm run build -- --outDir dist-e2e
)

echo "==> 启动 rc26_xhu_viewer web 静态预览"
(
  cd "$FRONTEND_DIR"
  exec python3 -m http.server "$FRONTEND_PORT" --bind "$FRONTEND_BIND_HOST" --directory "$FRONTEND_DIST"
) >"$FRONTEND_LOG" 2>&1 &
FRONTEND_PID=$!

wait_for_http "${FRONTEND_ORIGIN}/" "rc26_xhu_viewer web 静态预览" "$FRONTEND_PID" "$FRONTEND_LOG"

echo "==> 运行 rc26_xhu_viewer web 浏览器 E2E"
exec env \
  E2E_FRONTEND_URL="$FRONTEND_ORIGIN" \
  E2E_ARTIFACT_DIR="$ARTIFACT_DIR" \
  python3 "$ROOT_DIR/docs/test/e2e/xhu_viewer_flow.py"
