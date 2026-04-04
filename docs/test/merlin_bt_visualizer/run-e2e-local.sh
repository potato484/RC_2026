#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
FRONTEND_DIR="$ROOT_DIR/merlin-bt-visualizer"
ARTIFACT_DIR="${MERLIN_E2E_ARTIFACT_DIR:-$ROOT_DIR/artifacts/merlin-bt-visualizer/e2e}"
LOG_DIR="$ARTIFACT_DIR/logs"
SAVE_TARGET_DIR="$ARTIFACT_DIR/save-targets"
PREVIEW_LOG="$LOG_DIR/preview.log"
PREVIEW_BUILD_LOG="$LOG_DIR/preview-build.log"
DEV_LOG="$LOG_DIR/dev.log"
REQUESTED_PREVIEW_PORT="${MERLIN_E2E_PREVIEW_PORT:-4173}"
REQUESTED_DEV_PORT="${MERLIN_E2E_DEV_PORT:-4174}"

PREVIEW_PID=""
DEV_PID=""

pick_available_port() {
  python3 - "$@" <<'PY'
import socket
import sys

start = int(sys.argv[1])
blocked = {int(value) for value in sys.argv[2:] if value}

for port in range(start, 65536):
    if port in blocked:
        continue

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("127.0.0.1", port))
        except OSError:
            continue

    print(port)
    break
else:
    raise SystemExit("No available TCP port found.")
PY
}

wait_for_http() {
  local url="$1"
  local label="$2"
  local pid="$3"
  local log_file="$4"

  for _ in $(seq 1 60); do
    if curl -fsS "$url" >/dev/null 2>&1; then
      return 0
    fi

    if ! kill -0 "$pid" >/dev/null 2>&1; then
      echo "==> $label 提前退出，日志尾部如下:" >&2
      tail -n 80 "$log_file" >&2 || true
      return 1
    fi

    sleep 1
  done

  echo "==> 等待 $label 超时，日志尾部如下:" >&2
  tail -n 80 "$log_file" >&2 || true
  return 1
}

cleanup() {
  if [[ -n "$PREVIEW_PID" ]] && kill -0 "$PREVIEW_PID" >/dev/null 2>&1; then
    kill "$PREVIEW_PID" >/dev/null 2>&1 || true
    wait "$PREVIEW_PID" >/dev/null 2>&1 || true
  fi

  if [[ -n "$DEV_PID" ]] && kill -0 "$DEV_PID" >/dev/null 2>&1; then
    kill "$DEV_PID" >/dev/null 2>&1 || true
    wait "$DEV_PID" >/dev/null 2>&1 || true
  fi
}

trap cleanup EXIT

rm -rf "$ARTIFACT_DIR"
mkdir -p "$LOG_DIR" "$SAVE_TARGET_DIR"

PREVIEW_PORT="$(pick_available_port "$REQUESTED_PREVIEW_PORT")"
DEV_PORT="$(pick_available_port "$REQUESTED_DEV_PORT" "$PREVIEW_PORT")"

if [[ "$PREVIEW_PORT" != "$REQUESTED_PREVIEW_PORT" ]]; then
  echo "==> 静态预览端口 $REQUESTED_PREVIEW_PORT 已被占用，改用 $PREVIEW_PORT"
fi

if [[ "$DEV_PORT" != "$REQUESTED_DEV_PORT" ]]; then
  echo "==> 开发态端口 $REQUESTED_DEV_PORT 已被占用，改用 $DEV_PORT"
fi

bash "$ROOT_DIR/docs/test/merlin_bt_visualizer/ensure-playwright-ready.sh"

echo "==> 构建 merlin-bt-visualizer 预览产物"
(
  cd "$FRONTEND_DIR"
  exec npm run build
) >"$PREVIEW_BUILD_LOG" 2>&1

echo "==> 启动 merlin-bt-visualizer 静态预览"
(
  cd "$FRONTEND_DIR"
  exec npm run preview -- --host 127.0.0.1 --port "$PREVIEW_PORT" --strictPort
) >"$PREVIEW_LOG" 2>&1 &
PREVIEW_PID=$!

wait_for_http "http://127.0.0.1:${PREVIEW_PORT}/" "静态预览" "$PREVIEW_PID" "$PREVIEW_LOG"

echo "==> 运行查看态 / 编辑态联动 E2E"
E2E_BASE_URL="http://127.0.0.1:${PREVIEW_PORT}" \
PLAYWRIGHT_HTML_REPORT="$ARTIFACT_DIR/playwright-report/viewer-editor" \
PLAYWRIGHT_TEST_OUTPUT_DIR="$ARTIFACT_DIR/test-results/viewer-editor" \
npm --prefix "$FRONTEND_DIR" run test:e2e:specs -- e2e/viewer-editor.spec.ts

kill "$PREVIEW_PID" >/dev/null 2>&1 || true
wait "$PREVIEW_PID" >/dev/null 2>&1 || true
PREVIEW_PID=""

echo "==> 准备开发态写回临时源文件"
cp "$ROOT_DIR/src/rc26_decision/behavior_trees/mf_tree.xml" "$SAVE_TARGET_DIR/mf_tree.xml"
cp "$ROOT_DIR/src/rc26_decision/behavior_trees/mc_tree.xml" "$SAVE_TARGET_DIR/mc_tree.xml"
cp "$ROOT_DIR/src/rc26_decision/behavior_trees/combat_tree.xml" "$SAVE_TARGET_DIR/combat_tree.xml"

echo "==> 启动 merlin-bt-visualizer 开发态服务"
(
  cd "$FRONTEND_DIR"
  exec env MERLIN_BT_SAVE_DIR="$SAVE_TARGET_DIR" npm run dev -- --host 127.0.0.1 --port "$DEV_PORT" --strictPort
) >"$DEV_LOG" 2>&1 &
DEV_PID=$!

wait_for_http "http://127.0.0.1:${DEV_PORT}/" "开发态服务" "$DEV_PID" "$DEV_LOG"

echo "==> 运行开发态写回 E2E"
E2E_BASE_URL="http://127.0.0.1:${DEV_PORT}" \
E2E_SAVE_DIR="$SAVE_TARGET_DIR" \
PLAYWRIGHT_HTML_REPORT="$ARTIFACT_DIR/playwright-report/save-to-source" \
PLAYWRIGHT_TEST_OUTPUT_DIR="$ARTIFACT_DIR/test-results/save-to-source" \
npm --prefix "$FRONTEND_DIR" run test:e2e:specs -- e2e/save-to-source.spec.ts

echo "==> merlin-bt-visualizer E2E 完成"
echo "    - 产物目录: $ARTIFACT_DIR"
echo "    - 预览日志: $PREVIEW_LOG"
echo "    - 开发态日志: $DEV_LOG"
