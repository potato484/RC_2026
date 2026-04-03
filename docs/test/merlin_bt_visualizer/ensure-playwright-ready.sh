#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
FRONTEND_DIR="$ROOT_DIR/merlin-bt-visualizer"

if [[ ! -d "$FRONTEND_DIR/node_modules" ]]; then
  echo "==> merlin-bt-visualizer 缺少 node_modules，先安装依赖"
  npm --prefix "$FRONTEND_DIR" ci
fi

echo "==> 准备 Playwright Chromium 运行时"
if [[ "${CI:-}" == "true" ]]; then
  npm --prefix "$FRONTEND_DIR" exec playwright install --with-deps chromium
else
  npm --prefix "$FRONTEND_DIR" exec playwright install chromium
fi
