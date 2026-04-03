#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: bash docs/test/e2e/ensure-playwright-ready.sh [--check]

Default behavior:
  - Ensures Python Playwright is installed.
  - Ensures both Chromium and chromium_headless_shell are installed.

Options:
  --check     Exit non-zero unless the runtime is already ready.
  -h, --help  Show this help.
EOF
}

MODE="ensure"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --check)
      MODE="check"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
  shift
done

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "缺少必要命令: $1" >&2
    exit 1
  fi
}

has_python_playwright() {
  python3 - <<'PY'
import importlib.util
raise SystemExit(0 if importlib.util.find_spec("playwright") else 1)
PY
}

ensure_python_pip() {
  if ! python3 -m pip --version >/dev/null 2>&1; then
    echo "Python pip 不可用，无法自动安装 Playwright。" >&2
    exit 1
  fi
}

playwright_install_list() {
  python3 -m playwright install --list
}

has_playwright_chromium_bundle() {
  local install_list
  install_list="$(playwright_install_list 2>/dev/null)" || return 1

  grep -Eq '/chromium-[0-9]+' <<<"$install_list" || return 1
  grep -Eq '/chromium_headless_shell-[0-9]+' <<<"$install_list" || return 1
}

ensure_python_playwright() {
  if has_python_playwright; then
    return 0
  fi

  ensure_python_pip
  echo "==> 安装 Python Playwright 包"
  python3 -m pip install --upgrade pip
  python3 -m pip install playwright
}

ensure_playwright_browsers() {
  if has_playwright_chromium_bundle; then
    return 0
  fi

  echo "==> 安装 Playwright Chromium 与 headless shell"
  python3 -m playwright install chromium
}

require_command python3

if [[ "$MODE" == "check" ]]; then
  has_python_playwright || exit 1
  has_playwright_chromium_bundle || exit 1
  exit 0
fi

ensure_python_playwright
ensure_playwright_browsers

if has_playwright_chromium_bundle; then
  echo "==> Playwright Chromium 运行时已就绪"
  exit 0
fi

echo "Playwright Chromium 运行时仍未就绪。" >&2
exit 1
