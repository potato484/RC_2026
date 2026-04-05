#!/usr/bin/env bash
set -uo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
FRONTEND_DIR="$ROOT_DIR/merlin-bt-visualizer"
REPORT_BASE_DIR="${MERLIN_PREFLIGHT_REPORT_BASE_DIR:-$ROOT_DIR/artifacts/merlin-bt-visualizer/preflight}"
TIMESTAMP="$(date '+%Y%m%d-%H%M%S')"
REPORT_DIR="$REPORT_BASE_DIR/$TIMESTAMP"
LOG_DIR="$REPORT_DIR/logs"
SUMMARY_FILE="$REPORT_DIR/summary.md"
LATEST_FILE="$REPORT_BASE_DIR/latest.md"

STRICT=0
REFRESH_DEPS=0
SKIP_E2E=0

declare -a STEP_IDS=()
declare -a NOTES=()
declare -A STEP_CATEGORY=()
declare -A STEP_LABEL=()
declare -A STEP_STATUS=()
declare -A STEP_DETAIL=()
declare -A STEP_LOG=()

usage() {
  cat <<'EOF'
Usage: bash docs/test/merlin_bt_visualizer/local-preflight.sh [options]

Options:
  --strict         Exit non-zero on WARN, BLOCKED or FAIL.
  --refresh-deps   Force npm ci for merlin-bt-visualizer.
  --skip-e2e       Skip docs/test/merlin_bt_visualizer/run-e2e-local.sh.
  -h, --help       Show this help.
EOF
}

require_command() {
  local name="$1"
  if ! command -v "$name" >/dev/null 2>&1; then
    echo "缺少必要命令: $name" >&2
    exit 1
  fi
}

record_note() {
  NOTES+=("$1")
}

record_step() {
  local id="$1"
  local category="$2"
  local label="$3"
  local status="$4"
  local detail="$5"
  local log_file="${6:-}"

  STEP_IDS+=("$id")
  STEP_CATEGORY["$id"]="$category"
  STEP_LABEL["$id"]="$label"
  STEP_STATUS["$id"]="$status"
  STEP_DETAIL["$id"]="$detail"
  STEP_LOG["$id"]="$log_file"
}

relative_path() {
  local target="$1"
  realpath --relative-to="$ROOT_DIR" "$target" 2>/dev/null || printf '%s' "$target"
}

run_logged_step() {
  local id="$1"
  local category="$2"
  local label="$3"
  local command_text="$4"
  local log_file="$LOG_DIR/${id}.log"

  {
    printf '$ %s\n\n' "$command_text"
    (
      cd "$ROOT_DIR"
      eval "$command_text"
    )
  } >"$log_file" 2>&1
  local exit_code=$?

  if [[ $exit_code -eq 0 ]]; then
    printf '==> PASS     %s\n' "$label"
    record_step "$id" "$category" "$label" "PASS" "Command succeeded." "$log_file"
    return 0
  fi

  printf '==> FAIL     %s [%s]\n' "$label" "$(relative_path "$log_file")"
  tail -n 60 "$log_file" || true
  record_step "$id" "$category" "$label" "FAIL" "Command exited with status $exit_code." "$log_file"
  return $exit_code
}

node_major_version() {
  node -p 'process.versions.node.split(".")[0]'
}

write_report() {
  mkdir -p "$REPORT_BASE_DIR"

  {
    printf '# Merlin Preflight Report\n\n'
    printf -- '- Generated at: `%s`\n' "$(date '+%Y-%m-%d %H:%M:%S %Z')"
    printf -- '- Workspace: `%s`\n' "$ROOT_DIR"
    printf -- '- Strict mode: `%s`\n\n' "$([[ $STRICT -eq 1 ]] && printf 'on' || printf 'off')"

    printf '## Step Results\n\n'
    printf '| Category | Step | Status | Detail | Log |\n'
    printf '| --- | --- | --- | --- | --- |\n'

    local id log_path
    for id in "${STEP_IDS[@]}"; do
      log_path="${STEP_LOG[$id]}"
      if [[ -n "$log_path" ]]; then
        log_path="$(relative_path "$log_path")"
      else
        log_path='-'
      fi

      printf '| %s | %s | %s | %s | %s |\n' \
        "${STEP_CATEGORY[$id]}" \
        "${STEP_LABEL[$id]}" \
        "${STEP_STATUS[$id]}" \
        "${STEP_DETAIL[$id]}" \
        "$log_path"
    done

    if [[ ${#NOTES[@]} -gt 0 ]]; then
      printf '\n## Notes\n\n'
      local note
      for note in "${NOTES[@]}"; do
        printf -- '- %s\n' "$note"
      done
    fi
  } >"$SUMMARY_FILE"

  ln -sfn "$SUMMARY_FILE" "$LATEST_FILE"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --strict)
      STRICT=1
      ;;
    --refresh-deps)
      REFRESH_DEPS=1
      ;;
    --skip-e2e)
      SKIP_E2E=1
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

require_command bash
require_command node
require_command npm
require_command curl

mkdir -p "$LOG_DIR"

NODE_MAJOR="$(node_major_version)"
if [[ "$NODE_MAJOR" == "22" ]]; then
  record_step "node-parity" "preflight" "Node 版本对齐" "PASS" "Detected Node ${NODE_MAJOR}.x." ""
else
  record_step "node-parity" "preflight" "Node 版本对齐" "WARN" "Detected Node ${NODE_MAJOR}.x; CI targets Node 22.x." ""
fi

if [[ ! -d "$FRONTEND_DIR/node_modules" || $REFRESH_DEPS -eq 1 ]]; then
  run_logged_step \
    "frontend-deps" \
    "ci" \
    "安装 merlin Node 依赖" \
    "npm --prefix \"$FRONTEND_DIR\" ci"
else
  record_step "frontend-deps" "ci" "安装 merlin Node 依赖" "PASS" "Reused existing node_modules in merlin-bt-visualizer." ""
fi

run_logged_step \
  "frontend-unit" \
  "ci" \
  "运行 merlin 单测" \
  "npm --prefix \"$FRONTEND_DIR\" test"

run_logged_step \
  "frontend-build" \
  "ci" \
  "构建 merlin 前端" \
  "npm --prefix \"$FRONTEND_DIR\" run build"

if [[ $SKIP_E2E -eq 1 ]]; then
  record_step "frontend-e2e" "e2e" "运行 merlin 浏览器 E2E" "SKIP" "Skipped by --skip-e2e." ""
else
  run_logged_step \
    "frontend-e2e" \
    "e2e" \
    "运行 merlin 浏览器 E2E" \
    "bash docs/test/merlin_bt_visualizer/run-e2e-local.sh"
fi

run_logged_step \
  "release-package" \
  "cd" \
  "打包 merlin release 目录" \
  "bash docs/test/merlin_bt_visualizer/package-release.sh"

record_note "浏览器 E2E 当前拆成两段：preview 覆盖查看/编辑中文链路，dev 覆盖保存到源文件的本地适配层。"
record_note "release 产物只包含静态站点与文档摘录，不包含开发态写回源文件能力。"

write_report

printf '==> 报告已生成: %s\n' "$(relative_path "$SUMMARY_FILE")"

exit_code=0
for id in "${STEP_IDS[@]}"; do
  status="${STEP_STATUS[$id]}"
  if [[ "$status" == "FAIL" ]]; then
    exit_code=1
    break
  fi
  if [[ $STRICT -eq 1 && ( "$status" == "WARN" || "$status" == "BLOCKED" ) ]]; then
    exit_code=1
    break
  fi
done

exit "$exit_code"
