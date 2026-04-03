#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
RELEASE_DIR="$ROOT_DIR/release/rc26_topo_sim_viewer"

usage() {
  cat <<'EOF'
Usage: bash docs/test/release/deploy-via-ssh.sh

Required environment variables:
  DEPLOY_HOST
  DEPLOY_USER
  DEPLOY_PATH

Optional environment variables:
  DEPLOY_PORT
  DEPLOY_SSH_KEY_PATH
EOF
}

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "缺少必要命令: $1" >&2
    exit 1
  fi
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

require_command ssh
require_command rsync

: "${DEPLOY_HOST:?DEPLOY_HOST is required}"
: "${DEPLOY_USER:?DEPLOY_USER is required}"
: "${DEPLOY_PATH:?DEPLOY_PATH is required}"

if [[ ! -d "$RELEASE_DIR" ]]; then
  bash "$ROOT_DIR/docs/test/release/package-release.sh"
fi

SSH_PORT="${DEPLOY_PORT:-22}"
SSH_KEY_PATH="${DEPLOY_SSH_KEY_PATH:-}"
SSH_ARGS=(-p "$SSH_PORT" -o StrictHostKeyChecking=no)

if [[ -n "$SSH_KEY_PATH" ]]; then
  SSH_ARGS+=(-i "$SSH_KEY_PATH")
fi

ssh "${SSH_ARGS[@]}" "${DEPLOY_USER}@${DEPLOY_HOST}" "mkdir -p '$DEPLOY_PATH'"
rsync -az --delete -e "ssh ${SSH_ARGS[*]}" "$RELEASE_DIR/" "${DEPLOY_USER}@${DEPLOY_HOST}:$DEPLOY_PATH/"

echo "==> Deploy completed: ${DEPLOY_USER}@${DEPLOY_HOST}:$DEPLOY_PATH"
