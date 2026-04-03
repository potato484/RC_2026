#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
FRONTEND_DIR="$ROOT_DIR/merlin-bt-visualizer"
PACKAGE_ROOT="$ROOT_DIR/release/merlin_bt_visualizer"
ARCHIVE_PATH="$ROOT_DIR/release/merlin_bt_visualizer.tgz"
SKIP_BUILD=0

usage() {
  cat <<'EOF'
Usage: bash docs/test/merlin_bt_visualizer/package-release.sh [--skip-build]

Options:
  --skip-build   Reuse existing merlin-bt-visualizer/dist.
  -h, --help     Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-build)
      SKIP_BUILD=1
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

if [[ $SKIP_BUILD -eq 0 ]]; then
  npm --prefix "$FRONTEND_DIR" run build
fi

rm -rf "$PACKAGE_ROOT" "$ARCHIVE_PATH"
mkdir -p "$PACKAGE_ROOT/frontend" "$PACKAGE_ROOT/docs"

cp -R "$FRONTEND_DIR/dist" "$PACKAGE_ROOT/frontend/dist"
cp "$FRONTEND_DIR/package.json" "$PACKAGE_ROOT/frontend/package.json"
cp "$ROOT_DIR/docs/test/merlin_bt_visualizer/README.md" "$PACKAGE_ROOT/docs/test-merlin-bt-visualizer.md"
cp "$ROOT_DIR/docs/frontend/README.md" "$PACKAGE_ROOT/docs/frontend-entry.md"
cp "$ROOT_DIR/docs/frontend/overview/README.md" "$PACKAGE_ROOT/docs/frontend-overview.md"
cp "$ROOT_DIR/docs/frontend/editor_mode/README.md" "$PACKAGE_ROOT/docs/frontend-editor-mode.md"

cat >"$PACKAGE_ROOT/README.md" <<'EOF'
# merlin_bt_visualizer release

- `frontend/dist/`：可直接由任意静态文件服务托管的前端构建产物。
- `docs/`：当前前端入口、总览、编辑模式与测试链路说明摘录。
- 当前 release 只收口静态站点，不包含开发态“写回源文件”适配层。
EOF

tar -czf "$ARCHIVE_PATH" -C "$ROOT_DIR/release" merlin_bt_visualizer

echo "==> Release package ready:"
echo "    - directory: $PACKAGE_ROOT"
echo "    - archive:   $ARCHIVE_PATH"
