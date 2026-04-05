#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SIM_VIEWER_DIR="$ROOT_DIR/src/rc26_topo_nav/sim_viewer"
PACKAGE_ROOT="$ROOT_DIR/release/rc26_topo_sim_viewer"
ARCHIVE_PATH="$ROOT_DIR/release/rc26_topo_sim_viewer.tgz"
SKIP_BUILD=0

usage() {
  cat <<'EOF'
Usage: bash docs/test/release/package-release.sh [--skip-build]

Options:
  --skip-build   Reuse existing src/rc26_topo_nav/sim_viewer/dist.
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
  npm --prefix "$SIM_VIEWER_DIR" run build
fi

rm -rf "$PACKAGE_ROOT" "$ARCHIVE_PATH"
mkdir -p "$PACKAGE_ROOT/frontend" "$PACKAGE_ROOT/adapter"

cp -R "$SIM_VIEWER_DIR/dist" "$PACKAGE_ROOT/frontend/dist"
cp "$SIM_VIEWER_DIR/package.json" "$PACKAGE_ROOT/frontend/package.json"
cp "$ROOT_DIR/src/rc26_topo_nav/scripts/topo_sim_server.py" "$PACKAGE_ROOT/adapter/topo_sim_server.py"
cp "$ROOT_DIR/src/rc26_topo_nav/scripts/topo_sim_algorithms.py" "$PACKAGE_ROOT/adapter/topo_sim_algorithms.py"
cp "$ROOT_DIR/src/rc26_topo_nav/scripts/render_graph_sim_html.py" "$PACKAGE_ROOT/adapter/render_graph_sim_html.py"
cp -R "$ROOT_DIR/src/rc26_topo_nav/config" "$PACKAGE_ROOT/adapter/config"
cp -R "$ROOT_DIR/src/rc26_topo_nav/sim_assets" "$PACKAGE_ROOT/adapter/sim_assets"

tar -czf "$ARCHIVE_PATH" -C "$ROOT_DIR/release" rc26_topo_sim_viewer

echo "==> Release package ready:"
echo "    - directory: $PACKAGE_ROOT"
echo "    - archive:   $ARCHIVE_PATH"
