#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
VIEWER_DIR="$ROOT_DIR/src/rc26_visualization/viewer"
PACKAGE_ROOT="$ROOT_DIR/release/rc26_visualization_viewer"
ARCHIVE_PATH="$ROOT_DIR/release/rc26_visualization_viewer.tgz"
SKIP_BUILD=0

usage() {
  cat <<'EOF'
Usage: bash docs/test/release/package-release.sh [--skip-build]

Options:
  --skip-build   Reuse existing src/rc26_visualization/viewer/dist.
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
  npm --prefix "$VIEWER_DIR" run build
fi

rm -rf "$PACKAGE_ROOT" "$ARCHIVE_PATH"
mkdir -p "$PACKAGE_ROOT/frontend" "$PACKAGE_ROOT/scripts" "$PACKAGE_ROOT/config"

cp -R "$VIEWER_DIR/dist" "$PACKAGE_ROOT/frontend/dist"
cp "$VIEWER_DIR/package.json" "$PACKAGE_ROOT/frontend/package.json"
cp "$ROOT_DIR/src/rc26_visualization/scripts/visualization_server.py" "$PACKAGE_ROOT/scripts/visualization_server.py"
cp "$ROOT_DIR/src/rc26_visualization/scripts/visualization_algorithms.py" "$PACKAGE_ROOT/scripts/visualization_algorithms.py"
cp "$ROOT_DIR/src/rc26_visualization/scripts/render_graph_sim_html.py" "$PACKAGE_ROOT/scripts/render_graph_sim_html.py"
cp -R "$ROOT_DIR/src/rc26_topo_nav/sim_assets" "$PACKAGE_ROOT/sim_assets"
cp "$ROOT_DIR/src/rc26_visualization/config/field_scene_manifest.yaml" "$PACKAGE_ROOT/config/field_scene_manifest.yaml"
cp "$ROOT_DIR/src/rc26_topo_nav/config/r2_field_graph_blue.yaml" "$PACKAGE_ROOT/config/r2_field_graph_blue.yaml"
cp "$ROOT_DIR/src/rc26_topo_nav/config/r2_field_graph_red.yaml" "$PACKAGE_ROOT/config/r2_field_graph_red.yaml"
cp "$ROOT_DIR/src/rc26_topo_nav/config/r2_surface_graph_blue.yaml" "$PACKAGE_ROOT/config/r2_surface_graph_blue.yaml"
cp "$ROOT_DIR/src/rc26_topo_nav/config/r2_surface_graph_red.yaml" "$PACKAGE_ROOT/config/r2_surface_graph_red.yaml"

tar -czf "$ARCHIVE_PATH" -C "$ROOT_DIR/release" rc26_visualization_viewer

echo "==> Release package ready:"
echo "    - directory: $PACKAGE_ROOT"
echo "    - archive:   $ARCHIVE_PATH"
