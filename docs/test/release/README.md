# Release 打包

当前 release 脚本打包的是 `rc26_visualization` 的 Web 可视化链路，而不是旧的 topo sim viewer。

## 当前入口

- `docs/test/release/package-release.sh`
- `docs/test/release/deploy-via-ssh.sh`

## 当前打包内容

- `src/rc26_visualization/viewer/dist`
- `src/rc26_visualization/scripts/visualization_server.py`
- `src/rc26_visualization/scripts/visualization_algorithms.py`
- `src/rc26_visualization/scripts/render_graph_sim_html.py`
- `src/rc26_visualization/config/field_scene_manifest.yaml`
- `src/rc26_topo_nav/config/r2_*graph*.yaml`
- `src/rc26_topo_nav/sim_assets`

## 当前输出目录

- `release/rc26_visualization_viewer/`
- `release/rc26_visualization_viewer.tgz`
