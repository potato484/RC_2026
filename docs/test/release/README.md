# Release 打包

当前 release 脚本打包的是 `rviz2_rc26` 的 Web 可视化链路，而不是旧的 topo sim viewer。

## 当前入口

- `docs/test/release/package-release.sh`
- `docs/test/release/deploy-via-ssh.sh`

## 当前打包内容

- `src/rc26_xhu_viewer/rviz2/viewer/dist`
- `src/rc26_xhu_viewer/rviz2/scripts/rviz2_rc26_server.py`
- `src/rc26_xhu_viewer/rviz2/scripts/visualization_algorithms.py`
- `src/rc26_topo_nav/scripts/render_graph_sim_html.py`
- `src/rc26_xhu_viewer/rviz2/config/field_scene_manifest.yaml`
- `src/rc26_topo_nav/config/r2_*graph*.yaml`
- `src/rc26_topo_nav/sim_assets`

## 当前输出目录

- `release/rviz2_rc26_web/`
- `release/rviz2_rc26_web.tgz`
