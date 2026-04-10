# rviz2

## 当前状态

`src/rc26_xhu_viewer/rviz2/` 已于 2026-04-10 随整棵 `src/rc26_xhu_viewer/` 删除，不再是当前工作区源码。

## 历史职责

- RC26 定制 GUI 壳层
- 6 份 `.rviz` preset
- 本地 Web adapter 与浏览器前端
- 若干 RViz fork 与 vendor 资产

## 当前口径

- `rc26_bringup` 与 `odometry.launch.py` 都已收口为 headless
- 根仓库不再提供 `start_rviz2_rc26_web.sh`、`rviz2_rc26_server.py` 或 `viewer/` 前端入口
- 如果后续重新引入 GUI，应视为新的架构变更，而不是继续假定本目录仍然存在
