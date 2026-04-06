# 前端总览

## 当前工程

- `merlin-bt-visualizer`
- `src/rc26_visualization/viewer`

其中 `src/rc26_visualization/viewer` 已经替代原先挂在 `rc26_topo_nav` 下的 `sim_viewer`，成为仓库内唯一维护中的比赛场地 Web 可视化前端。

## 可视化平台链路

当前主链路是：

`rc26_topo_nav graph/world/sim_assets`
-> `rc26_visualization/scripts/visualization_server.py`
-> `src/rc26_visualization/viewer`

这条链路的职责拆分是：

- `rc26_topo_nav` 提供 graph、surface graph、planner CLI 和 world 资产
- `rc26_visualization` 提供浏览器消费模型、live bridge 和布局元数据
- viewer 只负责渲染与交互，不拥有运行时真源

## 当前入口文件

- [src/rc26_visualization/viewer/src/main.tsx](/home/potato/RC_2026/src/rc26_visualization/viewer/src/main.tsx)
- [src/rc26_visualization/viewer/src/App.tsx](/home/potato/RC_2026/src/rc26_visualization/viewer/src/App.tsx)
- [src/rc26_visualization/scripts/visualization_server.py](/home/potato/RC_2026/src/rc26_visualization/scripts/visualization_server.py)
