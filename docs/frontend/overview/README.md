# 前端总览

## 当前工程

- `merlin-bt-visualizer`
- `src/rc26_xhu_viewer/rviz2/viewer`

其中 `src/rc26_xhu_viewer/rviz2/viewer` 已经替代原先挂在 `rc26_topo_nav` 下的 `sim_viewer`，并在本轮重构后与 `rviz2` 包内的 RC26 GUI / Web 资产统一收口，成为仓库内唯一维护中的比赛场地 Web 可视化前端。

## 可视化平台链路

当前主链路是：

`rc26_topo_nav graph/world/sim_assets`
-> `rviz2/scripts/rviz2_rc26_server.py`
-> `src/rc26_xhu_viewer/rviz2/viewer`

这条链路的职责拆分是：

- `rc26_topo_nav` 提供 graph、surface graph、planner CLI、`render_graph_sim_html.py` 真源和 world 资产
- `rviz2` 提供浏览器消费模型、live bridge、布局元数据与本地工程入口
- viewer 只负责渲染与交互，不拥有运行时真源

## 当前入口文件

- [src/rc26_xhu_viewer/rviz2/viewer/src/main.tsx](/home/potato/RC_2026/src/rc26_xhu_viewer/rviz2/viewer/src/main.tsx)
- [src/rc26_xhu_viewer/rviz2/viewer/src/App.tsx](/home/potato/RC_2026/src/rc26_xhu_viewer/rviz2/viewer/src/App.tsx)
- [src/rc26_xhu_viewer/rviz2/viewer/src/components/scene/BabylonSceneManager.ts](/home/potato/RC_2026/src/rc26_xhu_viewer/rviz2/viewer/src/components/scene/BabylonSceneManager.ts)
- [src/rc26_xhu_viewer/rviz2/scripts/rviz2_rc26_server.py](/home/potato/RC_2026/src/rc26_xhu_viewer/rviz2/scripts/rviz2_rc26_server.py)

## 本轮结构更新

- `SceneCanvas.tsx` 已缩成 React 壳层，Babylon 细节移到 `components/scene/BabylonSceneManager.ts`
- trace 结果组装移到 `features/trace/traceModel.ts`
- live 状态归一化移到 `features/live/liveBridge.ts`
- viewer 文案/格式化移到 `features/viewer/formatting.ts`

这次拆分的目标只是降低维护复杂度，不改变 `rc26_topo_nav`、`rc26_xhu_viewer_status` 与 `rc26_bringup` 的权威边界。
