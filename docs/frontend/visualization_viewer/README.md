# visualization_viewer

## 模块定位

`src/rc26_visualization/viewer` 是 R2 当前维护中的本地 Web 可视化平台前端。

它已经不再只是“任意点 3D 路线观察台”，而是统一承载：

- 场地 mesh 与 topo/surface-route 路线观察
- 表面路线搜索回放
- 局部规划案例回放
- live 运行态、定位健康、keepout、机构状态和 BT 快照

## 当前关键文件

- [src/main.tsx](/home/potato/RC_2026/src/rc26_visualization/viewer/src/main.tsx)
- [src/App.tsx](/home/potato/RC_2026/src/rc26_visualization/viewer/src/App.tsx)
- [src/store.ts](/home/potato/RC_2026/src/rc26_visualization/viewer/src/store.ts)
- [src/layerModel.ts](/home/potato/RC_2026/src/rc26_visualization/viewer/src/layerModel.ts)
- [src/components/SceneCanvas.tsx](/home/potato/RC_2026/src/rc26_visualization/viewer/src/components/SceneCanvas.tsx)
- [src/api.ts](/home/potato/RC_2026/src/rc26_visualization/viewer/src/api.ts)

## 当前数据入口

- `/api/scene-manifest`
- `/api/surface-route/preview`
- `/api/surface-route/trace-from-nodes`
- `/api/surface-route/execute`
- `/api/local-planner/scenarios`
- `/api/local-planner/trace`
- `/api/live/start`
- `/api/live/events`

## 当前真实实现

- 页面标题、布局预设和阶段区来自 `field_scene_manifest.yaml`
- `field_scene_manifest.yaml` 里的阶段区现在按三块粗粒度 BT phase band 定义：`MCAreaTree / MFAreaTree / CombatAreaTree`，不再把梅林坡道入口/出口单独画成额外阶段条带
- `operator / engineering / diagnostic` 三套 preset 会直接改写图层集合
- 触发 `surface-route` 生成时，viewer 会自动重新打开 `route` 图层，避免诊断布局把新生成的路线完全隐藏
- preview 路线会额外渲染高对比底衬、亮色主线和路径珠点，保证在俯视缩放较远时也能直接看见路线走向
- 当 body-aware surface route 因车体约束失败时，viewer 会继续渲染 `visualization_server.py` 回退出的 `legacy` 参考路线，并把它标成“仅供观察、不可执行”
- 失败原因的中文化不再只支持精确匹配；像 `heading change ...`、`node clearance ...` 这类 body planner 细节现在也会直接显示成中文摘要
- 场景里会同时渲染 route、corridor、lookahead、robotPose、phaseZones 和 keepout
- `controlState.pose` 会优先作为机器人 live 位姿
- `btSnapshot.activeSubtreeId` 会驱动阶段区高亮

## 当前边界

- viewer 不拥有 planner 真源，回放仍以 `rc26_topo_nav` CLI 输出为准
- viewer 不是 bringup 或 ROS2 的权威进程
- 页面允许执行 surface route，但这只是受控 action 下发，不是浏览器直控
