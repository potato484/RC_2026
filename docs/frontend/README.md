# 前端当前实现与边界

`docs/frontend/README.md` 现在作为前端文档入口页，用来快速回答两件事：

- 当前仓库里前端到底已经实现了什么。
- 这些前端工具当前不是什么系统。

更细的实现拆解已经按模块拆到独立文档，避免单文件继续膨胀后难以维护。

## 1. 当前已经成型的前端工程

当前仓库里已经有两套成型的本地前端工具：

- `merlin-bt-visualizer`
  - 围绕 `rc26_decision` 行为树 XML 的单页本地工作台，不是在线驾驶舱。
  - 技术栈：`Vite + React 18 + TypeScript + Tailwind CSS + Zustand + @xyflow/react + dagre + framer-motion + lucide-react`
  - 工程入口：`merlin-bt-visualizer/src/main.tsx` -> `merlin-bt-visualizer/src/App.tsx`
  - 启动脚本：`npm run dev`、`npm run build`、`npm run preview`
  - 测试与交付入口：`npm --prefix merlin-bt-visualizer run test`、`npm run merlin:test:e2e`、`npm run merlin:preflight`、`npm run merlin:cd:package`
  - 原始输入源：`src/rc26_decision/behavior_trees/mf_tree.xml`、`mc_tree.xml`、`combat_tree.xml`
- `src/rc26_topo_nav/sim_viewer`
  - 围绕 `rc26_topo_nav` 的单用途三维路线观察工具，负责渲染完整场地 mesh、任意点 3D 路线和 surface-route 搜索回放。
  - 技术栈：`Vite + React 18 + TypeScript + Zustand + Babylon.js (WebGPU 优先)`
  - 工程入口：`src/rc26_topo_nav/sim_viewer/src/main.tsx` -> `src/rc26_topo_nav/sim_viewer/src/App.tsx`
  - 启动脚本：`npm run dev`、`npm run build`
  - 数据入口：`src/rc26_topo_nav/scripts/topo_sim_server.py` 提供本地 HTTP adapter，消费 `surface_graph`、Gazebo world 和 KFS 对齐配置
  - **最新变更说明**：此工具已经从旧的多模式仿真页收口成“任意点 3D 路线观察台”。页面不再暴露 `Topo 节点模式`、`RRT / DWA`、live ROS 观察和 run 播放控制，只保留：场景选起点/终点、`POST /api/surface-route/trace` 生成完整路线与逐帧 A* trace、滑块回放、可选执行当前路线。

## 2. 当前能力总览

当前前端已经成型的能力可以归成两条工具链：

- `merlin-bt-visualizer`
  - 查看模式：读取三份行为树 XML，解析成展示模型，完成分区切换、树切换、节点详情、执行日志和黑板可视化。
  - 编辑模式：把原始 XML 解析成可逆编辑语义，当前已经支持官方节点与机器人模块注册表、中文节点库、拖拽插入、任意位置分支插入、节点包裹、复合节点切换、端口/黑板绑定编辑和 XML 导出；开发态还可通过本地保存适配层把当前区域 XML 写回源文件。
  - 本地模拟执行：在浏览器里用一套确定性演示逻辑跑单次行为树执行，驱动节点状态、时间线和黑板更新。
  - 自动化验证：当前已补齐独立 GitHub CI / CD 与 Playwright E2E，查看态、编辑态联动和开发态写回都能通过脚本复现。
- `src/rc26_topo_nav/sim_viewer`
  - 完整三维场景：渲染 `robocon2026_v2_aligned.world` / `robocon2026.dae` 提取出的 mesh 面、材质色、光照和阴影。使用 Babylon.js 渲染管线。
  - 清晰路径层：用更细的三维 tube / marker 显示当前 surface route、前沿点和已探查点；当前只保留 `scene / frontier / expanded / shadows` 四个图层开关。
  - 多视角交互：支持 `orbit / top_ortho / side_perspective` 三个观察视角，专门服务路线读图。
  - 本地 adapter：主链是 `surface_route_cli -> planner_trace_cli -> /api/surface-route/trace`，由后端返回完整 3D 路线和逐帧 A* 推导过程。
  - Web 侧当前还会把这条生成链路的结构化规划日志一并展示出来，方便直接判断卡在 `surface_route_cli` 还是 `planner_trace_cli`。
  - **最新实现补充**：`surface-route` trace 已改为“真实规划 + 压缩回放”链路。`planner_trace_cli` 只输出采样帧和本次回放涉及节点的 `node_poses`，浏览器本地按 `nodeId` 复原可视化点位，避免重复传输 pose-heavy 大对象与重复解析 dense `surface_graph` YAML。
  - 任意点路线：浏览器直接在地面、坡面、阶梯表面选取起终点，生成 dense `surface_graph` 上的完整路径，并可选下发 `navigate_surface_route`。
  - 视觉与体验：保持工业战术沙盘风格，但主界面已经删掉旧的模式/算法切换和 debug 抽屉，只保留与“看一条 3D 路线如何被算出来”直接相关的操作。

当前仍然没有的能力也必须一开始就看清：

- 没有浏览器直接成为机器人控制入口的能力。
- 没有浏览器直接写回仓库文件的持久化链路。
- 没有把前端变成机器人运行时权威后端；联机能力如果存在，也必须通过单独 adapter 且保持只读或受控边界。

## 3. 模块文档索引

后续维护前端文档时，优先从这里进入，再按改动范围进入对应模块文档：

- [`overview`](overview/README.md): 前端总览、工程骨架、入口与共享约束，适合先建立全局上下文。`(file: overview/README.md)`
- [`viewer_mode`](viewer_mode/README.md): 查看模式的数据流、查看链关键文件和组件职责。`(file: viewer_mode/README.md)`
- [`editor_mode`](editor_mode/README.md): 编辑模式的 round-trip 语义链、编辑链关键文件和交互入口。`(file: editor_mode/README.md)`
- [`editor_mode/bt_terms_mapping`](editor_mode/bt_terms_mapping.md): 编辑器使用的官方节点、机器人模块、属性键、黑板键和枚举值中文映射表。`(file: editor_mode/bt_terms_mapping.md)`
- [`local_simulator`](local_simulator/README.md): `App.tsx` 里的本地模拟执行器实现、运行步骤与误读风险。`(file: local_simulator/README.md)`
- [`topo_sim_viewer`](topo_sim_viewer/README.md): `rc26_topo_nav` 3D 仿真 viewer、FastAPI adapter、Babylon.js 场景与运行边界。`(file: topo_sim_viewer/README.md)`
- [`boundaries`](boundaries/README.md): 当前前端的明确边界、未实现能力和准确定位。`(file: boundaries/README.md)`

## 4. 推荐阅读顺序

如果是第一次接触当前前端，建议按下面顺序阅读：

1. 先看本文，确认它是本地工程工具而不是在线系统。
2. 再看 [overview/README.md](overview/README.md)，建立入口、输入源和模块分层认知。
3. 如果关注行为树查看链路，看 [viewer_mode/README.md](viewer_mode/README.md)。
4. 如果关注 XML 编辑和导出链路，看 [editor_mode/README.md](editor_mode/README.md)。
5. 如果关注行为树本地执行演示，看 [local_simulator/README.md](local_simulator/README.md)。
6. 如果要对照编辑器里的中文术语和模块库映射，补看 [editor_mode/bt_terms_mapping.md](editor_mode/bt_terms_mapping.md)。
7. 如果关注 topo 导航三维仿真，看 [topo_sim_viewer/README.md](topo_sim_viewer/README.md)。
8. 如果要判断能力边界、对外描述或需求是否越界，最后回到 [boundaries/README.md](boundaries/README.md) 对照。

## 5. 按改动位置同步文档

为了让拆分后的文档真正便于维护，后续改动可以按下面方式同步：

- 改 `src/main.tsx`、`src/App.tsx` 页面装配、输入源或总体结构时，优先更新 [overview/README.md](overview/README.md)。
- 改 `useStore`、`btParser`、`TreeVisualizer`、`RightPanel`、查看态 `Sidebar` 时，优先更新 [viewer_mode/README.md](viewer_mode/README.md)。
- 改 `useEditorStore`、`editorParser`、`editorProjection`、`editorSerializer`、`EditorVisualizer`、`EditorRightPanel` 时，优先更新 [editor_mode/README.md](editor_mode/README.md)。
- 改节点注册表、中文术语表、黑板映射或模块库分组时，优先更新 [editor_mode/bt_terms_mapping.md](editor_mode/bt_terms_mapping.md)。
- 改 `App.tsx` 里的本地执行逻辑、时间线、黑板、模拟/实机提示时，优先更新 [local_simulator/README.md](local_simulator/README.md)。
- 改 `src/rc26_topo_nav/sim_viewer/*`、`src/rc26_topo_nav/scripts/topo_sim_server.py`、`src/rc26_topo_nav/scripts/topo_sim_algorithms.py` 或 `planner_trace_cli` 对外链路时，优先更新 [topo_sim_viewer/README.md](topo_sim_viewer/README.md)。
- 改能力描述、接入范围、在线化说法或运行时边界表述时，必须同步更新 [boundaries/README.md](boundaries/README.md)。

## 6. 当前准确定位

截至当前代码状态，这个仓库里的前端更准确的定义是：

- `merlin-bt-visualizer`: 读取 `rc26_decision` 行为树 XML 的本地可视化/演示工具，加上一套注册表驱动、默认全中文的行为树编辑工作台。
- `src/rc26_topo_nav/sim_viewer`: 读取 surface graph、仿真 world 和 planner trace 的本地三维路线观察工具（基于 Babylon.js），不是导航运行时权威后端。
