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
  - 原始输入源：`src/rc26_decision/behavior_trees/mf_tree.xml`、`mc_tree.xml`、`combat_tree.xml`
- `src/rc26_topo_nav/sim_viewer`
  - 围绕 `rc26_topo_nav` 的三维路径规划仿真与观测工具，负责渲染完整场地 mesh、路径回放和只读实时观察。
  - 技术栈：`Vite + React 18 + TypeScript + Zustand + @react-three/fiber + drei + Three.js`
  - 工程入口：`src/rc26_topo_nav/sim_viewer/src/main.tsx` -> `src/rc26_topo_nav/sim_viewer/src/App.tsx`
  - 启动脚本：`npm run dev`、`npm run build`
  - 数据入口：`src/rc26_topo_nav/scripts/topo_sim_server.py` 提供本地 HTTP / WebSocket adapter，消费 topo 图、Gazebo world、KFS 对齐配置和运行时只读 topic

## 2. 当前能力总览

当前前端已经成型的能力可以归成两条工具链：

- `merlin-bt-visualizer`
  - 查看模式：读取三份行为树 XML，解析成展示模型，完成分区切换、树切换、节点详情、执行日志和黑板可视化。
  - 编辑模式：把原始 XML 解析成可逆编辑语义，支持基础属性修改、添加子节点、删除非根节点，并导出 XML。
  - 本地模拟执行：在浏览器里用一套确定性演示逻辑跑单次行为树执行，驱动节点状态、时间线和黑板更新。
- `src/rc26_topo_nav/sim_viewer`
  - 完整三维场景：渲染 `robocon2026_v2_aligned.world` / `robocon2026.dae` 提取出的 mesh 面、材质色、光照和阴影。
  - 清晰路径层：用三维 tube / line 显示 A* / RRT / DWA 路径、关键点、open set、扩展树和候选轨迹。
  - 多视角交互：支持 `orbit / follow / first_person / top_ortho / side_ortho`。
  - 本地 adapter：离线模式下回放 A* 运行时 trace 与 RRT / DWA 仿真帧；实时模式下只读消费 `/topo_nav/route`、`/topo_nav/corridor`、`/xhu_nav/active_edge`、`/xhu_nav/semantic_gate`、`/mf_block_overlay`、`/xhu_nav/tracking_state`。

当前仍然没有的能力也必须一开始就看清：

- 没有浏览器直接成为机器人控制入口的能力。
- 没有浏览器直接写回仓库文件的持久化链路。
- 没有把前端变成机器人运行时权威后端；联机能力如果存在，也必须通过单独 adapter 且保持只读或受控边界。

## 3. 模块文档索引

后续维护前端文档时，优先从这里进入，再按改动范围进入对应模块文档：

- [`overview`](overview/README.md): 前端总览、工程骨架、入口与共享约束，适合先建立全局上下文。`(file: overview/README.md)`
- [`viewer_mode`](viewer_mode/README.md): 查看模式的数据流、查看链关键文件和组件职责。`(file: viewer_mode/README.md)`
- [`editor_mode`](editor_mode/README.md): 编辑模式的 round-trip 语义链、编辑链关键文件和交互入口。`(file: editor_mode/README.md)`
- [`local_simulator`](local_simulator/README.md): `App.tsx` 里的本地模拟执行器实现、运行步骤与误读风险。`(file: local_simulator/README.md)`
- [`topo_sim_viewer`](topo_sim_viewer/README.md): `rc26_topo_nav` 3D 仿真 viewer、FastAPI adapter、Three.js 场景与运行边界。`(file: topo_sim_viewer/README.md)`
- [`boundaries`](boundaries/README.md): 当前前端的明确边界、未实现能力和准确定位。`(file: boundaries/README.md)`

## 4. 推荐阅读顺序

如果是第一次接触当前前端，建议按下面顺序阅读：

1. 先看本文，确认它是本地工程工具而不是在线系统。
2. 再看 [overview/README.md](overview/README.md)，建立入口、输入源和模块分层认知。
3. 如果关注行为树查看链路，看 [viewer_mode/README.md](viewer_mode/README.md)。
4. 如果关注 XML 编辑和导出链路，看 [editor_mode/README.md](editor_mode/README.md)。
5. 如果关注行为树本地执行演示，看 [local_simulator/README.md](local_simulator/README.md)。
6. 如果关注 topo 导航三维仿真，看 [topo_sim_viewer/README.md](topo_sim_viewer/README.md)。
7. 如果要判断能力边界、对外描述或需求是否越界，最后回到 [boundaries/README.md](boundaries/README.md) 对照。

## 5. 按改动位置同步文档

为了让拆分后的文档真正便于维护，后续改动可以按下面方式同步：

- 改 `src/main.tsx`、`src/App.tsx` 页面装配、输入源或总体结构时，优先更新 [overview/README.md](overview/README.md)。
- 改 `useStore`、`btParser`、`TreeVisualizer`、`RightPanel`、查看态 `Sidebar` 时，优先更新 [viewer_mode/README.md](viewer_mode/README.md)。
- 改 `useEditorStore`、`editorParser`、`editorProjection`、`editorSerializer`、`EditorVisualizer`、`EditorRightPanel` 时，优先更新 [editor_mode/README.md](editor_mode/README.md)。
- 改 `App.tsx` 里的本地执行逻辑、时间线、黑板、模拟/实机提示时，优先更新 [local_simulator/README.md](local_simulator/README.md)。
- 改 `src/rc26_topo_nav/sim_viewer/*`、`src/rc26_topo_nav/scripts/topo_sim_server.py`、`src/rc26_topo_nav/scripts/topo_sim_algorithms.py` 或 `planner_trace_cli` 对外链路时，优先更新 [topo_sim_viewer/README.md](topo_sim_viewer/README.md)。
- 改能力描述、接入范围、在线化说法或运行时边界表述时，必须同步更新 [boundaries/README.md](boundaries/README.md)。

## 6. 当前准确定位

截至当前代码状态，这个仓库里的前端更准确的定义是：

- `merlin-bt-visualizer`: 读取 `rc26_decision` 行为树 XML 的本地可视化/演示工具，加上一套浏览器内基础属性编辑与 XML 导出的初步编辑器。
- `src/rc26_topo_nav/sim_viewer`: 读取 topo 图、仿真 world 和只读运行时状态的本地三维观测工具，不是导航运行时权威后端。
