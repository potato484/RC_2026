# 前端当前实现与边界

`docs/frontend/README.md` 现在作为前端文档入口页，用来快速回答两件事：

- 当前仓库里前端到底已经实现了什么。
- `merlin-bt-visualizer` 当前不是一个什么系统。

更细的实现拆解已经按模块拆到独立文档，避免单文件继续膨胀后难以维护。

## 1. 当前唯一成型的前端工程

当前仓库里唯一已经成型的自研 Web 前端是 `merlin-bt-visualizer`。它不是多页面站点，也不是在线驾驶舱，而是一个围绕 `rc26_decision` 行为树 XML 展开的单页本地工作台。

- 技术栈：`Vite + React 18 + TypeScript + Tailwind CSS + Zustand + @xyflow/react + dagre + framer-motion + lucide-react`
- 工程入口：`merlin-bt-visualizer/src/main.tsx` -> `merlin-bt-visualizer/src/App.tsx`
- 启动脚本：`npm run dev`、`npm run build`、`npm run preview`
- 本地文件读取边界：`merlin-bt-visualizer/vite.config.ts:12-16` 通过 `server.fs.allow: ['..']` 允许开发态读取前端目录上一级的 XML
- 原始输入源：`src/rc26_decision/behavior_trees/mf_tree.xml`、`mc_tree.xml`、`combat_tree.xml`

## 2. 当前能力总览

当前前端已经成型的能力可以归成三块：

- 查看模式：读取三份行为树 XML，解析成展示模型，完成分区切换、树切换、节点详情、执行日志和黑板可视化。
- 编辑模式：把原始 XML 解析成可逆编辑语义，支持基础属性修改、添加子节点、删除非根节点，并导出 XML。
- 本地模拟执行：在浏览器里用一套确定性演示逻辑跑单次行为树执行，驱动节点状态、时间线和黑板更新。

当前仍然没有的能力也必须一开始就看清：

- 没有真实 ROS2 / WebSocket / HTTP API 接入。
- 没有浏览器直接写回仓库文件的持久化链路。
- 没有把前端变成机器人运行时控制面的在线化适配层。

## 3. 模块文档索引

后续维护前端文档时，优先从这里进入，再按改动范围进入对应模块文档：

- [`overview`](overview/README.md): 前端总览、工程骨架、入口与共享约束，适合先建立全局上下文。`(file: overview/README.md)`
- [`viewer_mode`](viewer_mode/README.md): 查看模式的数据流、查看链关键文件和组件职责。`(file: viewer_mode/README.md)`
- [`editor_mode`](editor_mode/README.md): 编辑模式的 round-trip 语义链、编辑链关键文件和交互入口。`(file: editor_mode/README.md)`
- [`local_simulator`](local_simulator/README.md): `App.tsx` 里的本地模拟执行器实现、运行步骤与误读风险。`(file: local_simulator/README.md)`
- [`boundaries`](boundaries/README.md): 当前前端的明确边界、未实现能力和准确定位。`(file: boundaries/README.md)`

## 4. 推荐阅读顺序

如果是第一次接触当前前端，建议按下面顺序阅读：

1. 先看本文，确认它是本地工程工具而不是在线系统。
2. 再看 [overview/README.md](overview/README.md)，建立入口、输入源和模块分层认知。
3. 如果关注只读可视化链路，看 [viewer_mode/README.md](viewer_mode/README.md)。
4. 如果关注 XML 编辑和导出链路，看 [editor_mode/README.md](editor_mode/README.md)。
5. 如果关注“播放”“执行中节点”“时间线”“黑板”这类行为，看 [local_simulator/README.md](local_simulator/README.md)。
6. 如果要判断能力边界、对外描述或需求是否越界，最后回到 [boundaries/README.md](boundaries/README.md) 对照。

## 5. 按改动位置同步文档

为了让拆分后的文档真正便于维护，后续改动可以按下面方式同步：

- 改 `src/main.tsx`、`src/App.tsx` 页面装配、输入源或总体结构时，优先更新 [overview/README.md](overview/README.md)。
- 改 `useStore`、`btParser`、`TreeVisualizer`、`RightPanel`、查看态 `Sidebar` 时，优先更新 [viewer_mode/README.md](viewer_mode/README.md)。
- 改 `useEditorStore`、`editorParser`、`editorProjection`、`editorSerializer`、`EditorVisualizer`、`EditorRightPanel` 时，优先更新 [editor_mode/README.md](editor_mode/README.md)。
- 改 `App.tsx` 里的本地执行逻辑、时间线、黑板、模拟/实机提示时，优先更新 [local_simulator/README.md](local_simulator/README.md)。
- 改能力描述、接入范围、在线化说法或运行时边界表述时，必须同步更新 [boundaries/README.md](boundaries/README.md)。

## 6. 当前准确定位

截至当前代码状态，这个前端更准确的定义是：

一个读取 `rc26_decision` 行为树 XML 的本地可视化/演示工具，加上一套浏览器内基础属性编辑与 XML 导出的初步编辑器。
