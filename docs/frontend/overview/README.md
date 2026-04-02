# 前端总览与工程骨架

## 1. 工程定位

当前仓库里已经有两套成型的自研 Web 前端，但这份总览主要记录 `merlin-bt-visualizer` 这条行为树工具链。

- `merlin-bt-visualizer` 的核心任务不是联机控车，而是围绕 `rc26_decision` 行为树 XML 做本地查看、基础编辑和演示。
- `src/rc26_topo_nav/sim_viewer` 是另一条独立的 3D topo 仿真 viewer 链，详见 [topo_sim_viewer/README.md](/home/potato/RC_2026/docs/frontend/topo_sim_viewer/README.md)。
- 这两套工具都不拥有机器人运行时控制权。

## 2. 入口、输入源与开发边界

- `merlin-bt-visualizer/src/main.tsx:1-10`
  - 只负责挂载 React 根节点，把全局样式和 `App` 接起来。
- `merlin-bt-visualizer/src/App.tsx:10-225`
  - 是真正的页面编排器，决定当前是查看模式还是编辑模式，并在查看模式下驱动本地模拟执行器。
- `merlin-bt-visualizer/vite.config.ts:12-16`
  - 通过 `server.fs.allow: ['..']` 允许开发态读取前端目录上一级的 XML。
- 原始输入源
  - `src/rc26_decision/behavior_trees/mf_tree.xml`
  - `src/rc26_decision/behavior_trees/mc_tree.xml`
  - `src/rc26_decision/behavior_trees/combat_tree.xml`

这里的本地文件读取边界只是一种开发便利，不是通用部署协议。如果后续要脱离当前仓库目录结构部署，必须重新设计数据输入边界。

## 3. 工程骨架与职责拆分

前端不是一坨混在一起的 UI，而是已经拆成三条相对清晰的链路：

- 查看链
  - `src/store/useStore.ts:9-189` 是查看模式的状态真源，负责区域切换、当前子树切换、节点运行态、执行日志、黑板和模拟/实机模式标志。
  - `src/utils/btParser.ts:160-429` 是查看链专用解析器和布局器，会做中文标签翻译、装饰器压缩、子树展开等展示增强。
- 编辑链
  - `src/store/useEditorStore.ts:42-209` 是编辑模式的状态真源，负责 XML 载入、当前 `BehaviorTree` 选择、画布投影、属性修改、增删节点和 XML 导出。
  - `src/utils/editorParser.ts:7-128` + `src/utils/editorProjection.ts:13-106` + `src/utils/editorSerializer.ts:7-103` 组成 round-trip 三件套，保留的是可逆语义，不是只为展示服务的简化模型。
- 本地模拟执行链
  - `src/App.tsx:18-203` 实现浏览器内本地演示执行逻辑，驱动节点运行态、时间线和黑板。

## 4. 页面装配导读

### 4.1 页面壳层

- `merlin-bt-visualizer/src/App.tsx:10-16`
  - 读取查看模式状态，并维护异步执行期的 `isPlayingRef`。
- `merlin-bt-visualizer/src/App.tsx:211-222`
  - 页面骨架非常直接：`Header + Sidebar + 中央画布 + 右侧面板`，只是中间和右侧会在查看/编辑两套组件之间切换。

### 4.2 组件层的职责共识

- `src/components/*.tsx`
  - UI 组件层基本只消费 store 和 parser 输出，没有自己维护另一套 XML 真源。
- `merlin-bt-visualizer/src/components/Header.tsx:13-24`
  - 查看/编辑模式切换时，决定加载哪份原始 XML 进入编辑 store。
- `merlin-bt-visualizer/src/components/Sidebar.tsx:60-129`
  - 同一个组件承担两种模式下的树列表展示，但实际消费的是不同 store。
- `merlin-bt-visualizer/src/components/RightPanel.tsx:21-147`
  - 负责查看模式下的节点详情、执行日志和黑板。
- `merlin-bt-visualizer/src/components/EditorRightPanel.tsx:22-118`
  - 负责编辑模式下的属性操作入口和实时 XML 预览。

## 5. 维护时的共识

- 查看模型和编辑模型不能重新混成一套。
- 组件应继续只消费状态和工具输出，不要在组件内部悄悄复制 XML 解释逻辑。
- 本地模拟执行器只能当演示逻辑维护，不能把它写成真实运行时适配层。
- 文档描述必须和真实能力一致，不能把本地演示说成联机能力。
