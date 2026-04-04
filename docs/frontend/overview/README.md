# 前端总览与工程骨架

## 1. 工程定位

当前仓库里已经有两套成型的自研 Web 前端，但这份总览主要记录 `merlin-bt-visualizer` 这条行为树工具链。

- `merlin-bt-visualizer` 的核心任务不是联机控车，而是围绕 `rc26_decision` 行为树 XML 做本地查看、基础编辑和演示。
- `src/rc26_topo_nav/sim_viewer` 是另一条独立的 3D topo 仿真 viewer 链，详见 [topo_sim_viewer/README.md](/home/potato/RC_2026/docs/frontend/topo_sim_viewer/README.md)。
- 这两套工具都不拥有机器人运行时控制权。

## 2. 入口、输入源与开发边界

- `merlin-bt-visualizer/src/main.tsx:1-10`
  - 只负责挂载 React 根节点，把全局样式和 `App` 接起来。
- `merlin-bt-visualizer/src/App.tsx:12-230`
  - 是真正的页面编排器，决定当前是查看模式还是编辑模式，在查看模式下驱动本地模拟执行器，并在编辑模式下把当前区域同步到编辑 store。
- `merlin-bt-visualizer/src/utils/behaviorTreeSources.ts:1-10`
  - 集中管理三份区域 XML 的原始导入，避免查看链和编辑链各自维护一份区域来源。
- `merlin-bt-visualizer/vite.config.ts:1-70`
  - 开发态通过 `server.fs.allow: ['..']` 读取前端目录上一级的 XML，并额外提供一个本地保存 API，把当前区域 XML 写回工作区源文件。
- `merlin-bt-visualizer/playwright.config.ts` + `merlin-bt-visualizer/e2e/*.spec.ts`
  - 收口 `merlin-bt-visualizer` 的浏览器自动化验证，覆盖查看态/编辑态联动和开发态写回两条真实链路。
- `docs/test/merlin_bt_visualizer/*`
  - 收口 `merlin-bt-visualizer` 的本地 preflight、E2E 与 release 打包脚本，供根仓库 `package.json` 和 GitHub workflow 复用。
- 原始输入源
  - `src/rc26_decision/behavior_trees/mf_tree.xml`
  - `src/rc26_decision/behavior_trees/mc_tree.xml`
  - `src/rc26_decision/behavior_trees/combat_tree.xml`

这里的本地文件读取边界只是一种开发便利，不是通用部署协议。如果后续要脱离当前仓库目录结构部署，必须重新设计数据输入边界。

## 3. 工程骨架与职责拆分

前端不是一坨混在一起的 UI，而是已经拆成三条相对清晰的链路：

- 查看链
  - `src/store/useStore.ts:7-185` 是查看模式的状态真源，负责区域切换、当前子树切换、节点运行态、执行日志、黑板和模拟/实机模式标志。
  - `src/utils/btDisplay.ts:1-205` 提供查看链和编辑链共用的中文名称、参数和值翻译规则。
  - `src/utils/btParser.ts:5-168` 是查看链专用解析器和布局器，消费共享翻译规则，并继续负责装饰器压缩、子树展开等展示增强。
- 编辑链
  - `src/store/useEditorStore.ts` 是编辑模式的状态真源，负责按区域缓存草稿、当前 `BehaviorTree` 选择、画布投影、定义内参数修改、同支线插入、显式新增支线、节点包裹、复合节点切换和 XML 导出；当前同支线插入会强制要求用户选择包装控制节点，并始终生成一层显式包装结构。
  - `src/utils/editorParser.ts:7-121` + `src/utils/editorProjection.ts:16-111` + `src/utils/editorSerializer.ts:7-103` 组成 round-trip 三件套，保留的是可逆语义，不是只为展示服务的简化模型。
  - `src/generated/btNodeRegistry.ts` + `src/utils/btRegistry.ts` 是编辑链新增的节点定义真源，统一维护官方节点、机器人模块、已声明端口、默认属性和复合节点切换规则。
  - `src/utils/editorTreeView.ts:1-33` 从 `SubTree` 引用关系派生出和查看模式一致的树列表层级与中文树名。
  - `vite.config.ts` 的本地保存适配层只在 `npm run dev` 下可用；它把当前区域 XML 写回 `src/rc26_decision/behavior_trees/*.xml`，不等于前端拥有通用后端持久化能力。
- 本地模拟执行链
  - `src/App.tsx:21-217` 实现浏览器内本地演示执行逻辑，并在编辑态切区时驱动区域草稿恢复。

## 4. 页面装配导读

### 4.1 页面壳层

- `merlin-bt-visualizer/src/App.tsx:10-16`
  - 读取查看模式状态，并维护异步执行期的 `isPlayingRef`。
- `merlin-bt-visualizer/src/App.tsx:214-229`
  - 页面骨架非常直接：`Header + Sidebar + 中央画布 + 右侧面板`，只是中间和右侧会在查看/编辑两套组件之间切换。

### 4.2 组件层的职责共识

- `src/components/*.tsx`
  - UI 组件层基本只消费 store 和 parser 输出，没有自己维护另一套 XML 真源。
- `merlin-bt-visualizer/src/components/Header.tsx:8-14`
  - 只负责查看/编辑模式切换，不再自己决定加载哪份 XML。
- `merlin-bt-visualizer/src/components/Sidebar.tsx:13-162`
  - 同一个组件承担两种模式下的树列表展示，两边都按主树/子树层级组织列表；编辑模式会跟着当前区域草稿同步切换。
- `merlin-bt-visualizer/src/components/EditorVisualizer.tsx:12-129`
  - 负责编辑画布、节点库、右键菜单、导出源文件，以及开发态“保存到源文件”按钮。
- `merlin-bt-visualizer/src/components/RightPanel.tsx:21-147`
  - 负责查看模式下的节点详情、执行日志和黑板。
- `merlin-bt-visualizer/src/components/EditorRightPanel.tsx:23-243`
  - 负责编辑模式下的定义内参数编辑、同支线插入、显式新增支线、节点包裹、结构预览和实时 XML 预览，同时展示中文节点标题和原始标签名；当前同支线插入会把“控制包装选择”和“新增支线”明确拆开。

## 5. 维护时的共识

- 查看模型和编辑模型不能重新混成一套。
- 组件应继续只消费状态和工具输出，不要在组件内部悄悄复制 XML 解释逻辑。
- 如果要调整中文展示口径，优先改 `src/utils/btDisplay.ts`，不要让查看态和编辑态各自维护一份翻译表。
- 如果要新增或修改可编辑节点，优先更新 `src/generated/btNodeRegistry.ts` 与 `src/i18n/btTerms.ts`，再改具体组件交互。
- 编辑模式现在按区域保留内存草稿；切区或暂时退出编辑不应丢失当前会话内的改动，但刷新页面后仍会从 XML 真源重新加载。
- 如果要维护“保存到源文件”能力，必须继续把它限制在明确的本地适配层里，而不是让 React 页面直接假装拥有文件系统写权限。
- 如果要维护自动化测试链路，优先继续复用 `docs/test/merlin_bt_visualizer/*` 和 `merlin-bt-visualizer/e2e/*.spec.ts`，不要再把 merlin 的测试入口散落回仓库根目录。
- 本地模拟执行器只能当演示逻辑维护，不能把它写成真实运行时适配层。
- 文档描述必须和真实能力一致，不能把本地演示说成联机能力。

## 6. 2026-04-03 编辑链补充

- `merlin-bt-visualizer` 当前已从“基础 XML 改写”升级为“注册表驱动的中文行为树编辑工作台”。
- 当前编辑链新增了：
  - 节点库搜索与拖拽插入
  - 连线中点一键插入节点
  - 复合节点一键切换
  - 沿当前支线的前插、后插
  - 控制节点上的显式新增支线
  - 装饰器与复合节点包裹
  - 按区域缓存的撤销 / 重做历史
  - 定义内参数 / 黑板绑定可视化编辑
  - 原始但未注册属性的只读保留与导出
  - 统一中文映射与默认中文界面
- 2026-04-04 的最新结构编排约束补充：
  - 同支线插入只能选“动作 / 条件 / 子树”，不能把控制节点混作普通动作去插。
  - 用户必须显式选择 `Sequence / SequenceWithMemory / ReactiveSequence / Fallback / ReactiveFallback / RoundRobin / Parallel / ParallelAll` 之一作为包装控制。
  - 每次同支线插入都会新建一层可见包装节点，把“原节点 + 新节点”包起来，不再直接往现有顺序父节点 `splice` 一个新兄弟。
- 2026-04-04 起，查看态和编辑态的画布主题已经收口到共享的节点尺寸、边样式和布局参数；编辑态只在选中、悬停或抽屉里额外暴露编辑控件，不再维护另一套明显不同的节点外观。
- 页面壳层现在也开始按“画布优先”响应式收口：
  - 桌面端仍是侧栏 + 画布 + 面板
  - 窄屏下改为画布主视图，决策树和检查器通过抽屉进入，编辑节点库通过底部 sheet 进入
- 这部分的详细设计和完整映射表分别见：
  - [editor_mode/README.md](/home/potato/RC_2026/docs/frontend/editor_mode/README.md)
  - [editor_mode/bt_terms_mapping.md](/home/potato/RC_2026/docs/frontend/editor_mode/bt_terms_mapping.md)
