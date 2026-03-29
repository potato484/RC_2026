**核心原则**
- 检索项目代码或实现方案时，优先使用 `ace-tool`。
- 如果 `ace-tool` / `grok` / 在当前会话不可用，明确说明不确定性后，使用本地代码与可用工具继续推进，不阻塞任务。
- **目标** 当前项目仅为R2这个自动机器人设计，R1只是手动机器人
**判断依据**
- 以项目代码和可获取的搜索结果作为主要判断依据，避免无依据猜测。
- 在调用编程语言的非内置库时，优先查阅官方文档或权威资料（`grok`  / `context7` 等）；若无法联网检索，先标注风险再编码。
- **Project Scope**: 优先处理当前仓库内容；
- **工作 Environment**: Linux Ubuntu 22.04。涉及 Python 命令统一使用 `python3`。
- **编译验证** 统一用 `MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 --packages-select <pkg...>` 进行编译验证；需要提速时优先小幅调高 `MAKEFLAGS`，不要直接提高 `--parallel-workers`
- **R2算力平台** 基于 Qualcomm® QCS8550 平台，采用领先的 4nm 工艺，CPU 算力达 300k DMIPS，并集成 Adreno 740 GPU（3000 GFLOPS）。系统提供高达 48 TOPS 的 INT8 AI 推理能力，支持 AidLux (Android 13 + Ubuntu 22.04) 深度融合环境。硬件配置 16GB LPDDR5x + 256GB UFS 4.0 顶级存储组合，具备卓越的 8K 视频编解码 性能，是高性能边缘计算的理想选择。
- **R2基本信息** 四驱麦克纳姆轮底盘，高精度陀螺仪放在底盘中心用来进行位姿融合下发（达妙陀螺仪在比赛时间内没有明显漂移，在可控范围内），参照rc26_merge_odom，rc26_telecontrol用来人为遥控测试R2机器人

**当前前端实现与骨架**
- 当前仓库里唯一成型的自研 Web 前端是 `merlin-bt-visualizer`；`src/rc26_bringup/foxglove/*.json` 属于 Foxglove 布局模板，不是自研前端页面工程。
- 技术栈仍是 `Vite + React 18 + TypeScript + Tailwind CSS + Zustand + @xyflow/react + dagre + framer-motion + lucide-react`，并已引入 `Vitest + JSDOM + Testing Library` 作为前端测试依赖；构建脚本仍在 `merlin-bt-visualizer/package.json`：`npm run dev` / `npm run build` / `npm run preview`，当前还没有单独的 `npm test` script；`vite.config.ts` 通过 `server.fs.allow: ['..']` 允许开发态读取前端目录上一级位置的行为树 XML。
- 工程入口仍是 `merlin-bt-visualizer/src/main.tsx` -> `merlin-bt-visualizer/src/App.tsx`。当前没有路由、没有多页面、没有 SSR、没有鉴权，但已不再只是单纯查看器，而是一个单页的“行为树查看 + 初步 XML 编辑”工作台。
- 页面骨架在查看模式下仍是 3 段式：顶部 `Header`，左侧 `Sidebar`，中间 `TreeVisualizer`，右侧 `RightPanel`。切到编辑模式后，中间切换为 `EditorVisualizer`，右侧切换为 `EditorRightPanel`。整体视觉仍采用玻璃拟态风格，核心样式在 `merlin-bt-visualizer/src/index.css` 与 `merlin-bt-visualizer/tailwind.config.ts`。
- `Header` 现在除了显示当前区域与当前执行节点、提供播放/暂停、复位、模拟/实机模式切换外，还提供“进入编辑 / 退出编辑”按钮；进入编辑模式时，会根据当前区域把对应的原始 XML 载入 `useEditorStore`。
- `Sidebar` 仍负责切换 `武馆区 / 梅林区 / 对抗区`。查看模式下，它按 `parentTreeId` 展示当前区域内的主树与子树层级；编辑模式下，它改为列出当前 `EditorDocument` 中的 `BehaviorTree` 列表并切换编辑目标树。
- 查看模式的数据依然不是从后端接口拉取，而是直接 `?raw` 导入 `src/rc26_decision/behavior_trees/mf_tree.xml`、`mc_tree.xml`、`combat_tree.xml`，在 `merlin-bt-visualizer/src/store/useStore.ts` 初始化时解析为展示用树结构；默认初始区域仍是 `梅林区`。
- 查看模式的解析和布局核心仍在 `merlin-bt-visualizer/src/utils/btParser.ts`：`parseBTXml()` 会解析多棵 `BehaviorTree`、展开 `SubTree`、补齐中文标签与参数说明，并在压缩阶段把装饰器/条件附着到实际执行节点；`layoutNodes()` 使用 `dagre` 计算从左到右的树形布局。这个模型本质上仍是“展示模型”，不是原样可逆的 XML 真源。
- 查看模式的全局状态由 `Zustand` 管理，核心字段包括 `appMode`、`isSimulating`、`isPlaying`、`activePhase`、`activeTreeId`、`trees`、`nodes`、`edges`、`activeNodeId`、`timeline`、`blackboard`；切区时会重置日志和黑板，切树时会切换当前展示树。
- 编辑模式新增了独立的语义模型与状态管理：`merlin-bt-visualizer/src/types/editor.ts` 定义 `EditorDocument / EditorTree / EditorNode`；`useEditorStore.ts` 维护 `document`、`activeTreeId`、`selectedNodeId`、`collapsedNodes`、`flowNodes`、`flowEdges`，并提供 `loadXml`、`updateNodeAttributes`、`addChildNode`、`deleteNode`、`exportXml` 等动作。
- 编辑模式的核心链路是：`editorParser.ts` 把原始 XML 解析成 `EditorDocument`，`editorProjection.ts` 把当前树投影为 React Flow 画布节点/连线，`editorSerializer.ts` 再把编辑后的语义树序列化回 XML。`EditorVisualizer` 负责画布展示与 XML 下载，`EditorRightPanel` 负责属性编辑、增删属性、添加子节点、删除非根节点，以及实时 XML 预览，`EditorNode.tsx` 负责区分控制节点、装饰器、叶子节点、子树节点的可视化外观。
- `TreeVisualizer` 仍支持点击节点查看详情、双击节点折叠/展开子树，运行中的边会高亮动画；节点本体由 `CustomNode.tsx` 渲染，不同类型使用不同图标、尺寸和状态样式。编辑模式使用的是另一套 `editorNode` 节点类型，不与查看模式复用同一个数据模型。
- 当前“模拟模式”仍完全在前端本地执行，入口逻辑在 `App.tsx`。但它已经不是旧描述里的确定性回放了：动作节点使用 `Math.random() > 0.1` 决定成功/失败，`sequence` / `selector` / `decorator` / `subtree` 都有对应分支，导航与脚本节点会向 `timeline` / `blackboard` 写入演示数据；单步延迟大约 `800ms`，一轮结束后的再次启动间隔约 `1500ms`，日志最多保留 `50` 条，黑板最多保留 `10` 条。
- 当前“实机模式”依然没有真正接入 ROS 2 / WebSocket / Foxglove / rosbridge / HTTP API。代码里仍然没有 `fetch`、`axios`、`WebSocket`、`react-router` 等联机接入层实现，切到实机模式时仍只是输出“等待接收真实行为树状态”的日志提示。
- 当前编辑器也还不是完整生产级行为树 IDE：没有保存回仓库文件的能力，没有后端持久化，没有拖线重连/节点拖拽编辑语义，没有 schema 校验、撤销重做、冲突处理或多人协作。
- 当前前端的准确定位应更新为：“一个读取 `rc26_decision` 行为树 XML 的本地可视化/演示工具，外加一套可在浏览器内做基础属性编辑并导出 XML 的初步编辑器，以及一套 Foxglove 布局模板。”不要把它误判为已经接通真实机器人状态流的完整在线驾驶舱。
