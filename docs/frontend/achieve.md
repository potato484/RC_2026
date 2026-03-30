# 前端当前实现与边界

## 1. 当前唯一成型的前端工程

当前仓库里唯一已经成型的自研 Web 前端是 `merlin-bt-visualizer`。它不是多页面站点，也不是在线驾驶舱，而是一个基于本地行为树 XML 的单页工作台。

- 技术栈：`Vite + React 18 + TypeScript + Tailwind CSS + Zustand + @xyflow/react + dagre + framer-motion + lucide-react`
- 工程入口：`merlin-bt-visualizer/src/main.tsx` -> `merlin-bt-visualizer/src/App.tsx`
- 启动脚本：`npm run dev`、`npm run build`、`npm run preview`
- 核心样式：`merlin-bt-visualizer/src/index.css`、`merlin-bt-visualizer/tailwind.config.ts`
- 开发态文件访问：`vite.config.ts` 通过 `server.fs.allow: ['..']` 允许读取前端目录上一级的行为树 XML

## 2. 当前已经实现的前端能力

### 2.1 查看模式

查看模式仍是当前前端的主能力，页面骨架固定为：

- 顶部 `Header`
- 左侧 `Sidebar`
- 中间 `TreeVisualizer`
- 右侧 `RightPanel`

这一模式下，前端不会从后端接口拉数据，而是直接通过 `?raw` 导入：

- `src/rc26_decision/behavior_trees/mf_tree.xml`
- `src/rc26_decision/behavior_trees/mc_tree.xml`
- `src/rc26_decision/behavior_trees/combat_tree.xml`

然后在 `merlin-bt-visualizer/src/store/useStore.ts` 中初始化为展示态数据。默认区域是 `梅林区`。

查看模式的关键实现包括：

- `Header`：显示当前区域、当前执行节点、播放/暂停、复位、模拟/实机模式切换、进入编辑按钮
- `Sidebar`：切换 `武馆区 / 梅林区 / 对抗区`，并按 `parentTreeId` 展示主树和子树层级
- `TreeVisualizer`：基于 React Flow 画行为树图
- `RightPanel`：显示节点详情、执行日志、黑板变量，并支持手动清空日志
- `btParser.ts`：把原始 XML 解析成展示模型，展开 `SubTree`，补中文标签和参数说明，并做装饰器/条件压缩
- `layoutNodes()`：使用 `dagre` 做从左到右的树布局

### 2.2 编辑模式

前端已经不只是只读查看器，还做了一套浏览器内的基础 XML 编辑链路。

- `useEditorStore.ts` 维护 `EditorDocument / EditorTree / EditorNode` 的编辑态状态
- `editorParser.ts` 负责把原始 XML 解析为可逆的编辑语义模型
- `editorProjection.ts` 把编辑语义树投影到 React Flow 画布
- `editorSerializer.ts` 把编辑后的语义树重新序列化回 XML
- `EditorVisualizer` 负责画布和 XML 下载
- `EditorRightPanel` 负责属性编辑、增删属性、添加子节点、删除非根节点、实时 XML 预览
- `EditorNode.tsx` 区分控制节点、装饰器、叶子节点、子树节点的可视化外观

当前编辑模式已经支持：

- 载入当前区域对应的原始 XML
- 在多棵 `BehaviorTree` 之间切换
- 修改节点属性
- 添加子节点
- 删除非根节点
- 导出 XML 文件

### 2.3 本地模拟执行

`App.tsx` 里实现了一套完全前端本地的、确定性的行为树单次执行器。

- 动作节点默认成功
- 部分条件节点按硬编码规则返回失败，例如 `CheckR1Blocking`、`CheckExitCondition`
- `sequence`、`selector`、`decorator`、`subtree` 都有对应执行分支
- 导航和脚本节点会向 `timeline` 与 `blackboard` 写入演示数据
- 单步延迟约 `800ms`
- 日志最多保留 `50` 条
- 黑板最多保留 `10` 条
- 整棵树执行结束后会自动停止播放并追加“执行完毕”日志

## 3. 当前前端的明确边界

当前前端的边界必须说清楚，否则很容易高估它的成熟度。

- 它不是完整的机器人在线驾驶舱，没有真正接入 ROS 2、WebSocket、rosbridge、Foxglove live stream 或 HTTP API
- 代码里当前没有 `fetch`、`axios`、`WebSocket`、`react-router` 这一类联机接入层
- “实机模式”目前只是提示“等待接收真实行为树状态”，没有真实状态流接入
- 它没有鉴权、没有路由、没有 SSR、没有多页面
- 它不能直接把 XML 保存回仓库文件，也没有后端持久化
- 它没有撤销/重做、schema 校验、冲突处理、多人协作、拖线重连后的完整编辑语义
- `src/rc26_bringup/foxglove/*.json` 只是 Foxglove 布局模板，不是这个前端工程本身

## 4. 当前准确定位

截至当前代码状态，这个前端更准确的定义是：

一个读取 `rc26_decision` 行为树 XML 的本地可视化/演示工具，加上一套浏览器内基础属性编辑与 XML 导出的初步编辑器。
