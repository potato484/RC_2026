# 查看模式

## 1. 查看模式的数据流

查看模式不是“打开一个 XML 然后直接画”，中间有明确的数据转换链。

1. `src/store/useStore.ts:9-14`
   在模块初始化阶段通过 `?raw` 直接导入三份区域 XML，并立刻调用 `parseBTXml()`。
2. `src/store/useStore.ts:44-62`
   建立默认区域 `梅林区`、默认主树和展示初始状态。
3. `src/utils/btParser.ts:160-370`
   把原始 BehaviorTree XML 转成 `ParsedArea -> ParsedTree -> BTNode[]/edges[]` 的展示模型。
4. `src/utils/btParser.ts:104-158`
   在转换后执行装饰器压缩，把部分 Decorator 节点折叠进真正执行节点，避免画布过碎。
5. `src/utils/btParser.ts:372-429`
   使用 `dagre` 做从左到右的树布局，再交给 React Flow。
6. `src/components/TreeVisualizer.tsx:13-87`
   只负责把 store 中的 `nodes/edges` 交给 React Flow，并把点击和折叠行为回写 store。
7. `src/components/Sidebar.tsx:60-107`
   按 `parentTreeId` 把主树和子树组织成分层列表，而不是简单平铺树名。
8. `src/components/RightPanel.tsx:14-150`
   消费 `activeNodeId`、`timeline` 和 `blackboard`，显示节点细节、执行日志和黑板变量。

## 2. 查看链关键文件与行段导读

### 2.1 查看态 store

- `src/store/useStore.ts:9-14`
  - 三个区域 XML 的原始导入和首轮解析。
- `src/store/useStore.ts:16-42`
  - 查看模式状态接口定义，能看出这个 store 关心的是展示态、时间线和黑板，而不是可逆 XML。
- `src/store/useStore.ts:74-100`
  - 区域切换和子树切换逻辑。
- `src/store/useStore.ts:104-116`
  - 时间线和黑板的展示裁剪策略。
- `src/store/useStore.ts:118-167`
  - 节点运行态和折叠态会同步回写到对应 `ParsedTree`，而不是只改当前画布副本。
- `src/store/useStore.ts:169-187`
  - 全树状态复位。

### 2.2 查看态解析与布局

- `src/utils/btParser.ts:5-102`
  - 中文动作名、参数名和子树名翻译表，前端的中文展示基本都从这里来。
- `src/utils/btParser.ts:104-158`
  - 装饰器压缩逻辑。
- `src/utils/btParser.ts:160-370`
  - BehaviorTree XML 到展示模型的主解析流程。
- `src/utils/btParser.ts:372-429`
  - `dagre` 横向布局。

### 2.3 查看态组件

- `src/components/TreeVisualizer.tsx:13-87`
  - 画布渲染和节点点击/折叠交互。
- `src/components/Sidebar.tsx:60-107`
  - 查看模式树列表按主树/子树层级展示。
- `src/components/RightPanel.tsx:21-79`
  - 节点详情。
- `src/components/RightPanel.tsx:82-116`
  - 执行日志。
- `src/components/RightPanel.tsx:119-147`
  - 黑板变量面板。

## 3. 维护提示

- 查看模式的数据结构允许为展示服务做增强，但不能拿来当 XML 导出的真源。
- 如果改了 `btParser` 的展示增强逻辑，要确认只是改变可视化解释，不会悄悄污染编辑链语义。
- 如果改了 `Sidebar` 或 `TreeVisualizer` 的交互，也要同步检查 `useStore` 中的树切换、折叠和活动节点逻辑是否仍然自洽。
