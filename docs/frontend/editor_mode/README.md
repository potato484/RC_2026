# 编辑模式

## 1. 编辑模式的数据流

编辑模式不是把查看态图结构直接改一改，而是单独维护一套可逆编辑语义。

1. `src/components/Header.tsx:13-24`
   在从查看模式切进编辑模式时，按当前区域把原始 XML 载入 `useEditorStore`。
2. `src/store/useEditorStore.ts:50-66`
   通过 `xmlToEditorDocument()` 解析 XML，生成 `EditorDocument`，并把第一棵 `BehaviorTree` 设为当前树。
3. `src/utils/editorParser.ts:7-49`
   先读取 `<root>` 级属性和 `<include>`，再解析每棵 `BehaviorTree`。
4. `src/utils/editorParser.ts:52-95`
   把每个 XML 节点保留为 `EditorNode`，包括原始标签名、原始属性和真实子节点层级。
5. `src/store/useEditorStore.ts:193-208`
   调用 `projectTreeToFlow()` 把编辑语义树投影成 React Flow 所需的 `flowNodes/flowEdges`。
6. `src/utils/editorProjection.ts:20-58`
   先做树遍历和可见性过滤，`59-105` 再用 `dagre` 布局。
7. `src/components/EditorVisualizer.tsx:12-92`
   负责编辑画布、节点选中状态同步和 XML 下载按钮。
8. `src/components/EditorRightPanel.tsx:22-94`
   通过递归查找当前选中节点，并提供属性增删改、添加子节点、删除非根节点和实时 XML 预览。
9. `src/store/useEditorStore.ts:89-185`
   所有编辑操作都先改 `EditorDocument`，然后统一触发 `_updateFlow()`，避免 UI 组件自行拼图。
10. `src/utils/editorSerializer.ts:7-103`
    最终把 `EditorDocument` 重新序列化成 XML 字符串，并做基础格式化。

## 2. 编辑链关键文件与行段导读

### 2.1 编辑态 store

- `src/store/useEditorStore.ts:50-66`
  - XML 载入与编辑文档初始化。
- `src/store/useEditorStore.ts:89-147`
  - 修改属性、添加子节点。
- `src/store/useEditorStore.ts:149-185`
  - 删除非根节点。
- `src/store/useEditorStore.ts:193-208`
  - 统一把编辑语义投影成画布节点与边。

### 2.2 解析、投影与序列化

- `src/utils/editorParser.ts:7-49`
  - `<root>` 级解析。
- `src/utils/editorParser.ts:52-95`
  - `BehaviorTree` 和节点递归解析。
- `src/utils/editorProjection.ts:20-58`
  - 遍历树并根据折叠态决定可见节点。
- `src/utils/editorProjection.ts:59-105`
  - 编辑器画布布局。
- `src/utils/editorSerializer.ts:7-42`
  - 文档重新序列化回 XML。
- `src/utils/editorSerializer.ts:62-103`
  - 基础 XML 格式化器。

### 2.3 编辑态组件

- `src/components/Header.tsx:13-24`
  - 查看/编辑模式切换时，决定加载哪份原始 XML 进入编辑 store。
- `src/components/Sidebar.tsx:108-129`
  - 编辑模式树列表只在当前 `EditorDocument` 内切树。
- `src/components/EditorVisualizer.tsx:12-92`
  - 编辑画布、选中状态同步和 XML 下载。
- `src/components/EditorRightPanel.tsx:22-50`
  - 在编辑文档里递归查找当前选中节点。
- `src/components/EditorRightPanel.tsx:52-94`
  - 属性增删改、添加子节点、删除节点的交互入口。
- `src/components/EditorRightPanel.tsx:115-118`
  - 实时 XML 预览。

## 3. 维护提示

- 修改 `editorParser`、`editorProjection`、`editorSerializer` 时，必须把它们视为一组 round-trip 能力一起维护。
- 新增节点类型、属性能力或树结构操作时，优先先把编辑语义模型补完整，再考虑画布怎么展示。
- 只做到“看起来能显示”不算完成，必须继续验证导出后的 XML 语义仍然可逆。
