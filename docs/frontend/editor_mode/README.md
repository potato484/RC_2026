# 编辑模式

## 1. 编辑模式的数据流

编辑模式不是把查看态图结构直接改一改，而是单独维护一套可逆编辑语义。

1. `src/App.tsx:214-217`
   进入编辑模式或在编辑模式下切换区域时，按当前 `activePhase` 调用 `ensurePhaseLoaded()`，统一驱动编辑态和查看态的区域联动。
2. `src/store/useEditorStore.ts:96-129`
   首次进入某个区域时通过 `xmlToEditorDocument()` 解析 XML，生成该区域的编辑草稿；再次切回时直接恢复缓存草稿，而不是重载真源。
3. `src/utils/editorParser.ts:7-49`
   先读取 `<root>` 级属性和 `<include>`，再解析每棵 `BehaviorTree`。
4. `src/utils/editorParser.ts:52-95`
   把每个 XML 节点保留为 `EditorNode`，包括原始标签名、原始属性和真实子节点层级。
5. `src/store/useEditorStore.ts:131-267`
   所有编辑操作都先改当前区域草稿里的 `EditorDocument`，再同步更新当前区域缓存，避免切区后丢失未导出的改动。
6. `src/utils/editorProjection.ts:23-43`
   在投影节点时调用 `btDisplay` 生成 `displayLabel/displayDesc`，让编辑画布的节点中文显示和查看模式保持一致。
7. `src/utils/editorTreeView.ts:19-32`
   根据 `SubTree` 引用关系派生编辑态树列表的主树/子树层级和中文树名。
8. `src/components/EditorVisualizer.tsx:12-92`
   负责编辑画布、节点选中状态同步、源文件导出，以及开发态“保存到源文件”按钮。
9. `src/components/EditorRightPanel.tsx:95-243`
   通过递归查找当前选中节点，并提供属性增删改、添加子节点、删除非根节点和中文结构预览；默认优先展示中文属性名和值，只在显式展开时露出原始 XML 内容。
10. `src/utils/editorSerializer.ts:7-103`
    最终把 `EditorDocument` 重新序列化成 XML 字符串，并做基础格式化。
11. `merlin-bt-visualizer/vite.config.ts:1-70`
    开发态额外提供本地保存 API，把当前区域 XML 写回 `src/rc26_decision/behavior_trees/*.xml`，并让前端内存中的区域源同步更新。

## 2. 编辑链关键文件与行段导读

### 2.1 编辑态 store

- `src/store/useEditorStore.ts:96-129`
  - 按区域初始化或恢复编辑草稿。
- `src/store/useEditorStore.ts:131-225`
  - 修改属性、添加子节点，并把变化同步回当前区域草稿。
- `src/store/useEditorStore.ts:228-267`
  - 删除非根节点，并同步清理当前区域选中态。
- `src/store/useEditorStore.ts:276-290`
  - 统一把编辑语义投影成画布节点与边。

### 2.2 解析、投影与序列化

- `src/utils/editorParser.ts:7-49`
  - `<root>` 级解析。
- `src/utils/editorParser.ts:52-95`
  - `BehaviorTree` 和节点递归解析。
- `src/utils/btDisplay.ts:1-205`
  - 查看态和编辑态共用的中文名称、参数、属性名和值翻译逻辑。
- `src/utils/editorProjection.ts:23-58`
  - 遍历树、补充中文显示字段与属性摘要，并根据折叠态决定可见节点。
- `src/utils/editorProjection.ts:65-109`
  - 编辑器画布布局。
- `src/utils/editorTreeView.ts:19-32`
  - 编辑态树列表的层级派生和中文树名转换。
- `src/utils/editorTreeView.ts:35-48`
  - 当前树的中文结构预览。
- `src/utils/editorSerializer.ts:7-42`
  - 文档重新序列化回 XML。
- `src/utils/editorSerializer.ts:62-103`
  - 基础 XML 格式化器。
- `merlin-bt-visualizer/vite.config.ts:1-70`
  - 开发态本地保存适配层。

### 2.3 编辑态组件

- `src/components/Header.tsx:8-14`
  - 只负责模式切换；区域 XML 的装载和恢复交由 `App + useEditorStore`。
- `src/components/Sidebar.tsx:112-156`
  - 编辑模式树列表按主树/子树层级展示，并跟着当前区域草稿同步切换。
- `src/components/EditorVisualizer.tsx:12-92`
  - 编辑画布、选中状态同步、源文件导出和保存到源文件。
- `src/components/EditorNode.tsx:57-82`
  - 编辑画布节点主标题和摘要都优先显示中文解释，不再默认暴露原始标签名。
- `src/components/EditorRightPanel.tsx:23-51`
  - 在编辑文档里递归查找当前选中节点。
- `src/components/EditorRightPanel.tsx:53-93`
  - 属性增删改、添加子节点、删除节点的交互入口。
- `src/components/EditorRightPanel.tsx:95-122`
  - 中文结构预览。

## 3. 维护提示

- 修改 `editorParser`、`editorProjection`、`editorSerializer` 时，必须把它们视为一组 round-trip 能力一起维护。
- 如果是改中文显示或树名规则，优先改 `btDisplay.ts` 和 `editorTreeView.ts`，不要在 `EditorNode` 或 `Sidebar` 组件里再复制一套解释逻辑。
- 右侧属性区现在默认隐藏原始英文键值；如果要继续保持“编辑态默认不露英文”，新增属性展示时也应先补中文属性映射，而不是直接把原始键值露出来。
- 编辑模式现在按区域保留浏览器内存草稿；切区、切回或暂时退出编辑时应能恢复该区域草稿，但刷新页面后仍会回到 XML 真源。
- “保存到源文件”当前只在 `npm run dev` 下可用，因为它依赖 Vite 的本地适配层；`build/preview` 仍然只保证导出源文件，不保证直接写回工作区文件。
- 当前 `docs/test/merlin_bt_visualizer/run-e2e-local.sh` 会把开发态写回目标重定向到临时目录，因此浏览器 E2E 可以验证这条链路而不污染仓库真源。
- 新增节点类型、属性能力或树结构操作时，优先先把编辑语义模型补完整，再考虑画布怎么展示。
- 只做到“看起来能显示”不算完成，必须继续验证导出后的 XML 语义仍然可逆。

## 4. 2026-04-03 实现补充

这次编辑模式已经从“基础属性改写器”升级成“注册表驱动的完整行为树编辑器”，当前真实实现如下：

- `merlin-bt-visualizer/src/generated/btNodeRegistry.ts`
  - 统一维护官方节点和机器人模块节点的注册表。
  - 每个节点都显式给出 `tagName / 中文名 / 来源 / 类别 / 子节点策略 / 默认属性 / 端口定义 / 检索关键词`。
  - 当前覆盖范围包括：
    - 官方控制节点：`Sequence / SequenceWithMemory / SequenceStar / ReactiveSequence / Fallback / ReactiveFallback / Parallel / ParallelAll / IfThenElse / WhileDoElse / RoundRobin / Switch2-6`
    - 官方装饰节点：`Inverter / ForceSuccess / ForceFailure / Repeat / RetryUntilSuccessful / KeepRunningUntilFailure / Delay / Timeout`
    - 官方叶子与结构节点：`Script / ScriptCondition / AlwaysSuccess / AlwaysFailure / SubTree`
    - 当前项目机器人模块：武馆区、梅林区、导航、对抗区、视觉相关 Action / Condition
- `merlin-bt-visualizer/src/utils/btRegistry.ts`
  - 负责把原始 XML 节点补充为带定义元数据的 `EditorNode`。
  - 统一处理端口绑定解析、节点来源识别、节点类别识别、复合节点切换候选、以及“从定义创建新节点”。
- `merlin-bt-visualizer/src/store/useEditorStore.ts`
  - 当前已经支持：
    - 单属性更新
    - 在当前节点前后插入兄弟节点
    - 作为首个或末尾子节点插入
    - 用装饰器或复合节点包裹当前节点
    - 一键切换复合节点类型，并保留原有子节点和属性
    - 区域级草稿缓存与 XML round-trip 导出
  - 2026-04-03 额外修复了 `wrap` 包裹操作在非根节点上丢失结构替换的问题。
- `merlin-bt-visualizer/src/components/EditorPalette.tsx`
  - 新增节点库面板，按“机器人模块 / 官方节点”分栏展示。
  - 支持搜索、拖拽插入、以及直接插入到当前选中节点。
- `merlin-bt-visualizer/src/components/EditorNode.tsx`
  - 节点卡片新增快速插入槽：
    - 前插
    - 后插
    - 子插
  - 对支持切换的复合节点提供一键切换按钮。
- `merlin-bt-visualizer/src/components/EditorContextMenu.tsx`
  - 新增右键菜单，当前支持折叠/展开、切换复合节点类型、包裹为结果反转、包裹为重试直到成功、删除节点。
- `merlin-bt-visualizer/src/components/EditorRightPanel.tsx`
  - 右侧面板现在同时承担：
    - 中文节点信息展示
    - 端口/黑板绑定编辑
    - 附加属性编辑
    - 结构插入与包裹
    - 当前树结构预览
    - 当前 XML 源文件预览
- `merlin-bt-visualizer/src/i18n/useLocaleStore.ts`
  - 当前默认语言固定为 `zh-CN`。
  - `en-US` 字典只保留为后续调试与扩展准备，当前产品界面默认仍强制中文。
- `merlin-bt-visualizer/src/i18n/btTerms.ts` + `merlin-bt-visualizer/src/utils/btDisplay.ts`
  - 当前已经统一承接树名、实例名、属性键、枚举值、黑板键的中文映射与摘要生成。
  - 完整映射表见 [bt_terms_mapping.md](/home/potato/RC_2026/docs/frontend/editor_mode/bt_terms_mapping.md)。

### 4.1 当前编辑交互的真实边界

- 当前编辑器仍然是本地工程工具，不拥有机器人运行时权威。
- 当前“保存到源文件”依旧只在开发态通过 Vite 本地适配层生效。
- 当前节点库以静态注册表为真源；如果 `rc26_decision` 新增了 BT 节点注册或新的中文本地化条目，必须同步补 `btNodeRegistry.ts` 与 `btTerms.ts`。

### 4.2 本次验证结果

- `npm --prefix merlin-bt-visualizer run test`
  - 通过，已覆盖编辑草稿缓存、节点插入/包裹/切换、XML round-trip、中文显示与查看态 XML 刷新。
- `npm --prefix merlin-bt-visualizer run build`
  - 通过，当前编辑模式相关 TypeScript 与生产构建已稳定。
