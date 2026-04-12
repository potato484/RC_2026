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
   当前桌面端已经移除画布左侧常驻节点库，ReactFlow 只需要给顶部工具栏预留安全区；节点解释改由工具栏触发的覆盖式知识库承接，打开后会完整盖住当前编辑画布区域，避免底层编辑内容继续透出；新增节点改由独立插入菜单承接。
9. `src/components/EditorRightPanel.tsx`
   通过递归查找当前选中节点，并提供定义内数值参数编辑、同支线插入、显式新增支线、删除非根节点，以及“当前树”的中文结构预览和 XML 预览；右侧 XML 预览会保留完整 `<root>` 包裹和 `include`，但只展示当前 `activeTree`，与结构预览保持同一作用域。未注册但原始存在的属性只读展示并在导出时保留。
10. `src/utils/editorSerializer.ts:7-103`
    最终把 `EditorDocument` 重新序列化成 XML 字符串，并做基础格式化。
11. `merlin-bt-visualizer/vite.config.ts:1-70`
    开发态额外提供本地保存 API，把当前区域 XML 写回 `src/rc26_decision/behavior_trees/*.xml`，并让前端内存中的区域源同步更新。

## 2. 编辑链关键文件与行段导读

### 2.1 编辑态 store

- `src/store/useEditorStore.ts:96-129`
  - 按区域初始化或恢复编辑草稿。
- `src/store/useEditorStore.ts`
  - 修改已声明参数、同支线插入、显式新增支线与节点包裹，并把变化同步回当前区域草稿。
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
- `src/utils/btCanvasTheme.ts`
  - 查看态与编辑态共用的节点尺寸、边样式和 dagre 布局参数。
- `src/utils/editorTreeView.ts:19-32`
  - 编辑态树列表的层级派生和中文树名转换。
- `src/utils/editorTreeView.ts:35-48`
  - 当前树的中文结构预览。
- `src/utils/editorInsertCatalog.ts`
  - 统一生成“插入节点”目录和“节点知识库”目录；知识库当前按业务域分成梅林区模块、导航模块、视觉模块、对抗区模块、武馆区模块、官方节点、当前文档子树，并在当前类目内做搜索。
- `src/utils/btKnowledgeBaseDocs.ts`
  - 为节点知识库补充面向纯小白的教程式说明，包括“什么时候用 / 应该放在哪里 / 运行结果怎么理解 / 新手避坑 / 一个典型用法”，并给端口补充“新手提示 / 示例填法”。
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
  - 2026-04-04 起，桌面端不再常驻左侧节点库，改由工具栏触发“节点知识库”和“插入节点”；ReactFlow 只给顶部工具栏预留安全区。
- `src/components/EditorInsertEdge.tsx`
  - 把编辑态连线升级成可点击的自定义 edge，中点可直接打开插入菜单。
- `src/components/EditorKnowledgeBase.tsx`
  - 把原先占画布宽度的节点库改成只读知识库，打开后会覆盖当前编辑画布区域；知识库先按业务类目分栏，再在当前类目里搜索节点说明、子节点约束、声明端口和检索关键词。
  - 右侧详情当前已经升级成教程式说明页，不再只有一句短描述，而是固定展示：一句话认识、什么时候用、应该放在哪里、运行结果怎么理解、参数怎么填、新手避坑和一个典型用法。
  - 节点列表列和右侧详情列现在各自承担内部滚动，避免列表被裁切后无法上下滑动。
- `src/components/EditorInsertMenu.tsx`
  - 统一承接“常用机器人模块 / 动作 / 条件 / 子树”插入目录，支持边中点/快捷键的固定前后插，也支持工具栏场景下显式选择前插或后插。
  - 2026-04-05 起，插入菜单和右侧检查器选择器已经统一到同一套分层式弹层壳层：头部改成标题 + 短提示 + 独立搜索区，外层边框和底色也从原来的轻玻璃白卡收口成更实的浅色面板，避免标题区继续透出底层画布。
- `src/components/EditorNode.tsx:57-82`
  - 编辑画布节点现在复用查看态风格的节点壳层，只在选中或悬停时露出前插 / 后插控件。
- `src/components/EditorRightPanel.tsx:23-51`
  - 在编辑文档里递归查找当前选中节点。
- `src/components/EditorRightPanel.tsx`
  - 定义内数值参数编辑、显式新增支线、同支线插入、删除节点的交互入口。
  - 2026-04-05 起，右侧检查器里的结构选择入口不再使用原生下拉框；“同支线插入”的插入位置 / 控制包装 / 节点模板、“新增支线”的节点模板，以及“包裹当前节点”的包裹模板，统一改成覆盖式选择弹层。弹层只负责回填当前选择，不会在点选时直接改树结构，真正执行仍然依赖原有按钮。
- `src/components/EditorRightPanel.tsx:95-122`
  - 当前树的中文结构预览，以及保留完整 `<root>` 包裹的当前树 XML 预览。

## 3. 维护提示

- 修改 `editorParser`、`editorProjection`、`editorSerializer` 时，必须把它们视为一组 round-trip 能力一起维护。
- 修改 `btCanvasTheme`、`CustomNode`、`EditorNode` 或 edge 主题时，要同时检查查看态和编辑态的视觉是否仍然同源，不要再让两条链的节点尺寸、边样式和折叠空间处理重新漂移。
- 如果是改中文显示或树名规则，优先改 `btDisplay.ts` 和 `editorTreeView.ts`，不要在 `EditorNode` 或 `Sidebar` 组件里再复制一套解释逻辑。
- 右侧参数区现在只允许编辑注册表里已声明的参数；如果后端动作新增了真实可配置字段，应先补 `btNodeRegistry.ts`，不要在浏览器里临时开“附加属性”口子。
- 当前节点知识库的小白说明真源由两部分组成：
  - `btNodeRegistry.ts` 继续维护节点短摘要、端口定义、默认值和检索关键词。
  - `btKnowledgeBaseDocs.ts` 负责维护教程式说明、端口新手提示和示例填法。
- 编辑模式现在按区域保留浏览器内存草稿；切区、切回或暂时退出编辑时应能恢复该区域草稿，但刷新页面后仍会回到 XML 真源。
- “保存到源文件”当前只在 `npm run dev` 下可用，因为它依赖 Vite 的本地适配层；`build/preview` 仍然只保证导出源文件，不保证直接写回工作区文件。
- 右侧“源文件预览”当前只用于帮助对照当前树结构，因此它和“结构预览”一样只聚焦当前 `activeTree`；下载 XML 和开发态写回仍然处理整份 phase XML，不要把右侧预览误认成完整导出结果。
- 当前 `docs/test/merlin_bt_visualizer/run-e2e-local.sh` 会把开发态写回目标重定向到临时目录，因此浏览器 E2E 可以验证这条链路而不污染仓库真源。
- 新增节点类型、属性能力或树结构操作时，优先先把编辑语义模型补完整，再考虑画布怎么展示。
- 只做到“看起来能显示”不算完成，必须继续验证导出后的 XML 语义仍然可逆。

## 4. 2026-04-03 实现补充

这次编辑模式已经从“基础属性改写器”升级成“注册表驱动、边界收紧的行为树编辑器”，当前真实实现如下：

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
  - 统一处理端口绑定解析、节点来源识别、节点类别识别，以及“从定义创建新节点”。
- `merlin-bt-visualizer/src/store/useEditorStore.ts`
  - 当前已经支持：
    - 已声明参数更新
    - 沿当前支线的前插 / 后插
    - 控制节点上的显式新增支线
    - 用装饰器或复合节点包裹当前节点
    - 区域级草稿缓存与 XML round-trip 导出
  - 2026-04-03 额外修复了 `wrap` 包裹操作在非根节点上丢失结构替换的问题。
- `merlin-bt-visualizer/src/components/EditorKnowledgeBase.tsx`
  - 节点知识库现在改为覆盖当前编辑画布区域的只读工作区，打开后不再透出底层旧内容。
  - 知识库当前先按业务域分成“梅林区模块 / 导航模块 / 视觉模块 / 对抗区模块 / 武馆区模块 / 官方节点 / 当前文档子树”，再展示类目内的具体节点说明。
  - 搜索只作用在当前类目内，并同时命中中文名、英文 tag、关键词、端口名、端口新手提示和教程式说明正文；知识库本身不再直接插入节点。
  - 右侧详情当前固定展开：一句话认识、什么时候用、应该放在哪里、运行结果怎么理解、参数怎么填、新手避坑、一个典型用法、检索关键词。
- `merlin-bt-visualizer/src/components/EditorNode.tsx`
  - 节点卡片新增快速插入槽：
    - 前插
    - 后插
- `merlin-bt-visualizer/src/components/EditorContextMenu.tsx`
  - 新增右键菜单，当前支持折叠/展开、包裹为结果反转、包裹为重试直到成功、删除节点。
- `merlin-bt-visualizer/src/components/EditorRightPanel.tsx`
  - 右侧面板现在同时承担：
    - 中文节点信息展示
    - 定义内数值参数编辑
    - 未注册原始属性的只读展示
    - 同支线插入、新增支线与包裹
    - 当前树结构预览
    - 当前树 XML 源文件预览（保留 `<root>` 与 `include`，但不再展示兄弟树）
- `merlin-bt-visualizer/src/i18n/useLocaleStore.ts`
  - 当前默认语言固定为 `zh-CN`。
  - `en-US` 字典只保留为后续调试与扩展准备，当前产品界面默认仍强制中文。
- `merlin-bt-visualizer/src/i18n/btTerms.ts` + `merlin-bt-visualizer/src/utils/btDisplay.ts`
  - 当前已经统一承接树名、实例名、属性键、枚举值、黑板键的中文映射与摘要生成。
  - 完整映射表见 [bt_terms_mapping.md](/home/potato/RC_2026/docs/frontend/editor_mode/bt_terms_mapping.md)。

### 4.1 当前编辑交互的真实边界

- 当前编辑器仍然是本地工程工具，不拥有机器人运行时权威。
- 当前“保存到源文件”依旧只在开发态通过 Vite 本地适配层生效。
- 当前节点知识库以静态注册表、教程式说明文本和业务类目映射为真源；如果 `rc26_decision` 新增了 BT 节点注册或新的中文本地化条目，必须同步补 `btNodeRegistry.ts`、`btKnowledgeBaseDocs.ts`、`btTerms.ts`，并确认 `editorInsertCatalog.ts` 里的知识库类目映射仍然准确。
- 当前浏览器不会凭空新增任意 XML 属性；未在注册表声明的字段最多只读展示，真实可编辑参数必须回到节点定义里补齐。

### 4.3 2026-04-04 编辑体验重构补充

- 编辑态连线现在支持“点边即插入”：
  - 点击任意可见连线的中点，会立刻弹出插入菜单。
  - 插入菜单和键盘 `A / Shift+A` 弹层现在统一提供：常用机器人模块、动作节点、条件节点、子树节点。
  - 边插入的固定语义是“沿当前支线前插到目标节点之前”。
  - 用户必须先显式选择包装控制节点，再选择要插入的动作、条件或子树。
  - 当前允许作为包装控制的节点只有：`Sequence / SequenceWithMemory / ReactiveSequence / Fallback / ReactiveFallback / RoundRobin / Parallel / ParallelAll`。
  - 不管目标节点原来挂在什么父节点下，同支线插入都会新建一层可见包装节点，把“原节点 + 新节点”包起来；编辑器不再直接往现有顺序父节点里 `splice` 新兄弟，也不再偷偷固定补一层 `Sequence`。
- 编辑态现在把“同支线插入”和“新增支线”拆成两套显式入口：
  - 节点前后插槽、工具栏插入菜单、键盘 `A / Shift+A`、连线中点菜单都统一走“同支线插入”，但每次都必须先明确控制包装关系。
  - 新增支线只允许在右侧检查器里对控制节点显式加 child，不再通过节点卡片侧边快捷槽隐式触发。
- 编辑态工具栏现在显式拆成两种入口：
  - `节点知识库` 只负责查节点功能解释、子节点约束和端口说明，不再承担拖拽或一键插入。
  - 知识库打开后会以覆盖式工作区盖住当前编辑画布，避免底层旧内容继续干扰阅读。
  - 知识库当前先选业务类目，再浏览该类里的具体节点；搜索范围只限当前类目。
  - `插入节点` 只在已选中目标节点时可用，并且必须先明确是前插还是后插，避免工具栏入口默认偷偷后插。
- 右侧检查器与预览补充：
  - `结构预览` 和 `源文件预览` 已改成真正保留换行与缩进的预格式化面板，不再把多行文本挤成一段。
  - `源文件预览` 现在和 `结构预览` 一样只聚焦当前 `activeTree`；为保证可复制和可阅读，预览内容仍保留完整 `<root>` 包裹和 `include`。
  - 节点信息面板不再提供“附加属性 / 添加附加属性”入口。
- 编辑态现在补齐了按区域草稿历史：
  - `Ctrl/Cmd+Z` 撤销
  - `Shift+Ctrl/Cmd+Z` 或 `Ctrl/Cmd+Y` 重做
  - 结构插入、边插入、属性修改、删除都会进入历史栈。
- 编辑态节点外观、边样式和布局参数已经和查看态统一到同一套画布主题：
  - 控制节点、装饰节点、叶子节点的尺寸与间距不再另起一套。
  - 折叠后只按可见节点重新布局，不再保留多余空洞。
  - 英文 `tagName` 保留为辅助标识，中文名称和中文说明始终优先展示。
- 编辑页壳层已做响应式收口：
  - 桌面端继续保留左侧树列表、中央画布、右侧检查器。
  - 窄屏下改为“画布优先 + 决策树抽屉 + 检查器抽屉 + 覆盖式知识库工作区 + 插入菜单底部 sheet”，不再把三栏强行挤在一行里。
- `src/generated/btNodeRegistry.ts`
  - 所有官方节点和机器人模块节点的中文说明都已改成更口语化、1-2 行内的说明文本，后续如果再补节点，继续沿用这套表达风格。
- `src/utils/btKnowledgeBaseDocs.ts`
  - 当前为所有知识库节点补充了面向纯小白的教程式说明。
  - 动态“当前文档子树”项也会生成同样结构的说明，并直接给出子树 `ID` 的示例填法，降低新人把中文树名误填进 `ID` 的概率。

### 4.4 2026-04-04 检查器参数收口补充

- `merlin-bt-visualizer/src/components/EditorRightPanel.tsx`
  - 右侧检查器现在只显示“纯数字 + 纯固定值输入”的已声明参数；字符串、布尔、输出端口、黑板绑定端口和 `both` 端口都不再出现在“已定义参数”区。
  - 已显示的参数当前只保留数值输入框，不再在这个区块里暴露“固定值 / 黑板 / 根黑板”切换。
  - 已声明但被隐藏的端口不会转移到“保留的原始属性”区；它们仍然参与解析与导出，只是不再作为终端用户常规编辑入口。
  - 动作节点的 `error_code` 不再作为常规“已定义参数”暴露；如果旧 XML 里仍然手工写了 `error_code`，检查器会把它放到“保留的原始属性”里只读展示并继续 round-trip 保留。
- `src/components/EditorVisualizer.tsx`
  - 编辑态全局快捷键现在会避开输入框、下拉框和可编辑区域；在参数输入过程中按 `Backspace` 不会再误删整个节点。
  - 画布级删除快捷键收口为 `Delete`；`Backspace` 回到文本编辑自身的原生语义。
- 当前项目里如果需要排查最近一次机构动作失败，更推荐直接查看 `last_action_error_code` 黑板键。
  - 该键由决策节点订阅机制状态后主动写入黑板和发布链路，不依赖每个动作节点在 XML 里单独 remap `error_code`。

### 4.5 2026-04-05 检查器结构选择弹层化

- `merlin-bt-visualizer/src/components/EditorOptionPicker.tsx`
  - 新增通用覆盖式选择器，桌面端以居中浮层呈现，窄屏以底部 sheet 呈现，可复用于右侧检查器里的结构选择场景。
- `merlin-bt-visualizer/src/components/EditorOverlayPanel.tsx`
  - 新增选择弹层共用壳层，统一承接标题、短提示、关闭按钮、搜索区，以及新的边框/底色/阴影样式。
- `merlin-bt-visualizer/src/components/EditorRightPanel.tsx`
  - “同支线插入”的插入位置 / 控制包装 / 节点模板不再走原生 `<select>`，改成点击后弹出具体候选项。
  - “新增支线”的节点模板也改成弹层目录，方便从完整注册表里挑选起始节点。
  - “包裹当前节点”里的包裹模板同样改成弹层选择，但仍然要手动点击“执行包裹”才会真正改树，避免点选时误触发结构变更。
- 本次视觉收口补充：
  - 所有选择类弹层现在都改成更实的浅色面板，不再让头部区域继续和底层画布叠成一片。
  - 说明文案改成短提示条，搜索框也从标题区里独立出来，方便编辑者更快扫到“先选什么、再做什么”。
  - “同支线插入”“新增支线”和工具栏 `+插入节点` 里的节点模板卡片现在统一成两列网格，和“控制包装”保持同一套扫描节奏，不再混用单列列表。
  - 工具栏 `+插入节点` 与移动端底部 sheet 重新收口为“头部固定、模板区单独纵向滚动”的结构；节点很多时可以继续向下滚到末尾，不会再整块卡死。
- 当前真实交互边界：
  - 选择器弹层只负责回填当前值，不直接执行 `insertAlongBranch / insertBranch / wrapNode`。
  - 真正改动文档结构的入口仍然是右侧原有的“执行同支线插入 / 新增分支 / 执行包裹”按钮。

### 4.2 本次验证结果

- `npm --prefix merlin-bt-visualizer run test`
  - 通过，已覆盖编辑草稿缓存、节点插入/包裹、XML round-trip、中文显示与查看态 XML 刷新。
- `npm --prefix merlin-bt-visualizer run build`
  - 通过，当前编辑模式相关 TypeScript 与生产构建已稳定。
