# 行为树中文本地化模块

## 1. 模块整体设计方案

### 目标

给 `rc26_decision` 的行为树运行态增加一层独立的中文解释模块：

- 不改原有 BT 执行逻辑
- 原始英文名继续保留为内部 ID
- 前端默认显示中文
- 节点、黑板键、黑板值都能给出中文解释
- 支持通过配置文件热更新

### 总体链路

```text
BehaviorTree.CPP 运行态
  -> BtRuntimePublisher
  -> ChineseLocalizationModule
  -> r2/bt/localization
  -> web/r2_dashboard
  -> 中文节点名 / 中文黑板 / 解释面板 / 中文事件时间线
```

### 核心思路

1. `r2/bt/model`、`r2/bt/snapshot`、`r2/bt/blackboard`、`r2/bt/events` 继续保持原职责。
2. 新增 `r2/bt/localization` 话题，只发布本地化元数据。
3. 后端启动时一次加载 `bt_localization.yaml`。
4. 定时检查文件修改时间，变更后自动热更新并重新发布。
5. 前端收到 `r2/bt/localization` 后，只改显示层，不改英文内部字段。

### 映射优先级

节点中文解析采用以下优先级：

1. `full_paths`
2. `instances`
3. `custom_types`
4. `registrations`
5. 默认回退为原始英文名

子树中文名单独从 `subtrees` 目录解析。

黑板键中文解析来自 `blackboard_keys`。

黑板值中文解析来自对应键下的 `values` 映射。

### 后端统一 API

后端实际接口位于：

- `ChineseLocalizationModule::getChineseNode(...)`
- `ChineseLocalizationModule::getChineseBlackboardKey(...)`
- `ChineseLocalizationModule::buildMessage(...)`

前端对应 helper：

```ts
getChineseNode(node: BtNodeModel, localization: BtLocalization | null)
getChineseBlackboardKey(key: string, localization: BtLocalization | null)
getChineseBlackboardValue(key: string, rawValue: string, localization: BtLocalization | null)
```

## 2. 目录结构

```text
docs/
  bt_localization_module.md

src/rc26_interfaces/msg/
  BehaviorTreeLocalizedValue.msg
  BehaviorTreeLocalizationEntry.msg
  BehaviorTreeLocalizedNode.msg
  BehaviorTreeLocalization.msg

src/rc26_decision/include/rc26_decision/bt/
  chinese_localization_module.hpp
  bt_runtime_publisher.hpp

src/rc26_decision/src/bt/
  chinese_localization_module.cpp
  bt_runtime_publisher.cpp

src/rc26_decision/config/
  bt_localization.yaml
  decision_params.yaml

web/r2_dashboard/src/lib/bt/
  localization.ts
  layout.ts

web/r2_dashboard/src/components/
  BtCanvas.tsx
  BtNodeComponent.tsx
  NodeDetailPanel.tsx
  BlackboardPanel.tsx
  BlackboardDetailPanel.tsx
  LocalizedExplanationBlock.tsx
  EventTimeline.tsx
```

## 3. 核心代码实现

### 后端加载与热更新

文件：

- `src/rc26_decision/include/rc26_decision/bt/chinese_localization_module.hpp`
- `src/rc26_decision/src/bt/chinese_localization_module.cpp`

职责：

- 读取 YAML
- 解析节点、黑板、子树、服务、事件、自定义类型目录
- 根据运行态节点生成最终中文展示对象
- 定时检查文件时间戳，支持热更新

核心结构：

```cpp
struct LocalizedEntrySpec {
  std::string display_name;
  std::string tooltip;
  std::string summary;
  std::string scenario;
  std::string attention;
  std::string markdown;
  std::vector<std::string> related_blackboard_keys;
  std::map<std::string, LocalizedValueSpec> values;
};
```

### 后端发布话题

文件：

- `src/rc26_decision/src/bt/bt_runtime_publisher.cpp`

新增参数：

```yaml
bt_runtime.localization_topic: "r2/bt/localization"
bt_runtime.localization_reload_ms: 1000
bt_runtime.localization_config: ""
```

新增话题：

```text
r2/bt/localization
```

建议 QoS：

- `reliable`
- `transient_local`
- `KeepLast(1)`

### 前端查询 helper

文件：

- `web/r2_dashboard/src/lib/bt/localization.ts`

职责：

- 把 `r2/bt/localization` 转成索引
- 提供 `getChineseNode` / `getChineseBlackboardKey`
- 支持按 UID 查节点中文名
- 支持黑板值枚举翻译

## 4. 示例配置

实际配置文件：

- `src/rc26_decision/config/bt_localization.yaml`

首版已内置：

- 控制/装饰器/条件等通用注册项
- 20+ 个当前项目常用节点
- 20+ 个当前项目常用黑板键
- 多个黑板值枚举示例
- 子树中文目录
- 服务 / 事件预留目录

### 配置模板

```yaml
version: "2026-03-15"
locale: "zh-CN"

custom_types:
  GrabTip:
    display_name: "抓取矛头"
    tooltip: "控制机构去抓取武馆区的矛头。"
    summary: "武馆阶段的第一步。"
    scenario: "开局进入武馆区后执行。"
    attention: "失败时优先检查机构状态和错误码。"
    related_blackboard_keys: ["mechanism_tip_state", "last_action_error_code"]

blackboard_keys:
  loc_level:
    display_name: "定位健康等级"
    tooltip: "定位链路当前风险等级。"
    summary: "数值越高风险越高。"
    scenario: "主任务突然变慢或进入恢复时先看它。"
    attention: "要结合 loc_reason 一起看。"
    related_blackboard_keys: ["loc_reason", "loc_recommended_profile"]
    values:
      "0":
        display_name: "绿色"
        explanation: "定位正常。"
      "1":
        display_name: "黄色"
        explanation: "轻度退化。"
```

### 解释文案模板

配置里推荐维护以下四段：

```md
## 功能说明
这个节点 / 黑板键是做什么的。

## 使用场景
什么时候会看到它，现场一般在什么情况下用。

## 注意事项
最容易踩的坑，或者最该联动观察的地方。

## 相关黑板键
- key_a
- key_b
```

当前实现会根据 `summary / scenario / attention / related_blackboard_keys` 自动生成 Markdown 文本。

## 5. 前端调用示例

### 订阅新话题

```ts
const TOPICS = [
  'r2/bt/model',
  'r2/bt/snapshot',
  'r2/bt/blackboard',
  'r2/bt/events',
  'r2/bt/localization',
];
```

### 节点中文显示

```ts
const localized = getChineseNode(node, localization);

return {
  label: localized.displayName,
  tooltip: localized.tooltip,
  originalName: localized.originalName,
};
```

### 黑板键中文显示

```ts
const localizedKey = getChineseBlackboardKey(entry.key, localization);
const localizedValue = getChineseBlackboardValue(entry.key, entry.value, localization);
```

### 点击查看解释

当前页面行为：

- 点节点：右侧显示节点中文解释
- 点黑板键：右侧切换为黑板解释卡片
- 点解释里的相关黑板键：继续跳转到该黑板键详情

## 6. 维护与扩展说明

### 新增一个新节点

1. 在 `bt_localization.yaml` 的 `custom_types` 或 `instances` 下新增条目。
2. 填写 `display_name / tooltip / summary / scenario / attention`。
3. 如果依赖黑板键，把键名写到 `related_blackboard_keys`。
4. 若有实例级特殊文案，优先放到 `instances`。
5. 保存文件后，等待热更新自动生效。

### 新增一个新黑板键

1. 在 `blackboard_keys` 下增加键名。
2. 填写中文名和解释。
3. 如果值是枚举或状态码，补上 `values`。
4. 若需要前端实际看到它，还要同步加入 `bt_runtime.blackboard_whitelist`。

### 新增一个新子树

1. 在 `subtrees` 下增加子树 ID。
2. 写清楚这个子树解决什么问题。
3. 若是守护型子树，注意把“何时抢占主任务”写在 `attention` 里。

### 新增一个新服务或事件

1. 在 `services` 或 `events` 下补目录项。
2. 当前前端主界面还没有直接消费它们，但后端目录已支持维护。
3. 后续若新增 UI 面板，可直接复用现有目录结构。

### 推荐维护规则

- 运维能看懂优先，不要写内部代码名式解释。
- 一条解释尽量只说一件事。
- 注意区分“节点成功”和“业务成功”。
- 若某项暂时只是预留状态，要明确写“预留”。
- 黑板值是状态枚举时，尽量把常见值都补齐。
