# 行为树中文映射表

这份文档记录 `merlin-bt-visualizer` 当前已经落地的中文映射真相，便于后续维护节点库、属性面板和中文 UI 时对照。

## 1. 代码真源

- 节点注册表：`merlin-bt-visualizer/src/generated/btNodeRegistry.ts`
- 节点解释与摘要生成：`merlin-bt-visualizer/src/utils/btDisplay.ts`
- 树名、实例名、属性键、黑板键、枚举值：`merlin-bt-visualizer/src/i18n/btTerms.ts`
- 默认语言：`merlin-bt-visualizer/src/i18n/useLocaleStore.ts`

当前页面默认语言固定为 `zh-CN`。`en-US` 字典仅作为保留能力存在，当前界面不提供对外切换入口。

2026-04-04 起，`btNodeRegistry.ts` 里的节点 `descriptionZh` 已统一改成“中文主叙述 + 1-2 行内能看懂”的说明文本。后续补节点时，不再沿用内部实现术语或代码式句子，优先保持新手也能一眼看懂的口径。

## 2. 官方节点映射

### 2.1 控制节点

| 英文标签 | 中文名称 | 说明 |
| --- | --- | --- |
| `Sequence` | 顺序节点 | 依次执行子节点，前一个成功才继续。 |
| `SequenceWithMemory` | 记忆顺序节点 | 失败或运行中时保留当前执行位置。 |
| `SequenceStar` | 记忆顺序节点 | `SequenceWithMemory` 的兼容旧写法。 |
| `ReactiveSequence` | 响应式顺序节点 | 每次 tick 都重新检查前置条件。 |
| `Fallback` | 回退节点 | 依次尝试子节点，直到某个成功。 |
| `ReactiveFallback` | 响应式回退节点 | 高优先级分支可立即抢占。 |
| `Parallel` | 并行节点 | 按成功阈值和失败阈值聚合多个子节点结果。 |
| `ParallelAll` | 全并行节点 | 等待全部子节点完成后再聚合。 |
| `IfThenElse` | 条件分支节点 | 固定条件/满足/不满足三段结构。 |
| `WhileDoElse` | 循环分支节点 | 固定条件/循环体/否则分支结构。 |
| `RoundRobin` | 轮询节点 | 轮流从不同子节点开始尝试。 |
| `Switch2` | 多路切换节点（2 路） | 依据变量值匹配 2 个条件。 |
| `Switch3` | 多路切换节点（3 路） | 依据变量值匹配 3 个条件。 |
| `Switch4` | 多路切换节点（4 路） | 依据变量值匹配 4 个条件。 |
| `Switch5` | 多路切换节点（5 路） | 依据变量值匹配 5 个条件。 |
| `Switch6` | 多路切换节点（6 路） | 依据变量值匹配 6 个条件。 |

### 2.2 装饰节点

| 英文标签 | 中文名称 | 说明 |
| --- | --- | --- |
| `Inverter` | 结果反转装饰器 | 成功和失败语义反转。 |
| `ForceSuccess` | 强制成功装饰器 | 无论子节点结果如何都按成功返回。 |
| `ForceFailure` | 强制失败装饰器 | 无论子节点结果如何都按失败返回。 |
| `Repeat` | 重复装饰器 | 按次数重复执行子节点。 |
| `RetryUntilSuccessful` | 重试直到成功装饰器 | 在允许次数内持续重试。 |
| `KeepRunningUntilFailure` | 持续运行直到失败装饰器 | 子节点非失败时持续运行。 |
| `Delay` | 延时装饰器 | 先等待再执行子节点。 |
| `Timeout` | 超时装饰器 | 子节点超过时限后强制结束。 |

### 2.3 叶子与结构节点

| 英文标签 | 中文名称 | 说明 |
| --- | --- | --- |
| `Script` | 脚本赋值节点 | 使用脚本语法读写黑板。 |
| `ScriptCondition` | 脚本条件节点 | 使用脚本表达式判断真假。 |
| `AlwaysSuccess` | 恒成功节点 | 直接返回成功。 |
| `AlwaysFailure` | 恒失败节点 | 直接返回失败。 |
| `SubTree` | 子树调用节点 | 调用另一棵行为树。 |

## 3. 机器人模块映射

### 3.1 武馆区

| 英文标签 | 中文名称 | 类型 |
| --- | --- | --- |
| `GrabTip` | 抓取端头 | 动作 |
| `AssembleWeapon` | 组装武器 | 动作 |
| `CheckManualRobot` | 检查手动机器人 | 条件 |

### 3.2 梅林区

| 英文标签 | 中文名称 | 类型 |
| --- | --- | --- |
| `StairClimb` | 上楼梯 | 动作 |
| `StairDescend` | 下楼梯 | 动作 |
| `GrabKFS` | 抓取 KFS | 动作 |
| `CheckKFS` | 检查 KFS 状态 | 条件 |
| `CheckLoad` | 检查装载数量 | 条件 |
| `ScanSurroundings` | 扫描周围环境 | 动作 |
| `SelectNextGrid` | 选择下一格动作 | 动作 |
| `CheckExitCondition` | 检查退出条件 | 条件 |
| `CheckR1Blocking` | 检查 R1 阻挡 | 条件 |
| `IncrementKFSCount` | 累加 KFS 数量 | 动作 |
| `UpdateMapKFS` | 更新地图中的 KFS 状态 | 动作 |

### 3.3 导航

| 英文标签 | 中文名称 | 类型 |
| --- | --- | --- |
| `NavToPose` | 导航到地图位姿 | 动作 |

### 3.4 对抗区

| 英文标签 | 中文名称 | 类型 |
| --- | --- | --- |
| `PlaceKFSGrid` | 放置 KFS 到九宫格 | 动作 |
| `GimbalMove` | 云台移动 | 动作 |
| `FollowManualRobot` | 跟随手动机器人 | 动作 |

### 3.5 视觉

| 英文标签 | 中文名称 | 类型 |
| --- | --- | --- |
| `VisionStart` | 启动视觉 | 动作 |
| `VisionStop` | 停止视觉 | 动作 |
| `VisionSetModel` | 切换视觉模型 | 动作 |
| `WaitVisionTarget` | 等待视觉目标 | 动作 |

## 4. 树与实例名映射

### 4.1 树标识

| 英文标识 | 中文名称 |
| --- | --- |
| `MainTree` | 主任务树 |
| `MCAreaTree` | 武馆主任务 |
| `MFAreaTree` | 梅林主任务 |
| `CombatAreaTree` | 对抗主任务 |
| `MF_Entry` | 梅林进门阶段 |
| `MF_Loop` | 梅林循环阶段 |
| `GrabKFSSeq` | 抓取 KFS 子流程 |
| `MoveToGridSeq` | 移动到下一格子流程 |
| `MF_Exit` | 梅林离场阶段 |

### 4.2 实例名

| 英文实例名 | 中文名称 |
| --- | --- |
| `Combat_Sequence` | 对抗顺序流程 |
| `goto_combat` | 前往对抗区 |
| `place_sequence` | 放置序列 |
| `grab_tip` | 取端头 |
| `assemble` | 执行组装 |
| `Entry_Seq` | 进门顺序流程 |
| `Loop_Body` | 循环主体 |
| `GrabKFSSeq` | 抓取 KFS 子流程 |
| `MoveToGridSeq` | 移动到下一格子流程 |
| `Exit_Seq` | 离场顺序流程 |
| `MC_Sequence` | 武馆顺序流程 |
| `MF_Main` | 梅林主流程 |

## 5. 属性键映射

| 英文键 | 中文名称 |
| --- | --- |
| `name` | 节点名称 |
| `id` | 节点标识 |
| `ID` | 子树标识 |
| `_autoremap` | 自动映射 |
| `timeout_sec` | 超时时间 |
| `error_code` | 错误码 |
| `node_id` | 拓扑节点 |
| `task_tag` | 任务标签 |
| `route_tag` | 路径标签 |
| `grid_id` | 格子编号 |
| `grid_position` | 九宫格位置 |
| `kfs_type` | KFS 类型 |
| `expected_state` | 期望状态 |
| `min_load` | 最小装载数 |
| `max_load` | 最大装载数 |
| `delay_msec` | 延迟时长 |
| `msec` | 超时毫秒数 |
| `num_cycles` | 循环次数 |
| `num_attempts` | 重试次数 |
| `success_count` | 成功阈值 |
| `failure_count` | 失败阈值 |
| `threshold` | 阈值 |
| `variable` | 判定变量 |
| `case_1` | 条件一 |
| `case_2` | 条件二 |
| `case_3` | 条件三 |
| `case_4` | 条件四 |
| `case_5` | 条件五 |
| `case_6` | 条件六 |
| `code` | 脚本 |
| `model_id` | 模型标识 |
| `target_attr` | 目标属性 |
| `max_dist` | 最远距离 |
| `timeout` | 等待超时 |
| `pitch` | 俯仰角 |
| `yaw` | 偏航角 |
| `follow_distance` | 跟随距离 |
| `lost_timeout` | 丢失超时 |
| `layer` | 目标层 |
| `selected_layer` | 实际层 |
| `distance_threshold` | 距离阈值 |
| `static_time` | 静止时间 |
| `next_action` | 输出动作 |
| `target_grid` | 输出目标格 |

## 6. 黑板键映射

| 英文键 | 中文名称 |
| --- | --- |
| `current_grid` | 当前格子 |
| `target_grid` | 目标格子 |
| `next_action` | 下一步动作 |
| `exit_grid` | 出口格子 |
| `target_kfs_count` | 目标 KFS 数量 |
| `kfs_on_board` | 已装载 KFS 数量 |
| `merlin_last_transition_reason` | 梅林转移原因 |
| `current_level` | 当前楼层 |
| `stair_delta` | 台阶高度差 |
| `base_ground_stable` | 地面稳定状态 |
| `stair_climb_done` | 上楼梯完成 |
| `stair_descend_done` | 下楼梯完成 |
| `timeout_sec` | 超时时间 |
| `error_code` | 错误码 |
| `selected_layer` | 实际层号 |
| `mechanism_hal_open` | 机构链路在线状态 |
| `mechanism_current_cmd_id` | 当前机构命令 ID |
| `last_action_error_code` | 最近动作错误码 |
| `vision_running` | 视觉运行状态 |
| `vision_ok` | 视觉可用状态 |
| `vision_current_model` | 当前视觉模型 |
| `vision_has_target` | 视觉是否有目标 |
| `vision_distance_m` | 视觉距离 |
| `vision_attr_kind` | 视觉目标属性 |
| `loc_level` | 定位健康等级 |
| `loc_reason` | 定位原因 |
| `loc_recommended_profile` | 推荐导航档位 |
| `loc_guard_required` | 是否需要定位守护 |
| `loc_guard_reason` | 定位守护原因 |
| `loc_last_profile` | 上次导航档位 |
| `location` | 当前位置 |

## 7. 枚举值映射

| 英文值 | 中文值 |
| --- | --- |
| `GRAB` | 抓取 |
| `MOVE` | 移动 |
| `WAIT` | 等待 |
| `AUTO_KFS` | 自动识别 KFS |
| `EMPTY` | 空位 |
| `normal` | 常规档 |
| `loc_yellow` | 黄色保守档 |
| `loc_orange` | 橙色保守档 |
| `loc_red_hold` | 红色保持档 |
| `Truth` | 真目标 |
| `False` | 假目标 |
| `R_R1` | 红队 R1 |
| `B_R1` | 蓝队 R1 |
| `true` | 是 |
| `false` | 否 |

## 8. 维护要求

- 新增节点时，先补 `btNodeRegistry.ts`，再补 `btTerms.ts`，最后验证 `btDisplay.ts` 的中文摘要是否成立。
- 如果 XML 中出现了新节点但注册表没有收录，编辑器仍能 round-trip 保存，但会落到“未注册节点”分支；这不符合当前维护目标，应尽快补齐映射。
- 如果调整了中文术语，测试需要同步覆盖：
  - `tests/btDisplay.test.ts`
  - `tests/editorStore.test.ts`
  - `tests/useStore.test.ts`
