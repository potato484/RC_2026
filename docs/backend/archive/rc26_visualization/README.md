# rc26_visualization

## 模块定位

`rc26_visualization` 负责把定位、控制、keepout、地形、机构和导航运行时状态压成统一的诊断语义。

## 当前实现

- 构建产物:
  - `visualization_status_core`
  - `rc26_visualization_status_node`
- 关键配置:
  - `config/visualization_status.yaml`
- 关键输入:
  - `/xhu_nav/semantic_gate`
  - `/xhu_nav/motion_mode_state`
  - `/xhu_nav/tracking_state`
  - `/xhu_nav/local_planner_state`
  - `/xhu_nav/recovery_state`
  - `/xhu_nav/semantic_layer_summary`
  - `/mf_block_overlay`
  - `/kfs_filter_mask`
  - `/kfs_keepout_heartbeat`

## 当前诊断输入口径

- 诊断输入已经完全收口到 xhu 主链与 keepout 约束输入
- 地形侧只消费 `terrain_obstacles`、`terrain_drop` 与 `/terrain_grid_map_local`，不再监控 `terrain_speed_limit`
- 当前 `nav_safety` 诊断已经补充局部规划、恢复动作和语义层摘要三条运行时输入，但对外 `OperatorStatus.msg` 结构保持不变

## 当前边界

- 只做状态聚合与事件生成
- 不参与导航控制和决策

## 近期实现说明

- 当前节点新增三组 topic 参数和 watchdog：
  - `topics.local_planner_state`
  - `topics.recovery_state`
  - `topics.semantic_layer_summary`
- 当前会把以下运行时信号提升成统一事件：
  - `LOCAL_PLANNER_WAITING`
  - `LOCAL_COLLISION_BLOCKED`
  - `LOCAL_RECOVERY_RUNNING`
  - `SEMANTIC_LAYER_BLOCKED`
  - `SEMANTIC_LAYER_SLOW_ONLY`
- `r2/nav_safety` 当前还会附带 planner/recovery/semantic 的补充字段，便于不改 `OperatorStatus.msg` 的前提下继续看清运行时根因。
