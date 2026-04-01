# rc26_topo_nav

## 模块定位

`rc26_topo_nav` 是 R2 比赛特化导航表达层，负责把目标意图（node/task/route）转换成可执行 edge 序列与 corridor，并在运行时驱动执行与重规划。

当前支持双执行后端：

- `nav2_follow_path`：历史兼容路径，仍调用 Nav2 `FollowPath`
- `xhu_direct`：直接发布 `XhuSemanticCorridor`，由 `xhu_motion_follower` 输出 `cmd_vel`

## 架构

内部保持五层结构：

| 层 | 文件 | 职责 |
|---|---|---|
| graph_loader | `graph_loader.cpp` | YAML 装载与合法性校验 |
| overlay_reducer | `overlay_reducer.cpp` | 运行态语义叠加（定位/地形/KFS/base_ground） |
| planner | `planner.cpp` | 图搜索（A*/Dijkstra），输出 route 与 edge 序列 |
| edge_executor | `edge_executor.cpp` | 单边执行调度（Nav2 或 xhu_direct） |
| diagnostics | `diagnostics.cpp` | `/topo_nav/*` 与 `/xhu_nav/*` 诊断输出 |

## 对外接口

- **Action Server**：`navigate_topo_target` (`NavigateTopoTarget.action`)
  - Goal：`target_type` + `target_id` + `team` + `allow_replan`
  - Result：`success` + `terminal_node_id` + `failure_code/reason`
  - Feedback：`active_node_id` + `active_edge_id` + `exec_state` + `replan_count`
- **Published Topics**：
  - `/topo_nav/route`、`/topo_nav/corridor`、`/diagnostics`
  - `/xhu_nav/route`、`/xhu_nav/corridor`、`/xhu_nav/diagnostics`
  - `/xhu_nav/active_edge`、`/xhu_nav/semantic_gate`、`/xhu_nav/risk_markers`
  - `/xhu_nav/corridor_cmd` (`XhuSemanticCorridor`)：仅在 `xhu_direct` 路径被执行时用于下发当前 corridor
- **Subscribed Topics**：
  - `/localization/health`、`/localization/backend_status`、`/localization/route_observability`
  - `terrain_features`
  - `/mf_block_overlay`
  - `base_ground/level`、`base_ground/stable_terrain`、`base_ground/stable_operation`
  - `/xhu_nav/tracking_state` (`XhuTrackingState`)：`xhu_direct` 执行反馈
- **Service Clients**：
  - `set_nav_mode` (`SetNavMode`)：`nav2_follow_path` 模式
  - `set_xhu_motion_mode` (`SetXhuMotionMode`)：`xhu_direct` 模式

## 关键配置

- `config/topo_nav.yaml`
  - `execution_backend`：`nav2_follow_path` / `xhu_direct`
  - `xhu.exec_timeout_sec`
  - `xhu.hold_replan_timeout_sec`
  - `weights.*`：图搜索代价权重
- `config/r2_field_graph_blue.yaml` / `config/r2_field_graph_red.yaml`
  - 静态导航图真源；支持 `routes` 预定义路线和 `edges.control_points`

## Overlay 规则

- LocalizationHealth.YELLOW → 边降级为 `slow_only`
- LocalizationHealth.ORANGE 或 RouteObservability.HIGH → 禁止 `ramp/drop_risky` 边
- LocalizationHealth.RED → `hold`，中止当前 action
- KFS block confidence >= 0.7 → 节点 BLOCKED
- TerrainFeatureGrid `p_obstacle >= 0.6` 或 `p_drop >= 0.8` → 边 BLOCKED

## 边执行策略

- 每条边执行前先请求模式（`SetNavMode` 或 `SetXhuMotionMode`）
- `nav2_follow_path`：
  - corridor 交给 Nav2 `FollowPath`
  - 1 次本地重试，失败后进入重规划语义
- `xhu_direct`：
  - 生成 `XhuSemanticCorridor` 发布到 `/xhu_nav/corridor_cmd`
  - 等待 `/xhu_nav/tracking_state`，识别 `PASS/HOLD/REPLAN/ABORT`
  - `HOLD` 超时后转 `REPLAN`，整体执行超时后也转 `REPLAN`

## 2026-04 运行时收口

- `navigate_topo_target` 现已按单飞行语义运行：并发 goal 会被直接拒绝，避免多个 BT 或重复请求同时争抢同一 `edge_executor`。
- `topo_nav_node` 不再用 `detach()` 放飞执行线程；析构时会显式 `cancel + join`，并在执行异常时统一回写 `INTERNAL_ERROR` 与 diagnostics。
- `edge_executor` 的 service/action 等待已经从 `spin_until_future_complete()` 改为后台轮询 future，避免节点已被 executor 托管时再次临时 spin 的运行时冲突。
- `xhu_direct` 执行链补上了 corridor 接收超时、tracking 心跳超时、首包前重发和 stale tracking 清理，减少 follower 尚未就绪或反馈中断时的黑盒卡死。

## 启动

- `navigation_stack_mode:=topo`：`execution_backend=nav2_follow_path`
- `navigation_stack_mode:=xhu_direct`：`execution_backend=xhu_direct`
- 独立启动 `topo_nav.launch.py` 时，`graph_file` 为空会按 `team` 自动选择红蓝图
