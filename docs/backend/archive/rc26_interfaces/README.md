# rc26_interfaces

## 模块定位

`rc26_interfaces` 是整个 R2 仓库的跨包接口真源。

## 当前导航相关契约

- `NavigateTopoTarget.action`
- `NavigateSurfaceRoute.action`
- `SetXhuMotionMode.srv`
- `MfBlockOverlay.msg`
- `MfBlockOverlayCell.msg`
- `SurfaceGraphOverlay.msg`
- `XhuSemanticCorridor.msg`
- `XhuMotionModeState.msg`
- `XhuTrackingState.msg`
- `XhuSemanticLayerSummary.msg`
- `XhuLocalPlannerState.msg`
- `XhuRecoveryState.msg`

## 当前视觉与机构端头契约

- `TipDetection.msg`、`TipDetectionArray.msg` 与 `/vision/tip_detections` 是视觉端头检测稳定契约。
- `GrabTip.action` 与 `/mechanism/grab_tip` 继续作为机构抓取端头的动作契约。
- 本轮 `rc26_vision` 内部实验链从旧拼音命名收口到 `tip`，不改变上述公共 ROS 接口外形。

当前导航契约最近补充了两组语义：

- `MfBlockOverlayCell.state` 现在额外支持 `SLOW=3`，用于表达“可通行但需要保守降速”的 overlay 单元。
- `XhuSemanticCorridor` 现在额外携带：
  - `preferred_linear_speed`
  - `allow_in_place_rotate`
  - `speed_limit_reason`
  - `active_risk_sources`
- `XhuTrackingState.status` 当前除了 `PASS | HOLD | REPLAN | ABORT` 外，还会稳定出现：
  - `WAITING_ON_BLOCK`
  - `LOCAL_COLLISION_BLOCKED`
  - `RECOVERY_RUNNING`
- `XhuSemanticLayerSummary` 用于把地形风险和 keepout overlay 归并成轻量级摘要，供局部规划/执行链按 revision 消费。
- `XhuLocalPlannerState` 用于暴露局部规划打分结果、建议速度和 observe-only/runtime 状态。
- `XhuRecoveryState` 用于暴露局部恢复动作建议或运行中恢复状态。

`NavigateSurfaceRoute.action` 的当前语义是：上游传入世界坐标系下的 `start_pose / goal_pose`，`rc26_xhu_nav` 先把点击点投影到 dense `surface_graph`，产出 `projected_start_pose / projected_goal_pose / planned_path`，再按 surface segment 顺序执行。当前不负责“从机器人当前位置自动接驳到起点”，运行前提是机器人已经足够接近点击起点。

`NavigateSurfaceRoute.result.failure_code` 当前已稳定为可区分的运行时契约，至少包含：

- 点投影失败：
  - `START_POINT_NOT_PROJECTABLE`
  - `GOAL_POINT_NOT_PROJECTABLE`
- overlay 导致的点不可达：
  - `START_POINT_BLOCKED_BY_OVERLAY`
  - `GOAL_POINT_BLOCKED_BY_OVERLAY`
- body-aware 几何约束导致的点不可达：
  - `START_POINT_BLOCKED_BY_BODY_CONSTRAINT`
  - `GOAL_POINT_BLOCKED_BY_BODY_CONSTRAINT`
- 图与路径级失败：
  - `SURFACE_GRAPH_DISCONNECTED`
  - `SURFACE_PATH_BLOCKED_BY_RUNTIME_OVERLAY`
  - `SURFACE_PATH_BLOCKED_BY_DYNAMIC_OVERLAY`
  - `BODY_CONSTRAINT_UNSATISFIED`
- 运行时守护与执行失败：
  - `ROBOT_GEOMETRY_UNAVAILABLE`
  - `SURFACE_GRAPH_NOT_BODY_AWARE`
  - `LOC_RED_HOLD`
  - `NO_TF`
  - `START_POSE_MISMATCH`
  - `SEGMENT_EXEC_FAILED`
  - `MAX_REPLAN_EXCEEDED`

## 当前边界

- 只定义消息、服务、动作
- `SurfaceGraphOverlay.msg` 当前用于把离散 blocked `node_id / edge_id` 和 TTL 传给 `rc26_xhu_nav`，它只表达 runtime 动态阻塞输入，不改变 `NavigateSurfaceRoute` action 形态
- 新增的 local planner / semantic summary 消息只补充执行链内部状态，不改变 `NavigateTopoTarget` 或 `NavigateSurfaceRoute` 的 action 外形
- 接口是否存在以 [src/rc26_interfaces/CMakeLists.txt](/home/potato/RC_2026/src/rc26_interfaces/CMakeLists.txt) 为准

## 本轮收口

- 随 `src/rc26_xhu_viewer` 一起退役的诊断接口已从 `rc26_interfaces` 移除：
  - `OperatorStatus.msg`
  - `VisualizationEvent.msg`
  - `VisualizationEventArray.msg`
- `docs/middle/openapi.yaml` 不再维护 `diagnostics` 模块索引
- 当前 `docs/middle` 里仍保留的模块索引只覆盖行为树、定位、导航、视觉、机构与流媒体这些真实接口
