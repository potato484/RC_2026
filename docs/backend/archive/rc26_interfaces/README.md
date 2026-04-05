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

`NavigateSurfaceRoute.action` 的当前语义是：上游传入世界坐标系下的 `start_pose / goal_pose`，`rc26_topo_nav` 先把点击点投影到 dense `surface_graph`，产出 `projected_start_pose / projected_goal_pose / planned_path`，再按 surface segment 顺序执行。当前不负责“从机器人当前位置自动接驳到起点”，运行前提是机器人已经足够接近点击起点。

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
- `SurfaceGraphOverlay.msg` 当前用于把离散 blocked `node_id / edge_id` 和 TTL 传给 `rc26_topo_nav`，它只表达 runtime 动态阻塞输入，不改变 `NavigateSurfaceRoute` action 形态
- 接口是否存在以 [src/rc26_interfaces/CMakeLists.txt](/home/aidlux/RC_2026/src/rc26_interfaces/CMakeLists.txt) 为准
