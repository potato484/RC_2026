# rc26_interfaces

## 模块定位

`rc26_interfaces` 是整个 R2 仓库的跨包接口真源。

## 当前导航相关契约

- `NavigateTopoTarget.action`
- `NavigateSurfaceRoute.action`
- `SetXhuMotionMode.srv`
- `MfBlockOverlay.msg`
- `MfBlockOverlayCell.msg`
- `XhuSemanticCorridor.msg`
- `XhuMotionModeState.msg`
- `XhuTrackingState.msg`

`NavigateSurfaceRoute.action` 的当前语义是：上游传入世界坐标系下的 `start_pose / goal_pose`，`rc26_topo_nav` 先把点击点投影到 dense `surface_graph`，产出 `projected_start_pose / projected_goal_pose / planned_path`，再按 surface segment 顺序执行。当前不负责“从机器人当前位置自动接驳到起点”，运行前提是机器人已经足够接近点击起点。

## 当前边界

- 只定义消息、服务、动作
- 接口是否存在以 [src/rc26_interfaces/CMakeLists.txt](/home/potato/RC_2026/src/rc26_interfaces/CMakeLists.txt) 为准
