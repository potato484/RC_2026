# rc26_visualization

`rc26_visualization` 负责把 R2 当前自研导航链上的定位、控制、keepout、地形、机构和运行时语义聚合成统一的诊断输出。

## 当前输入

- 定位:
  - `/localization/health`
  - `/localization/backend_status`
- 控制:
  - `/control_degraded`
  - `control_degenerate_score`
  - `compute_time_ms`
  - `pose_age_ms`
  - `collision_d_min`
  - `/xhu_nav/semantic_gate`
  - `/xhu_nav/tracking_state`
- 导航运行时:
  - `/xhu_nav/motion_mode_state`
- Keepout:
  - `/mf_block_overlay`
  - `/kfs_filter_mask`
  - `/kfs_keepout_heartbeat`
- 地形:
  - `terrain_obstacles`
  - `terrain_drop`
  - `/terrain_grid_map_local`
- 机构:
  - `/mechanism/state`

## 当前输出

- `r2/diag/summary`
- `r2/diag/operator_status`
- `r2/diag/events`
- `r2/diag/reset_topic_timeout_count`

## 当前定位

- 不直接控制机器人
- 不替代各子模块自身诊断
- 负责把分散状态收敛为值守可读语义
- 当前诊断语义已完全围绕 xhu 自研导航话题组织，`OperatorStatus` 也不再暴露 `terrain_speed_limited`

核心实现见：

- [src/visualization_status_node.cpp](/home/potato/RC_2026/src/rc26_visualization/src/visualization_status_node.cpp)
- [src/visualization_status_core.cpp](/home/potato/RC_2026/src/rc26_visualization/src/visualization_status_core.cpp)
