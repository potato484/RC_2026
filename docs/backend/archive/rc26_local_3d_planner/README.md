# rc26_local_3d_planner

## 模块定位

`rc26_local_3d_planner` 是 R2 当前新增的可复用局部 3D 规划 core 包，负责把 corridor、terrain risk 和 semantic summary 归并成局部速度候选评分结果。

## 当前实现

- 构建产物:
  - `rc26_local_3d_planner_core`
  - `local_3d_planner_node`
  - `local_planner_trace_cli`
- 关键源码:
  - [include/rc26_local_3d_planner/planner_core.hpp](/home/potato/RC_2026/src/rc26_local_3d_planner/include/rc26_local_3d_planner/planner_core.hpp)
  - [src/planner_core.cpp](/home/potato/RC_2026/src/rc26_local_3d_planner/src/planner_core.cpp)
  - [src/local_3d_planner_node.cpp](/home/potato/RC_2026/src/rc26_local_3d_planner/src/local_3d_planner_node.cpp)
  - [src/local_planner_trace_cli.cpp](/home/potato/RC_2026/src/rc26_local_3d_planner/src/local_planner_trace_cli.cpp)
- 关键配置:
  - [config/local_3d_planner.yaml](/home/potato/RC_2026/src/rc26_local_3d_planner/config/local_3d_planner.yaml)
  - [scenarios/](/home/potato/RC_2026/src/rc26_local_3d_planner/scenarios)

## 当前接口

- 订阅:
  - `/xhu_nav/corridor_cmd`
  - `/xhu_nav/motion_mode_state`
  - `terrain_features`
  - `/xhu_nav/semantic_layer_summary`
  - `control_state`
- 发布:
  - `/xhu_nav/local_planner_state`
  - `/xhu_nav/recovery_state`
  - `/xhu_nav/local_planner_preview`

## 当前规划口径

- `PlannerCore` 当前按 tracked-diff 口径采样 `linear.x + angular.z` 速度对，并基于以下项做打分：
  - path alignment
  - heading alignment
  - preferred speed 偏差
  - angular effort
  - terrain clearance
- 地形碰撞检查当前直接使用 `TerrainFeatureGrid` 的 `p_obstacle / p_drop` 网格，并结合 `stop_envelope_half_width_m` 做中心线与左右包络采样。
- semantic summary 当前只提供粗粒度 blocked/slow 语义：
  - `slow_cells > 0` 时整体收紧线速度上限
  - 无解且 `blocked_cells > 0` 时返回 `WAITING_ON_BLOCK`
  - 无解但允许原地旋转且 goal heading 偏差足够大时返回 `RECOVERY_RUNNING`
  - 其余无解情况返回 `LOCAL_COLLISION_BLOCKED`
- `PlannerCore::plan(...)` 当前支持可选输出 `PlannerTrace`：
  - 逐条候选轨迹的采样点、得分和淘汰原因
  - 最终选中轨迹与最终状态
  - 供 CLI 和前端离线观察直接复用，不再额外复制一套假想候选生成逻辑

## 当前边界

- 负责局部候选采样、评分和 preview
- 允许作为 observe-only 节点独立运行，也允许被 `rc26_omni_controller` 直接复用
- 不直接拥有 `cmd_vel` 权威
- 不负责 topo 图规划、全局 route 重规划或比赛策略
- 当前还不是 swept-volume 级的完整 3D 动态避障器；它只消费已有 corridor 和 terrain/keepout 摘要

## 近期实现说明

- 当前新增 `local_3d_planner_node` 作为 observe-only 观测节点，主要用于在 legacy follower 链下先暴露 planner state / recovery state / preview。
- 当前 `PlannerCore` 已被 `xhu_motion_runtime_node` 复用，避免在执行器包里复制一套局部评分逻辑。
- CMake 当前只导出可复用的 `rc26_local_3d_planner_core` 库，不把 `local_3d_planner_node` 可执行文件作为跨包链接目标导出。
- 当前新增 `local_planner_trace_cli --snapshot <yaml>`，可以把局部规划快照导出成 JSON trace，作为 `topo_sim_server + sim_viewer` 的局部规划案例真源。
- 包内当前内置四个最小回归场景：
  - `pass_straight`
  - `waiting_on_block`
  - `rotate_recovery`
  - `local_collision_blocked`
- 当前测试已经覆盖两层回归：
  - `test_planner_core.cpp` 验证 PASS / WAITING_ON_BLOCK / RECOVERY_RUNNING / LOCAL_COLLISION_BLOCKED 状态判定
  - `test_local_planner_trace_cli.py` 验证 CLI 能稳定导出上述案例的 trace 结构
