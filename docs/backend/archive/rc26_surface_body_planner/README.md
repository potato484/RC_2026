# rc26_surface_body_planner

## 模块定位

`rc26_surface_body_planner` 是 `rc26_topo_nav` 之外的独立 body planner 库，负责在固定比赛 `surface_graph` 上做 heading-aware、受地表约束的 `SE(2.5)` 全车体全局搜索。

## 当前实现

- 当前是 library + CLI 包，不导出 ROS node / topic / service / action
- 主能力：
  - 读取 `surface_graph` YAML
  - 在 `surface_node + heading_bin` 状态空间上做 A*
  - 基于 `nominal_yaw + node/edge clearance + 车体 half_length/half_width` 近似评估转向扫掠
  - 输出 `node_path / edge_path / heading_path / blocked transition reason`
- 当前 CLI：
  - `surface_body_planner_cli`

## 当前输入口径

- 静态 `surface_graph` 几何事实
- 上层传入的 node / edge overlay
- `rc26_robot_geometry` 提供的静态几何 profile，经 `rc26_topo_nav` 转成：
  - `half_length_m`
  - `half_width_m`
- planner 配置参数：
  - `heading_bin_count`
  - `max_heading_change_deg`
  - `turn_cost_weight`
  - `node_turn_clearance_gain`
  - `edge_turn_clearance_gain`

## 当前集成方式

- 当前不直接面对 `rc26_decision` 或控制器，只由 `rc26_topo_nav` 在 `surface_planner_backend=body_planner` 时调用
- `rc26_topo_nav` 仍然负责：
  - 点投影
  - runtime overlay 归并
  - body-aware overlay
  - `NavigateSurfaceRoute` action 契约
  - segment execution 与 route-level replan
- 这个包只补足“更高保真的全车体全局搜索”，不接管导航表达层权威

## 当前边界

- 不负责 TF / localization 权威
- 不负责动态障碍预测与感知提取
- 不负责底层速度控制与 corridor 跟踪
- 不拥有机构状态机
- 不做自由 `SE(3)` / 6DoF planner；当前仍是沿地表的 `SE(2.5)` planner
