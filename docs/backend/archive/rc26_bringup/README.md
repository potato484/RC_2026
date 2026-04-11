# rc26_bringup

## 模块定位

`rc26_bringup` 是 R2 整车链路的统一装配入口，负责决定哪些运行时包被拉起，但不拥有这些包内部算法真源。

## 当前可视化装配口径

`bringup.launch.py` 现在固定按 headless 口径装配：

- `visualization_profile:=headless`

其中：

- `headless`：车端默认口径；不启动 GUI，也不再装配 `rc26_xhu_viewer_status_node`

以下参数为了兼容旧脚本仍然保留，但已经退化成 no-op：

- `visualization_backend:=none`
- `visualization_layout:=*`
- `visualization_status_enable:=*`
- `use_rviz:=true | false`

旧里程计调试入口 `launch/odometry.launch.py` 也已对齐：

- `odometry_use_rviz` 与 `odometry_visualization_layout` 只保留兼容参数
- 当前始终只保留 headless 里程计链

Foxglove 和已删除的 `src/rc26_xhu_viewer` 子树都不再是 bringup 的主链路后端。

## 当前关键文件

- [launch/bringup.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/bringup.launch.py)
- [launch/odometry.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/odometry.launch.py)
- `launch/localization.launch.py`
- `config/localization.yaml`
- `rc26_xhu_nav/config/topo_nav.yaml`
- `rc26_xhu_nav/config/local_3d_planner.yaml`
- `rc26_xhu_nav/config/xhu_motion_runtime.yaml`

## 当前边界

- 负责装配，不承载 planner、控制器或可视化平台的实现本体
- 当前工作区默认不再装配第一方 GUI 或操作员聚合包
- 如需可视化，应由工作区外部工具只读消费现有 ROS2 输出

## 当前真实变化

- bringup 默认口径仍是 `visualization_profile:=headless`
- `src/rc26_xhu_viewer/` 已整体删除，不再作为 bringup 依赖
- 3D 导航装配已经收口到 `rc26_xhu_nav`，当前固定装配 `topo_nav_node + xhu_motion_mode_manager_node + xhu_motion_runtime_node`
- `team`、topo graph、robot geometry、local planner/runtime 配置都由 bringup 统一装配给 `rc26_xhu_nav`
- `local_execution_backend` 与 `enable_local_3d_planner_observe` 已从主启动入口移除，不再保留 follower / observe-only planner 切换
- `visualization_profile/backend/layout/status_enable/use_rviz` 仅保留兼容参数
- `odometry.launch.py` 已同步收口为 headless
