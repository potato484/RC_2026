# rc26_bringup

## 模块定位

`rc26_bringup` 是 R2 整车链路的统一装配入口，负责决定哪些运行时包被拉起，但不拥有这些包内部算法真源。

## 当前装配口径

`bringup.launch.py` 与 `odometry.launch.py` 现在都固定按 headless 口径装配，不再声明或透传任何 viewer / RViz / Foxglove 兼容参数。

- `bringup.launch.py`
  - 装配 Point-LIO、里程计接口、定位、terrain、keepout、`rc26_xhu_nav` 和决策
- `odometry.launch.py`
  - 装配 Point-LIO、`rc26_odom_interface`、`rc26_sensor_scan`，并可按 `enable_terrain_grid_map` 额外带起 terrain grid map

如需图形观察，应手工启动工作区外部可视化工具只读消费当前 ROS2 输出。仓库内仍保留两个可复用的 RViz 预设：

- `src/rc26_bringup/rviz/slam.rviz`
- `src/rc26_bringup/rviz/navigation_default.rviz`

## 当前关键文件

- [launch/bringup.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/bringup.launch.py)
- [launch/odometry.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/odometry.launch.py)
- `launch/localization.launch.py`
- `launch/test_mapping.launch.py`
- `launch/test_localization_chain.launch.py`
- `launch/test_relocalization.launch.py`
- `launch/test_loop_closure.launch.py`
- `launch/test_navigation.launch.py`
- `config/localization.yaml`
- `rc26_xhu_nav/config/topo_nav.yaml`
- `rc26_xhu_nav/config/local_3d_planner.yaml`
- `rc26_xhu_nav/config/xhu_motion_runtime.yaml`

## 当前边界

- 负责装配，不承载 planner、控制器或可视化平台的实现本体
- 当前工作区默认不再装配第一方 GUI 或操作员聚合包
- 如需可视化，应由工作区外部工具只读消费现有 ROS2 输出

## 本轮收口

- `bringup.launch.py` 删除了 `visualization_profile`、`visualization_backend`、`visualization_layout`、`visualization_status_enable` 和 `use_rviz`
- `odometry.launch.py` 删除了 `odometry_use_rviz` 与 `odometry_visualization_layout`
- `test_odometry_chain.launch.py` 不再透传任何可视化兼容参数
- 仓库内不再维护 `src/rc26_bringup/foxglove/*.json` 旧资产
- 3D 导航装配已经收口到 `rc26_xhu_nav`，当前固定装配 `topo_nav_node + xhu_motion_mode_manager_node + xhu_motion_runtime_node`
- `team`、topo graph、robot geometry、local planner/runtime 配置都由 bringup 统一装配给 `rc26_xhu_nav`
- `local_execution_backend` 与 `enable_local_3d_planner_observe` 已从主启动入口移除，不再保留 follower / observe-only planner 切换
- 定位相关调参项 `competition_mode`、`enable_graph_backend`、`p4_candidate_enable`、`min_inliers` 已由 `bringup.launch.py` 透传到 `localization.launch.py`
- 当前整车联调入口统一查看仓库根目录 `调试/` 目录，按“遥控 → 建图 → 定位 → 重定位 → 回环 → 导航”顺序执行
