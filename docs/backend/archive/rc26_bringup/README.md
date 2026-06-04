# rc26_bringup

## 模块定位

`rc26_bringup` 是 R2 整车链路的统一装配入口，负责决定哪些运行时包被拉起，但不拥有这些包内部算法真源。

## 当前装配口径

`bringup.launch.py` 与 `odometry.launch.py` 固定按 headless 口径装配，不再声明或透传任何 viewer / RViz / Foxglove 兼容参数。面向现场建图的 `test_mapping.launch.py` 是调试入口，默认复用纯建图链路并额外打开 RViz2，可用 `use_rviz:=false` 关闭图形界面。

- `bringup.launch.py`
  - 装配 Point-LIO、里程计接口、定位、地面高度估计、terrain、keepout runtime manager、Nav2 基础导航栈和决策
  - `slam=false` 时启动 `map_server` 与 Nav2 `navigation_launch.py`
  - `rc26_localization` 继续作为 `map -> odom` 权威，不启动 AMCL
  - `/cmd_vel` 由 Nav2 controller/velocity_smoother 输出
- `odometry.launch.py`
  - 装配 Point-LIO、`rc26_odom_interface`、`rc26_sensor_scan`，并可按 `enable_terrain_grid_map` 额外带起 terrain grid map
  - 读取 `rc26_sensor_extrinsics` 的 YAML profile 发布静态 TF
- `test_navigation.launch.py`
  - 复用整车 bringup，默认 `use_decision=false`
  - 用于定位 + Nav2 基础导航联调
- `test_mapping.launch.py`
  - 默认等价于 `slam:=true pure_mapping_mode:=true use_rviz:=true`

当前 `slam=false` 的导航 bringup 中，keepout 相关装配是：

- 一个初始为空的 `ComposableNodeContainer`
- 一个常驻 `kfs_keepout_runtime_manager_node`

真正的 keepout 组件由 `rc26_decision` 进入/离开 `MFAreaTree` 时按需装载和卸载。本轮不把 keepout 输出接入 Nav2 costmap。

## 当前关键文件

- [launch/bringup.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/bringup.launch.py)
- [launch/test_navigation.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/test_navigation.launch.py)
- [launch/odometry.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/odometry.launch.py)
- [config/nav2_params.yaml](/home/potato/RC_2026/src/rc26_bringup/config/nav2_params.yaml)
- [rviz/navigation_default.rviz](/home/potato/RC_2026/src/rc26_bringup/rviz/navigation_default.rviz)
- [launch/localization.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/localization.launch.py)
- [config/localization.yaml](/home/potato/RC_2026/src/rc26_bringup/config/localization.yaml)

## 当前边界

- 负责装配、参数选择和生命周期拉起，不承载 planner、控制器或可视化平台的实现本体
- Nav2 参数文件归 bringup 托管，是本轮基础导航栈的部署配置
- 除 `test_mapping.launch.py` 建图调试入口外，如需可视化，应由工作区外部工具只读消费现有 ROS2 输出

## 本轮收口

- 删除旧导航包和旧配置文件引用，导航模式改为 include Nav2 `navigation_launch.py`
- 新增 `config/nav2_params.yaml`，帧固定为 `map / odom / base_link`，控制器使用 Humble 兼容的 DWB 配置并允许麦克纳姆横移速度
- `test_navigation.launch.py` 的验收目标改为 `/navigate_to_pose`、Nav2 lifecycle nodes、costmap/plan topics 与 `/cmd_vel`
- RViz 预设改为观察 Nav2 plan、local/global costmap、TF、RobotModel 和 terrain 输出
- `src/rc26_bringup/map/default.yaml` 仍只是默认占位入口；实机导航验收应通过 `nav2_map_file` 传入有效 2D occupancy map
