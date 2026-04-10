# 后端归档文档索引

`docs/backend/archive/` 采用“每个 ROS2 包一个目录”的结构。各目录下的 `README.md` 是当前真实实现的入口文档。

## 包目录索引

### 装配与决策

- [`rc26_bringup`](archive/rc26_bringup/README.md): R2 整车链路统一装配入口；导航模式下固定装配 topo/xhu 自研导航链。`(file: archive/rc26_bringup/README.md)`
- [`rc26_decision`](archive/rc26_decision/README.md): R2 主决策包；通过 topo/xhu 导航链发起导航调用。`(file: archive/rc26_decision/README.md)`
- [`rc26_interfaces`](archive/rc26_interfaces/README.md): R2 自定义 ROS 2 接口契约包；覆盖 topo/xhu 与决策运行时接口，当前包含 `NavigateTopoTarget` 与 `NavigateSurfaceRoute` 两条导航 action。`(file: archive/rc26_interfaces/README.md)`

### 里程计、定位与点云主链

- [`rc26_mid360_driver`](archive/rc26_mid360_driver/README.md): Livox Mid-360 驱动。`(file: archive/rc26_mid360_driver/README.md)`
- [`rc26_point_lio`](archive/rc26_point_lio/README.md): LiDAR-Inertial Odometry 主链。`(file: archive/rc26_point_lio/README.md)`
- [`rc26_lio_state_predictor`](archive/rc26_lio_state_predictor/README.md): LIO 前向预测链。`(file: archive/rc26_lio_state_predictor/README.md)`
- [`rc26_localization`](archive/rc26_localization/README.md): 激光重定位主模块。`(file: archive/rc26_localization/README.md)`
- [`rc26_merge_odom`](archive/rc26_merge_odom/README.md): 多源里程计融合与位姿下发。`(file: archive/rc26_merge_odom/README.md)`
- [`rc26_odom_interface`](archive/rc26_odom_interface/README.md): 上游里程计到下游统一底盘坐标系的接口层。`(file: archive/rc26_odom_interface/README.md)`
- [`rc26_sensor_scan`](archive/rc26_sensor_scan/README.md): 点云与里程计时空对齐模块。`(file: archive/rc26_sensor_scan/README.md)`
- [`rc26_small_gicp`](archive/rc26_small_gicp/README.md): 点云配准基础库。`(file: archive/rc26_small_gicp/README.md)`

### 控制与执行

- [`rc26_topo_nav`](archive/rc26_topo_nav/README.md): 拓扑导航表达与单边执行器；当前同时支持 topo 节点目标和基于 dense `surface_graph` 的任意点 3D 路线，并统一通过 `set_xhu_motion_mode + /xhu_nav/corridor_cmd` 驱动执行。`(file: archive/rc26_topo_nav/README.md)`
- [`rc26_surface_body_planner`](archive/rc26_surface_body_planner/README.md): 独立的 heading-aware surface body planner 库；当前由 `rc26_topo_nav` 作为 `navigate_surface_route` 的可选后端调用。`(file: archive/rc26_surface_body_planner/README.md)`
- [`rc26_nav_mode_manager`](archive/rc26_nav_mode_manager/README.md): 自研导航运动模式管理器；提供 `set_xhu_motion_mode` 与 `/xhu_nav/motion_mode_state` 主线。`(file: archive/rc26_nav_mode_manager/README.md)`
- [`rc26_local_3d_planner`](archive/rc26_local_3d_planner/README.md): 可复用的局部 3D 规划 core 与 observe-only 观测节点；向执行器暴露局部评分、preview 与 recovery 状态。`(file: archive/rc26_local_3d_planner/README.md)`
- [`rc26_omni_controller`](archive/rc26_omni_controller/README.md): 自研走廊跟踪执行器宿主包；当前同时承载旧 `xhu_motion_follower_node` 和基于 `rc26_local_3d_planner` 的 `xhu_motion_runtime_node`。`(file: archive/rc26_omni_controller/README.md)`
- [`rc26_robot_geometry`](archive/rc26_robot_geometry/README.md): R2 机器人车体轮廓与安全包络共享配置真源；当前通过参数契约供 `rc26_topo_nav` 和 `rc26_omni_controller` 消费。`(file: archive/rc26_robot_geometry/README.md)`
- [`rc26_mechanism`](archive/rc26_mechanism/README.md): 机构执行与生命周期管理。`(file: archive/rc26_mechanism/README.md)`
- [`rc26_telecontrol`](archive/rc26_telecontrol/README.md): 人工遥控测试包。`(file: archive/rc26_telecontrol/README.md)`
- [`rc26_serial`](archive/rc26_serial/README.md): 串口通信基础库。`(file: archive/rc26_serial/README.md)`

### 地形与规则安全

- [`rc26_base_ground`](archive/rc26_base_ground/README.md): 基础标高与离散层级估计。`(file: archive/rc26_base_ground/README.md)`
- [`rc26_kfs_keepout`](archive/rc26_kfs_keepout/README.md): KFS keepout 融合模块；当前面向 `/mf_block_overlay` 与 `/kfs_filter_mask`。`(file: archive/rc26_kfs_keepout/README.md)`
- [`rc26_terrain`](archive/rc26_terrain/README.md): 地形感知与语义栅格生成包。`(file: archive/rc26_terrain/README.md)`

### 感知与可视化

- [`rc26_vision`](archive/rc26_vision/README.md): 视觉推理与弹头定位。`(file: archive/rc26_vision/README.md)`

### 历史可视化目录

- [`rc26_xhu_viewer`](archive/rc26_xhu_viewer/README.md): `src/rc26_xhu_viewer/` 已于 2026-04-10 整体从工作区删除；该目录下文档仅保留历史说明。`(file: archive/rc26_xhu_viewer/README.md)`
