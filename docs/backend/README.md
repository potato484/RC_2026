# 后端归档文档索引

`docs/backend/archive/` 采用“按当前 ROS2 包保留入口文档，必要时补充少量历史资料”的结构。

当前调试文档口径也已经收口：

- 各包原先散落在包内 `docs` 目录中的调试文档已删除
- 当前统一改为仓库根目录 `调试/rc26_*调试.md` 作为按包调试入口
- `调试/建图启动.md`、`调试/定位启动.md`、`调试/重定位启动.md`、`调试/回环启动.md`、`调试/导航启动.md`、`调试/决策启动.md`、`调试/感知启动.md`、`调试/遥控启动.md` 与 `调试/联调顺序.md` 作为按场景入口

当前 3D 导航文档已经收口：

- `rc26_topo_nav`、`rc26_surface_body_planner`、`rc26_local_3d_planner`、`rc26_nav_mode_manager`、`rc26_omni_controller` 的独立归档页已删除
- 当前导航实现统一查看 [`rc26_xhu_nav`](archive/rc26_xhu_nav/README.md)
- 已经落地的架构变更直接归档在对应包 README 中，这里不再单独维护长期变更流水账入口

## 当前 ROS2 自动验证入口

- `.github/workflows/ros2-workspace-ci.yml`：当前 ROS2 工作区的 GitHub Actions smoke CI。固定覆盖 `rc26_interfaces`、`rc26_robot_geometry`、`rc26_serial`、`rc26_telecontrol`、`rc26_xhu_nav` 这条 headless 主链，执行 `colcon build + colcon test`。
- `scripts/ci/run-ros2-workspace-smoke.sh`：本地与 CI 复用的 smoke 验证脚本，统一使用仓库规定的 `MAKEFLAGS='-j2 -l2'`、`--executor sequential` 与 `--parallel-workers 1` 口径。
- `src/` 当前不提供通用 GitHub CD 部署流程。机器人运行时仍以 QCS8550 / AidLux 实机环境和 `rc26_bringup` 装配入口为准，不能把仓库 workflow 伪装成一条通用云部署链。

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

- [`rc26_xhu_nav`](archive/rc26_xhu_nav/README.md): R2 当前唯一的 3D 导航实现宿主包；统一承载 topo/surface graph、body-aware planner、local planner、mode manager 与 runtime executor。`(file: archive/rc26_xhu_nav/README.md)`
- [`rc26_robot_geometry`](archive/rc26_robot_geometry/README.md): R2 机器人车体轮廓与安全包络共享配置真源；当前通过参数契约供 `rc26_xhu_nav` 消费。`(file: archive/rc26_robot_geometry/README.md)`
- [`rc26_mechanism`](archive/rc26_mechanism/README.md): 机构执行与生命周期管理。`(file: archive/rc26_mechanism/README.md)`
- [`rc26_telecontrol`](archive/rc26_telecontrol/README.md): 人工遥控测试包。`(file: archive/rc26_telecontrol/README.md)`
- [`rc26_serial`](archive/rc26_serial/README.md): 串口通信基础库。`(file: archive/rc26_serial/README.md)`

### 地形与规则安全

- [`rc26_base_ground`](archive/rc26_base_ground/README.md): 基础标高与离散层级估计。`(file: archive/rc26_base_ground/README.md)`
- [`rc26_kfs_keepout`](archive/rc26_kfs_keepout/README.md): KFS keepout 融合模块；当前面向 `/mf_block_overlay` 与 `/kfs_filter_mask`。`(file: archive/rc26_kfs_keepout/README.md)`
- [`rc26_terrain`](archive/rc26_terrain/README.md): 地形感知与语义栅格生成包。`(file: archive/rc26_terrain/README.md)`

### 感知与可视化

- [`rc26_vision`](archive/rc26_vision/README.md): 视觉推理与弹头定位。`(file: archive/rc26_vision/README.md)`
