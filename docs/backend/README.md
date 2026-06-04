# 后端归档文档索引

`docs/backend/archive/` 采用“按当前 ROS2 包保留入口文档，必要时补充少量历史资料”的结构。

当前调试文档口径统一收口到仓库根目录 `调试/`：

- `调试/建图启动.md`、`调试/定位启动.md`、`调试/重定位启动.md`、`调试/回环启动.md`、`调试/导航启动.md`、`调试/决策启动.md`、`调试/感知启动.md`、`调试/遥控启动.md` 与 `调试/联调顺序.md` 作为按场景入口
- `调试/rc26_*调试.md` 作为按包调试入口

## 当前导航口径

R2 运行时导航权威已经迁移到 Nav2。`rc26_bringup` 在 `slam=false` 时装配 `map_server + nav2_bringup/navigation_launch.py`，`/cmd_vel` 由 Nav2 controller/velocity_smoother 输出。

`rc26_decision` 不再发送自定义导航 action，而是在 BT XML 中显式写 Nav2 pose 目标，通过 `NavToPose` 节点调用 `/navigate_to_pose`。

本轮是基础可运行迁移：`rc26_terrain` 与 `rc26_kfs_keepout` 仍保留自己的输出和决策支撑职责，但暂不接入 Nav2 costmap filter 或 obstacle layer。

## 当前 ROS2 自动验证入口

- `.github/workflows/ros2-workspace-ci.yml`：当前 ROS2 工作区的 GitHub Actions smoke CI。默认先构建 `rc26_bringup` 的本地运行依赖闭包、`rc26_decision` 和基础遥控/视觉包，再只测试 smoke 目标包，避免历史依赖包 lint 阻塞导航装配验证。
- `scripts/ci/run-ros2-workspace-smoke.sh`：本地与 CI 复用的 smoke 验证脚本，统一使用仓库规定的 `MAKEFLAGS='-j2 -l2'`、`--executor sequential` 与 `--parallel-workers 1` 口径；无参数时按默认构建闭包/测试目标执行，显式传包时对传入包同时 build/test。
- `src/` 当前不提供通用 GitHub CD 部署流程。机器人运行时仍以犀牛派 X1 / AidLux 实机环境和 `rc26_bringup` 装配入口为准。

## 当前 IDE 索引入口

- `scripts/dev/refresh-compile-commands.sh`：当前工作区统一的 C/C++ 编译数据库刷新入口。
- 仓库内可追踪的 VS Code C/C++ 配置当前以 `.vscode/c_cpp_properties.json` 为入口，统一指向仓库根目录 `compile_commands.json`。
- 如果清理过 `build/ install/`，或新增 / 重命名了 C++ 源文件与包，需重新执行一次该脚本，避免跳转仍落在过期声明或直接跳转失败。

## 包目录索引

### 装配与决策

- [`rc26_bringup`](archive/rc26_bringup/README.md): R2 整车链路统一装配入口；导航模式下装配定位、terrain、keepout runtime、Nav2 基础导航栈和决策。`(file: archive/rc26_bringup/README.md)`
- [`rc26_decision`](archive/rc26_decision/README.md): R2 主决策包；梅林区行为树通过 `NavToPose` 调用 Nav2 `/navigate_to_pose`。`(file: archive/rc26_decision/README.md)`
- [`rc26_interfaces`](archive/rc26_interfaces/README.md): R2 自定义 ROS 2 接口契约包；当前保留行为树、定位、机构、terrain 和 MF keepout 支撑接口。`(file: archive/rc26_interfaces/README.md)`

### 里程计、定位与点云主链

- [`rc26_mid360_driver`](archive/rc26_mid360_driver/README.md): Livox Mid-360 驱动。`(file: archive/rc26_mid360_driver/README.md)`
- [`rc26_sensor_extrinsics`](archive/rc26_sensor_extrinsics/README.md): R2 静态传感器安装外参 YAML 真源。`(file: archive/rc26_sensor_extrinsics/README.md)`
- [`rc26_point_lio`](archive/rc26_point_lio/README.md): LiDAR-Inertial Odometry 主链。`(file: archive/rc26_point_lio/README.md)`
- [`rc26_localization`](archive/rc26_localization/README.md): 激光重定位主模块，继续作为 `map -> odom` 权威。`(file: archive/rc26_localization/README.md)`
- [`rc26_merge_odom`](archive/rc26_merge_odom/README.md): 多源里程计融合与位姿下发。`(file: archive/rc26_merge_odom/README.md)`
- [`rc26_odom_interface`](archive/rc26_odom_interface/README.md): 上游里程计到下游统一底盘坐标系的接口层。`(file: archive/rc26_odom_interface/README.md)`
- [`rc26_sensor_scan`](archive/rc26_sensor_scan/README.md): 点云与里程计时空对齐模块。`(file: archive/rc26_sensor_scan/README.md)`
- [`rc26_small_gicp`](archive/rc26_small_gicp/README.md): 点云配准基础库。`(file: archive/rc26_small_gicp/README.md)`

### 控制与执行

- [`rc26_robot_geometry`](archive/rc26_robot_geometry/README.md): R2 机器人车体轮廓与安全包络共享配置真源。`(file: archive/rc26_robot_geometry/README.md)`
- [`rc26_mechanism`](archive/rc26_mechanism/README.md): 机构执行与生命周期管理。`(file: archive/rc26_mechanism/README.md)`
- [`rc26_telecontrol`](archive/rc26_telecontrol/README.md): 人工遥控测试包。`(file: archive/rc26_telecontrol/README.md)`
- [`rc26_serial`](archive/rc26_serial/README.md): 串口通信基础库。`(file: archive/rc26_serial/README.md)`

### 地形与规则安全

- [`rc26_base_ground`](archive/rc26_base_ground/README.md): 基础标高与离散层级估计。`(file: archive/rc26_base_ground/README.md)`
- [`rc26_kfs_keepout`](archive/rc26_kfs_keepout/README.md): KFS keepout 融合模块；当前面向 `/mf_block_overlay`、`/kfs_filter_mask` 和决策侧 keepout gate。`(file: archive/rc26_kfs_keepout/README.md)`
- [`rc26_terrain`](archive/rc26_terrain/README.md): 地形感知与语义栅格生成包。`(file: archive/rc26_terrain/README.md)`

### 感知与可视化

- [`rc26_vision`](archive/rc26_vision/README.md): 视觉推理与端头定位。`(file: archive/rc26_vision/README.md)`
