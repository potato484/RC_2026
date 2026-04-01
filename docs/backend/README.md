# 后端归档文档索引

`docs/backend/archive/` 现在采用“每个 ROS2 包一个目录”的结构。每个包目录内的 `README.md` 是当前入口文档，后续如果某个包需要继续拆分更细的实现说明、调试指南或专题分析，可以直接放在同一目录下，不再继续把所有内容堆回单个平铺文件。

## 推荐使用方式

- 想快速建立某个包的上下文时，先进入对应目录的 `README.md`。
- 如果后续需要为某个包补充更细的实现解剖、调用链、时序图或故障排查文档，优先继续放在该包目录内。
- 文档如果涉及职责边界、输入输出、权威归属或接口语义变化，仍然需要同步更新相关 README、`docs/fitness/` 和 `docs/middle/` 契约文档。

## 当前专题状态

- [`xhu_direct_current_status_2026-04-01`](xhu_direct_current_status_2026-04-01.md): 记录当前导航架构真实落地状态，以及 2026-04-01 根目录外部参考仓库清理结果。`(file: xhu_direct_current_status_2026-04-01.md)`

## 包目录索引

### 装配与决策

- [`rc26_bringup`](archive/rc26_bringup/README.md): R2 自动机器人整车链路的统一启动与装配包，负责把各个算法包、驱动包、可视化包和测试入口组织起来。`(file: archive/rc26_bringup/README.md)`
- [`rc26_decision`](archive/rc26_decision/README.md): R2 自动机器人当前的主决策包，采用 BehaviorTree.CPP 实现比赛流程级决策。`(file: archive/rc26_decision/README.md)`
- [`rc26_interfaces`](archive/rc26_interfaces/README.md): 整个 R2 仓库的自定义 ROS 2 接口契约包，统一承载消息、服务和动作定义。`(file: archive/rc26_interfaces/README.md)`

### 里程计、定位与点云主链

- [`rc26_mid360_driver`](archive/rc26_mid360_driver/README.md): R2 面向 Livox Mid-360 的专用雷达驱动包，负责直接接收雷达 UDP 数据并发布标准 ROS 2 点云与 IMU。`(file: archive/rc26_mid360_driver/README.md)`
- [`rc26_point_lio`](archive/rc26_point_lio/README.md): R2 当前的 LiDAR-Inertial Odometry 主链路，实现定制版 Point-LIO 建图/里程计。`(file: archive/rc26_point_lio/README.md)`
- [`rc26_lio_state_predictor`](archive/rc26_lio_state_predictor/README.md): 用来解决 LIO 输出相对控制回路存在延迟的问题，把上游里程计前向预测到更接近“当前时刻”的状态。`(file: archive/rc26_lio_state_predictor/README.md)`
- [`rc26_localization`](archive/rc26_localization/README.md): R2 当前的激光重定位主模块，基于仓库内置的 `rc26_small_gicp` 和 GTSAM 进行局部配准与可选图后端优化。`(file: archive/rc26_localization/README.md)`
- [`rc26_merge_odom`](archive/rc26_merge_odom/README.md): R2 当前的多源里程计融合与位姿下发包，负责把轮速、CAN 里程计、达妙 IMU 和控制下发整合到同一条运行链路里。`(file: archive/rc26_merge_odom/README.md)`
- [`rc26_odom_interface`](archive/rc26_odom_interface/README.md): Point-LIO 与 Nav2/底盘坐标系之间的标准化接口层，负责把上游里程计结果转换成下游统一消费的底盘里程计和 TF。`(file: archive/rc26_odom_interface/README.md)`
- [`rc26_sensor_scan`](archive/rc26_sensor_scan/README.md): 点云与里程计的时空对齐模块，用来给下游局部感知提供“已经同步并投影到传感器视角”的干净输入。`(file: archive/rc26_sensor_scan/README.md)`
- [`rc26_small_gicp`](archive/rc26_small_gicp/README.md): 仓库内置的点云配准基础库，被 `rc26_localization` 直接以内嵌源码方式依赖。`(file: archive/rc26_small_gicp/README.md)`

### 控制与执行

- [`rc26_topo_nav`](archive/rc26_topo_nav/README.md): R2 比赛特化拓扑导航中层；当前支持 `nav2_follow_path` 与 `xhu_direct` 双执行后端，负责 route/corridor 产出和 `/xhu_nav/*` 运行时可观测输出。`(file: archive/rc26_topo_nav/README.md)`
- [`rc26_mechanism`](archive/rc26_mechanism/README.md): R2 的机构执行与生命周期管理模块，负责把上层动作语义可靠地下发给下位机，并回传状态。`(file: archive/rc26_mechanism/README.md)`
- [`rc26_nav_mode_manager`](archive/rc26_nav_mode_manager/README.md): R2 的导航/运动模式管理宿主包；同时承载 legacy `SetNavMode` 管理器和 `xhu_direct` 运行时 `xhu_motion_mode_manager`。`(file: archive/rc26_nav_mode_manager/README.md)`
- [`rc26_nmpc_controller`](archive/rc26_nmpc_controller/README.md): 挂载到 Nav2 `controller_server` 上的定位感知型 NMPC 控制器插件。`(file: archive/rc26_nmpc_controller/README.md)`
- [`rc26_omni_controller`](archive/rc26_omni_controller/README.md): R2 麦克纳姆底盘控制宿主包；保留 Nav2 控制器插件，同时新增 `xhu_motion_follower` 独立节点用于 `xhu_direct` 直连跟踪。`(file: archive/rc26_omni_controller/README.md)`
- [`rc26_telecontrol`](archive/rc26_telecontrol/README.md): R2 的人工遥控测试包，用来在调试和联调阶段通过手柄向底盘发送速度指令。`(file: archive/rc26_telecontrol/README.md)`
- [`rc26_serial`](archive/rc26_serial/README.md): 整个 R2 仓库的串口通信基础库，给机构控制、底盘下发和其他串口链路复用。`(file: archive/rc26_serial/README.md)`

### 地形与规则安全

- [`rc26_base_ground`](archive/rc26_base_ground/README.md): R2 的基础标高与离散层级估计模块，用于把连续高度变化压成导航和机构更容易消费的地形层级语义。`(file: archive/rc26_base_ground/README.md)`
- [`rc26_kfs_keepout`](archive/rc26_kfs_keepout/README.md): R2 在梅林区等场景下的动态 Keepout 生成模块，负责把 KFS 状态融合成 Nav2 可消费的禁行掩码。`(file: archive/rc26_kfs_keepout/README.md)`
- [`rc26_terrain`](archive/rc26_terrain/README.md): R2 当前的地形感知与语义栅格生成包，用来把点云转换成导航可消费的地形风险结果。`(file: archive/rc26_terrain/README.md)`
- [`rc26_terrain_nav2`](archive/rc26_terrain_nav2/README.md): `rc26_terrain` 与 Nav2 之间的适配层，负责把地形结果接到 costmap 和速度限制链路中。`(file: archive/rc26_terrain_nav2/README.md)`

### 感知与可视化

- [`rc26_vision`](archive/rc26_vision/README.md): R2 当前的视觉推理与弹头定位包，负责模型装载、图像推理、深度融合和目标三维定位。`(file: archive/rc26_vision/README.md)`
- [`rc26_visualization`](archive/rc26_visualization/README.md): R2 的状态聚合与运维诊断包，负责把定位、控制、地形、机构、导航安全等多路状态压成统一的可视化语义输出。`(file: archive/rc26_visualization/README.md)`
