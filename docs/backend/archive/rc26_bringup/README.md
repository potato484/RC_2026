# rc26_bringup

## 模块定位

`rc26_bringup` 是 R2 整车链路的统一装配入口，负责决定哪些运行时包被拉起，但不拥有这些包内部算法真源。

## 当前装配口径

`bringup.launch.py` 与 `odometry.launch.py` 固定按 headless 口径装配，不再声明或透传任何 viewer / RViz / Foxglove 兼容参数。面向现场建图的 `test_mapping.launch.py` 是调试入口，默认复用纯建图链路并额外打开 RViz2，可用 `use_rviz:=false` 关闭图形界面。

- `bringup.launch.py`
  - 装配 Point-LIO、里程计接口、定位、Nav2 基础导航栈、`pose_sender_node` 执行桥和决策
  - `slam=false` 时启动 `map_server` 与 Nav2 `navigation_launch.py`
  - `rc26_localization` 继续作为 `map -> odom` 权威，不启动 AMCL
  - 定位装配只透传先验 PCD 和定位参数文件，不再透传图后端、P4、重试区或 overlay 参数
  - Nav2 默认直接消费 `/sensor_scan` (`PointCloud2`) 作为 obstacle layer 输入，不再依赖 `/scan`
  - `/cmd_vel` 由 Nav2 controller/velocity_smoother 输出，并默认交给 `rc26_merge_odom/pose_sender_node` 下发到底盘目标 MCU
- `odometry.launch.py`
  - 装配 Point-LIO、`rc26_odom_interface`、`rc26_sensor_scan`
  - 读取 `rc26_sensor_extrinsics` 的 YAML profile 发布 `base_link -> livox_frame` 等对外静态 TF
  - 推导 `base_link -> point_lio.body_frame` 内部外参并注入 `rc26_odom_interface`，不再把这条内部边发布到 TF 树
- `test_navigation.launch.py`
  - 复用整车 bringup，默认 `use_decision=false`
  - 用于定位 + Nav2 基础导航联调；默认也会拉起 `pose_sender_node`
- `test_mapping.launch.py`
  - 默认等价于 `slam:=true pure_mapping_mode:=true use_rviz:=true`
  - `slam.rviz` 默认观察 `/point_lio/map_cloud` 完整累计地图、`/registered_scan` 实时点云与 `/Laser_map` 初始地图

`rc26_terrain`、`rc26_base_ground` 与 `rc26_kfs_keepout` 已归档退出当前装配。`bringup.launch.py`、`odometry.launch.py` 和 `test_mapping.launch.py` 不再声明 terrain/base-ground/keepout 参数，不再启动这些节点，也不再向 `rc26_decision` 透传 keepout heartbeat 或 runtime service 配置。

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
- `config/localization.yaml` 当前只维护开局一次重定位与连续 GICP 跟踪的核心参数，定位健康度、图后端和路径可观测性参数已移除
- `config/odom_interface.yaml` 只保留 `rc26_odom_interface` 的参数契约；`base_link -> point_lio.body_frame` 推导值只允许在 `odometry.launch.py` 中按真源配置注入
- `config/nav2_params.yaml` 当前按 `sensor_scan` (`PointCloud2`) 作为 Nav2 默认障碍输入维护，不再维护 `/scan` LaserScan 兼容链
- 除 `test_mapping.launch.py` 建图调试入口外，如需可视化，应由工作区外部工具只读消费当前主链 ROS2 输出；`/point_lio/map_cloud` 是现场建图观察输出，不作为定位或导航权威

## 本轮收口

- 删除旧导航包和旧配置文件引用，导航模式改为 include Nav2 `navigation_launch.py`
- 新增 `config/nav2_params.yaml`，帧固定为 `map / odom / base_link`，控制器使用 Humble 兼容的 DWB 配置并允许麦克纳姆横移速度
- Nav2 obstacle layer 当前默认直接消费 `/sensor_scan` (`PointCloud2`)；`registered_scan` 继续只供定位链使用
- `bringup.launch.py` 与 `test_navigation.launch.py` 新增 `start_pose_sender`、`pose_sender_feedback_serial_port`、`pose_sender_target_serial_port` 与 `pose_sender_baudrate`，默认把 `/cmd_vel` 接到 `pose_sender_node`
- `test_navigation.launch.py` 的验收目标改为 `/navigate_to_pose`、`sensor_scan`、Nav2 lifecycle nodes、costmap/plan topics、`/cmd_vel` 与 `pose_sender_node`
- `config/nav2_params.yaml` 现已补齐主要字段的中文注释，现场调试应优先以该文件中的分区说明和速度/代价图参数注释为准
- RViz 预设只观察 Nav2 plan、local/global costmap、TF、RobotModel 与点云/里程计主链，不再订阅 terrain 或 keepout 输出
- `src/rc26_bringup/map/default.yaml` 仍只是默认占位入口；它只用于把 Nav2 lifecycle、costmap 和执行桥链路拉起，实机规划验收仍应通过 `nav2_map_file` 传入有效 2D occupancy map
- `odometry.launch.py` 新增 `start_point_lio`、`start_sensor_scan` 开关，供 `test_odom_interface.launch.py`、`odometry_mock.launch.py` 等入口复用同一套静态 TF / 内部外参装配
- `odometry_mock.launch.py` 复用同一套 `odometry.launch.py` 装配；`mock_point_lio.py` 默认先输出一小段静止里程计，再进入运动阶段，以满足 `rc26_odom_interface` 现有的启动静止归零约束
- 若现场只有目标 MCU 下发串口，应显式传 `pose_sender_feedback_serial_port:=__disabled__`，避免 bringup 因缺少反馈串口而报双串口初始化异常
- 本轮归档 `rc26_terrain`、`rc26_base_ground` 与 `rc26_kfs_keepout`：主启动、建图调试、验收探针、RViz 预设和 `package.xml` 均不再接入这些包
