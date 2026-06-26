# rc26_bringup

## 模块定位

`rc26_bringup` 是 R2 整车链路的统一装配入口，负责决定哪些运行时包被拉起，但不拥有这些包内部算法真源。

## 当前装配口径

`bringup.launch.py` 与 `odometry.launch.py` 固定按 headless 口径装配，不再声明或透传任何 viewer / RViz / Foxglove 兼容参数。面向现场联调的 `test_mapping.launch.py` 与 `test_navigation.launch.py` 是调试入口，默认复用对应链路并额外打开 RViz2，可用 `use_rviz:=false` 关闭图形界面，也可用 `rviz_config_file:=...` 替换 RViz 配置。

- `bringup.launch.py`
  - 装配 Point-LIO、里程计接口、定位、Nav2 基础导航栈和决策
  - `run_mode:=navigation` 时启动 `rc26_localization`、`map_server` 与 Nav2 `navigation_launch.py`
  - `run_mode:=mapping` 时进入建图链路，跳过 Nav2 导航分支，定位侧发布静态 `map -> odom`
  - `rc26_localization` 继续作为 `map -> odom` 权威，不启动 AMCL
  - 定位装配只透传先验 PCD 和定位参数文件，不再透传图后端、P4、重试区或 overlay 参数
  - 当前默认运行时关闭 local/global costmap 的 obstacle layer；`/sensor_scan` (`PointCloud2`) 链路仍保留，且不再依赖 `/scan`
  - `/cmd_vel` 由 Nav2 controller/velocity_smoother 输出；默认由 `rc26_mcu_transport` 消费并下发 `POSE_TARGET(0x0C)`
  - `rc26_bringup` 不再启动 `rc26_merge_odom` 或 `pose_sender_node`，也不再提供 `/merge_odom`
  - `/mechanism/send_command` 与 `/mechanism/command_feedback` 由 `rc26_mcu_transport` 提供；涉及机构动作的运行链必须启动该目标 MCU 串口 owner
  - `config/r2_runtime.yaml` 是整车运行配置真源：集中维护点云文件、Nav2 地图文件、行为树 XML 绝对路径与决策参数
  - `r2_runtime.decision.ros__parameters` 中的 `mc_nav_*` 参数是当前红方武馆区行为树去程 `NavToPose` 目标真源；`mc_tree.xml` 从黑板读取这些值，不再在 XML 中写死取端头导航点。`mc_nav_behavior_tree_file` 当前指向 `config/nav2_bt_mc_red_positive_xy.xml`，让去程 Nav2 goal 显式使用红方 MC 正向 controller。MC 树完成视觉夹取和原地旋转后直接进入 `WaitForever`，当前不再维护固定返程目标，也不再声明 `mc_return_nav_*` 参数
  - `r2_runtime.decision.ros__parameters` 中的 `mc_align_*` 参数维护武馆区视觉伺服横移对线口径；当前默认按后置相机安装反转横移方向，并启用端头目标锁定以避免双框同屏切换。`mc_align_heading_*` 当前默认开启，通过 `mc_odom_topic=odom` 消费 `rc26_odom_interface` 的雷达标准 odom yaw，并在视觉伺服阶段补发 `cmd_vel.angular.z` 保持车身朝向
  - 完整 MC 决策链依赖 `/odom` 作为雷达标准里程计，服务视觉 heading hold 和 180° 旋转 yaw 闭环；底盘执行与机构 transport 均由 `rc26_mcu_transport` 承担
  - `r2_runtime.decision.ros__parameters` 中声明为 double 的参数必须在 YAML 中写成小数形式，例如 `10.0`，不要写成裸整数 `10`；ROS2 会区分 integer 和 double，类型不一致会导致 `decision_node` 启动时报 `InvalidParameterTypeException`
  - `config/nav2_params.yaml` 中 `planner_server.GridBased.tolerance` 当前收紧为 `0.05m`，避免 NavFn 在目标点附近半米内选替代终点后仍被上层当作真实到点；`controller_server.progress_checker` 当前按低速实车调为 `20s` 内至少前进 `0.05m`，减少慢速起步或局部恢复期间的误判卡死。普通导航继续使用 `FollowPath`，红方 MC 去程专用 Nav2 BT 使用 `MCPositiveXYRed`，只采样车体系 `linear.x/y` 正向和角速度。`MCNegativeXYRed` 仍作为可手工选择的负向约束 controller 配置保留，但当前默认 MC 树不再引用它。梅林预选 2 号入口导航专用 Nav2 BT 使用 `MFPreselectEntryXPositiveYNegative`，只采样车体系 `linear.x >= 0`、`linear.y <= 0`、`|angular.z| <= 0.10rad/s`，并且 recovery 只做 costmap clear，避免恢复动作绕过方向约束
  - `r2_runtime.decision.ros__parameters` 同时维护 `stair_*` 台阶动作参数；这些参数供独立 `stair_climb_tree.xml` / `stair_descend_tree.xml`、红方中列测试树和 MF `GridTransition` 离散格间动作复用。普通直行速度为便于上/下阶梯独立调速，已拆为 `stair_climb_drive_speed_mps` 与 `stair_descend_drive_speed_mps`，旧 `stair_drive_speed_mps` 不再是当前配置契约；上台阶的前推杆伸出后零速等待、前收+后伸后零速等待、后推杆收回后零速等待，以及下台阶的后推杆伸出后零速等待、后收+前伸后零速等待、前推杆收回后零速等待也在这里配置；当前两激光下台阶链路还在这里配置前推杆收回前的 `x` 负向定时行驶速度与时长，默认 `0.025m/s` 持续 `4.0s`。`stair_odom_topic` 与 `stair_heading_*` 参数用于上/下阶梯前的 yaw 对齐和直行期间 heading hold
  - `r2_runtime.decision.ros__parameters` 中的 `kfs_*` 参数维护 KFS 阶梯等待测试链：上/下阶梯机械臂预调命令与完成反馈、`R_R1/B_R1` 阻塞标签、深度有效范围、出现/消失稳定帧、等待超时和 topic/service 名。当前 KFS v1 只等待其它机器人目标消失，不会下发预留的 `GRAB_KFS_UP/DOWN`
  - `r2_runtime.decision.ros__parameters` 中的 `mf_preselect_*` 参数维护梅林区预选赛专属正式链路：2 号入口可选 Nav2 目标、R2/R1/假 KFS 标签、深度阈值、入口 1/3 阶梯横移探测距离和速度、R2 KFS 夹取上限、`ARM_HIGH_RAISE(0x0D)` / `ARM_HIGH_RAISE_DONE(0x09)`、普通机械臂升降、相对移动、转向和最终离场停车参数。当前默认 `behavior_tree_file` 指向 `mf_preselection_tree.xml`，入口导航默认关闭，现场标定后只需启用并填写 `mf_preselect_entry2_nav_*`；入口导航的 `mf_preselect_entry2_nav_behavior_tree_file` 默认指向 `config/nav2_bt_mf_preselect_entry_x_positive_y_negative.xml`
  - 决策测试不再通过 `rc26_decision` 单节点 launch 入口进行；需要测试决策时使用本完整 bringup 入口，并确认 `rc26_mcu_transport` 已按 `r2_runtime.mcu_transport` 启动
  - `r2_runtime.chassis_runtime.merge_odom` 已删除；默认 bringup 不再读取或透传 `merge_odom_*`、`start_pose_sender`、`pose_sender_*` 参数
  - 自动导航主链的 `/odom` 仍由 `rc26_odom_interface` 提供给 Nav2；`/merge_odom` 不再是当前默认运行时话题
- `odometry.launch.py`
  - 装配 Point-LIO、`rc26_odom_interface`、`rc26_sensor_scan`
  - 读取 `rc26_sensor_extrinsics` 的 YAML profile 发布 `base_link -> livox_frame` 等对外静态 TF
  - 直接从 `config/odom_interface.yaml` 读取 `base_link_height_above_base_footprint_m`
  - 推导 `base_link -> point_lio.body_frame` 内部外参并注入 `rc26_odom_interface`，不再把这条内部边发布到 TF 树
  - 自动导航链当前发布 `odom -> base_footprint -> base_link -> livox_frame`
- `test_navigation.launch.py`
  - 复用整车 bringup，默认 `use_decision=false`
  - 用于定位 + Nav2 基础导航联调；默认不启动 `rc26_mcu_transport`，只验证 Nav2 输出 `/cmd_vel` 与定位/地图/点云观察链
  - 默认 `use_rviz=true`，加载 `rviz/navigation_default.rviz` 观察地图、costmap、路径、定位、里程计与点云主链
- `kfs_stair_test.launch.py`
  - 独立 KFS 阶梯等待测试入口，按 `direction:=climb|descend` 选择 `kfs_stair_climb_test_tree.xml` 或 `kfs_stair_descend_test_tree.xml`
  - 默认拉起 odometry、RealSense D455、`rc26_mcu_transport` 与 `decision_node`；可通过 `start_odometry`、`use_realsense`、`start_mcu_transport` 开关裁剪硬件链路
  - `camera_namespace` 默认空字符串，保持 RealSense 发布 `/camera/color/image_raw`、`/camera/aligned_depth_to_color/image_raw` 与 `/camera/color/camera_info`，匹配 KFS 视觉默认订阅；顶级 ROS `namespace` 不会自动改相机 topic
  - `use_kfs_vision_ui:=false` 默认关闭摄像头 UI；开启后额外启动 `rc26_vision/kfs_vision_test_node` 作为只读 OpenCV overlay，复用同一组 RealSense topic，不重复启动相机。该 UI 会按 `kfs_model_id` 与 `kfs_depth_min_m/max_m` 显示检测框、标签特征色、置信度和中心 ROI 深度有效性；开启时会额外跑一份 KFS 推理，只用于现场观察，不发布 `/cmd_vel`，也不调用机构 service
  - 不启动 Nav2 controller、定位导航栈或遥控节点，确保 `/cmd_vel` 只由 KFS 等待/台阶 BT 与 `rc26_mcu_transport` 这一条测试链消费/执行
- `scripts/capture_nav_points.py`
  - 现场 Nav2 导航点采集工具；定位链和 `odom_interface` 已发布 `map -> odom -> base_footprint` 后，人工遥控到目标位置并在终端按 `Enter` 即记录当前 `map -> base_footprint` 的 `x/y/yaw`
  - 脚本只读 TF，不发布 `/cmd_vel`，不修改定位，不调用 Nav2 action；输出 `.txt` 中同时包含点位表和可复制进 `rc26_decision` 行为树的 `<NavToPose .../>` 片段
- `test_mapping.launch.py`
  - 默认等价于 `run_mode:=mapping pure_mapping_mode:=true use_rviz:=true`
  - `slam.rviz` 默认观察 `/point_lio/map_cloud` 完整累计地图、`/registered_scan` 实时点云与 `/Laser_map` 初始地图

旧地形、base-ground 与 MF keepout 包及相关兼容接口已从工作区删除。`bringup.launch.py`、`odometry.launch.py` 和 `test_mapping.launch.py` 不声明 terrain/base-ground/keepout 参数，不启动这些节点，也不向 `rc26_decision` 透传旧 keepout heartbeat 或 runtime service 配置。

## 当前关键文件

- [launch/bringup.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/bringup.launch.py)
- [launch/test_navigation.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/test_navigation.launch.py)
- [launch/odometry.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/odometry.launch.py)
- [launch/kfs_stair_test.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/kfs_stair_test.launch.py)
- [config/r2_runtime.yaml](/home/potato/RC_2026/src/rc26_bringup/config/r2_runtime.yaml)
- [config/nav2_params.yaml](/home/potato/RC_2026/src/rc26_bringup/config/nav2_params.yaml)
- [config/nav2_bt_mc_red_positive_xy.xml](/home/potato/RC_2026/src/rc26_bringup/config/nav2_bt_mc_red_positive_xy.xml)
- [config/nav2_bt_mf_preselect_entry_x_positive_y_negative.xml](/home/potato/RC_2026/src/rc26_bringup/config/nav2_bt_mf_preselect_entry_x_positive_y_negative.xml)
- [rviz/navigation_default.rviz](/home/potato/RC_2026/src/rc26_bringup/rviz/navigation_default.rviz)
- [launch/localization.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/localization.launch.py)
- [config/localization.yaml](/home/potato/RC_2026/src/rc26_bringup/config/localization.yaml)
- [scripts/capture_nav_points.py](/home/potato/RC_2026/src/rc26_bringup/scripts/capture_nav_points.py)

## 当前边界

- 负责装配、参数选择和生命周期拉起，不承载 planner、控制器或可视化平台的实现本体
- Nav2 参数文件归 bringup 托管，是本轮基础导航栈的部署配置
- `config/localization.yaml` 当前只维护开局一次重定位与连续 GICP 跟踪的核心参数，定位健康度、图后端和路径可观测性参数已移除
- `config/odom_interface.yaml` 只保留 `rc26_odom_interface` 的参数契约；其中 `base_link_height_above_base_footprint_m` 当前直接在该 YAML 维护，`base_link -> point_lio.body_frame` 推导值仍只允许在 `odometry.launch.py` 中注入
- `config/nav2_params.yaml` 当前保留 `sensor_scan` (`PointCloud2`) 到 obstacle layer 的参数块，但默认关闭 local/global obstacle layer，不再维护 `/scan` LaserScan 兼容链；该文件的实际 YAML 配置项已经逐行补充中文行尾说明，便于现场直接查参数语义
- `scripts/capture_nav_points.py` 是现场只读采点辅助入口，依赖当前主链 TF 结果生成 Nav2 目标点文本，不接管 `map -> odom`、`/navigate_to_pose` 或 `/cmd_vel` 权威
- 除 `test_mapping.launch.py` 与 `test_navigation.launch.py` 联调入口外，如需可视化，应由工作区外部工具只读消费当前主链 ROS2 输出；`/point_lio/map_cloud` 是现场建图观察输出，不作为定位或导航权威

## 本轮收口

- 删除旧导航包和旧配置文件引用，导航模式改为 include Nav2 `navigation_launch.py`
- `bringup.launch.py` 向 Humble Nav2 `navigation_launch.py` 透传 `use_composition=False` 与 `use_respawn=False` 时使用 Python 布尔字面量大写，避免 Nav2 内部 `PythonExpression(['not ', use_composition])` 将小写 `false` 当作未定义名称求值
- 新增 `config/nav2_params.yaml`，当前自动导航链帧固定为 `map / odom / base_footprint`，控制器使用 Humble 兼容的 DWB 配置并允许麦克纳姆横移速度
- Nav2 obstacle layer 参数块当前保留在 `config/nav2_params.yaml` 中，但默认关闭；`/sensor_scan` (`PointCloud2`) 链路仍保留，`registered_scan` 继续只供定位链使用
- 本轮自动导航链将 `base_footprint` 固定为地面投影 2D 基座，`base_link` 固定为底盘最下层刚性主板中心，二者高度差当前由 `config/odom_interface.yaml` 维护，当前为 `0.2m`
- `rc26_sensor_scan` 不再把导航基座到雷达的关系当成静态缓存，而是按时间戳查询 `base_footprint -> livox_frame` 组合 TF，保证 `base_link` 保留 roll/pitch 时点云投影仍正确
- 本轮停用默认运行链中的 `rc26_merge_odom` 装配；源码保留在 `src/rc26_merge_odom`，但 `bringup.launch.py` 与 `test_navigation.launch.py` 不再声明或透传 `merge_odom_*`、`start_pose_sender`、`pose_sender_*` 参数
- 本轮新增 `rc26_mcu_transport` 作为机构指令共享串口 provider；`bringup.launch.py` 默认按 `r2_runtime.mcu_transport` 启动它，`test_navigation.launch.py` 显式关闭它
- 点云、地图、行为树入口现在由 `r2_runtime.paths.prior_pcd_file`、`r2_runtime.paths.nav2_map_file` 与 `r2_runtime.paths.behavior_tree_file` 配置，三者必须写绝对路径；`prior_pcd_file:=...` 与 `nav2_map_file:=...` 仍可在 launch 命令中临时覆盖
- `test_navigation.launch.py` 的验收目标改为 `/navigate_to_pose`、`sensor_scan` 链路存在、Nav2 lifecycle nodes、costmap/plan topics 与 `/cmd_vel` 输出；当前默认不再把 `sensor_scan` 作为 obstacle layer 成功条件
- `config/nav2_params.yaml` 现已为每一行实际 YAML 配置项补齐中文行尾注释，现场调试应优先以该文件中的分区说明和参数行说明为准
- `test_navigation.launch.py` 新增 `use_rviz` 与 `rviz_config_file`，默认随导航联调入口启动 RViz2；主 `bringup.launch.py` 继续保持 headless
- `navigation_default.rviz` 默认观察 Nav2 map、local/global costmap、plan、footprint、TF、RobotModel、`/odom`、定位位姿与 `registered_scan`/`sensor_scan` 点云主链；静态 map 与 costmap 采用半透明显示，点云采用固定高对比色，并保留 Nav2 RViz 面板和 GoalTool；不再订阅 terrain 或 keepout 输出
- `src/rc26_bringup/map/test.yaml` 是当前默认 Nav2 map 入口；当前绑定 `test.png`，`resolution=0.05`，`origin=[-2.567935467, -3.759390831, 0.0]`，由 `src/rc26_point_lio/PCD/scan.pcd` 按 `0.05 <= z <= 2.0` 与 `min_points_per_cell=3` 过滤投影生成。它可用于基础导航链联调，但现场实机规划验收仍应通过 `nav2_map_file` 传入现场有效 2D occupancy map
- 当前 `src/rc26_bringup/map/test.yaml` 字段按 Nav2 map YAML 的 `image / resolution / origin / negate / occupied_thresh / free_thresh` 口径维护；排查 map_server 加载问题时应先确认 `/map_server` 处于 active 且 `/map` 有 publisher
- 排查 PCD 与 Nav2 map YAML 的尺寸、origin 和覆盖关系时，使用 `rc26_point_lio/scripts/pcd_map_inspector.py --map-yaml <yaml>` 做只读校验；需要从 PCD 生成黑白 Nav2 静态地图时，使用 `rc26_point_lio/scripts/pcd_to_nav2_map.py`
- 当前仓库的 `test_mapping.launch.py` / Point-LIO 建图链默认导出的是 PCD 点云；PCD 作为 localization 先验输入，Nav2 `map_server` 则消费 `pcd_to_nav2_map.py` 生成的 `PNG + YAML` 或其它建图链保存出的 occupancy map，最后通过 `nav2_map_file` 传给导航入口
- `odometry.launch.py` 新增 `start_point_lio`、`start_sensor_scan` 开关，供 `test_odom_interface.launch.py`、`odometry_mock.launch.py` 等入口复用同一套静态 TF / 内部外参装配
- `odometry_mock.launch.py` 复用同一套 `odometry.launch.py` 装配；`mock_point_lio.py` 默认先输出一小段静止里程计，再进入运动阶段，以满足 `rc26_odom_interface` 现有的启动静止归零约束
- 当前 bringup / test_navigation 已移除 `pose_sender_feedback_serial_port` 等历史独立反馈口参数；目标 MCU 机构串口 owner 统一由 `rc26_mcu_transport` 提供
- `bringup.launch.py` 默认通过 `rc26_mcu_transport` 提供底盘 `/cmd_vel` consumer；`test_navigation.launch.py` 仍关闭该节点，只观察 `/cmd_vel` 输出。
- 新增 `scripts/capture_nav_points.py` 作为现场 Nav2 目标点采集工具；它通过 TF 读取当前 `map -> base_footprint`，生成中文 `.txt` 和可复制的 `<NavToPose .../>`，仅服务人工标定行为树目标点。
- 2026-06-13 同步：`bringup.launch.py` 与内部联调入口删除旧 `slam` 参数，统一改用 `run_mode:=navigation|mapping`；完整导航/决策链路使用 `run_mode:=navigation use_decision:=true`，建图链路使用 `run_mode:=mapping pure_mapping_mode:=true`。
- 2026-06-13 同步：删除分散的 `rc26_decision/config/decision_params.yaml` 与 `rc26_bringup/config/chassis_runtime.yaml`，并移除 `rc26_decision` 独立 launch 测试入口；完整 bringup 统一从 `rc26_bringup/config/r2_runtime.yaml` 读取运行配置，决策验收必须在所有相关节点拉起后进行。
- 2026-06-22 同步：`rc26_merge_odom` 源码保留但退出默认运行装配；机构 transport provider 改由独立 `rc26_mcu_transport` 承担。
- 2026-06-23 同步：`rc26_mcu_transport` 默认提供底盘 `/cmd_vel` consumer，以 `POSE_TARGET(0x0C)` 下发速度，线速度和角速度默认上限均为 `2.0`。
- 2026-06-26 同步：删除旧地形、base-ground 与 MF keepout 包及相关兼容接口；主启动、建图调试、验收探针、RViz 预设和 `package.xml` 均不接入这些历史链路
- 2026-06-26 同步：新增梅林区预选赛专属 `mf_preselection_tree.xml` 运行入口和 `mf_preselect_*` 参数组，默认行为树切到预选赛链路；入口 2 号导航点保留配置占位，未标定前由状态机假设机器人已处于入口预备姿态。
