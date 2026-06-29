# rc26_localization

## 模块定位

`rc26_localization` 是 R2 当前的最小先验地图定位模块。它默认使用可信比赛起点 `init_pose` 作为 `map -> odom` 初值，负责连续 small_gicp 局部跟踪，并作为 `map -> odom` 的唯一动态 TF 权威。无先验开局全局重定位和运行中在线重定位均默认关闭，仍沿用现有 `prior_pcd_file` PCD。

## 当前实现

- 构建方式：共享库组件 + 可执行节点
- 导出节点：`rc26_localization_node`
- 整车启动入口：`rc26_bringup/launch/localization.launch.py`
- 包内调试入口：`launch/sentry_localization.launch.py`
- 主源码：`src/localization.cpp`

运行时链路已经按比赛最小可用口径收口：

- 输入 `registered_scan`，默认由 `rc26_odom_interface` 提供，假设已经在 `odom` 坐标系表达
- `registered_scan` 订阅队列由 `input_cloud_queue_size` 控制，默认 30，用于吸收启动阶段和短时回调拥塞；累计点云仍受内部上限保护，避免无界占用内存
- 默认 `robot_base_frame=base_footprint`，连续定位和 `initialpose` 接管都以 2D 导航基座为准
- 加载 `prior_pcd_file`，假设先验 PCD 已经在 `map` 坐标系表达
- 启动后继续发布配置初值 TF，并直接用该初值进入局部 GICP 跟踪；默认 `init_pose` 是 `class_plus` 固定起点附近的 `map->odom` 可信初值，不是地图 origin
- 默认 `startup_relocalization_enable=false` 且 `startup_global_grid_enable=false`：现场 rosbag 验证表明无先验开局全局候选会被 `class_plus` 重复结构稳定带到远端假位姿，因此不进入比赛默认主链
- 默认 `require_initial_pose_for_local_tracking=true`：没有非零 `init_pose`、开局重定位成功、`initialpose` 接管或在线重定位接管形成的可信初值时，连续局部跟踪不会从全零身份变换开始盲配
- 2Hz 执行 small_gicp 局部配准，默认按 `previous_result_t` 在先验 PCD 下采样目标中裁剪局部目标点云；局部目标点数不足时冻结上一帧 TF，不回退到整图盲配准
- 连续局部配准质量通过 `min_inliers`、`max_normalized_error` 和单次建议跳变门控共同决定；默认 `num_threads=2`，按 X1 低算力整车链路限制 GICP 并行度，避免与 Point-LIO、决策侧运动闭环和相机驱动抢满 CPU
- 接受的连续局部配准结果默认用 `registration_smoothing_alpha` 在上一帧和 GICP 建议结果之间插值后发布，降低静止场景中 `map -> odom` 的厘米级随机游走；门控仍以未平滑的 GICP 建议结果为准
- 20Hz 发布 `map -> odom`，同时发布 `/localization/pose_with_cov`
- 发布 `/localization/diagnostics`，记录本帧是否接受、收敛、内点数、归一化误差、可信初值状态、局部目标点数、建议跳变量、拒绝原因和平滑配置；`status.message` 继续保留英文 reason code，同时新增 `human_message` 中文说明字段供现场排查
- 订阅 `initialpose`，用 `map_to_odom = map_to_base * inverse(odom_to_base)` 接管定位；当前查询的是 `odom -> base_footprint`
- 面向现场操作者的控制台日志和验收脚本提示按通俗中文输出；topic、frame、参数名和 diagnostics 既有英文 key 继续作为机器契约保留
- 实验性在线重定位由 `online_relocalization_enable` 控制，默认关闭；开启后在局部跟踪连续失败达到阈值时后台执行一次 PCD 全局粗配准 + small_gicp 精配准，成功后只更新本节点内部 `map -> odom`

## 当前边界

- 不负责建图、点云去畸变、里程计融合或传感器外参处理
- 默认不运行运行中自动全局重定位；该能力只作为显式开启的实验分支存在，不进入默认比赛主链
- 不内置 Scan Context 先验目录、重试区、UWB、BEVPlace、ESIKF、P4 候选或在线图后端
- 不再发布定位健康度、后端状态、路径可观测性、关键帧、闭环或重定位调试话题
- 开局无先验全局重定位默认关闭；默认丢定位恢复仍由人工或外部单次重定位节点向 `initialpose` 注入结果
- 实验性在线重定位成功时会造成 `/localization/pose_with_cov` 和 `map -> odom` 跳转，这是该开关开启后的预期行为；它不会回写 Point-LIO 滤波状态

## 实验性在线重定位

- 默认参数位于 [src/rc26_bringup/config/localization.yaml](/home/potato/RC_2026/src/rc26_bringup/config/localization.yaml)，`online_relocalization_enable=false`。
- 自动触发条件是连续局部跟踪失败达到 `online_relocalization_trigger_after_failures`，并满足冷却时间和最大尝试次数限制。
- 后台 worker 会在 `online_relocalization_collect_ms` 收集窗口后，使用最近累计 `registered_scan` 对现有 PCD 先验执行 FPFH/SAC-IA 粗配准和 small_gicp 精配准。
- `initialpose` 优先级高于后台在线重定位；人工接管后会清空失败计数和尝试计数，并取消待应用的后台结果。
- `/localization/diagnostics` 追加 `online_relocalization_state`、`online_relocalization_attempts`、`online_relocalization_reason`、`last_relocalization_source`，原有 key 和 `status.message` reason code 保持兼容。

## 运维入口

- [config/localization.yaml](/home/potato/RC_2026/src/rc26_bringup/config/localization.yaml)：最小定位参数
- [scripts/run_localization_acceptance.sh](/home/potato/RC_2026/src/rc26_localization/scripts/run_localization_acceptance.sh)：最小链路验收脚本
- [scripts/publish_synthetic_loc_inputs.py](/home/potato/RC_2026/src/rc26_localization/scripts/publish_synthetic_loc_inputs.py)：合成 `registered_scan` 输入

验收脚本现在会强制检查 `/localization/pose_with_cov` 与 `/localization/diagnostics` 均存在 publisher 且能采到频率；若走 synthetic 输入，会补一条静态 `odom -> base_footprint` 供最小链路 smoke 使用。默认小 PCD 只用于验证节点、TF/pose/diagnostics 链路可启动；比赛配准质量仍以真实地图或 rosbag 为准。

## 调试信息口径

- 定位节点的先验地图加载、开局重定位、`initialpose` 接管、实时点云不足、局部配准通过/冻结等控制台提示已经改为中文，便于现场直接判断下一步该检查地图、点云还是 TF。
- `/localization/diagnostics` 兼顾人读和机器读：`status.message`、`accepted`、`converged`、`inliers`、`normalized_error`、`startup_relocalization` 等原有英文字段保持不变；新增 `human_message` 字段给出中文原因说明，并追加实验性在线重定位状态字段。
- 连续跟踪稳定性字段包括 `tracking_initialized`、`active_target_points`、`local_target_used`、`registration_rejection_reason`、`registration_delta_translation_m`、`registration_delta_z_m`、`registration_delta_yaw_rad`、`registration_smoothing_enable` 和 `registration_smoothing_alpha`。现场排查时，若看到 `local_tracking_waiting_for_initial_pose`，应先确认开局重定位或 `/initialpose`；若看到 `jump_gate`，说明本轮 GICP 建议跳变过大，`map -> odom` 已冻结。
- `scripts/run_localization_acceptance.sh`、`publish_synthetic_loc_inputs.py`、`setup_realtime_aidlux.sh` 和 `profile_localization_perf.sh` 的用户可见提示同步改为中文；脚本参数名、raw 日志文件名和 ROS 话题名不变。

默认配置只调整定位节点的固定起点初值、局部目标、配准门控和发布平滑，不修改 `class_plus.pcd`、2D YAML/PNG 地图或地图 origin。若比赛实际起点改变，需要同步调整 `init_pose` 或用 `/initialpose` 接管，否则局部跟踪会围绕旧起点做目标裁剪；不要把无先验全局重定位作为重复结构场地的默认兜底。
