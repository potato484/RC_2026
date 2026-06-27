# rc26_localization

## 模块定位

`rc26_localization` 是 R2 当前的最小先验地图定位模块。它负责开局一次 SAC-IA + `rc26_small_gicp` 重定位、连续 small_gicp 局部跟踪，并作为 `map -> odom` 的唯一动态 TF 权威。运行中在线重定位已作为实验性能力恢复，默认关闭，仍沿用现有 `prior_pcd_file` PCD。

## 当前实现

- 构建方式：共享库组件 + 可执行节点
- 导出节点：`rc26_localization_node`
- 整车启动入口：`rc26_bringup/launch/localization.launch.py`
- 包内调试入口：`launch/sentry_localization.launch.py`
- 主源码：`src/localization.cpp`

运行时链路已经按比赛最小可用口径收口：

- 输入 `registered_scan`，默认由 `rc26_odom_interface` 提供，假设已经在 `odom` 坐标系表达
- 默认 `robot_base_frame=base_footprint`，连续定位和 `initialpose` 接管都以 2D 导航基座为准
- 加载 `prior_pcd_file`，假设先验 PCD 已经在 `map` 坐标系表达
- 启动后继续发布配置初值 TF，并用开局累计点云做一次 FPFH/SAC-IA 粗配准 + small_gicp 精配准
- 2Hz 执行 small_gicp 局部配准，质量通过 `min_inliers` 和 `max_normalized_error` 门控
- 20Hz 发布 `map -> odom`，同时发布 `/localization/pose_with_cov`
- 发布 `/localization/diagnostics`，记录本帧是否接受、收敛、内点数、归一化误差和开局重定位状态；`status.message` 继续保留英文 reason code，同时新增 `human_message` 中文说明字段供现场排查
- 订阅 `initialpose`，用 `map_to_odom = map_to_base * inverse(odom_to_base)` 接管定位；当前查询的是 `odom -> base_footprint`
- 面向现场操作者的控制台日志和验收脚本提示按通俗中文输出；topic、frame、参数名和 diagnostics 既有英文 key 继续作为机器契约保留
- 实验性在线重定位由 `online_relocalization_enable` 控制，默认关闭；开启后在局部跟踪连续失败达到阈值时后台执行一次 PCD 全局粗配准 + small_gicp 精配准，成功后只更新本节点内部 `map -> odom`

## 当前边界

- 不负责建图、点云去畸变、里程计融合或传感器外参处理
- 默认不运行运行中自动全局重定位；该能力只作为显式开启的实验分支存在，不进入默认比赛主链
- 不内置 Scan Context 先验目录、重试区、UWB、BEVPlace、ESIKF、P4 候选或在线图后端
- 不再发布定位健康度、后端状态、路径可观测性、关键帧、闭环或重定位调试话题
- 开局重定位只在启动阶段尝试一次；默认丢定位恢复仍由人工或外部单次重定位节点向 `initialpose` 注入结果
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
- `scripts/run_localization_acceptance.sh`、`publish_synthetic_loc_inputs.py`、`setup_realtime_aidlux.sh` 和 `profile_localization_perf.sh` 的用户可见提示同步改为中文；脚本参数名、raw 日志文件名和 ROS 话题名不变。
