# rc26_localization 模块说明

`rc26_localization` 现在是 R2 的最小比赛定位链路：加载先验 PCD，订阅 `registered_scan`，默认使用可信比赛起点 `init_pose` 作为 `map -> odom` 初值，随后用 `rc26_small_gicp` 连续局部配准，并发布 `map -> odom`。无先验开局全局重定位和运行中在线重定位均默认关闭，仍只使用现有 `prior_pcd_file` PCD。

## 输入

- `registered_scan` (`sensor_msgs/msg/PointCloud2`)：默认来自 `rc26_odom_interface`，假设已经在 `odom` 坐标系表达。
- `initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)：人工或外部单次重定位结果，用于重置 `map -> odom`。
- `prior_pcd_file`：先验地图 PCD，假设已经在 `map` 坐标系表达。

## 输出

- 动态 TF `map -> odom`：20Hz 发布，本节点是唯一权威。
- `/localization/pose_with_cov` (`geometry_msgs/msg/PoseWithCovarianceStamped`)：当前 `map -> odom` 位姿和简化协方差。
- `/localization/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`)：配准是否接受、是否收敛、内点数、归一化误差等状态。`status.message` 和既有 KeyValue key 继续保留英文机器字段，同时新增 `human_message` 中文说明，便于现场直接判断定位卡在地图、点云、TF 还是配准质量。实验性在线重定位会追加 `online_relocalization_state`、`online_relocalization_attempts`、`online_relocalization_reason`、`last_relocalization_source`。

## 保留的运行逻辑

- 启动后按 `init_pose` 继续发布 `map -> odom`，并直接进入连续局部 GICP 跟踪。当前默认 `init_pose` 是 `class_plus` 固定起点附近的 `map->odom` 可信初值；它不是地图 origin。
- 默认 `startup_relocalization_enable=false`，不开局执行无先验 SAC-IA/全局栅格重定位。现场测试表明 `class_plus` 重复结构会把无先验全局候选稳定推到远端假位姿，不能作为默认主链。
- 默认 `require_initial_pose_for_local_tracking=true`，若没有非零 `init_pose`、开局重定位成功、`initialpose` 接管或在线重定位接管形成的可信初值，连续局部跟踪不会从全零身份变换开始盲配。
- 连续跟踪阶段 2Hz 累积并下采样 `registered_scan`。
- 以 `previous_result_t` 为初值执行 small_gicp；默认 `local_target_enable=true`，只在初值附近裁剪先验 PCD 的局部目标点云参与连续跟踪，局部目标点数不足时冻结而不是退回整图盲配准。
- 只有 `converged && inliers >= min_inliers && normalized_error <= max_normalized_error`，并且单次配准建议跳变未超过 `max_registration_translation_delta_m`、`max_registration_z_delta_m`、`max_registration_yaw_delta_rad` 时更新 `map -> odom`。
- 连续跟踪接受的结果默认按 `registration_smoothing_alpha` 在上一帧和 GICP 建议结果之间插值，用于抑制静止时 `map -> odom` 随机游走；质量门控仍按未平滑的 GICP 建议结果判断。
- 配准质量不达标时冻结上一帧 TF，并按内部常量放大 pose covariance。
- 收到 `initialpose` 后查询 `odom -> base_footprint`，按 `map_to_odom = map_to_base * inverse(odom_to_base)` 接管。

## 实验性在线重定位

`online_relocalization_enable=false` 是默认主链口径。关闭时，定位链路只执行可信 `init_pose` 下的连续局部跟踪和 `initialpose` 接管，不启动后台在线重定位 worker，也不会在局部跟踪失败时跑全局配准。

开启后，当连续局部跟踪失败次数达到 `online_relocalization_trigger_after_failures`，节点会在后台收集最近 `registered_scan`，对现有 `prior_pcd_file` 生成的目标执行一次 FPFH/SAC-IA 粗配准和 small_gicp 精配准。成功时只更新本节点内部 `map -> odom`，不改 Point-LIO 滤波器状态；20Hz TF 发布不会等待全局配准完成。

`initialpose` 仍是人工优先接管入口。收到人工初值后会立即按当前实现重置 `map -> odom`，清空在线重定位失败计数和尝试计数，并取消待应用的后台在线重定位结果。

## 仍不恢复的能力

在线图后端、关键帧、闭环、Scan Context 先验目录、重试区、UWB、BEVPlace、ESIKF、P4 外部候选、路径可观测性、定位健康度和定位自定义接口消息仍不属于当前主链。本轮在线重定位只支持现有 PCD 先验地图，不支持参考仓库的 `scd/optimized_pose.txt` 目录格式。

## 最小验收

`scripts/run_localization_acceptance.sh` 用于 smoke 验证当前链路：它会启动 `rc26_bringup` 的定位 launch，可选发布合成 `registered_scan` 和静态 `odom -> base_footprint`，并强制检查 `/localization/pose_with_cov`、`/localization/diagnostics` 均存在 publisher 且能采到频率。默认小 PCD 只适合验证节点、TF/pose/diagnostics 链路可启动；真实配准质量仍需用比赛地图或 rosbag 验证。

## 调试信息

定位节点、合成输入脚本、验收脚本和 AidLux/性能运维脚本的用户可见提示按中文输出。topic、frame、参数名、reason code、raw 日志文件名和 diagnostics 既有英文 key 不变，外部脚本或看板仍应按原字段消费；现场人工排查优先看控制台中文日志和 `/localization/diagnostics` 的 `human_message`、`online_relocalization_state` 字段。

连续跟踪稳定性相关的诊断字段会随 `/localization/diagnostics` 发布：

- `tracking_initialized`：是否已经获得可信初值。
- `active_target_points`、`local_target_used`：本轮连续跟踪使用的目标点数，以及是否使用局部裁剪目标。
- `registration_rejection_reason`：本轮拒绝原因，常见值包括 `waiting_for_initial_pose`、`not_converged`、`high_normalized_error`、`jump_gate`。
- `registration_delta_translation_m`、`registration_delta_z_m`、`registration_delta_yaw_rad`：GICP 建议结果相对上一帧的跳变量。
- `registration_smoothing_enable`、`registration_smoothing_alpha`：接受更新后的发布平滑配置。

默认参数位于 `rc26_bringup/config/localization.yaml`。当前口径不修改 `prior_pcd_file` 指向的 PCD 或 2D YAML/PNG 地图，只通过定位节点的固定起点初值、局部目标、质量门控和发布平滑约束 `map -> odom` 的更新。若实车起点换到地图内其它位置，必须同步调整 `init_pose` 或通过 `/initialpose` 接管，否则节点会按旧起点附近做局部跟踪；不要重新打开无先验全局开局重定位来兜底重复结构场景。
