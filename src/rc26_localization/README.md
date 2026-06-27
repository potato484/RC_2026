# rc26_localization 模块说明

`rc26_localization` 现在是 R2 的最小比赛定位链路：加载先验 PCD，订阅 `registered_scan`，开局执行一次 SAC-IA + `rc26_small_gicp` 重定位，随后用 `rc26_small_gicp` 连续局部配准，并发布 `map -> odom`。当前新增运行中在线重定位是实验性能力，默认关闭，仍只使用现有 `prior_pcd_file` PCD。

## 输入

- `registered_scan` (`sensor_msgs/msg/PointCloud2`)：默认来自 `rc26_odom_interface`，假设已经在 `odom` 坐标系表达。
- `initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)：人工或外部单次重定位结果，用于重置 `map -> odom`。
- `prior_pcd_file`：先验地图 PCD，假设已经在 `map` 坐标系表达。

## 输出

- 动态 TF `map -> odom`：20Hz 发布，本节点是唯一权威。
- `/localization/pose_with_cov` (`geometry_msgs/msg/PoseWithCovarianceStamped`)：当前 `map -> odom` 位姿和简化协方差。
- `/localization/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`)：配准是否接受、是否收敛、内点数、归一化误差等状态。`status.message` 和既有 KeyValue key 继续保留英文机器字段，同时新增 `human_message` 中文说明，便于现场直接判断定位卡在地图、点云、TF 还是配准质量。实验性在线重定位会追加 `online_relocalization_state`、`online_relocalization_attempts`、`online_relocalization_reason`、`last_relocalization_source`。

## 保留的运行逻辑

- 启动后按 `init_pose` 继续发布 `map -> odom`，同时累计短时间 `registered_scan`。
- 开局累计完成后，用 FPFH/SAC-IA 生成粗初值，再用 small_gicp 精配准；成功后接管 `map -> odom`。
- 开局重定位只尝试一次；失败后保留 `init_pose` 或上一帧 TF，并进入连续局部跟踪。
- 连续跟踪阶段 2Hz 累积并下采样 `registered_scan`。
- 以 `previous_result_t` 为初值执行 small_gicp。
- 只有 `converged && inliers >= min_inliers && normalized_error <= max_normalized_error` 时更新 `map -> odom`。
- 配准质量不达标时冻结上一帧 TF，并按内部常量放大 pose covariance。
- 收到 `initialpose` 后查询 `odom -> base_footprint`，按 `map_to_odom = map_to_base * inverse(odom_to_base)` 接管。

## 实验性在线重定位

`online_relocalization_enable=false` 是默认主链口径。关闭时，定位链路只执行现有开局一次重定位、连续局部跟踪和 `initialpose` 接管，不启动后台在线重定位 worker，也不会在局部跟踪失败时跑全局配准。

开启后，当连续局部跟踪失败次数达到 `online_relocalization_trigger_after_failures`，节点会在后台收集最近 `registered_scan`，对现有 `prior_pcd_file` 生成的目标执行一次 FPFH/SAC-IA 粗配准和 small_gicp 精配准。成功时只更新本节点内部 `map -> odom`，不改 Point-LIO 滤波器状态；20Hz TF 发布不会等待全局配准完成。

`initialpose` 仍是人工优先接管入口。收到人工初值后会立即按当前实现重置 `map -> odom`，清空在线重定位失败计数和尝试计数，并取消待应用的后台在线重定位结果。

## 仍不恢复的能力

在线图后端、关键帧、闭环、Scan Context 先验目录、重试区、UWB、BEVPlace、ESIKF、P4 外部候选、路径可观测性、定位健康度和定位自定义接口消息仍不属于当前主链。本轮在线重定位只支持现有 PCD 先验地图，不支持参考仓库的 `scd/optimized_pose.txt` 目录格式。

## 最小验收

`scripts/run_localization_acceptance.sh` 用于 smoke 验证当前链路：它会启动 `rc26_bringup` 的定位 launch，可选发布合成 `registered_scan` 和静态 `odom -> base_footprint`，并强制检查 `/localization/pose_with_cov`、`/localization/diagnostics` 均存在 publisher 且能采到频率。默认小 PCD 只适合验证节点、TF/pose/diagnostics 链路可启动；真实配准质量仍需用比赛地图或 rosbag 验证。

## 调试信息

定位节点、合成输入脚本、验收脚本和 AidLux/性能运维脚本的用户可见提示按中文输出。topic、frame、参数名、reason code、raw 日志文件名和 diagnostics 既有英文 key 不变，外部脚本或看板仍应按原字段消费；现场人工排查优先看控制台中文日志和 `/localization/diagnostics` 的 `human_message`、`online_relocalization_state` 字段。
