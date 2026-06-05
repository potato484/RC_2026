# rc26_localization 模块说明

`rc26_localization` 现在是 R2 的最小比赛定位链路：加载先验 PCD，订阅 `registered_scan`，开局执行一次 SAC-IA + `rc26_small_gicp` 重定位，随后用 `rc26_small_gicp` 连续局部配准，并发布 `map -> odom`。

## 输入

- `registered_scan` (`sensor_msgs/msg/PointCloud2`)：默认来自 `rc26_odom_interface`，假设已经在 `odom` 坐标系表达。
- `initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)：人工或外部单次重定位结果，用于重置 `map -> odom`。
- `prior_pcd_file`：先验地图 PCD，假设已经在 `map` 坐标系表达。

## 输出

- 动态 TF `map -> odom`：20Hz 发布，本节点是唯一权威。
- `/localization/pose_with_cov` (`geometry_msgs/msg/PoseWithCovarianceStamped`)：当前 `map -> odom` 位姿和简化协方差。
- `/localization/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`)：配准是否接受、是否收敛、内点数、归一化误差等状态。`status.message` 和既有 KeyValue key 继续保留英文机器字段，同时新增 `human_message` 中文说明，便于现场直接判断定位卡在地图、点云、TF 还是配准质量。

## 保留的运行逻辑

- 启动后按 `init_pose` 继续发布 `map -> odom`，同时累计短时间 `registered_scan`。
- 开局累计完成后，用 FPFH/SAC-IA 生成粗初值，再用 small_gicp 精配准；成功后接管 `map -> odom`。
- 开局重定位只尝试一次；失败后保留 `init_pose` 或上一帧 TF，并进入连续局部跟踪。
- 连续跟踪阶段 2Hz 累积并下采样 `registered_scan`。
- 以 `previous_result_t` 为初值执行 small_gicp。
- 只有 `converged && inliers >= min_inliers && normalized_error <= max_normalized_error` 时更新 `map -> odom`。
- 配准质量不达标时冻结上一帧 TF，并按内部常量放大 pose covariance。
- 收到 `initialpose` 后查询 `odom -> base_footprint`，按 `map_to_odom = map_to_base * inverse(odom_to_base)` 接管。

## 已移除能力

在线图后端、关键帧、闭环、Scan Context、运行中自动全局重定位、重试区、UWB、BEVPlace、ESIKF、P4 外部候选、路径可观测性、定位健康度和定位自定义接口消息都已从主链删除。

开局重定位只解决启动初值问题。丢定位后的恢复入口仍只保留 `initialpose`。如果未来要重新引入运行中自动全局重定位，需要先明确比赛链路需求、接口契约和验证口径，再作为新的架构变更恢复。

## 最小验收

`scripts/run_localization_acceptance.sh` 用于 smoke 验证当前链路：它会启动 `rc26_bringup` 的定位 launch，可选发布合成 `registered_scan` 和静态 `odom -> base_footprint`，并强制检查 `/localization/pose_with_cov`、`/localization/diagnostics` 均存在 publisher 且能采到频率。默认小 PCD 只适合验证节点、TF/pose/diagnostics 链路可启动；真实配准质量仍需用比赛地图或 rosbag 验证。

## 调试信息

定位节点、合成输入脚本、验收脚本和 AidLux/性能运维脚本的用户可见提示按中文输出。topic、frame、参数名、reason code、raw 日志文件名和 diagnostics 既有英文 key 不变，外部脚本或看板仍应按原字段消费；现场人工排查优先看控制台中文日志和 `/localization/diagnostics` 的 `human_message` 字段。
