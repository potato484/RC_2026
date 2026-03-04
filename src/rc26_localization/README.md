# rc26_localization

`rc26_localization` 是 RC_2026 中 R2 自动机器人的点云重定位模块，基于 `small_gicp`，负责发布 `map -> odom`。

## 功能概览

- 局部跟踪：`registered_scan` 与先验地图持续配准
- 鲁棒估计：`GICPFactor` 外包 `Huber` 核，降低动态点/反光点污染
- 观测协方差：由 `result.H` 在线估计 `Sigma_obs`，不再固定硬编码常量
- 两层退化策略：软退化（协方差膨胀）+ 硬退化（`lambda_min` 门控）
- 优化器自适应：`gn_auto` 根据初值跳变在 GN/LM 间切换
- 退化场景稳态：子空间可观测性分析 + 方向约束更新（替代原 S2 二值门控主路径）
- 重定位通道：L0（IMU 快速恢复）/ L1（重试区与 UWB 种子）/ L2（Scan Context）
- 并行恢复：支持 L0/L1/L2 并行赛跑，首个满足阈值通道胜出
- 可观测性输出：`/localization/pose_with_cov` 与 `/localization/diagnostics`

## 当前恢复架构

```text
TRACKING
  │ (质量退化/超时/绑架)
  ▼
SUSPECT
  │ requestRelocalization()
  ▼
GLOBAL_RECOVERY (single-flight worker)
  ├─ 并行模式: L0 || L1 || L2 (winner-takes-all)
  └─ 串行模式: L0 -> L1 -> L2
       成功 -> TRACKING
       失败 -> RELOC_FAILED
```

## 输入输出

- 输入话题：`registered_scan` (`sensor_msgs/msg/PointCloud2`)
- 输入话题：`initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)
- 输入话题：`/livox/imu`（或配置的 `s1_imu_topic`）
- 输入话题：`/uwb/position`（可选，开启 `uwb_enable`）
- 输出 TF：`map -> odom`
- 输出话题：`/localization/pose_with_cov` (`geometry_msgs/msg/PoseWithCovarianceStamped`)
- 输出话题：`/localization/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`)

## 关键参数（新增能力）

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `degen_enable` | true | 启用子空间退化约束更新 |
| `degen_eigenvalue_ratio_threshold` | 0.01 | 退化方向判定阈值 |
| `esikf_enable` | false | 启用 ESIKF 预测/更新 |
| `dynamic_filter_enable` | false | 启用多帧一致性动态点过滤 |
| `l0_enable` | true | 启用 L0 快速恢复 |
| `l0_max_imu_gap_ms` | 1000 | L0 可接受 IMU 断档上限 |
| `parallel_reloc_enable` | true | 启用 L0/L1/L2 并行赛跑 |
| `uwb_enable` | false | 启用 UWB 绝对锚点种子 |
| `pose_cov_topic` | `/localization/pose_with_cov` | 协方差输出话题 |
| `diagnostics_topic` | `/localization/diagnostics` | 诊断输出话题 |

> 当前仓库未集成 `rc26_bevplace` 包。`bevplace_*` 参数已预留，运行时会自动回退 Scan Context。

## 执行方案1（P0/P1）参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `robust_enable` | true | Huber 鲁棒核总开关 |
| `huber_c` | 1.0 | Huber 核宽度（作用在 `sqrt(error)`） |
| `cov_from_hessian_enable` | true | true=用 Hessian 估计观测协方差，false=回退硬编码 |
| `cov_eig_floor` | 1.0 | Hessian 特征值下限（数值正则） |
| `cov_scale_enable` | true | 残差尺度校正开关 |
| `cov_scale_min` | 1e-4 | 残差尺度下限 |
| `cov_scale_max` | 10.0 | 残差尺度上限 |
| `hessian_degen_enable` | true | 硬退化门控开关 |
| `hessian_lambda_hard` | 10.0 | Hessian 最小特征值硬阈值 |
| `gicp_optimizer_mode` | `gn_auto` | `gn_auto` / `gn` / `lm` |
| `gn_auto_trans_threshold_m` | 0.05 | `gn_auto` 下初值平移跳变阈值 |

## 诊断字段（新增）

`/localization/diagnostics` 现在包含以下关键字段：
- `h_min_eig`
- `h_max_eig`
- `h_cond`
- `sigma_xy`
- `sigma_yaw`
- `obs_cov_source`（`hessian` / `hardcoded`）
- `hard_degen_consec`

## 回归开关检查

```bash
# 1) 协方差回退：obs_cov_source 应变为 hardcoded
ros2 param set /localization cov_from_hessian_enable false

# 2) 关闭鲁棒核：回退纯 GICP 行为
ros2 param set /localization robust_enable false

# 3) 关闭新硬退化：回退 legacy S2
ros2 param set /localization hessian_degen_enable false

# 4) 强制优化器
ros2 param set /localization gicp_optimizer_mode lm
ros2 param set /localization gicp_optimizer_mode gn_auto
```

## 结构化日志

本模块输出如下重定位日志，便于离线统计：

```text
RELOC_METRIC,trigger_reason,path_used,t_total_ms,t_l0_ms,t_l1_ms,t_l2_ms,
candidate_count,best_fitness,best_J,winner_channel,cancel_reason,accepted
```

## 快速开始

```bash
cd /home/potato/RC_2026
colcon build --parallel-workers 1 --packages-select rc26_localization
source install/setup.bash
ros2 launch rc26_bringup localization.launch.py prior_pcd_file:=/abs/path/prior.pcd
```

## 一键验收

```bash
cd /home/potato/RC_2026
./src/rc26_localization/scripts/run_localization_acceptance.sh \
  --map /abs/path/prior.pcd \
  --bag /abs/path/test.mcap \
  --duration 240
```

## 调试文档

- `src/rc26_localization/docs/调试与验收.md`

## 依赖

- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`
- `diagnostic_msgs`
- `tf2_ros`
- `pcl_conversions`
- `small_gicp`
