# RC26 模块测试指南

本目录包含各模块的独立测试 launch 文件，用于单独验证每个模块的功能。

## 前置条件

```bash
# 确保已编译并 source 环境
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 --packages-select rc26_bringup
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

---

## 联调阶段入口

当前建议先按整车联调顺序跑，再回到下面的模块级测试逐项排障。

### 0. 遥控

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
./start_r2_teleop.sh
```

### 1. 建图

```bash
ros2 launch rc26_bringup test_mapping.launch.py
```

### 2. 定位

```bash
ros2 launch rc26_bringup test_localization_chain.launch.py \
  prior_pcd_file:=${RC26_WS:-$HOME/RC_2026}/src/rc26_point_lio/PCD/scans.pcd
```

### 3. 重定位

```bash
ros2 launch rc26_bringup test_relocalization.launch.py \
  prior_pcd_file:=${RC26_WS:-$HOME/RC_2026}/src/rc26_point_lio/PCD/scans.pcd
```

### 4. 回环

```bash
ros2 launch rc26_bringup test_loop_closure.launch.py \
  prior_pcd_file:=${RC26_WS:-$HOME/RC_2026}/src/rc26_point_lio/PCD/scans.pcd
```

### 5. 导航

```bash
ros2 launch rc26_bringup test_navigation.launch.py \
  prior_pcd_file:=${RC26_WS:-$HOME/RC_2026}/src/rc26_point_lio/PCD/scans.pcd
```

---

## 测试指令汇总

### 1. 里程计接口测试 (rc26_odom_interface)

**功能**: 验证 Point-LIO → 统一 odom/tf 坐标转换是否正常

```bash
# 启动测试 (需要 rc26_point_lio 数据源)
ros2 launch rc26_bringup test_odom_interface.launch.py

# 验证输出话题
ros2 topic echo /odom --once
ros2 topic echo /registered_scan --once

# 检查协方差是否已透传（非全零）
ros2 topic echo /odom --once | grep covariance

# 检查 TF 树
ros2 run tf2_ros tf2_echo odom base_link
```

---

### 2. 传感器扫描测试 (rc26_sensor_scan)

**功能**: 验证点云坐标转换、里程计速度发布和 pose 协方差透传

```bash
# 启动测试 (需要 odom_interface 数据源)
ros2 launch rc26_bringup test_sensor_scan.launch.py

# 验证输出话题
ros2 topic echo /sensor_scan --once
ros2 topic echo /odometry --once

# 检查协方差是否继续透传
ros2 topic echo /odometry --once | grep covariance

# 检查 TF 树
ros2 run tf2_ros tf2_echo base_link laser_link
```

---

### 3. 定位模块测试 (rc26_localization)

**功能**: 验证基于 small_gicp 的点云配准定位

```bash
# 启动测试 (指定先验点云)
ros2 launch rc26_bringup test_localization.launch.py \
    prior_pcd_file:=${RC26_WS:-$HOME/RC_2026}/src/rc26_bringup/pcd/default.pcd

# 验证 TF 发布 (map -> odom)
ros2 run tf2_ros tf2_echo map odom

# 检查协方差与诊断
ros2 topic echo /localization/pose_with_cov --once
ros2 topic echo /localization/diagnostics --once
ros2 topic echo /localization/health --once
ros2 topic echo /localization/backend_status --once
ros2 topic echo /localization/route_observability --once
# diagnostics 中应包含:
# h_min_eig, h_max_eig, h_cond, sigma_xy, sigma_yaw, obs_cov_source, hard_degen_consec
# health 中应包含:
# level, reason, control_degraded, localization_state, sigma_xy, sigma_yaw, h_min_eig, h_cond
# backend_status 中应包含:
# optimizer_ready, optimizer_state, graph_health, last_local_reg_age_sec, imu_spike
# route_observability 中应包含:
# score, risk_level, repeat_structure_risk, dynamic_risk, recommended_nav_profile

# 一键验收（可选 synthetic 输入，无需 bag）
./src/rc26_localization/scripts/run_localization_acceptance.sh \
  --workspace "${RC26_WS:-$HOME/RC_2026}" \
  --synthetic-input \
  --duration 60 \
  --config-profile eval \
  --overlay-file "${RC26_WS:-$HOME/RC_2026}/src/rc26_bringup/config/localization_eval_overlay_synthetic.yaml" \
  --competition-mode false \
  --enable-graph-backend true

# P4 候选链路验收（在 synthetic 输入中自动发布 learned candidates）
./src/rc26_localization/scripts/run_localization_acceptance.sh \
  --workspace "${RC26_WS:-$HOME/RC_2026}" \
  --synthetic-input \
  --duration 30 \
  --config-profile eval \
  --overlay-file "${RC26_WS:-$HOME/RC_2026}/src/rc26_bringup/config/localization_eval_overlay_synthetic.yaml" \
  --competition-mode false \
  --enable-graph-backend true \
  --p4-candidate-enable true \
  --min-inliers 20
```

说明：

- 若使用实测 bag，`--bag` 参数可传 bag 目录（含 `metadata.yaml`）或 `.mcap/.db3` 文件路径。
- synthetic overlay 仅用于最小地图链路验收，不建议直接作为比赛默认参数。

---

### 4. 完整里程计链测试 (rc26_point_lio + odom_interface + sensor_scan + lio_state_predictor)

**功能**: 验证完整的里程计数据流

```bash
# 启动完整里程计链
ros2 launch rc26_bringup test_odometry_chain.launch.py

# 若雷达能 ping 通但 /livox/lidar 无数据，可在启动前自动恢复 Mid-360 host_ipcfg
ros2 launch rc26_bringup test_odometry_chain.launch.py start_mid360_driver:=true recover_mid360_stream:=true

# 验证数据流
ros2 topic list | grep -E "(state_estimation|odom|odometry|control_state|degenerate_score|control_degraded)"
ros2 topic echo /state_estimation --once
ros2 topic echo /odom --once
ros2 topic echo /odometry --once
ros2 topic echo /degenerate_score
ros2 topic echo /control_degraded
ros2 topic hz /control_state
ros2 topic hz /odometry

# 检查完整 TF 树
ros2 run tf2_tools view_frames
```

---

### 5. 决策系统测试 (rc26_decision)

（已删除过期的 rc26_bringup 决策/串口测试启动；请使用 `rc26_decision/launch/decision.launch.py` 进行独立测试。）

---

### 6. 自研导航链测试 (rc26_xhu_nav)

**功能**: 验证 topo/xhu 走廊下发、模式切换和执行反馈链路

```bash
ros2 launch rc26_bringup bringup.launch.py \
  slam:=false \
  use_decision:=false \
  prior_pcd_file:=${RC26_WS:-$HOME/RC_2026}/src/rc26_bringup/pcd/default.pcd

ros2 topic echo /xhu_nav/motion_mode_state
ros2 topic echo /xhu_nav/tracking_state
ros2 topic echo /xhu_nav/semantic_gate
ros2 topic echo /cmd_vel
```

说明：

- `rc26_bringup` 当前保持 headless，不再通过 launch 参数拉起仓库内 GUI
- 如需可视化，请改用工作区外部工具只读消费现有 topic，例如手工运行 `rviz2 -d /home/potato/RC_2026/src/rc26_bringup/rviz/navigation_default.rviz`

如需手动切模式：

```bash
ros2 service call /set_xhu_motion_mode rc26_interfaces/srv/SetXhuMotionMode \
  "{mode: 'plane_move', timeout: 0.0, reason: 'manual_check'}"
```

---

## 验证清单

| 模块 | 话题/TF | 预期结果 |
|------|---------|----------|
| odom_interface | `/odom` | odom→base_link 变换 |
| sensor_scan | `/sensor_scan` | laser_link 坐标系点云，`/odometry` 协方差透传 |
| lio_state_predictor | `/control_state` | 约 200Hz 预测里程计 |
| rc26_point_lio | `/degenerate_score` | 退化分数持续输出 |
| localization | `/localization/pose_with_cov` + `/localization/diagnostics` + `/localization/health` + `/localization/backend_status` + `/localization/route_observability` | 持续发布且包含扩展字段 |
| rc26_xhu_nav | `/xhu_nav/motion_mode_state` + `/xhu_nav/tracking_state` + `/xhu_nav/local_planner_state` | 模式、执行反馈和局部规划状态持续更新 |
| rc26_xhu_nav runtime | `/cmd_vel` | 速度指令由 `xhu_motion_runtime_node` 输出 |

---

## 调试技巧

```bash
# 查看所有节点
ros2 node list

# 查看所有话题
ros2 topic list

# 若整车 bringup 前需要自动恢复 Mid-360，可追加：recover_mid360_stream:=true
# 例如：ros2 launch rc26_bringup bringup.launch.py slam:=false use_decision:=false recover_mid360_stream:=true
# 首次执行会自动拉取并编译官方 Livox-SDK2，耗时会明显更长；后续直接复用本地缓存。

# 检查节点日志
ros2 run rqt_console rqt_console

# TF 树可视化
ros2 run tf2_tools view_frames && evince frames.pdf
```
