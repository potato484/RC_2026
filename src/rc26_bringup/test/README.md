# RC26 模块测试指南

本目录包含各模块的独立测试 launch 文件，用于单独验证每个模块的功能。

## 前置条件

```bash
# 确保已编译并 source 环境
cd "${RC26_WS:-$HOME/RC_2026}"
colcon build --symlink-install --parallel-workers 3 --cmake-args -DCMAKE_BUILD_TYPE=Release
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

---

## 测试指令汇总

### 1. 里程计接口测试 (rc26_odom_interface)

**功能**: 验证 Point-LIO → Nav2 坐标转换是否正常

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
# diagnostics 中应包含:
# h_min_eig, h_max_eig, h_cond, sigma_xy, sigma_yaw, obs_cov_source, hard_degen_consec
# health 中应包含:
# level, reason, control_degraded, localization_state, sigma_xy, sigma_yaw, h_min_eig, h_cond
# backend_status 中应包含:
# optimizer_ready, optimizer_state, graph_health, last_local_reg_age_sec, imu_spike
```

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

### 6. 控制器插件测试 (rc26_omni_controller)

**功能**: 验证全向运动控制器 (需要 Nav2 环境)

```bash
# 启动最小化 Nav2 + 控制器测试
ros2 launch rc26_bringup test_omni_controller.launch.py

# 检查隔离后的测试速度输出
ros2 topic echo /cmd_vel_test --once

# 注意：该测试默认使用 test_map -> test_odom -> test_base_link，
# 不再占用在线系统的 map / odom / base_link / cmd_vel
```

---

## 验证清单

| 模块 | 话题/TF | 预期结果 |
|------|---------|----------|
| odom_interface | `/odom` | odom→base_link 变换 |
| sensor_scan | `/sensor_scan` | laser_link 坐标系点云，`/odometry` 协方差透传 |
| lio_state_predictor | `/control_state` | 约 200Hz 预测里程计 |
| rc26_point_lio | `/degenerate_score` | 退化分数持续输出 |
| localization | `/localization/pose_with_cov` + `/localization/diagnostics` + `/localization/health` + `/localization/backend_status` | 持续发布且包含扩展字段 |
| omni_controller | `/cmd_vel` | 速度指令 |

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
