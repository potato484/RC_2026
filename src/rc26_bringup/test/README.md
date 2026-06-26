# RC26 模块测试指南

本目录包含 R2 运行链路的测试 launch 文件，用于验证模块链路和整车装配效果。决策相关测试默认按完整链路拉起，不再使用只启动 `decision_node` 的独立入口。

## 前置条件

```bash
# 确保已编译并 source 环境
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 --packages-select rc26_bringup rc26_sensor_scan rc26_mcu_transport
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

---

## 联调阶段入口

当前建议先按整车联调顺序跑，再回到下面的模块级测试逐项排障。涉及真实运动或机构动作时需要启动 `rc26_mcu_transport`；完整 bringup 默认会按 `r2_runtime.mcu_transport` 启动它，`test_navigation.launch.py` 则显式关闭它以避免底盘真实运动。

完整导航/决策链默认从 `src/rc26_bringup/config/r2_runtime.yaml` 读取点云、Nav2 地图和行为树入口绝对路径；现场切换 PCD / map / BT 时优先修改该文件。

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
ros2 launch rc26_bringup test_localization_chain.launch.py
```

### 3. 重定位

```bash
ros2 launch rc26_bringup test_relocalization.launch.py
```

### 4. 导航/决策完整链路

```bash
ros2 launch rc26_bringup test_navigation.launch.py \
  use_rviz:=true
```

```bash
ros2 launch rc26_bringup bringup.launch.py \
  run_mode:=navigation \
  use_decision:=true
```

### 5. Grid heading 正式入口

```bash
ros2 launch rc26_bringup grid_heading.launch.py
```

方向由 `src/rc26_bringup/config/r2_runtime.yaml` 维护：

```yaml
grid_heading_direction: "left"  # forward | left | right | backward
```

本入口只执行 `GridTurn -> GridHeadingAlign`，直接发布 `cmd_vel.angular.z`，不触发推杆或激光事件。实车运行前必须停用 Nav2、遥控和其它 `/cmd_vel` 发布者；若 `/odom` 和 `rc26_mcu_transport` 已由其它入口提供，可关闭重复链路：

```bash
ros2 launch rc26_bringup grid_heading.launch.py \
  start_odometry:=false \
  start_mcu_transport:=false
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
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
```

---

### 2. 传感器扫描测试 (rc26_sensor_scan)

**功能**: 验证点云坐标转换、里程计速度发布、pose 协方差透传，以及供导航链调试或后续恢复 obstacle layer 使用的 `sensor_scan` 点云输出

```bash
# 启动测试 (需要 odom_interface 数据源)
ros2 launch rc26_bringup test_sensor_scan.launch.py

# 验证输出话题
ros2 topic echo /sensor_scan --once
ros2 topic echo /odometry --once

# 检查协方差是否继续透传
ros2 topic echo /odometry --once | grep covariance

# 检查 TF 树
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_link livox_frame
```

---

### 3. 定位模块测试 (rc26_localization)

**功能**: 验证开局一次重定位与基于 small_gicp 的连续点云配准定位

```bash
# 启动测试
ros2 launch rc26_bringup test_localization.launch.py

# 验证 TF 发布 (map -> odom)
ros2 run tf2_ros tf2_echo map odom

# 检查协方差与诊断
ros2 topic echo /localization/pose_with_cov --once
ros2 topic echo /localization/diagnostics --once
# diagnostics 中应包含:
# accepted, converged, inliers, normalized_error, map_loaded, target_ready, startup_relocalization

# 一键验收（可选 synthetic 输入，无需 bag）
./src/rc26_localization/scripts/run_localization_acceptance.sh \
  --workspace "${RC26_WS:-$HOME/RC_2026}" \
  --synthetic-input \
  --duration 60
```

说明：

- 若使用实测 bag，`--bag` 参数可传 bag 目录（含 `metadata.yaml`）或 `.mcap/.db3` 文件路径。
- 现场切换先验点云时，完整链路优先修改 `src/rc26_bringup/config/r2_runtime.yaml`；模块级定位 launch 仍可用 `prior_pcd_file:=...` 临时覆盖。
- 当前定位链只在启动阶段尝试一次自动重定位；运行中丢定位仍通过 `initialpose` 接管，不再提供回环测试入口。

---

### 4. 完整里程计链测试 (rc26_point_lio + odom_interface + sensor_scan)

**功能**: 验证完整的里程计数据流

```bash
# 启动完整里程计链
ros2 launch rc26_bringup test_odometry_chain.launch.py

# 若雷达能 ping 通但 /livox/lidar 无数据，可在启动前自动恢复 Mid-360 host_ipcfg
ros2 launch rc26_bringup test_odometry_chain.launch.py start_mid360_driver:=true recover_mid360_stream:=true

# 验证数据流
ros2 topic list | grep -E "(state_estimation|odom|odometry|registered_scan|sensor_scan)"
ros2 topic echo /state_estimation --once
ros2 topic echo /odom --once
ros2 topic echo /odometry --once
ros2 topic echo /sensor_scan --once
ros2 topic hz /odom
ros2 topic hz /odometry
ros2 topic hz /sensor_scan

# 检查完整 TF 树
ros2 run tf2_tools view_frames
```

---

### 5. 决策系统测试 (rc26_decision)

完整决策流程使用完整 bringup 拉起定位、Nav2、`decision_node` 与 `rc26_mcu_transport`；`/cmd_vel` 默认由 `rc26_mcu_transport` 承接：

```bash
ros2 launch rc26_bringup bringup.launch.py \
  run_mode:=navigation \
  use_decision:=true
```

武馆区、主树或其它完整行为树入口通过 `src/rc26_bringup/config/r2_runtime.yaml` 的 `r2_runtime.paths.behavior_tree_file` 切换。Grid heading 使用正式独立入口 `grid_heading.launch.py`，不需要改默认行为树入口。

---

### 6. Nav2 基础导航链测试

**功能**: 验证定位、Nav2 lifecycle、`/navigate_to_pose` action、静态地图规划链、costmap 话题，以及 `/cmd_vel` 输出；`/sensor_scan` 仅检查链路存在

```bash
# 开发机 / 图结构验收：不启动 rc26_mcu_transport，只观察导航输出
ros2 launch rc26_bringup test_navigation.launch.py

ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 action info /navigate_to_pose
ros2 topic echo /sensor_scan --once
ros2 topic echo /plan --once
ros2 topic echo /local_costmap/costmap --once
ros2 topic echo /global_costmap/costmap --once
ros2 topic echo /cmd_vel
```

说明：

- `rc26_bringup` 当前保持 headless，不再通过 launch 参数拉起仓库内 GUI
- Nav2 当前默认关闭 local/global obstacle layer；`/sensor_scan` (`PointCloud2`) 仍会输出，但不再默认参与 costmap 障碍投影
- `test_navigation.launch.py` 不拉起 `rc26_mcu_transport`；需要真实运动时请改用完整 `bringup.launch.py`，或显式启动 `rc26_mcu_transport`
- 点云、地图和行为树入口默认来自 `src/rc26_bringup/config/r2_runtime.yaml`，其中 `paths.*` 必须使用绝对路径；`prior_pcd_file:=...` 和 `nav2_map_file:=...` 仍可临时覆盖
- `src/rc26_bringup/map/test.yaml` 是当前默认样本地图，由 `scan.pcd` 过滤投影为黑白 `test.png`；可用于基础导航链联调，不代表可直接通过现场真实规划验收
- 如需可视化，请改用工作区外部工具只读消费现有 topic，例如手工运行 `rviz2 -d /home/potato/RC_2026/src/rc26_bringup/rviz/navigation_default.rviz`

如需手动发送目标：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.2, y: 0.0, z: 0.0}, orientation: {z: 0.7071, w: 0.7071}}}}"
```

---

## 验证清单

| 模块 | 话题/TF | 预期结果 |
|------|---------|----------|
| odom_interface | `/odom` | odom→base_footprint 里程计，TF 同时提供 base_footprint→base_link |
| sensor_scan | `/sensor_scan` | `livox_frame` 坐标系点云；当前保留供导航链调试或后续恢复 obstacle layer 使用，`/odometry` 协方差继续透传 |
| rc26_point_lio | `/state_estimation` + `/cloud_registered` | LIO 里程计与原生配准点云持续输出 |
| localization | `map->odom` + `/localization/pose_with_cov` + `/localization/diagnostics` | TF 和标准定位观测持续发布 |
| Nav2 | `/navigate_to_pose` + `/plan` + costmap topics + `/sensor_scan` | action server、路径和 costmap 可观察；当前默认不接入动态障碍层，`/sensor_scan` 只校验链路存在 |
| rc26_mcu_transport | `/cmd_vel` | 完整 bringup/遥控入口默认消费速度命令并下发 `POSE_TARGET(0x0C)`；`test_navigation.launch.py` 默认关闭该节点 |

---

## 调试技巧

```bash
# 查看所有节点
ros2 node list

# 查看所有话题
ros2 topic list

# 若整车 bringup 前需要自动恢复 Mid-360，可追加：recover_mid360_stream:=true
# 例如：ros2 launch rc26_bringup bringup.launch.py run_mode:=navigation use_decision:=false recover_mid360_stream:=true
# 首次执行会自动拉取并编译官方 Livox-SDK2，耗时会明显更长；后续直接复用本地缓存。

# 检查节点日志
ros2 run rqt_console rqt_console

# TF 树可视化
ros2 run tf2_tools view_frames && evince frames.pdf
```
