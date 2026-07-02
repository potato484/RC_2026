# RC26 模块测试指南

本目录记录 R2 运行链路的测试入口。决策相关测试默认按完整链路拉起，不再使用只启动 `decision_node` 的独立入口。

## 前置条件

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --symlink-install --executor sequential --parallel-workers 1 --packages-select rc26_bringup rc26_decision rc26_mcu_transport
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 联调阶段入口

涉及真实运动或机构动作时必须启动 `rc26_mcu_transport`，并确认同一时刻只有一个 `/cmd_vel` 发布者。完整导航/决策链默认由 [r2_active_side.yaml](/home/potato/RC_2026/src/rc26_bringup/config/r2_active_side.yaml) 选择 [r2_red.yaml](/home/potato/RC_2026/src/rc26_bringup/config/r2_red.yaml) 或 [r2_blue.yaml](/home/potato/RC_2026/src/rc26_bringup/config/r2_blue.yaml) 读取点云路径、行为树入口和决策参数；现场切换 BT 或调整分段距离时优先修改对应红/蓝配置文件。

### 遥控

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
./start_r2_teleop.sh
```

遥控入口与完整导航/决策入口不能同时接管 `/cmd_vel`。

### 建图

```bash
ros2 launch rc26_bringup test_mapping.launch.py
```

### 定位

```bash
ros2 launch rc26_bringup test_localization_chain.launch.py
```

### 重定位

```bash
ros2 launch rc26_bringup test_relocalization.launch.py
```

### Odom-Only 导航/决策完整链路

```bash
ros2 launch rc26_bringup bringup.launch.py \
  run_mode:=navigation \
  use_decision:=true \
  use_rviz:=false
```

预期节点包含 odometry 相关节点、`/rc26_decision` 和 `rc26_mcu_transport`。预期 topic 至少包含 `/odom` 与 `/cmd_vel`。导航模式下 odometry 会以 `start_sensor_scan:=false` 装配，决策节点会等待 `/odom` 连续新鲜且低速稳定后再 tick 行为树。

### Grid Heading 正式入口

```bash
ros2 launch rc26_bringup grid_heading.launch.py
```

方向由当前红/蓝运行配置维护：

```yaml
grid_heading_direction: "left"  # forward | left | right | backward
grid_heading_turn_max_speed_radps: 1.0
grid_heading_align_max_speed_radps: 0.30
```

本入口只执行 `GridTurn -> GridHeadingAlign`，直接发布 `cmd_vel.angular.z`，不触发推杆或激光事件。实车运行前必须停用遥控和其它 `/cmd_vel` 发布者；若 `/odom` 和 `rc26_mcu_transport` 已由其它入口提供，可关闭重复链路：

```bash
ros2 launch rc26_bringup grid_heading.launch.py \
  start_odometry:=false \
  start_mcu_transport:=false
```

### Odom 闭环右转分段入口

```bash
ros2 launch rc26_bringup odom_right_turn_nav.launch.py
```

本入口启动后会先等 `/odom` 连续低速稳定，满足通用 startup odom gate 后才加载并 tick 右转验证树。若默认由本入口启动 odometry，会关闭 bootstrap `/odom`，避免把启动占位姿态当成可运动依据。

树默认执行：

```text
OdomDriveX(+0.40m) -> RelativeYawTarget(-pi/2) -> OdomTurnToYaw -> OdomDriveX(-0.70m)
```

右转验证树内固定验收距离和相对 yaw，速度、容差、topic 和超时复用通用 odom 相对导航参数。实车运行前必须停用遥控和其它 `/cmd_vel` 发布者；若 `/odom` 和 `rc26_mcu_transport` 已由其它入口提供，可关闭重复链路：

```bash
ros2 launch rc26_bringup odom_right_turn_nav.launch.py \
  start_odometry:=false \
  start_mcu_transport:=false
```

## 模块级测试

### 里程计接口测试

```bash
ros2 launch rc26_bringup test_odom_interface.launch.py
ros2 topic echo /odom --once
ros2 topic echo /registered_scan --once
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_footprint base_link
```

预期 `/odom` 非空，协方差按上游透传，TF 中存在 `odom -> base_footprint -> base_link`。

### 传感器扫描测试

```bash
ros2 launch rc26_bringup test_sensor_scan.launch.py
ros2 topic echo /sensor_scan --once
ros2 topic echo /odometry --once
ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_link livox_frame
```

`rc26_sensor_scan` 当前只是点云/里程计时空对齐模块。默认导航模式不启动它；需要单独验证局部点云整理能力时使用本测试入口。

### 定位模块测试

```bash
ros2 launch rc26_bringup test_localization.launch.py
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /localization/pose_with_cov --once
ros2 topic echo /localization/diagnostics --once
```

定位链用于建图/定位联调，不属于默认导航模式必需链路。

### 完整里程计链测试

```bash
ros2 launch rc26_bringup test_odometry_chain.launch.py
ros2 topic list | grep -E "(state_estimation|odom|odometry|registered_scan|sensor_scan)"
ros2 topic echo /state_estimation --once
ros2 topic echo /odom --once
ros2 topic echo /registered_scan --once
ros2 topic hz /odom
ros2 run tf2_tools view_frames
```

若雷达能 ping 通但 `/livox/lidar` 无数据，可追加：

```bash
ros2 launch rc26_bringup test_odometry_chain.launch.py start_mid360_driver:=true recover_mid360_stream:=true
```

### 决策系统测试

```bash
ros2 launch rc26_bringup bringup.launch.py \
  run_mode:=navigation \
  use_decision:=true
```

武馆区、主树、MF 预选赛或独立验收树通过 `r2_runtime.paths.behavior_tree_file` 切换。`relative_segment_nav_tree.xml` 可用于验收三个独立导航动作：`OdomDriveX` 只发 `linear.x`，`OdomDriveY` 只发 `linear.y`，`OdomTurnToYaw` 只发 `angular.z`；到达容差、halt、odom 过期或超时时都应停车。

## 验证清单

| 模块 | 话题/TF | 预期结果 |
|------|---------|----------|
| odom_interface | `/odom` | `odom -> base_footprint` 里程计与动态 TF 持续发布 |
| rc26_point_lio | `/state_estimation` + `/cloud_registered` | LIO 里程计与原生配准点云持续输出 |
| rc26_decision | `/cmd_vel` | 完整导航链中按行为树阶段串行发布速度命令 |
| rc26_mcu_transport | `/cmd_vel` + `/mechanism/*` | 消费速度命令并下发 `POSE_TARGET(0x0C)`，同时提供机构 raw transport |
| localization | `map -> odom` + diagnostics | 仅在定位/建图联调入口中作为定位状态输出 |
| sensor_scan | `/sensor_scan` + `/odometry` | 仅在传感器扫描或完整里程计测试入口中验证点云整理输出 |

## 调试技巧

```bash
ros2 node list
ros2 topic list

# 导航模式验收时应能看到 /odom 和 /cmd_vel。
ros2 topic echo /odom --once
ros2 topic echo /cmd_vel

# 若整车 bringup 前需要自动恢复 Mid-360，可追加：
# recover_mid360_stream:=true
ros2 launch rc26_bringup bringup.launch.py run_mode:=navigation use_decision:=false recover_mid360_stream:=true

ros2 run rqt_console rqt_console
ros2 run tf2_tools view_frames && evince frames.pdf
```
