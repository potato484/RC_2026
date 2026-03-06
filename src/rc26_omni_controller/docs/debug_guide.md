# rc26_omni_controller 调试指南

本文档提供关于如何逐步调试和测试 `rc26_omni_controller` 模块的详细操作指令。

## 1. 编译模块

首先，我们需要限定编译核心数并单独编译全向控制器模块，以验证代码的正确性：

```bash
cd /home/potato/RC_2026
colcon build --parallel-workers 1 --packages-select rc26_omni_controller
```

如果涉及启动文件或配置文件的修改，建议同时编译 `rc26_bringup`：

```bash
colcon build --parallel-workers 1 --packages-select rc26_omni_controller rc26_bringup
```

编译完成后，刷新环境变量：

```bash
source /home/potato/RC_2026/install/setup.bash
```

## 2. 启动控制器与仿真测试环境

使用专门的测试 launch 文件启动全向控制器环境：

```bash
ros2 launch rc26_bringup test_omni_controller.launch.py
```

*注意：启动后请检查终端输出，确认 `FollowPath` 控制器成功创建并激活。预期插件名称为 `rc26_omni_controller::OmniPidPursuitController`。*

## 3. 监控关键调试 Topic

控制器运行期间，会发布多个性能与状态指标。打开新的终端，刷新环境变量后，通过以下指令监控不同层面的运行状态：

### 3.1 性能耗时监控
用于评估控制器的计算开销（Phase 1 优化目标为 P95 ≤ 2ms）：

```bash
# 监控整体计算耗时
ros2 topic echo /compute_time_ms

# 监控碰撞检测段独立耗时
ros2 topic echo /collision_check_ms
```

### 3.2 碰撞与制动验证
用于验证物理制动距离模型是否正常工作：

```bash
# 监控当前检测到的最小威胁距离
ros2 topic echo /collision_d_min

# 监控基于威胁距离计算出的安全速度上限
ros2 topic echo /v_safe
```

### 3.3 基础控制信息监控
检查导航控制与底盘的交互：

```bash
# 监控下发给底盘的速度指令
ros2 topic echo /cmd_vel

# 监控里程计反馈
ros2 topic echo /odometry
```

## 4. 动态参数在线调试

`rc26_omni_controller` 支持通过 ROS 2 的参数机制在线动态调参。在不断开控制器的前提下，打开新终端执行以下指令调节各项功能：

### 4.1 制动模型参数标定
调节碰撞响应的敏感度与制动强度：

```bash
# 调整最近威胁距离安全余量（默认 0.15 m）
ros2 param set /controller_server FollowPath.brake_margin 0.15

# 调整制动减速度模型参数（默认 0.8 m/s²）
ros2 param set /controller_server FollowPath.brake_accel 0.8
```

### 4.2 轨迹跟踪参数标定
优化弯道内切和轨迹贴合度：

```bash
# 调整横向误差收敛增益（推荐从小值开始标定，默认 1.5）
ros2 param set /controller_server FollowPath.lateral_error_gain 1.5

# 调整横向误差限幅，防止极端情况下指令爆炸（默认 0.3 m）
ros2 param set /controller_server FollowPath.lateral_error_max 0.3
```

### 4.3 加速度限幅与曲率前馈标定
优化麦克纳姆轮底盘在各轴的动态表现：

```bash
# 开启或关闭曲率角速度前馈（验证符号正确前请保持 false）
ros2 param set /controller_server FollowPath.enable_curvature_ff true

# 调整纵向加速度限幅（默认 1.0 m/s²）
ros2 param set /controller_server FollowPath.a_lim_x 1.0

# 调整横向加速度限幅（默认 0.6 m/s²，建议偏保守）
ros2 param set /controller_server FollowPath.a_lim_y 0.6
```

## 5. 录制运行数据 (Rosbag) 验收

为进行指标验收（如横向偏差 RMS、制动距离方差等），需要在运行时录制对应 topic 的数据包。

开启录制（在给定的测试路线 A、B、C 场景中各录制至少 3 次）：

```bash
ros2 bag record /odometry /tf /tf_static /cmd_vel /compute_time_ms /collision_check_ms /collision_d_min /v_safe /DM_IMU
```

录制完成后，使用 `Ctrl+C` 结束，将生成的数据包用于后续的离线分析以确认改进方案指标是否达标。

## 6. 常见问题排查

若看不到 `/compute_time_ms`、`/collision_check_ms` 等调试 topic，请先确认 `controller_server` 已成功加载 `rc26_omni_controller::OmniPidPursuitController`，并检查 `nav2_params.yaml` 中 `FollowPath` 对应插件名是否与实际库导出名称一致。

- **`/v_safe` 长时间过低**：优先检查 `/collision_d_min` 是否持续偏小，以及碰撞半径、制动距离参数是否设置过于保守。
- **底盘出现横摆抖动或跟踪振荡**：重点回看 `FollowPath.a_lim_x`、`FollowPath.a_lim_y` 与角速度前馈参数，确认横向限幅没有过激，同时结合 `/odometry` 检查反馈噪声。
- **控制器无法加载**：若 `controller_server` 启动时报插件不存在，检查插件 XML 导出、安装产物以及 `nav2_params.yaml` 中类名是否完全匹配。
