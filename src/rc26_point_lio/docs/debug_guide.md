# rc26_point_lio 调试指南

本文档提供针对 RC2026 机器人里程计模块（Point-LIO）的详细调试与测试步骤。

## 1. 编译与环境准备

在开始测试前，请确保工作空间已正确编译。建议使用以下命令单独编译里程计相关包，以节省时间。

```bash
# 编译 Point-LIO 及其下游核心节点
colcon build --parallel-workers 1 --packages-select rc26_point_lio rc26_odom_interface rc26_sensor_scan rc26_lio_state_predictor rc26_bringup
```

确保 source 环境：

```bash
source install/setup.bash
```

## 2. 基础功能测试（回放数据包）

使用录制好的 rosbag 进行离线验证是最高效的调试方式。

### 步骤 2.1：启动 Point-LIO

在一个终端中启动里程计节点（默认加载 Mid360 配置）：

```bash
ros2 launch rc26_point_lio point_lio.launch.py rviz:=true
```

### 步骤 2.2：播放数据包

在另一个终端中播放 rosbag。建议只发布 LiDAR 和 IMU 话题，避免干扰：

```bash
# 请将 <path_to_your_bag> 替换为实际路径
ros2 bag play <path_to_your_bag> --topics /livox/lidar /livox/imu
```

### 步骤 2.3：验证核心话题输出

检查里程计是否正常工作。

**1. 检查状态估计频率**

```bash
# 预期：若 config/mid360.yaml 中 publish_odometry_without_downsample: True
# 频率应接近 LiDAR 采样率或更高（取决于 IMU 处理机制），且稳定
ros2 topic hz /state_estimation
```

**2. 检查里程计数据完整性**

```bash
# 检查是否包含协方差 (covariance) 数据
# 预期：pose.covariance 和 twist.covariance 不应全为 0
ros2 topic echo /state_estimation --once --field pose.covariance
```

**3. 检查 TF 树**

```bash
# 验证 odom -> body 的 TF 是否发布
ros2 run tf2_ros tf2_monitor odom body
```

## 3. 进阶性能验证

### 3.1 验证控制延迟（Phase 0/3 验收）

Point-LIO 经过配置优化，应输出实时的状态估计以供控制使用。

**检查命令**：

```bash
# 比较消息头时间戳与当前时间（需在实机或模拟时间同步环境下）
ros2 topic echo /state_estimation --once | grep stamp -A 2
```

**验证标准**：
- 如果启用了 `rc26_lio_state_predictor`，请检查 `/control_state` 话题。
- 确认 `publish_odometry_without_downsample` 为 `True`。

### 3.2 验证退化检测（Phase 2 验收）

在走廊或特征稀疏区域，退化分数应下降。

**监控命令**：

```bash
# 实时监控退化分数
# 正常场景：> 1.0 (典型值)
# 退化场景（如长走廊）：< 0.1 或接近 0
ros2 topic echo /degenerate_score
```

### 3.3 验证鲁棒性（Phase 1/4 验收）

针对剧烈运动（如原地快速旋转）的测试。

**操作**：
1. 机器人进行快速原地旋转（角速度 > 2 rad/s）。
2. 观察 Rviz 中点云是否分层或漂移。
3. 检查是否触发二次迭代（需查看终端日志）。

**日志检查**：

```bash
# 如果启用了 adaptive_second_iter_enable
# 在 launch 终端中查找类似日志：
# "[Phase4] second iter triggered: res=... omega=..."
```

## 4. 全链路联调测试

验证里程计数据是否正确传递给 Nav2 导航栈。

### 步骤 4.1：启动完整里程计链路

```bash
# 启动包含 sensor_scan 和 odom_interface 的完整链路
ros2 launch rc26_bringup odometry.launch.py
```

### 步骤 4.2：检查导航话题

```bash
# 检查最终输出给 Nav2 的话题（通常是 /odometry 或 /control_state）
ros2 topic echo /odometry --once
```

**验证点**：
- 确保 `child_frame_id` 为 `base_link`（即已完成从 `body` 到 `base_link` 的 TF 变换）。
- 确保协方差矩阵已正确透传。

## 5. 常见问题排查

| 现象 | 可能原因 | 排查建议 |
|------|----------|----------|
| **Rviz 中点云严重抖动** | 外参错误 或 时间同步失效 | 检查 `mid360.yaml` 中的 `extrinsic_T/R` 和 `time_diff_lidar_to_imu`。 |
| **轨迹飞出/定位丢失** | IMU 数据异常 或 初始重力未对齐 | 确认 IMU 安装方向，检查 `gravity` 参数是否与实际重力方向一致。 |
| **/state_estimation 频率低** | 下采样未关闭 | 检查 `publish_odometry_without_downsample` 是否为 `True`。 |
| **编译报错 std_msgs 缺失** | 依赖未安装 | 检查 `package.xml` 和 `CMakeLists.txt` 是否包含 `std_msgs`。 |

---

**备注**：所有参数配置位于 `src/rc26_point_lio/config/mid360.yaml`。修改后无需重新编译，重启节点即可生效。
