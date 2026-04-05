# rc26_lio_state_predictor 调试指南

本文档旨在指导开发者如何测试和验证 `rc26_lio_state_predictor` 模块的功能。

## 1. 编译模块

在进行测试前，请确保模块已正确编译。

```bash
MAKEFLAGS='-j4 -l4' colcon build --parallel-workers 2 --packages-select rc26_lio_state_predictor --cmake-args -DCMAKE_BUILD_TYPE=Release
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 2. 单元测试（离线 Bag 回放）

使用离线录制的 rosbag 数据进行功能验证，无需真机即可调试。

### 步骤 1：准备数据
确保你有一个包含以下话题的 rosbag：
- `/odometry` (nav_msgs/Odometry)
- `/livox/imu` (sensor_msgs/Imu)
- `/degenerate_score` (std_msgs/Float64, 可选)

### 步骤 2：启动预测节点
在一个终端中启动节点。这里我们手动指定参数以便观察。

```bash
ros2 run rc26_lio_state_predictor rc26_lio_state_predictor_node --ros-args \
    -p publish_rate_hz:=200.0 \
    -p odometry_topic:=/odometry \
    -p imu_topic:=/livox/imu \
    -p control_state_topic:=/control_state
```

### 步骤 3：回放数据
在另一个终端回放数据。

```bash
ros2 bag play your_bag_file_path.mcap --topics /odometry /livox/imu /degenerate_score
```

### 步骤 4：验证输出频率
检查输出话题 `/control_state` 是否稳定在 200Hz 左右。

```bash
ros2 topic hz /control_state
```

**预期结果**：平均频率应接近 200Hz。

### 步骤 5：验证预测延迟补偿
对比输入 `/odometry` 和输出 `/control_state` 的时间戳。

```bash
# 查看输入话题信息
ros2 topic echo /odometry --once | grep "stamp" -A 2

# 查看输出话题信息
ros2 topic echo /control_state --once | grep "stamp" -A 2
```

**预期结果**：`/control_state` 的 `header.stamp` 秒数应大于或等于 `/odometry` 的 `header.stamp`，且接近当前系统时间（在 play sim_time 模式下接近 bag 当前时间）。

### 步骤 6：验证退化标志
如果 bag 中包含退化场景（`degenerate_score` 很小），观察 `/control_degraded` 话题。

```bash
ros2 topic echo /control_degraded
```

**预期结果**：当 `/degenerate_score` 低于阈值（默认 0.02）时，`data` 应为 `True`。

## 3. 集成测试（Launch 启动）

在整车联调或仿真环境中，通常通过 launch 文件启动。

### 启动指令
```bash
ros2 launch rc26_bringup odometry.launch.py
```
*(注：假设该 launch 文件已集成 lio_state_predictor)*

### 检查节点状态
```bash
ros2 node list | grep lio_state_predictor
ros2 node info /lio_state_predictor
```

## 4. 常见问题排查

- **输出频率不稳定**：
  - 检查 CPU 占用率。
  - 检查 `publish_rate_hz` 参数设置。

- **控制状态无输出**：
  - 检查 `/odometry` 和 `/livox/imu` 是否有输入数据。
  - 检查话题名称是否与参数配置一致。

- **预测结果漂移严重**：
  - 检查 IMU 时间戳是否与里程计时间戳同步。
  - 检查 IMU 数据质量（零偏是否过大）。
