# rc26_sensor_scan 调试指南

本文档提供针对 `rc26_sensor_scan` 模块的具体调试步骤和命令行指令，用于验证其点云坐标转换和数据同步功能是否正常运行。

## 1. 编译模块

首先需要编译对应的模块，建议使用当前 AidLux 环境下实测更快的默认构建参数：

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j4 -l4' colcon build --parallel-workers 2 --packages-select rc26_sensor_scan rc26_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 2. 启动节点

目前该节点已集成在系统的里程计启动脚本中。请通过以下指令统一启动整个里程计和扫描处理链路：

```bash
ros2 launch rc26_bringup odometry.launch.py
```

*注：此节点依赖上游的点云数据和 `rc26_odom_interface` 提供的 `/odom` 数据，需确保上游链路通畅。*

## 3. 验证点云坐标转换

`rc26_sensor_scan` 的核心任务是将全局里程计坐标系下的点云转换回局部坐标系。

**检查转换后的点云话题输出：**
```bash
ros2 topic echo /sensor_scan --once
```
*预期结果：*
- 应当能看到点云数据输出。
- `header.frame_id` 应为雷达本体坐标系（例如 `livox_frame`）。
- 确认 `data` 数组不为空，说明点云成功进行了转换并发布。

**检查点云话题频率：**
```bash
ros2 topic hz /sensor_scan
```
*预期结果：频率应与原始的输入点云频率（通常约为 10Hz）基本一致。如果频率极低或没有输出，说明数据同步失败。*

## 4. 验证数据同步与透传里程计

该模块通过 `ApproxTime` 对点云和里程计进行时间戳同步，并同步透传里程计数据。

**检查透传的里程计话题：**
```bash
ros2 topic echo /odometry --once
```
*预期结果：*
- 输出包含位姿和速度数据的里程计信息。
- 比较 `/odometry` 和 `/odom` 话题，其速度部分（`twist.twist`）应当一致，不应有异常的高频噪声放大（因为取消了二次差分计算）。

**验证同步性能：**
使用命令同时查看点云和里程计的时间戳：
```bash
ros2 topic echo /sensor_scan/header/stamp --once
ros2 topic echo /odometry/header/stamp --once
```
*预期结果：两者的 `sec` 和 `nanosec` 应该非常接近，表明数据帧在时间轴上成功对齐。*

## 5. 常见问题排查

**问题：没有任何 `/sensor_scan` 或 `/odometry` 输出**
1. 检查输入话题 `/cloud_registered` 和 `/odom` 是否有数据：
   ```bash
   ros2 topic hz /cloud_registered
   ros2 topic hz /odom
   ```
2. 如果输入正常但无输出，通常是因为时间戳差异过大导致 `ApproxTime` 滤波器丢弃了数据。检查两个输入话题的时间戳差异。
3. 检查是否有 TF 错误（比如找不到 `odom` 到 `livox_frame` 的变换路径）：
   ```bash
   ros2 run tf2_ros tf2_echo odom livox_frame
   ```
   *预期结果：能正常输出 TF，说明 TF 树连通。*
