# Mid-360 雷达驱动调试指南

本文档提供 `mid360_driver` 在 RC_2026 项目（R2 机器人系统）中的详细调试与测试步骤。驱动已完成核心重构（事件驱动帧重组、时间同步滤波等），请按照以下步骤进行验证和排错。

## 1. 编译驱动

在工作空间根目录下，限制编译核心数以适应环境：

```bash
cd /home/potato/RC_2026
colcon build --parallel-workers 1 --packages-select mid360_driver
source install/setup.bash
```

## 2. 检查网络配置与系统参数

在启动节点前，需确保系统网络和 UDP 缓冲区已正确配置：

### 2.1 检查系统接收缓冲区上限
```bash
sysctl net.core.rmem_max
```
期望值应大于等于 `33554432` (32MB)。如果不足，可通过以下命令临时修改（需 sudo 权限）：
```bash
sudo sysctl -w net.core.rmem_max=33554432
```

### 2.2 检查与雷达的连通性
通常雷达的静态 IP 为 `192.168.1.1XX`，请根据实际配置 ping 雷达：
```bash
ping 192.168.1.100
```
确保无丢包且延迟在正常范围（<1ms）。

## 3. 启动雷达驱动节点

使用 launch 文件启动驱动节点加载默认参数：

```bash
ros2 launch mid360_driver mid360_driver.launch.py
```

观察终端输出日志：
- 正常情况应看到 `LidarPublisher` 成功初始化的提示。
- 若网络不通或雷达未开机，可能出现超时或无数据接收告警。
- 注意观察是否有丢包告警（如 `UDP packet loss detected`），若出现说明网络或缓冲区存在瓶颈。

## 4. 验证在线话题

开启一个新的终端，执行环境变量 source 后进行验证。

### 4.1 验证话题列表
```bash
source /home/potato/RC_2026/install/setup.bash
ros2 topic list
```
期望看到 `/livox/lidar` 和 `/livox/imu` 话题。

### 4.2 验证点云发布频率
```bash
ros2 topic hz /livox/lidar
```
期望：帧率稳定在 10Hz（即雷达设定的扫描频率）。由于驱动已从定时器改为基于协议帧的事件驱动发布，帧率应非常稳定，不再出现半帧抖动。

### 4.3 验证点云数据格式
```bash
ros2 topic echo /livox/lidar --once
```
期望：
- `fields` 列表严格包含 `x, y, z, intensity, timestamp`。
- `point_step` 必须为 `24`。

### 4.4 验证时间戳单调性与连续性
持续监控点云的头部时间戳，确保单调递增，无跳变回溯（NoSync 滤波验证）：
```bash
ros2 topic echo /livox/lidar --field header.stamp
```

### 4.5 验证 IMU 话题频率
```bash
ros2 topic hz /livox/imu
```
期望：帧率稳定在约 200Hz（驱动侧 IMU 定时器已调整为 4ms，能够完美覆盖该频率）。

## 5. 高阶调试：PTP 时间同步 (gPTP)

若环境启用了硬件 PTP 同步：

1. 确认 ptp4l 状态：
```bash
sudo ptp4l -i <网卡名> -H -m -f /etc/linuxptp/gptp-master.cfg
```
2. 确认 phc2sys 同步状态：
```bash
sudo phc2sys -c <网卡名> -s CLOCK_REALTIME -O 0
```
此时驱动接收到的时间戳将与系统时间（Host Time）高精度对齐，点云数据 `timestamp` 字段漂移被抑制。

## 6. 与下游节点联合测试

当雷达话题确认稳定后，可以带起 Point-LIO 等下游节点进行全链路验证：
```bash
ros2 launch rc26_bringup odometry.launch.py
```
监控 LIO 状态，观察是否出现由于特征提取导致的点云格式错位告警。由于 `Point` 结构体已与 PointCloud2 格式对齐（`{x,y,z,intensity,timestamp}`），兼容性应无问题。
