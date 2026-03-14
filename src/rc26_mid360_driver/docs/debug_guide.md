# rc26_mid360_driver 调试指南

本文档提供 `rc26_mid360_driver` 在 RC_2026 项目（R2 机器人系统）中的详细调试与测试步骤。驱动已完成核心重构（事件驱动帧重组、时间同步滤波等），请按照以下步骤进行验证和排错。

## 1. 编译驱动

在工作空间根目录下，限制编译核心数以适应环境：

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
colcon build --symlink-install --parallel-workers 3 --packages-select rc26_mid360_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
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

若需要在当前 AidLux / QTI 系统上开机自动生效，建议直接修改 `/etc/sysctl.conf`，再重新加载 sysctl 配置：
```bash
sudo sed -i 's/^net\.core\.rmem_max=.*/net.core.rmem_max=33554432/' /etc/sysctl.conf
sudo sysctl --system
sysctl net.core.rmem_max
```
注意：该系统的 `/etc/sysctl.d/99-sysctl.conf` 会链接回 `/etc/sysctl.conf`，如果仅新增单独的 `sysctl.d` 文件，后续仍可能被 `/etc/sysctl.conf` 中的旧值覆盖。

### 2.2 检查与雷达的连通性
当前 R2 实机中，Mid-360 雷达 IP 为 `192.168.1.140`，驱动接收端（`host_ip`）为本机 `br-lan` 地址 `192.168.1.50`。请 ping 雷达实际地址，而不是 ping 本机 `host_ip`：
```bash
ping 192.168.1.140
```
确保无丢包且延迟在正常范围（<1ms）。若 `ip route get 192.168.1.140` 显示为 `local ... dev lo`，说明目标 IP 被本机占用，当前连通性检查无效。

## 3. 启动雷达驱动节点

使用 launch 文件启动驱动节点加载默认参数：

```bash
ros2 launch rc26_mid360_driver mid360_driver.launch.py
```

观察终端输出日志：
- 正常情况应看到 `LidarPublisher` 成功初始化的提示。
- 若网络不通或雷达未开机，可能出现超时或无数据接收告警。
- 若 `/livox/imu` 正常但 `/livox/lidar` 无数据，优先检查雷达端固件是否正确更新协议帧计数；当前驱动已在 `frame_cnt` 恒定不变时回退到 `lidar_publish_time_interval` 时间窗口分帧。
- 注意观察是否有丢包告警（如 `UDP packet loss detected`），若出现说明网络或缓冲区存在瓶颈。

## 4. 验证在线话题

开启一个新的终端，执行环境变量 source 后进行验证。

### 4.1 验证话题列表
```bash
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
ros2 topic list
```
期望看到 `/livox/lidar` 和 `/livox/imu` 话题。

### 4.2 验证点云发布频率
```bash
ros2 topic hz /livox/lidar
```
期望：帧率稳定在 10Hz（即雷达设定的扫描频率）。驱动优先基于协议帧计数进行事件驱动发布；若设备固件未正确更新 `frame_cnt`，则自动回退到 `lidar_publish_time_interval`（默认 `0.1s`）的时间窗口分帧，因此该参数必须与雷达实际扫描频率一致。

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

## 7. 常见问题排查

- **节点启动成功但 `/livox/lidar` 无数据**：优先检查雷达供电、`ping 192.168.1.140` 是否连通，以及 `host_ip` 是否仍保持为本机 `192.168.1.50`，避免把雷达 IP 错配到本机网口。
- **主机完全收不到 `56301/56401` UDP 包**：当前 R2 实机已确认过一种特殊情况：雷达内部 `state_info_host_ipcfg` / `pointcloud_host_ipcfg` / `imu_host_ipcfg` 的 `src_port` 可能被写成 `0`。此时虽然能 `ping 192.168.1.140`，但 ROS 驱动和自写 UDP 探针都收不到任何点云/IMU。可直接执行：
  ```bash
  python3 src/rc26_mid360_driver/scripts/recover_mid360_stream.py --lidar-ip 192.168.1.140 --host-ip 192.168.1.50
  ```
  该脚本会复用官方 Livox-SDK2，把 `56200/56300/56400` 源端口重新写回雷达，并在必要时自动重启设备；实机验证后，`/livox/lidar` 可恢复到约 `10Hz`，`/livox/imu` 恢复到约 `200Hz`。
- **整链启动时自动恢复**：`src/rc26_bringup/launch/odometry.launch.py` 与 `src/rc26_bringup/launch/bringup.launch.py` 新增了 `recover_mid360_stream:=true` 开关。若要在启动整条里程计链前先自动执行恢复，可使用：
  ```bash
  ros2 launch rc26_bringup odometry.launch.py recover_mid360_stream:=true
  ros2 launch rc26_bringup bringup.launch.py slam:=false use_decision:=false recover_mid360_stream:=true
  ```
  首次执行会自动拉取并编译官方 `Livox-SDK2`，因此会比普通启动慢一些；后续会复用本地缓存。
- **只有 `/livox/imu` 有数据、`/livox/lidar` 无数据**：检查雷达固件是否正常更新 `frame_cnt`；当前驱动会用 `lidar_publish_time_interval` 兜底分帧，若该参数与实际扫描频率不一致，可能表现为点云不发布或频率异常。
- **点云频率抖动或出现丢包告警**：检查是否存在网卡抢占、交换机问题或省电模式；必要时独占网口直连雷达，并保持接收缓冲区不低于文档建议值。
- **时间戳异常导致下游拒收**：检查 PTP/gPTP 是否真正生效，以及系统时间是否稳定；若未启用硬件同步，需重点确认驱动日志中是否存在时间回跳或滤波异常提示。

## 8. R2 实机验证记录（2026-03-07）

以下结论已在当前 R2 实机上完成验证，可作为后续联调的默认参考：

- **网络拓扑**：本机 `br-lan` / `host_ip` = `192.168.1.50`，Mid-360 = `192.168.1.140`。
- **连通性检查**：应执行 `ping 192.168.1.140`，不要执行 `ping 192.168.1.50`，因为后者是本机接收口地址。
- **UDP 端口行为**：雷达点云从 `192.168.1.140:56300` 发往本机 `192.168.1.50:56301`，IMU 从 `192.168.1.140:56400` 发往本机 `192.168.1.50:56401`。
- **系统缓冲区**：`net.core.rmem_max` 需至少为 `33554432`，否则高频 UDP 数据可能在进入驱动前被系统丢弃。
- **持久化方式**：当前 AidLux / QTI 系统应直接修改 `/etc/sysctl.conf` 持久化 `net.core.rmem_max=33554432`，不要只新增单独的 `sysctl.d` 文件，否则可能被 `99-sysctl.conf` 回链配置覆盖。
- **固件兼容性**：当前实机上点云 UDP 包的 `frame_cnt` 可能持续为 `0`，不能仅依赖协议帧计数做分帧；驱动已验证可回退到 `lidar_publish_time_interval=0.1` 的时间窗口分帧。
- **实测结果**：`/livox/lidar` 稳定约 `10Hz`，`/livox/imu` 稳定约 `200Hz`，`/livox/lidar` 的 `fields` 为 `x, y, z, intensity, timestamp`，`point_step = 24`，头部时间戳单调递增。
- **补充恢复记录**：若雷达内部 host ipcfg 的 `src_port` 被异常写成 `0`，官方 SDK2 能正常发现设备，但点云/IMU不会出流；将 `state/point/imu` 的源端口修正为 `56200/56300/56400` 后，再执行一次软件重启，数据即可恢复。

建议在更换雷达、升级固件或重刷网络配置后，至少重新执行以下命令完成一次快速回归：

```bash
sysctl net.core.rmem_max
ping 192.168.1.140
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
ros2 launch rc26_mid360_driver mid360_driver.launch.py
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic echo /livox/lidar --once
ros2 topic echo /livox/lidar --field header.stamp
```
