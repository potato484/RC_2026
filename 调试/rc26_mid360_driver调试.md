# rc26_mid360_driver 调试

## 模块定位

`rc26_mid360_driver` 是 R2 当前的 Mid-360 雷达驱动，负责把 UDP 点云与 IMU 数据发布到 `/livox/lidar` 和 `/livox/imu`。

## 适用场景

- 单独验证雷达链路是否在线
- 给 `rc26_point_lio`、`rc26_sensor_scan`、`rc26_localization` 排查上游输入
- 排查“能 ping 通但没有点云/IMU”的网络与流恢复问题

## 前置条件

- Mid-360 实机地址为 `192.168.1.140`
- 主机接收地址 `host_ip` 为 `192.168.1.50`
- `sysctl net.core.rmem_max` 不低于 `33554432`

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_mid360_driver rc26_bringup
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐启动

```bash
ros2 launch rc26_mid360_driver mid360_driver.launch.py
```

如果雷达能 ping 通但 ROS 话题没有数据，可直接恢复数据流：

```bash
python3 src/rc26_mid360_driver/scripts/recover_mid360_stream.py \
  --lidar-ip 192.168.1.140 \
  --host-ip 192.168.1.50
```

或者通过总装链在启动前自动恢复：

```bash
ros2 launch rc26_bringup odometry.launch.py recover_mid360_stream:=true
```

## 最小验收

```bash
ping 192.168.1.140
ros2 topic list | grep livox
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic echo /livox/lidar --once
ros2 topic echo /livox/lidar --field header.stamp
```

## 优先排查

- 能 ping 通但没点云：先确认 `host_ip` 还是 `192.168.1.50`，再跑恢复脚本。
- 时间戳不连续：优先看网络抖动和恢复后的流状态。
- 只 ping `192.168.1.50` 没意义：这是本机接收口地址，必须 ping 雷达 `192.168.1.140`。

## 相关入口

- [建图启动](./建图启动.md)
- [感知启动](./感知启动.md)
- [rc26_point_lio调试](./rc26_point_lio调试.md)
