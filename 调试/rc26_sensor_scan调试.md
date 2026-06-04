# rc26_sensor_scan 调试

## 模块定位

`rc26_sensor_scan` 负责把点云和里程计做时空对齐，输出当前主链使用的 `/sensor_scan` 与 `/odometry`。

## 适用场景

- 排查 `/sensor_scan` 是否存在、频率是否正常
- 验证点云 frame_id 和 `/odometry` 时间戳对齐
- 给当前定位、导航和感知链排查输入问题

## 前置条件

- 已 `source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"`
- 上游 `/registered_scan`、`/odom` 已存在

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_sensor_scan rc26_bringup
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐启动

当前真实入口通过里程计总装链带起：

```bash
ros2 launch rc26_bringup odometry.launch.py
```

## 最小验收

```bash
ros2 topic hz /sensor_scan
ros2 topic echo /sensor_scan --once
ros2 topic echo /odometry --once
ros2 run tf2_ros tf2_echo odom livox_frame
```

## 优先排查

- `/sensor_scan` 没输出：先看 `/registered_scan` 和 `/odom` 是否都在。
- 时间戳不对齐：先对比 `/sensor_scan` 和 `/odometry` 的 `header.stamp`。
- frame 不对：先排查 `odom -> livox_frame` TF 和静态外参。

## 相关入口

- [感知启动](./感知启动.md)
- [建图启动](./建图启动.md)
