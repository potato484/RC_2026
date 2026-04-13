# rc26_odom_interface 调试

## 模块定位

`rc26_odom_interface` 是 `odom -> base_link` 动态 TF 的当前权威者，负责把上游里程计映射到底盘坐标系，并规范化输出 `/odom`。

## 适用场景

- 排查 TF 是否冲突或缺失
- 验证 `/odom` 是否正常透传位姿、速度和协方差
- 给 `rc26_sensor_scan`、`rc26_localization` 和导航链排查底盘坐标真源

## 前置条件

- 已 `source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"`
- 上游 `/state_estimation` 正常
- `base_link -> livox_frame` 静态 TF 已存在

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_odom_interface rc26_bringup
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐启动

当前真实入口通过里程计总装链带起：

```bash
ros2 launch rc26_bringup odometry.launch.py
```

## 最小验收

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_monitor odom base_link
ros2 topic echo /odom --once
ros2 topic hz /odom
```

## 优先排查

- `/odom` 没输出：先确认 `/state_estimation` 是否在线。
- TF 冲突：`odom -> base_link` 只能有一个权威发布者，先排查是否额外启动了其他底盘 TF 节点。
- 下游说雷达系不对：先确认 `base_link -> livox_frame` 静态 TF 正确。

## 相关入口

- [建图启动](./建图启动.md)
- [联调顺序](./联调顺序.md)
- [rc26_sensor_scan调试](./rc26_sensor_scan调试.md)
