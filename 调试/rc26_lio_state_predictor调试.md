# rc26_lio_state_predictor 调试

## 模块定位

`rc26_lio_state_predictor` 用来把上游 LIO 状态前向预测到更接近当前时刻的控制状态，输出 `/control_state` 和 `/control_degraded`。

## 适用场景

- 排查控制链为什么感觉“状态滞后”
- 验证 `control_state` 频率与时间戳是否符合控制器预期
- 离线回放 Bag 验证预测补偿与退化标志

## 前置条件

- 上游 `/odometry`、`/livox/imu`、`/degenerate_score` 正常
- 已 `source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"`

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_lio_state_predictor rc26_bringup
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐启动

最常见入口仍然是里程计总装链：

```bash
ros2 launch rc26_bringup odometry.launch.py
```

如果只想验证 predictor 自身，可单独运行：

```bash
ros2 run rc26_lio_state_predictor rc26_lio_state_predictor_node
```

## 最小验收

```bash
ros2 topic hz /control_state
ros2 topic echo /control_state --once
ros2 topic echo /control_degraded --once
ros2 topic echo /odometry --once | grep stamp -A 2
ros2 topic echo /control_state --once | grep stamp -A 2
```

离线回放时：

```bash
ros2 bag play your_bag_file_path.mcap --topics /odometry /livox/imu /degenerate_score
```

## 优先排查

- `/control_state` 频率上不去：先看上游 `/odometry` 与 IMU 是否正常，再检查是否真的启用了 predictor。
- 时间戳没有前推：先对比 `/odometry` 和 `/control_state` 的 `stamp`。
- `/control_degraded` 长期开启：先确认 `/degenerate_score` 是否持续低于阈值，再看上游定位链状态。

## 相关入口

- [建图启动](./建图启动.md)
- [联调顺序](./联调顺序.md)
- [rc26_point_lio调试](./rc26_point_lio调试.md)
