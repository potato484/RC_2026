# rc26_lio_state_predictor

`rc26_lio_state_predictor` 用于在里程计链路末端做控制态前向预测，降低控制使用的状态时延。

## 功能

- 订阅 `/odometry`（来自 `rc26_sensor_scan`）作为预测初始状态
- 订阅 `/livox/imu`，优先使用 IMU 角速度积分姿态
- 订阅 `/degenerate_score`，输出退化告警
- 以默认 `200Hz` 发布 `/control_state`（供 Nav2 使用）
- 发布 `/control_degenerate_score` 和 `/control_degraded`

## 话题

### 订阅
- `odometry` (`nav_msgs/msg/Odometry`)
- `livox/imu` (`sensor_msgs/msg/Imu`)
- `degenerate_score` (`std_msgs/msg/Float64`)

### 发布
- `control_state` (`nav_msgs/msg/Odometry`)
- `control_degenerate_score` (`std_msgs/msg/Float64`)
- `control_degraded` (`std_msgs/msg/Bool`)

## 参数（默认值）

- `publish_rate_hz: 200.0`
- `fixed_predict_ahead_sec: 0.0`
- `pos_cov_increase_m2_per_s: 0.02`
- `yaw_cov_increase_rad2_per_s: 0.01`
- `degeneracy_ratio_threshold: 0.02`

参数文件：`src/rc26_bringup/config/lio_state_predictor.yaml`

## 验收建议

```bash
ros2 topic hz /control_state
ros2 topic echo /control_degraded
ros2 topic echo /control_degenerate_score
```

期望：
- `/control_state` 频率约 `200Hz`
- 退化场景（走廊/贴墙）下 `control_degraded` 可触发 `true`
