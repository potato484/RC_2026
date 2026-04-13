# rc26_base_ground 调试

## 模块定位

`rc26_base_ground` 负责把连续高度变化压成离散地形层级，并发布地形稳定、操作稳定、被举起等安全语义，供导航与机构动作消费。

## 适用场景

- 单独验证台阶层级、稳定性和 `base_ground` TF
- 排查导航或机构动作为什么因为“地形不稳”被门控
- 联调 `rc26_xhu_nav`、`rc26_terrain`、`rc26_mechanism` 之前先确认地形语义真源

## 前置条件

- 已 `source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"`
- 上游 `/control_state` 或等价底盘状态已正常输出
- `odom -> base_link` TF 已存在

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_base_ground rc26_bringup
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐启动

```bash
ros2 launch rc26_base_ground base_ground_estimator.launch.py
```

如果只是整车联调，通常直接通过整车链路带起，不需要单独启动。

## 最小验收

```bash
ros2 topic echo /base_ground/level --once
ros2 topic echo /base_ground/stair_delta --once
ros2 topic echo /base_ground/stable_terrain --once
ros2 topic echo /base_ground/stable_operation --once
ros2 topic echo /base_ground/is_lifted --once
ros2 topic echo /base_ground/ground_z_continuous --once
ros2 run tf2_ros tf2_echo odom base_ground
```

## 常用在线调参

```bash
ros2 param set /base_ground_estimator h0_override_enable true
ros2 param set /base_ground_estimator h0_override_m 0.0
ros2 param set /base_ground_estimator tol_level_m 0.05
ros2 param set /base_ground_estimator tol_stable_z_std_m 0.02
ros2 param set /base_ground_estimator tol_stable_ang_vel_rps 0.08
```

## 优先排查

- `/base_ground/is_lifted` 误触发：先看起步阶段是否晃动过大，再检查 `h0_override_*` 和稳定窗口阈值。
- 层级一直不跳变：先确认上游高度输入和 `odom -> base_link` 正常，再看 `tol_level_m` 是否过大。
- `base_ground` TF 不存在：先确认节点已启动，再检查上游 `odom` 时间戳是否连续。

## 相关入口

- [导航启动](./导航启动.md)
- [感知启动](./感知启动.md)
- [联调顺序](./联调顺序.md)
