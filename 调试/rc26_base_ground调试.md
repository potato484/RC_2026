# rc26_base_ground 调试

## 模块定位

`rc26_base_ground` 已归档为 source-only 历史源码包。当前主链不编译它的运行时目标，不通过 bringup 启动它，也没有模块订阅 `base_ground/*` 输出。

本页仅用于显式恢复历史 base-ground 节点时的本地调试资料，不属于当前 R2 默认联调顺序。

## 适用场景

- 显式恢复归档目标后，单独验证台阶层级、稳定性和历史 `base_ground` TF
- 复现历史地形稳定性门控逻辑
- 确认历史参数与阈值，不得把 `base_ground/*` 输出接回当前主链

## 前置条件

- 已 `source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"`
- 上游 `/odom` 或等价底盘状态已正常输出
- `odom -> base_link` TF 已存在

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_base_ground \
  --cmake-args -DRC26_ENABLE_ARCHIVED_RUNTIME_TARGETS=ON
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐启动

```bash
ros2 launch rc26_base_ground base_ground_estimator.launch.py
```

当前整车联调不会通过 bringup 带起本包。

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

- [感知启动](./感知启动.md)
