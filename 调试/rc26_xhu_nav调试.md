# rc26_xhu_nav 调试

## 模块定位

`rc26_xhu_nav` 是 R2 当前唯一的 3D 导航实现宿主包，统一承载 topo、body-aware planner、local planner、mode manager 和 runtime executor。

## 适用场景

- 排查 `topo_nav_node`、`xhu_motion_mode_manager_node`、`xhu_motion_runtime_node`
- 验证 `/xhu_nav/*` 话题与 `cmd_vel` 权威链
- 排查导航为什么一直 `hold`、频繁 `replan` 或被语义 gate 阻塞

## 前置条件

- `map -> odom` 与 `/odom` 正常
- 上游定位、地形、keepout 链路在线
- 已 `source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"`

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_interfaces rc26_xhu_nav rc26_bringup
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐启动

最接近真实链路：

```bash
ros2 launch rc26_bringup bringup.launch.py \
  slam:=false \
  use_decision:=false \
  prior_pcd_file:=${RC26_WS:-$HOME/RC_2026}/src/rc26_bringup/pcd/default.pcd
```

只调 mode manager：

```bash
ros2 launch rc26_xhu_nav xhu_motion_mode_manager.launch.py
```

只调 topo action 入口：

```bash
ros2 launch rc26_xhu_nav topo_nav.launch.py
```

## 最小验收

```bash
ros2 topic echo /xhu_nav/corridor_cmd --once
ros2 topic echo /xhu_nav/motion_mode_state --once
ros2 topic echo /xhu_nav/tracking_state --once
ros2 topic echo /xhu_nav/local_planner_state --once
ros2 topic echo /xhu_nav/semantic_gate --once
ros2 topic echo /cmd_vel --once
ros2 service type /set_xhu_motion_mode
```

全向输出验收：

- corridor 存在横向偏移时，`/xhu_nav/local_planner_state` 与 `/xhu_nav/tracking_state` 的 `cmd_vy` 应允许出现非零值
- 若进入 `rotate_in_place` 恢复态，预期 `cmd_vx=0`、`cmd_vy=0`、`cmd_wz!=0`

## 常用模式切换

```bash
ros2 service call /set_xhu_motion_mode rc26_interfaces/srv/SetXhuMotionMode \
  "{mode: 'plane_move', timeout: 0.0, reason: 'manual_check'}"

ros2 service call /set_xhu_motion_mode rc26_interfaces/srv/SetXhuMotionMode \
  "{mode: 'mf_traverse', timeout: 10.0, reason: 'watchdog_check'}"
```

## 优先排查

- `/cmd_vel` 长期为零：先看 `/xhu_nav/motion_mode_state` 是否还在 `hold`，再看 `/xhu_nav/semantic_gate`。
- `cmd_vy` 长期为 0：优先检查 corridor/lookahead 是否真的需要横移，再看 `xhu_motion_runtime_node` 是否拿到了最新 `odom.twist.twist.linear.y`。
- `/xhu_nav/tracking_state` 频繁 `REPLAN`：优先检查 corridor、keepout 和地形输入。
- 单独拉起 mode manager 正常、整车链异常：先检查 bringup 传入的 graph 和上游健康度。

## 相关入口

- [导航启动](./导航启动.md)
- [决策启动](./决策启动.md)
- [rc26_localization调试](./rc26_localization调试.md)
- [rc26_terrain调试](./rc26_terrain调试.md)
