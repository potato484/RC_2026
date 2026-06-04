# rc26_decision 调试

## 模块定位

`rc26_decision` 是 R2 的主决策包，通过 BehaviorTree.CPP 组织比赛流程，并通过 `NavToPose`、机构动作和 keepout 约束驱动整车自动链路。

## 适用场景

- 单独验证行为树是否正常 tick
- 验证 Nav2、`rc26_kfs_keepout`、`rc26_mechanism` 是否已被决策正确消费
- 排查自动链路为什么不发导航 goal、为什么卡在 gate 或 fallback

## 前置条件

- `map -> odom` TF 正常
- `/navigate_to_pose` action server 在线
- `/mechanism/status` 正常发布
- `/kfs_filter_mask` 和 `/kfs_keepout_heartbeat` 可观察

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_interfaces rc26_kfs_keepout rc26_vision rc26_decision rc26_bringup
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐启动

单包调试：

```bash
ros2 launch rc26_decision decision.launch.py enable_vision:=false
```

整车自动链：

```bash
ros2 launch rc26_bringup bringup.launch.py \
  slam:=false \
  use_decision:=true \
  prior_pcd_file:=${RC26_WS:-$HOME/RC_2026}/src/rc26_bringup/pcd/default.pcd \
  nav2_map_file:=${RC26_WS:-$HOME/RC_2026}/src/rc26_bringup/map/default.yaml
```

带视觉的决策链：

```bash
ros2 launch rc26_decision decision.launch.py \
  enable_vision:=true \
  vision_config_file:=${RC26_WS:-$HOME/RC_2026}/src/rc26_vision/config/vision_models.yaml
```

## 最小验收

```bash
ros2 node list | grep rc26_decision
ros2 node info /rc26_decision
ros2 action info /navigate_to_pose
ros2 topic echo /r2/bt/debug_state --once
ros2 topic echo /r2/bt/blackboard --once
ros2 topic echo /kfs_filter_mask --once
ros2 topic hz /kfs_keepout_heartbeat
```

## 常用调试命令

```bash
ros2 launch rc26_decision decision.launch.py --ros-args --log-level rc26_decision:=debug
ros2 topic echo /r2/bt/trace --once
ros2 topic echo /r2/bt/events --once
```

## 优先排查

- 决策节点起来了但不发导航：先看 `/r2/bt/debug_state`、`/r2/bt/blackboard` 和行为树日志，再确认 `/navigate_to_pose` action server 已在线。
- `NavToPose` 失败：检查黑板 `nav_last_failure_code`、`nav_last_failure_reason`，再看 Nav2 lifecycle、costmap 和 TF。
- 自动链卡在 gate：先检查 `/kfs_keepout_heartbeat`、`/kfs_filter_mask`、`/mechanism/status` 和定位健康话题。
- 带视觉后卡住：先确认 `vision_models.yaml` 路径有效，再确认相机话题和视觉节点都在。

## 相关入口

- [决策启动](./决策启动.md)
- [rc26_kfs_keepout调试](./rc26_kfs_keepout调试.md)
- [Nav2导航调试](./Nav2导航调试.md)
- [rc26_mechanism调试](./rc26_mechanism调试.md)
