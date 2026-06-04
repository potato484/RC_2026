# Nav2 导航调试

## 模块定位

R2 当前导航运行权威是 Nav2。`rc26_bringup` 在 `slam=false` 时启动 `map_server` 与 Nav2 navigation stack，`rc26_localization` 继续发布 `map -> odom`，`/cmd_vel` 由 Nav2 controller/velocity_smoother 输出。

## 前置条件

- 已 `source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"`
- `map -> odom` 和 `odom -> base_link` TF 正常
- `/odom` 与 `/scan` 正常发布
- `nav2_map_file` 指向有效 2D occupancy map；默认 map 仅作占位入口

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_interfaces rc26_localization rc26_decision rc26_bringup
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐启动

```bash
ros2 launch rc26_bringup test_navigation.launch.py \
  prior_pcd_file:=${RC26_WS:-$HOME/RC_2026}/src/rc26_point_lio/PCD/scans.pcd \
  nav2_map_file:=${RC26_WS:-$HOME/RC_2026}/src/rc26_bringup/map/default.yaml
```

实机验收时请把 `nav2_map_file` 换成真实 occupancy map。

## 最小验收

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /velocity_smoother

ros2 action info /navigate_to_pose
ros2 topic echo /plan --once
ros2 topic echo /global_costmap/costmap --once
ros2 topic echo /local_costmap/costmap --once
ros2 topic echo /cmd_vel
```

发送一个最小目标：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.2, y: 0.0, z: 0.0}, orientation: {z: 0.7071, w: 0.7071}}}}"
```

## 优先排查

- action 不存在：先看 `/bt_navigator` lifecycle 是否 active，再检查 `nav2_bringup` 是否安装。
- costmap 没有数据：先确认 map_server 已 active、`/map` 有输出，再检查 `/scan` 与 TF。
- `/cmd_vel` 长期为零：先确认 goal 已 accepted，再看 `/plan`、controller 日志和 local costmap 是否阻塞。
- TF 报错：确认 `rc26_localization` 是 `map -> odom` 权威，`rc26_odom_interface` 是 `odom -> base_link` 权威。
- 目标无法规划：确认 occupancy map 覆盖目标点，且 `global_costmap` 可以收到 map。

## 相关入口

- [导航启动](./导航启动.md)
- [决策启动](./决策启动.md)
- [rc26_decision调试](./rc26_decision调试.md)
