# rc26_point_lio 调试

## 模块定位

`rc26_point_lio` 是 R2 当前的 LiDAR-Inertial Odometry 主链，负责输出 `/state_estimation`、`/registered_scan` 和可选的 `/laser_map_full`。

## 适用场景

- 建图、巡航和比赛 profile 切换
- 排查 `/state_estimation`、点云密度和累计地图
- 给 `rc26_localization`、`rc26_odom_interface`、`rc26_sensor_scan` 提供上游排障入口

## 前置条件

- Mid-360 已正常发布 `/livox/lidar` 与 `/livox/imu`
- 已 `source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"`
- 建图时建议使用 `mapping_dense`

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_point_lio rc26_bringup
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐启动

建图：

```bash
ros2 launch rc26_bringup bringup.launch.py \
  slam:=true \
  point_lio_profile:=mapping_dense \
  use_decision:=false
```

巡航/定位模式：

```bash
ros2 launch rc26_bringup bringup.launch.py \
  slam:=false \
  point_lio_profile:=cruise_light \
  use_decision:=false
```

只调里程计最小链：

```bash
ros2 launch rc26_bringup odometry.launch.py \
  slam:=true \
  point_lio_profile:=mapping_dense \
  enable_lio_state_predictor:=false \
  point_lio_publish_odometry_without_downsample:=false
```

## 最小验收

```bash
ros2 topic hz /state_estimation
ros2 topic echo /state_estimation --once --field pose.covariance
ros2 topic hz /registered_scan
ros2 topic echo /laser_map_full --once --field header.frame_id
ros2 topic echo /degenerate_score --once
```

## 常用在线调参

```bash
ros2 param set /point_lio point_keep_ratio 50.0
ros2 param set /point_lio filter_size_surf 0.1
ros2 param set /point_lio filter_size_map 0.1
ros2 param set /point_lio publish.map_full_publish_en true
ros2 param set /point_lio publish.map_full_publish_interval_sec 0.5
ros2 param set /point_lio output_filter.world_z_filter_en true
ros2 param set /point_lio output_filter.world_z_min -0.08
```

## 优先排查

- `state_estimation` 与点云时间戳错位：优先确认 `point_lio_publish_odometry_without_downsample:=false`。
- 累计地图太稀：先看 `point_keep_ratio`、`filter_size_surf` 和当前 profile。
- 建图结束后没 PCD：先确认使用了 `mapping_dense`，再正常 `Ctrl+C` 退出节点。

## 相关入口

- [建图启动](./建图启动.md)
- [重定位启动](./重定位启动.md)
- [rc26_mid360_driver调试](./rc26_mid360_driver调试.md)
