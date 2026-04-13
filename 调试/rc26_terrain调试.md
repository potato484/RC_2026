# rc26_terrain 调试

## 模块定位

`rc26_terrain` 负责把点云转换成障碍、跌落和 2.5D 栅格语义，是导航链的重要前置感知输入。

## 适用场景

- 排查 `/terrain_obstacles`、`/terrain_drop` 和 `/terrain_grid_map_local`
- 单独验证地形语义节点
- 给 `rc26_xhu_nav`、`rc26_base_ground`、`rc26_decision` 排查门控输入

## 前置条件

- 已 `source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"`
- 上游点云与 TF 正常
- 单独调试时建议先有 Bag 或真实雷达输入

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_interfaces rc26_terrain rc26_merge_odom rc26_bringup
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐启动

单独启动：

```bash
ros2 launch rc26_terrain terrain_semantic.launch.py
```

整车链路带 2.5D grid map：

```bash
ros2 launch rc26_bringup odometry.launch.py enable_terrain_grid_map:=true
```

## 最小验收

```bash
ros2 topic hz /terrain_obstacles
ros2 topic echo /terrain_obstacles --once
ros2 topic hz /terrain_drop
ros2 topic echo /terrain_drop --once
ros2 topic echo /terrain_features --once
```

如需外部观察：

```bash
rviz2 -d "${RC26_WS:-$HOME/RC_2026}/src/rc26_terrain/rviz/terrain_semantic.rviz"
```

## 常用在线调参

```bash
ros2 param set /terrain_semantic ground_ema_alpha 0.5
ros2 param set /terrain_semantic h_obstacle_m 0.35
ros2 param set /terrain_semantic h_drop_m 0.18
```

## 优先排查

- 平地上障碍物误报太多：先调 `h_obstacle_m`。
- 一直没有跌落输出：先确认输入点云覆盖到了落差区域，再看 `h_drop_m`。
- 只想看 grid map：记得在总装链里显式传 `enable_terrain_grid_map:=true`。

## 相关入口

- [感知启动](./感知启动.md)
- [导航启动](./导航启动.md)
- [rc26_sensor_scan调试](./rc26_sensor_scan调试.md)
