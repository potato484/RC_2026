# rc26_point_lio 调试

## 模块定位

`rc26_point_lio` 是 R2 当前的 LiDAR-Inertial Odometry 主链，原生输出 `/state_estimation`、`/cloud_registered`、`/cloud_registered_body`、`/Laser_map` 和 `/path`。

经 `rc26_odom_interface` 转换后，下游通常观察 `/odom` 与 `/registered_scan`。

## 适用场景

- 排查 Mid-360 到 Point-LIO 的输入链路
- 排查 `/state_estimation`、`/cloud_registered` 和 `/registered_scan`
- 验证车身 ROI 过滤热更新
- 验证 PCD 保存是否在完整 YAML 中显式开启

## 前置条件

- Mid-360 已正常发布 `/livox/lidar` 与 `/livox/imu`
- 已 `source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"`
- `rc26_sensor_extrinsics` 能发布 `base_link -> livox_frame` 静态 TF

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_point_lio rc26_bringup
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐启动

建图调试：

```bash
ros2 launch rc26_bringup test_mapping.launch.py
```

巡航/定位模式：

```bash
ros2 launch rc26_bringup bringup.launch.py \
  slam:=false \
  use_decision:=false
```

只调里程计最小链：

```bash
ros2 launch rc26_bringup odometry.launch.py \
  point_lio_publish_odometry_without_downsample:=false
```

需要导出 PCD 时，准备一份完整 Point-LIO YAML，在其中显式设置 `pcd_save.pcd_save_en: true`，然后：

```bash
ros2 launch rc26_bringup test_mapping.launch.py \
  point_lio_config_file:=/abs/path/to/point_lio_mapping.yaml
```

## 最小验收

```bash
ros2 topic hz /state_estimation
ros2 topic echo /state_estimation --once --field pose.covariance
ros2 topic hz /cloud_registered
ros2 topic hz /registered_scan
ros2 topic echo /Laser_map --once --field header.frame_id
ros2 topic list | grep -E "state_estimation|cloud_registered|registered_scan|Laser_map|path"
```

预期：

- `/state_estimation` 正常输出，协方差非全零；
- `/cloud_registered` 是 Point-LIO 原生配准点云；
- `/registered_scan` 是 `rc26_odom_interface` 转换后的下游统一点云；
- `/Laser_map` 只在初始地图发布时出现，不是持续累计地图。

## 运行时调参

当前只支持车身 ROI 热更新：

```bash
ros2 param set /point_lio filter_car_body false
ros2 param set /point_lio filter_car_body true
ros2 param set /point_lio body_x_min -0.45
ros2 param set /point_lio body_z_max 0.75
```

点云密度、体素滤波、量程、发布开关和保存开关都需要改完整 YAML 后重启。

## 优先排查

- `state_estimation` 与点云时间戳错位：优先确认 `point_lio_publish_odometry_without_downsample:=false`。
- `/cloud_registered` 有输出但 `/registered_scan` 没输出：检查 `rc26_odom_interface` 是否启动、`/odom` 是否可用。
- 建图结束后没 PCD：确认完整 YAML 中 `pcd_save.pcd_save_en` 已设为 `true`，并通过 `Ctrl+C` 正常退出节点。
- 车身 ROI 开启失败：检查 `base_link <- livox_frame` TF 是否存在。

## 相关入口

- [建图启动](./建图启动.md)
- [重定位启动](./重定位启动.md)
- [rc26_mid360_driver调试](./rc26_mid360_driver调试.md)
