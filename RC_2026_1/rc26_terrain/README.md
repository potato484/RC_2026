# rc26_terrain

基于点云的地形语义感知模块（RC26_Terrain），按《RC_2026 R2 地形语义感知模块（RC26_Terrain）需求规格说明书.md》交付：在单一 ROS 2 包内输出 Nav2 可用的“真障碍/跌落风险”点云，并提供 TF/QoS/迟滞/诊断与失效保护。

## 启动

```bash
ros2 launch rc26_terrain terrain_semantic.launch.py
```

参数文件默认使用 `config/terrain_semantic.yaml`，也可覆盖：

```bash
ros2 launch rc26_terrain terrain_semantic.launch.py params_file:=/path/to/terrain_semantic.yaml
```

## 接口（Topics）

- 输入点云：`registered_scan`（`sensor_msgs/PointCloud2`，可通过参数/Remap 配置）
- 输入里程计：`odom`（`nav_msgs/Odometry`，用于健康监测与失效保护备用位姿）
- 输出（Nav2 障碍层输入）：`terrain_obstacles`（`sensor_msgs/PointCloud2`）
- 输出（跌落/悬崖风险）：`terrain_drop`（`sensor_msgs/PointCloud2`）
- 输出（调试/定位用，可关闭）：`terrain_climbable`（`sensor_msgs/PointCloud2`，默认开启；置空可关闭）
- 输出诊断：`diagnostics`（`diagnostic_msgs/DiagnosticArray`，可 Remap 接入系统诊断汇聚）

## 坐标系与 TF

- 处理严格按点云时间戳进行 TF 变换；输出默认在 `target_frame=odom` 下发布。
- `base_frame` 与雷达外参必须与 Point-LIO/建图模块保持几何一致，否则会导致地面/台阶/悬崖判别失真。

## 参数字典（简表）

完整参数见 `config/terrain_semantic.yaml`，关键项：

- 话题：`input_cloud_topic` `odom_topic` `output_obstacles_topic` `output_drop_topic` `output_climbable_topic` `diagnostics_topic`
- Frame：`target_frame` `base_frame`
- TF/QoS：`tf_timeout_sec`/`transform_tolerance`，以及 `*_qos_*`
- 感知范围/栅格：`perception_radius_m` `grid_resolution_m` `voxel_leaf_size_m`
- 语义阈值：`h_climb_m` `h_obstacle_m` `h_drop_m` `climbable_min_dz_m`
- Unknown 策略：`unknown_policy`（`aggressive|conservative`）与 `unknown_output`（`drop|obstacles`）
- 迟滞与时效：`enable_hysteresis` `score_*` `*_on_score` `*_off_score` `stale_time_sec`
- 失效保护：`enable_fail_safe` `fail_safe_strategy`（`none|virtual_fence|emergency_stop`）及 `virtual_fence_*`

## RViz 调试

推荐使用 `rviz/terrain_semantic.rviz`：

```bash
rviz2 -d $(ros2 pkg prefix rc26_terrain)/share/rc26_terrain/rviz/terrain_semantic.rviz
```

## Bag 录制/回放

见 `docs/bag_record_playback.md`。

