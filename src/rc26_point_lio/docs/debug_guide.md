# rc26_point_lio 调试指南

本文档提供针对 R2 里程计模块（Point-LIO）的当前调试步骤。当前仓库不再通过 launch 参数内置 GUI，所有观察都按“headless 启动 + 外部工具手工只读订阅 topic”执行。

## 1. 编译与环境准备

建议单独编译 Point-LIO 与直接下游链路：

```bash
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_point_lio rc26_odom_interface rc26_sensor_scan rc26_lio_state_predictor rc26_bringup
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 2. 推荐启动方式

### 2.1 通过 bringup 自动选择 Point-LIO profile

```bash
# 建图模式：auto 会选择 mapping_dense
ros2 launch rc26_bringup bringup.launch.py slam:=true use_decision:=false

# 巡航/轻量模式：auto 会选择 cruise_light
ros2 launch rc26_bringup bringup.launch.py slam:=false use_decision:=false

# race_profile 需要显式指定
ros2 launch rc26_bringup bringup.launch.py slam:=false point_lio_profile:=race_profile use_decision:=false
```

### 2.2 显式指定 profile

```bash
ros2 launch rc26_bringup bringup.launch.py slam:=true point_lio_profile:=mapping_dense use_decision:=false
ros2 launch rc26_bringup bringup.launch.py slam:=false point_lio_profile:=cruise_light use_decision:=false
ros2 launch rc26_bringup bringup.launch.py slam:=false point_lio_profile:=race_profile use_decision:=false
```

### 2.3 显式指定 Point-LIO YAML

```bash
ros2 launch rc26_bringup bringup.launch.py \
  slam:=true \
  point_lio_config_file:=/abs/path/to/custom_point_lio.yaml \
  use_decision:=false
```

说明：

- `point_lio_config_file` 非空时优先级高于 `point_lio_profile`
- `point_lio_profile:=auto` 会根据 `slam` 自动切换
- `rc26_point_lio/launch/point_lio.launch.py` 现在只负责 headless Point-LIO 本体，不再声明 `rviz` 兼容参数

## 3. 基础功能测试

### 步骤 3.1：启动 Point-LIO 链路

```bash
ros2 launch rc26_bringup odometry.launch.py \
  slam:=true \
  point_lio_profile:=mapping_dense
```

如果需要观察点云和累计地图，另开一个终端手工启动：

```bash
rviz2 -d /home/potato/RC_2026/src/rc26_bringup/rviz/slam.rviz
```

如果要同时观察 terrain grid map，可在 `odometry.launch.py` 增加 `enable_terrain_grid_map:=true`，并改用：

```bash
rviz2 -d /home/potato/RC_2026/src/rc26_terrain/rviz/terrain_semantic.rviz
```

### 步骤 3.2：播放数据包

```bash
ros2 bag play <path_to_your_bag> --topics /livox/lidar /livox/imu
```

### 步骤 3.3：验证核心输出

```bash
ros2 topic hz /state_estimation
ros2 topic echo /state_estimation --once --field pose.covariance
ros2 run tf2_ros tf2_monitor odom point_lio_body
```

## 4. 点云密度与累计地图验证

### 4.1 动态调节点云保留比例

```bash
ros2 param set /point_lio point_keep_ratio 50.0
ros2 param set /point_lio point_keep_ratio 30.0
ros2 param set /point_lio point_keep_ratio 100.0
```

观察点：

- `/registered_scan` 的点数变化是否符合预期
- 终端是否出现 `PARAM_UPDATE` 日志
- 外部 RViz 中点云是否随之变稠或变稀

### 4.2 动态调累计地图发布

```bash
ros2 param set /point_lio publish.map_full_publish_en true
ros2 param set /point_lio publish.map_full_publish_interval_sec 0.5
ros2 topic hz /laser_map_full
ros2 topic echo /laser_map_full --once --field header.frame_id
```

预期：

- 话题名为 `/laser_map_full`
- `frame_id` 为 `odom`
- 外部 RViz 中累计地图会持续保留历史建图内容

### 4.3 单帧点云仍显得稀疏时的优先排查项

```bash
ros2 param set /point_lio filter_size_surf 0.1
ros2 param set /point_lio filter_size_map 0.1
```

说明：

- `point_keep_ratio` 控制输入点抽样比例
- `filter_size_surf` / `filter_size_map` 控制体素滤波强度
- 最终可见密度由两者共同决定

### 4.4 建图时地上也有点云，是否正常

通常正常。MID-360 会看到地面，当前仓库这版 Point-LIO 也不会主动删除地面点。若只是想让输出地图更干净，可只对输出做高度裁剪：

```bash
ros2 param set /point_lio output_filter.world_z_filter_en true
ros2 param set /point_lio output_filter.world_z_min -0.08
```

该过滤只作用于 `/cloud_registered`、`/laser_map_full` 和保存出来的 PCD，不会修改 Point-LIO 内部用于匹配的 ivox 地图。

## 5. 进阶性能验证

### 5.1 控制延迟

```bash
ros2 topic echo /state_estimation --once | grep stamp -A 2
```

验证要点：

- 如果启用了 `rc26_lio_state_predictor`，检查 `/control_state`
- `odometry.launch.py` 默认强制 `odometry.publish_odometry_without_downsample:=false`
- `rc26_odom_interface` 会吸收小幅时间戳抖动，只有严重失配才需要继续排查

### 5.2 退化检测

```bash
ros2 topic echo /degenerate_score
```

### 5.3 鲁棒性

1. 机器人进行快速原地旋转
2. 观察外部 RViz 中点云是否分层或漂移
3. 检查终端日志是否触发二次迭代

## 6. 地图保存与复用

### 6.1 保存建图结果

```bash
ros2 launch rc26_bringup bringup.launch.py slam:=true point_lio_profile:=mapping_dense use_decision:=false
```

保存说明：

- `mapping_dense` 默认开启 `pcd_save.pcd_save_en`
- 正常退出后会写入 `src/rc26_point_lio/PCD/scans.pcd`
- 若 `pcd_save.interval > 0`，会分段写成 `scans_1.pcd`、`scans_2.pcd`

### 6.2 复用到定位链路

```bash
ros2 launch rc26_bringup bringup.launch.py \
  slam:=false \
  prior_pcd_file:=${RC26_WS:-$HOME/RC_2026}/src/rc26_point_lio/PCD/scans.pcd \
  use_decision:=false
```

也可以先将生成的地图复制到 `src/rc26_bringup/pcd/` 再统一管理。

## 7. 常见问题排查

| 现象 | 可能原因 | 排查建议 |
|------|----------|----------|
| **Rviz 中点云严重抖动** | 外参错误 或 时间同步失效 | 检查 `extrinsic_T/R` 和 `time_diff_lidar_to_imu`。 |
| **当前帧点云偏稀** | `point_keep_ratio` 太低 或 `filter_size_surf` 太大 | 先提高 `point_keep_ratio`，再减小 `filter_size_surf`。 |
| **累计地图不显示** | `publish.map_full_publish_en=false` 或无订阅者 | 打开参数并确认 RViz/Foxglove 已订阅 `/laser_map_full`。 |
| **之前建好的内容没有留存显示** | 只在看 `/registered_scan` 单帧点云 | 切换观察 `/laser_map_full`。 |
| **建图时地上也有点云** | MID-360 本身会看到地面，且 Point-LIO 默认不删地面 | 若只是薄薄一层通常正常；若只想清理显示/PCD，使用 `output_filter.world_z_filter_*`；若地面厚、斜、漂，改查外参与重力方向。 |
| **建图结束后没有生成 PCD** | 未启用 `pcd_save` 或异常退出 | 使用 `mapping_dense` 并正常退出。 |
| **/state_estimation 频率低** | 输入频率异常、profile 配置过重或运行负载过高 | 检查 LiDAR/IMU 输入频率、当前 Point-LIO profile 与 CPU 负载。 |
| **偶发 `点云与里程计时间差 0.201 > 0.200`** | 点云和 odom 回调边界抖动 | 新版 `rc26_odom_interface` 已自动吸收约 5ms 抖动；若仍持续出现，重点排查上游 odom 是否真正卡顿。 |
| **持续出现 `点云落后最新 odom 0.301s/1.004s`** | Point-LIO 开启了 `publish_odometry_without_downsample`，导致 `state_estimation` 时间戳跑到当前扫描内部 | 使用 `odometry.launch.py` 默认配置，或显式传 `point_lio_publish_odometry_without_downsample:=false`。 |
| **编译报错 std_msgs 缺失** | 依赖未安装 | 检查 `package.xml` 和 `CMakeLists.txt` 是否包含 `std_msgs`。 |

---

**备注**：
- 直接改 YAML 后无需重新编译，但需要重启节点；
- 运行时热更新仅对白名单参数生效，例如 `point_keep_ratio`、`filter_size_surf`、`filter_size_map`、`publish.map_full_publish_en`、`publish.map_full_publish_interval_sec` 与 `output_filter.world_z_filter_*`。
