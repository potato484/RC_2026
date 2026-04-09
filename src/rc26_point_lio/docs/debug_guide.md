# rc26_point_lio 调试指南

本文档提供针对 RC2026 机器人里程计模块（Point-LIO）的详细调试与测试步骤。

## 1. 编译与环境准备

在开始测试前，请确保工作空间已正确编译。建议使用以下命令单独编译里程计相关包，以节省时间。

```bash
colcon build --symlink-install --parallel-workers 3 --packages-select rc26_point_lio rc26_odom_interface rc26_sensor_scan rc26_lio_state_predictor rc26_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
```

确保 source 环境：

```bash
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 2. 推荐启动方式

### 2.1 通过 bringup 自动选择 Point-LIO profile

```bash
# 建图模式：auto 会选择 mapping_dense
ros2 launch rc26_bringup bringup.launch.py slam:=true visualization_profile:=diagnostic_gui use_decision:=false

# 巡航/轻量模式：auto 会选择 cruise_light
ros2 launch rc26_bringup bringup.launch.py slam:=false visualization_profile:=diagnostic_gui use_decision:=false

# race_profile 需要显式指定，不会改写 auto
ros2 launch rc26_bringup bringup.launch.py slam:=false point_lio_profile:=race_profile use_decision:=false
```

### 2.2 显式指定 profile

```bash
# 强制高密建图
ros2 launch rc26_bringup bringup.launch.py slam:=true point_lio_profile:=mapping_dense use_decision:=false

# 强制轻量巡航
ros2 launch rc26_bringup bringup.launch.py slam:=false point_lio_profile:=cruise_light use_decision:=false

# 强制比赛最小链
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
- `point_lio_config_file` 非空时优先级高于 `point_lio_profile`。
- `point_lio_profile:=auto` 会根据 `slam` 自动切换。
- 当前仓库默认只保留 `mid360.yaml` 一份主配置，`cruise_light` / `mapping_dense` / `race_profile` 由 launch 在运行时做参数覆盖。
- `rc26_point_lio/launch/point_lio.launch.py` 现在是纯 headless 入口；若需要 GUI，请通过 `rc26_bringup/odometry.launch.py` 或直接运行 `rviz2`。

## 3. 基础功能测试（回放数据包）

使用录制好的 rosbag 进行离线验证是最高效的调试方式。

### 步骤 3.1：启动 Point-LIO 链路

在一个终端中启动里程计节点：

```bash
ros2 launch rc26_bringup odometry.launch.py \
  slam:=true \
  point_lio_profile:=mapping_dense \
  odometry_use_rviz:=true \
  odometry_visualization_layout:=diagnostic
```

说明：
- `odometry_use_rviz:=true` 现在启动的是默认进入 RC26 模式的魔改 `rviz2`。
- 如需完整原生 RViz 体验，请单独运行 `ros2 run rviz2 rviz2 --classic`。
- 若在无图形环境的车端调试，可直接去掉该参数或显式传 `odometry_use_rviz:=false`。

### 步骤 3.2：播放数据包

在另一个终端中播放 rosbag。建议只发布 LiDAR 和 IMU 话题，避免干扰：

```bash
# 请将 <path_to_your_bag> 替换为实际路径
ros2 bag play <path_to_your_bag> --topics /livox/lidar /livox/imu
```

### 步骤 3.3：验证核心话题输出

**1. 检查状态估计频率**

```bash
ros2 topic hz /state_estimation
```

**2. 检查里程计数据完整性**

```bash
ros2 topic echo /state_estimation --once --field pose.covariance
```

**3. 检查 TF 树**

```bash
# odometry.launch.py 中 Point-LIO body_frame 会被统一设置为 point_lio_body
ros2 run tf2_ros tf2_monitor odom point_lio_body
```

## 4. 点云密度与累计地图验证

### 4.1 动态调节点云保留比例

```bash
# 调成约 50%
ros2 param set /point_lio point_keep_ratio 50.0

# 调成约 30%
ros2 param set /point_lio point_keep_ratio 30.0

# 回到尽量全保留
ros2 param set /point_lio point_keep_ratio 100.0
```

观察点：
- `/registered_scan` 的点数变化是否符合预期；
- 终端中是否出现 `PARAM_UPDATE` 日志；
- RViz 中当前帧点云是否变得更稠或更稀。

### 4.2 动态调累计地图发布

```bash
# 开启累计地图持续发布
ros2 param set /point_lio publish.map_full_publish_en true

# 每 0.5 秒发布一次累计地图
ros2 param set /point_lio publish.map_full_publish_interval_sec 0.5
```

检查命令：

```bash
ros2 topic hz /laser_map_full
ros2 topic echo /laser_map_full --once --field header.frame_id
```

预期：
- 话题名为 `/laser_map_full`；
- `frame_id` 为 `odom`；
- RViz 中 `LaserMapFull` 会持续保留历史建图内容，而不是只显示当前帧。

### 4.3 单帧点云仍显得稀疏时的优先排查项

如果已经把 `point_keep_ratio` 调高，但 `RegisteredCloud` 仍然看起来稀疏，优先检查：

```bash
ros2 param set /point_lio filter_size_surf 0.1
ros2 param set /point_lio filter_size_map 0.1
```

说明：
- `point_keep_ratio` 控制输入点抽样比例；
- `filter_size_surf` / `filter_size_map` 控制体素滤波强度；
- 最终视觉密度由两者共同决定。

### 4.4 建图时地上也有点云，是否正常

通常是正常现象。

原因：
- MID-360 官方规格给出的垂直视场角为 `-7 deg ~ +52 deg`，本身就能扫到地面；
- 当前仓库这版 Point-LIO 默认不会主动做地面删除；
- 地面点对 `z / roll / pitch` 稳定性通常是有帮助的，因此不建议先入为主地把它当成噪声。

建议先区分两种情况：
- **只是地面有一层点**：通常正常，可不处理，或者仅对输出地图做高度裁剪。
- **地面很厚、倾斜、漂移、分层明显**：更像外参、重力方向、静态 TF、安装角或时间同步问题。

如果只是想让输出地图和保存的 PCD 更干净，可直接这样调：

```bash
ros2 param set /point_lio output_filter.world_z_filter_en true
ros2 param set /point_lio output_filter.world_z_min -0.08
```

调参方法：
- 先在 RViz 里观察 `odom` 系下地面大概落在哪个高度；
- 逐步上调 `output_filter.world_z_min`，例如 `-0.10 -> -0.08 -> -0.06`；
- 不要一次裁得太狠，否则容易误删低矮障碍物、底盘附近结构或坡面点。

注意：
- 该过滤只作用于 `/cloud_registered`、`/laser_map_full` 和保存出来的 PCD；
- 它不会修改 Point-LIO 内部用于里程计匹配的 ivox 地图；
- 如果实物安装存在俯仰角，但 `extrinsic_R` 或静态 TF 仍按零角度配置，地面表现会明显异常。

## 5. 进阶性能验证

### 5.1 验证控制延迟（Phase 0/3 验收）

Point-LIO 经过配置优化，应输出实时的状态估计以供控制使用。

```bash
ros2 topic echo /state_estimation --once | grep stamp -A 2
```

验证标准：
- 如果启用了 `rc26_lio_state_predictor`，请检查 `/control_state` 话题；
- `odometry.launch.py` 默认会强制 `odometry.publish_odometry_without_downsample:=false`，确保 `state_estimation` 与 `cloud_registered` 共用 `lidar_end_time` 时间戳；
- `rc26_odom_interface` 会自动吸收约 5ms 级的点云/里程计时间戳抖动；只有明显超出 `max_time_diff_sec` 的严重失配才应继续排查。

### 5.2 验证退化检测（Phase 2 验收）

在走廊或特征稀疏区域，退化分数应下降。

```bash
ros2 topic echo /degenerate_score
```

### 5.3 验证鲁棒性（Phase 1/4 验收）

操作：
1. 机器人进行快速原地旋转（角速度 > 2 rad/s）；
2. 观察 RViz 中点云是否分层或漂移；
3. 检查是否触发二次迭代（需查看终端日志）。

## 6. 地图保存与复用

### 6.1 保存建图结果

推荐直接使用高密建图 profile：

```bash
ros2 launch rc26_bringup bringup.launch.py slam:=true point_lio_profile:=mapping_dense use_decision:=false
```

保存说明：
- `mapping_dense` 默认开启 `pcd_save.pcd_save_en`;
- 节点正常退出时，会将累计点云写入 `src/rc26_point_lio/PCD/scans.pcd`;
- 若 `pcd_save.interval > 0`，会分段写成 `scans_1.pcd`、`scans_2.pcd` 等。
- 若启用了 `output_filter.world_z_filter_*`，保存出来的 PCD 也会按同样的世界系高度阈值裁剪。

建议：
- 建图结束后使用 `Ctrl+C` 正常退出，不要直接强杀进程；
- 大图场景可考虑设置 `interval > 0` 防止单文件过大。

### 6.2 复用到定位链路

将生成的 PCD 作为 `prior_pcd_file` 传给定位链路：

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
