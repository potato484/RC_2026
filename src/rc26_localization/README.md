# rc26_localization

`rc26_localization` 是 RC_2026 中 R2 的点云重定位模块，基于 `small_gicp` 实现。

## 功能

- 持续局部跟踪：将 `registered_scan` 与先验地图配准，发布 `map -> odom`
- 绑架检测：基于连续高误差/超时触发恢复
- L1 快速恢复：重试区先验 + `small_gicp` 精配准
- L2 全局恢复：Scan Context 检索 + `small_gicp` 精配准
- 统一状态机与异步 single-flight 重定位执行模型

## 当前重定位架构

```text
TRACKING
  │ (高误差计数/超时)
  ▼
SUSPECT
  │ requestRelocalization()
  ▼
FAST_RECOVERY (L1)
  ├─ 成功 -> TRACKING
  └─ 失败 -> GLOBAL_RECOVERY (L2)
           ├─ 成功 -> TRACKING
           └─ 失败 -> RELOC_FAILED
```

## 主要输入输出

- 输入话题: `registered_scan` (`sensor_msgs/msg/PointCloud2`)
- 输入话题: `initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)
- 输出 TF: `map -> odom`

## 关键参数

### 跟踪与局部配准

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `num_threads` | 4 | 并行线程数 |
| `num_neighbors` | 20 | 协方差估计近邻数 |
| `global_leaf_size` | 0.25 | 先验地图降采样体素 (m) |
| `registered_leaf_size` | 0.25 | 在线点云降采样体素 (m) |
| `max_dist_sq` | 1.0 | 内点距离阈值平方 (m²) |
| `gicp_max_iterations` | 20 | 局部配准最大迭代 |
| `min_points_for_registration` | 20 | 局部配准最少点数 |
| `max_delta_translation` | 0.5 | 未收敛时允许最大平移 (m) |
| `max_delta_rotation` | 0.3 | 未收敛时允许最大旋转 (rad) |

### 绑架检测与冻结门控

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `kidnap_threshold_count` | 5 | 连续高误差帧数阈值 |
| `kidnap_fitness_threshold` | 0.5 | 高误差判定阈值 |
| `registration_timeout_sec` | 10.0 | 成功配准超时阈值 (s) |
| `freeze_update_err` | 0.3 | 冻结 TF 更新的误差阈值 |
| `min_inliers` | 200 | 冻结 TF 更新的最小内点 |

### 全局恢复通用参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `global_icp_max_iterations` | 100 | 全局精配准最大迭代 |
| `global_icp_max_correspondence_distance` | 1.0 | 全局对应点距离上限 (m) |
| `global_fitness_threshold` | 0.1 | 全局恢复接受阈值 |
| `min_points_for_relocalization` | 50 | 恢复最小点数 |
| `global_downsample_leaf_size` | 0.5 | 全局恢复降采样体素 (m) |

### L1 重试区快速恢复

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `competition_mode` | true | 比赛模式强校验开关 |
| `retry_zone_enable` | false | 启用 L1 |
| `retry_zone_x` | 0.0 | 重试区中心 x (map) |
| `retry_zone_y` | 0.0 | 重试区中心 y (map) |
| `retry_zone_yaw_candidates_deg` | `[0,90,180,270]` | 重试区候选朝向 (deg) |
| `retry_zone_fast_accept_th` | 0.15 | L1 接受阈值 |
| `retry_zone_max_xy_offset` | 1.5 | 相对种子最大平移偏差 (m) |
| `retry_zone_max_yaw_offset_deg` | 60.0 | 相对种子最大航向偏差 (deg) |

> `competition_mode=true` 时若 `retry_zone_enable=false` 或 `retry_zone_x/y` 为占位值 `(0,0)`，节点会拒绝启动。

### L2 Scan Context

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `enable_scan_context` | true | 启用 L2 Scan Context |
| `sc_num_rings` | 20 | 描述子 ring 数 |
| `sc_num_sectors` | 60 | 描述子 sector 数 |
| `sc_max_radius` | 8.0 | 描述子最大半径 (m) |
| `sc_submap_radius` | 5.0 | 离线建库子图半径 (m) |
| `sc_grid_resolution` | 1.0 | 建库网格分辨率 (m) |
| `sc_min_points_per_submap` | 80 | 子图最少点数 |
| `sc_topk` | 5 | 在线检索候选数 |
| `sc_sim_threshold` | 0.18 | 相似度代价阈值 |

### I3 亚克力 ROI 过滤

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `acrylic_filter_enable` | false | 启用 ROI 过滤 |
| `acrylic_filter_max_stale_sec` | 1.0 | 可接受的 map->odom 新鲜度 (s) |
| `acrylic_roi_boxes` | `[]` | AABB 列表 `[xmin,ymin,zmin,xmax,ymax,zmax,...]` |

## 结构化日志

每次重定位输出以下字段，方便离线验收统计：

```text
RELOC_METRIC,trigger_reason,path_used,t_total_ms,t_l1_ms,t_l2_ms,candidate_count,best_fitness,best_J,accepted
```

## 使用方式

### 启动

```bash
# 推荐：通过 bringup 启动
ros2 launch rc26_bringup localization.launch.py prior_pcd_file:=/path/to/map.pcd

# 单独启动
ros2 launch rc26_localization sentry_localization.launch.py
```

### 设置初始位姿

在 RViz 中使用 `2D Pose Estimate`，或通过参数提供：

```yaml
init_pose: [x, y, z, roll, pitch, yaw]
```

## 坐标系

```text
map
 └── odom   (由本节点发布 map->odom)
      └── base_link
           └── laser_link
```

## 依赖

- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`
- `tf2_ros`
- `pcl_conversions`
- `small_gicp`

## 编译

```bash
colcon build --packages-select rc26_localization
```
