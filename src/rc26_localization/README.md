# rc26_localization

RC2026 点云重定位模块，基于 small_gicp 实现高精度定位。

## 功能

- 点云与先验地图配准
- 发布 map -> odom 变换
- 绑架检测与自动恢复
- 全局重定位 (ISS + FPFH + SAC-IA + NDT + ICP)
- 多假设初值策略

## 工作原理

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  先验点云地图    │     │   实时点云      │     │   配准结果      │
│  (PCD 文件)     │────▶│  (registered_   │────▶│  map -> odom    │
│                 │     │   scan)         │     │   TF 变换       │
└─────────────────┘     └─────────────────┘     └─────────────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │   small_gicp    │
                    │   点云配准      │
                    └─────────────────┘
                              │
                    ┌─────────┴─────────┐
                    ▼                   ▼
            ┌───────────┐       ┌───────────────┐
            │ 局部配准   │       │ 全局重定位     │
            │ (跟踪)    │       │ (绑架恢复)    │
            └───────────┘       └───────────────┘
```

## 数据流流向

```
里程计接口模块
    │
    ├─→ 配准点云数据 (registered_scan话题)
    │       │
    │       ▼
    │   定位模块 (rc26_localization)
    │       │
    │       ├─→ 加载先验点云地图
    │       │       │
    │       │       ▼
    │       │   构建地图数据结构
    │       │       │
    │       │       ▼
    │       │   地图缓存和优化
    │       │
    │       ├─→ 接收实时点云
    │       │       │
    │       │       ▼
    │       │   点云预处理
    │       │       │
    │       │       ├─→ 降采样
    │       │       ├─→ 去除噪声
    │       │       └─→ 特征提取
    │       │
    │       ├─→ 局部配准（跟踪）
    │       │       │
    │       │       ├─→ 使用上一帧位姿作为初值
    │       │       ├─→ 点云配准（small_gicp）
    │       │       ├─→ 计算位姿变换
    │       │       └─→ 更新当前位姿
    │       │               │
    │       │               ▼
    │       │           检查配准质量
    │       │               │
    │       │               ├─→ 如果质量好，使用结果
    │       │               └─→ 如果质量差，可能触发绑架检测
    │       │
    │       ├─→ 绑架检测
    │       │       │
    │       │       ├─→ 检查配准误差
    │       │       ├─→ 统计连续高误差帧数
    │       │       └─→ 如果超过阈值，判定为绑架
    │       │               │
    │       │               ▼
    │       │           触发全局重定位
    │       │
    │       ├─→ 全局重定位（绑架恢复）
    │       │       │
    │       │       ├─→ ISS关键点提取
    │       │       │       │
    │       │       │       ▼
    │       │       │   从点云中提取特征点
    │       │       │
    │       │       ├─→ FPFH特征计算
    │       │       │       │
    │       │       │       ▼
    │       │       │   为关键点计算特征描述符
    │       │       │
    │       │       ├─→ SAC-IA粗配准
    │       │       │       │
    │       │       │       ▼
    │       │       │   使用特征匹配进行粗略对齐
    │       │       │
    │       │       ├─→ 多假设初值生成（可选）
    │       │       │       │
    │       │       │       ▼
    │       │       │   生成多个可能的初始位姿
    │       │       │
    │       │       ├─→ NDT中间层精化（可选）
    │       │       │       │
    │       │       │       ▼
    │       │       │   使用NDT算法进一步优化位姿
    │       │       │
    │       │       ├─→ ICP精配准
    │       │       │       │
    │       │       │       ▼
    │       │       │   使用ICP算法进行精确配准
    │       │       │
    │       │       └─→ 验证配准质量
    │       │               │
    │       │               ├─→ 如果质量好，使用结果
    │       │               └─→ 如果质量差，重定位失败
    │       │
    │       └─→ 发布TF变换
    │               │
    │               ├─→ 计算map → odom变换
    │               │       │
    │               │       ▼
    │               │   根据配准结果计算变换
    │               │
    │               └─→ 发布到TF树
    │                       │
    │                       ▼
    │                   TF树 (map → odom)
    │                       │
    │                       ▼
    │                   其他模块 ← 查询使用
    │                   导航系统 ← 查询使用
    │
    ├─→ 初始位姿设置 (可选)
    │       │
    │       ├─→ RViz的2D Pose Estimate工具
    │       │       │
    │       │       ▼
    │       │   /initialpose话题
    │       │       │
    │       │       ▼
    │       │   定位模块接收初始位姿
    │       │       │
    │       │       ▼
    │       │   设置当前位姿
    │       │       │
    │       │       ▼
    │       │   更新map → odom变换
    │
    └─→ 输出到导航系统
            │
            ├─→ map → odom变换（通过TF树）
            │       │
            │       ▼
            │   Nav2导航系统 ← 查询使用
            │        │
            │        ▼
            │   全局路径规划
            │   障碍物避让
            │
            └─→ 定位状态信息（可选，如果发布话题）
                    │
                    ▼
                其他监控模块 ← 订阅使用
```

## 目录结构

```
rc26_localization/
├── include/rc26_localization/
│   └── localization.hpp       # 定位节点头文件
├── src/
│   └── localization.cpp       # 定位节点实现
├── launch/
│   └── sentry_localization.launch.py
├── CMakeLists.txt
└── package.xml
```

## 话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `registered_scan` | sensor_msgs/PointCloud2 | 订阅 | 配准点云输入 |
| `/initialpose` | geometry_msgs/PoseWithCovarianceStamped | 订阅 | 初始位姿 (RViz) |

## TF 变换

| 父坐标系 | 子坐标系 | 说明 |
|----------|----------|------|
| map | odom | 由本节点发布 |

## 参数

### 坐标系配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map_frame` | `map` | 地图坐标系 |
| `odom_frame` | `odom` | 里程计坐标系 |
| `base_frame` | `base_link` | 底盘坐标系 |
| `robot_base_frame` | `base_link` | 机器人基坐标系 |
| `lidar_frame` | `laser_link` | 激光雷达坐标系 |

### 输入配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `prior_pcd_file` | `""` | 先验点云地图路径 |
| `input_cloud_topic` | `registered_scan` | 输入点云话题 |
| `init_pose` | `[0,0,0,0,0,0]` | 初始位姿 [x,y,z,r,p,y] |

### small_gicp 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `num_threads` | 4 | 并行线程数 |
| `num_neighbors` | 20 | 协方差估计近邻数 |
| `global_leaf_size` | 0.25 | 先验地图体素大小 (m) |
| `registered_leaf_size` | 0.25 | 在线点云体素大小 (m) |
| `max_dist_sq` | 1.0 | 内点距离阈值平方 (m²) |
| `gicp_max_iterations` | 20 | GICP 最大迭代次数 |
| `min_points_for_registration` | 20 | 局部配准最少点数 |
| `max_delta_translation` | 0.5 | 未收敛最大平移 (m) |
| `max_delta_rotation` | 0.3 | 未收敛最大旋转 (rad) |

### 绑架检测参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `kidnap_threshold_count` | 5 | 连续高误差帧数阈值 |
| `kidnap_fitness_threshold` | 0.5 | 配准误差阈值 |

### 全局重定位参数 (SAC-IA)

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `sac_ia_num_samples` | 5 | SAC-IA 采样点数 |
| `sac_ia_min_sample_distance` | 0.1 | 采样点最小间距 (m) |
| `sac_ia_correspondence_randomness` | 50 | 特征匹配随机性 |
| `sac_ia_normal_ksearch` | 20 | 法向量估计近邻数 |
| `sac_ia_fpfh_ksearch` | 50 | FPFH 特征近邻数 |
| `min_points_for_relocalization` | 50 | 全局重定位最少点数 |
| `global_downsample_leaf_size` | 0.5 | 全局重定位体素大小 (m) |

### ISS 关键点参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_iss_keypoints` | true | 启用 ISS 关键点 |
| `iss_salient_radius` | 0.3 | 显著性半径 (m) |
| `iss_non_max_radius` | 0.15 | 非极大值抑制半径 (m) |
| `iss_threshold21` | 0.975 | 特征值比阈值 λ2/λ1 |
| `iss_threshold32` | 0.975 | 特征值比阈值 λ3/λ2 |
| `iss_min_neighbors` | 5 | 最小邻居数 |

### NDT 中间层参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_ndt_refinement` | true | 启用 NDT 中间层 |
| `ndt_resolution` | 1.0 | NDT 体素分辨率 (m) |
| `ndt_max_iterations` | 50 | NDT 最大迭代次数 |
| `ndt_step_size` | 0.1 | NDT 步长 |
| `ndt_transformation_epsilon` | 1e-6 | NDT 收敛阈值 |

### 多假设初值参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_multi_hypothesis` | true | 启用多假设初值 |
| `num_yaw_hypotheses` | 4 | yaw 方向假设数量 |

### 其他参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `tf_timeout_sec` | 1.0 | TF 查询超时 (s) |
| `global_icp_max_iterations` | 100 | 全局 ICP 最大迭代 |
| `global_icp_max_correspondence_distance` | 1.0 | 全局 ICP 最大对应距离 (m) |
| `globtness_threshold` | 0.1 | 全局重定位成功阈值 |

## 全局重定位流程

当检测到绑架 (连续多帧配准误差过高) 时，自动触发全局重定位：

```
1. ISS 关键点提取
       ↓
2. FPFH 特征计算
       ↓
3. SAC-IA 粗配准
       ↓
4. 多假设初值生成 (可选)
       ↓
5. NDT 中间层精化 (可选)
       ↓
6. ICP 精配准
       ↓
7. 验证 fitness_score
       ↓
8. 更新 map->odom 变换
```

## 使用方式

### 启动定位节点

```bash
# 通过 bringup 启动 (推荐)
ros2 launch rc26_bringup localization.launch.py prior_pcd_file:=/path/to/map.pcd

# 单独启动
ros2 launch rc26_localization sentry_localization.launch.py
```

### 设置初始位姿

在 RViz 中使用 "2D Pose Estimate" 工具设置初始位姿，或通过`yaml
init_pose: [1.0, 2.0, 0.0, 0.0, 0.0, 1.57]  # [x, y, z, roll, pitch, yaw]
```

## 坐标系约定

```
map (全局地图坐标系)
 │
 └── odom (里程计坐标系) ← 本节点发布 map->odom
      │
      └── base_link (底盘中心)
           │
           └── laser_link (激光雷达)
```

## 依赖

- rclcpp
- sensor_msgs
- geometry_msgs
- tf2_ros
- pcl_conversions
- small_gicp

## 编译

```bash
colcon build --packages-select rc26_localization
```
