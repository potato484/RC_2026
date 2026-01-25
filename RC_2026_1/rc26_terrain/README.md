# rc26_terrain

## 1. 模块简介 (Introduction)
`rc26_terrain` 是 RC2026 机器人导航系统的**地形分析与语义分割引擎**。它直接对接受来自 LiDAR 的原始点云数据，结合机器人当前的位姿信息，构建局部环境的高程模型。

本模块的核心目标是将非结构化的点云转化为结构化的**语义点云**，明确标识出：
*   **Obstacles**: 绝对不可通行的障碍物（如墙壁、敌方车辆）。
*   **Climbable**: 机器人底盘可越过的低矮地形（如斜坡、小台阶）。
*   **Drop/Cliffs**: 存在跌落风险的负障碍（如楼梯边缘、深坑）。

这些语义信息将直接作为 `Nav2` 代价地图 (Costmap) 的输入层，从根本上解决"悬崖识别难"和"误识别可通行区域"的痛点。

## 2. 核心功能 (Core Features)

*   **以机器人为中心的滚动栅格地图 (Rolling Grid Map)**:
    *   实时维护一个跟随机器人移动的局部高程网格（例如 $6.4m \times 6.4m$）。
    *   高效处理点云数据，统计每个栅格内的地面高度与顶部高度。
*   **精细化地形分类 (Semantic Classification)**:
    *   基于高度差 ($\Delta H$) 与相对高度 ($Z_{rel}$) 的几何规则判定。
    *   能够区分普通障碍物、悬空障碍物（如横梁）、可攀爬斜坡与跌落边缘。
*   **鲁棒的滤波机制**:
    *   **时间滞后 (Hysteresis) 滤波**: 引入积分状态机 (Score System)，只有连续多次检测到障碍物才确认为障碍，有效消除点云噪点造成的"闪烁"现象。
    *   **指数移动平均 (EMA)**: 平滑地面高度估计，减少测量误差带来的抖动。
    *   **时间衰减 (Time Decay)**: 自动清除过期的历史观测数据，适应动态变化的环境。
*   **失效保护 (Fail-Safe)**:
    *   当传感器数据中断或 TF 变换丢失时，自动在机器人周围生成一圈虚拟围栏 (Virtual Fence)，强制导航层停止规划，确保安全。
*   **Unknown 策略可配置**:
    *   `aggressive`: 忽略未知区域（默认，行为更激进）。
    *   `conservative`: 将未知区域视为风险点输出给 Nav2（更保守）。

## 3. 底层原理 (Underlying Principles)

### 3.1 高度统计与地面估计
对于落入同一栅格 $(i, j)$ 内的所有点云 $\{p_k\}$：
1.  **高度排序**: 对所有点的 $Z$ 坐标进行排序。
2.  **地面估计 ($Z_{ground}$)**: 取 $25\%$ 分位数的 $Z$ 值作为当前帧的观测值，并使用 EMA 更新历史估计：
    $$ Z_{ground}^{new} = \alpha \cdot Z_{obs} + (1 - \alpha) \cdot Z_{ground}^{old} $$
3.  **顶部估计 ($Z_{top}$)**: 取 $95\%$ 分位数的 $Z$ 值，代表该区域的最高点。

### 3.2 语义分类逻辑
计算相对于机器人基座 ($Z_{base}$) 的特征高度：

*   **Drop (跌落风险)**:
    $$ Z_{ground} < (Z_{base} - h_{drop\_thresh}) $$
    *   *条件*: 且该栅格位于机器人运动方向的前方扇区内（可配置 `drop_forward_sector_deg`）。
*   **Climbable (可跨越)**:
    $$ h_{min\_climb} < (Z_{top} - Z_{ground}) < h_{max\_climb} $$
    *   *说明*: 且地面高度接近机器人水平面，视为可通行的斜坡或矮阶梯。
*   **Obstacle (障碍物)**:
    $$ (Z_{top} - Z_{ground}) > h_{max\_climb} $$
    *   *说明*: 高度差超过越障能力的物体。

### 3.3 积分状态机 (Score System)
为了防止误报，每个栅格维护 `obstacle_score` 和 `drop_score` (范围: $0 \sim 10$)：
*   **Hit**: 当前帧检测到特征 $\rightarrow$ Score += 2
*   **Miss**: 当前帧未检测到 $\rightarrow$ Score -= 1
*   **状态翻转**:
    *   `Unknown` $\rightarrow$ `Occupied`: Score > 6
    *   `Occupied` $\rightarrow$ `Free`: Score < 3

### 3.4 失效保护策略
| 策略 | 说明 |
| :--- | :--- |
| `none` | 仅报错，不主动干预 |
| `virtual_fence` | 生成圆形虚拟墙，阻止机器人移动 |
| `emergency_stop` | 生成全向致密阻挡，强制急停 |

## 4. 接口说明 (Interface Description)

### 4.1 话题订阅 (Subscribed Topics)
| 话题 (Topic) | 类型 (Type) | 说明 | QoS |
| :--- | :--- | :--- | :--- |
| `registered_scan` | `sensor_msgs/msg/PointCloud2` | 原始点云 (Base Link 或 Lidar Frame) | Best Effort |
| `odom` | `nav_msgs/msg/Odometry` | 机器人里程计，提供实时高度 $Z_{base}$ | Reliable |

### 4.2 话题发布 (Published Topics)
| 话题 (Topic) | 类型 (Type) | 说明 |
| :--- | :--- | :--- |
| `terrain_obstacles` | `sensor_msgs/msg/PointCloud2` | 障碍物点云，用于局部代价地图的 Obstacle Layer |
| `terrain_drop` | `sensor_msgs/msg/PointCloud2` | 跌落/悬崖点云，用于 Voxel Layer 或标记禁行区 |
| `terrain_climbable` | `sensor_msgs/msg/PointCloud2` | 可通行区域点云 (可选可视化调试) |
| `diagnostics`| `diagnostic_msgs/msg/DiagnosticArray` | 模块健康状态、传感器延时报警 |

### 4.3 关键参数配置 (Parameters)

通过 `config/terrain_semantic.yaml` 配置：

| 参数域 | 参数名 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| **话题** | `input_cloud_topic` | `registered_scan` | 点云输入话题 |
| | `odom_topic` | `odom` | 里程计输入话题 |
| | `output_obstacles_topic` | `terrain_obstacles` | 障碍物输出话题 |
| | `output_drop_topic` | `terrain_drop` | 跌落风险输出话题 |
| | `output_climbable_topic` | `terrain_climbable` | 可攀爬区域输出话题 |
| **坐标系** | `target_frame` | `odom` | 目标坐标系 |
| | `base_frame` | `base_link` | 机器人基座坐标系 |
| | `tf_timeout_sec` | 0.2 | TF 查询超时时间 (秒) |
| **栅格** | `perception_radius_m` | 3.2 | 地图感知半径 (米) |
| | `grid_resolution_m` | 0.1 | 栅格分辨率 (米) |
| | `voxel_leaf_size_m` | 0.05 | 体素滤波叶子大小 (米) |
| **高度过滤** | `min_rel_z_m` | -1.5 | 相对机器人最小 Z 高度 |
| | `max_rel_z_m` | 0.5 | 相对机器人最大 Z 高度 |
| **地面估计** | `min_points_per_cell` | 5 | 每个栅格最小点数 |
| | `ground_quantile` | 0.25 | 地面高度分位数 |
| | `top_quantile` | 0.95 | 顶部高度分位数 |
| | `ground_ema_alpha` | 0.6 | EMA 平滑系数 |
| **语义阈值** | `h_obstacle_m` | 0.33 | 障碍物高度判定阈值 |
| | `h_climb_m` | 0.30 | 最大可越障/爬坡高度 |
| | `h_drop_m` | 0.15 | 允许的最大下落高度 |
| | `climbable_min_dz_m` | 0.05 | 可攀爬最小高度差 |
| **Unknown 策略** | `unknown_policy` | `aggressive` | `aggressive`/`conservative` |
| | `unknown_output` | `drop` | Unknown 输出到 `drop` 或 `obstacles` |
| **滤波** | `enable_hysteresis` | true | 是否启用积分滤波 |
| | `score_max` | 10 | 积分最大值 |
| | `score_inc` | 2 | 检测到时积分增量 |
| | `score_dec` | 1 | 未检测到时积分减量 |
| | `obstacle_on_score` | 6 | 障碍物确认阈值 |
| | `obstacle_off_score` | 3 | 障碍物清除阈值 |
| | `decay_time_sec` | 2.0 | 栅格数据老化清除时间 |
| **失效保护** | `enable_fail_safe` | true | 是否启用传感器失效保护 |
| | `fail_safe_strategy` | `virtual_fence` | `none`/`virtual_fence`/`emergency_stop` |
| | `virtual_fence_radius_m` | 0.6 | 虚拟围栏半径 |
| | `virtual_fence_num_points` | 36 | 虚拟围栏点数 |
| **QoS** | `cloud_qos_reliability` | `best_effort` | 点云 QoS 可靠性 |
| | `odom_qos_reliability` | `reliable` | 里程计 QoS 可靠性 |
| | `output_qos_reliability` | `best_effort` | 输出 QoS 可靠性 |

## 5. 启动示例 (Usage)

### 5.1 命令行启动
```bash
# 使用默认参数启动
ros2 launch rc26_terrain terrain_semantic.launch.py

# 指定命名空间和参数文件
ros2 launch rc26_terrain terrain_semantic.launch.py \
    namespace:=robot1 \
    use_sim_time:=true \
    params_file:=/path/to/custom_params.yaml
```

### 5.2 Launch 文件参数
| 参数 | 默认值 | 说明 |
| :--- | :--- | :--- |
| `namespace` | `""` | 顶级命名空间 |
| `use_sim_time` | `false` | 是否使用仿真时间 |
| `params_file` | `config/terrain_semantic.yaml` | 参数文件路径 |

### 5.3 在其他 Launch 文件中包含
```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('rc26_terrain'), 'launch', 'terrain_semantic.launch.py'
        ])
    ]),
    launch_arguments={
        'namespace': 'robot1',
        'use_sim_time': 'false',
        'params_file': '/path/to/params.yaml'
    }.items()
)
```

### 5.4 与 Nav2 Costmap 集成
在 Nav2 的 `local_costmap_params.yaml` 中添加 Obstacle Layer：
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: terrain_obstacles terrain_drop
        terrain_obstacles:
          topic: /terrain_obstacles
          sensor_frame: odom
          observation_persistence: 0.0
          expected_update_rate: 10.0
          data_type: "PointCloud2"
          clearing: true
          marking: true
          max_obstacle_height: 2.0
          min_obstacle_height: 0.0
        terrain_drop:
          topic: /terrain_drop
          sensor_frame: odom
          observation_persistence: 0.0
          expected_update_rate: 10.0
          data_type: "PointCloud2"
          clearing: false
          marking: true
          max_obstacle_height: 2.0
          min_obstacle_height: -2.0
```

## 6. 目录结构 (Directory Structure)
```
rc26_terrain/
├── include/rc26_terrain/
│   └── terrain_semantic_node.hpp   # 核心节点类定义
├── src/
│   └── terrain_semantic_node.cpp   # 节点实现
├── config/
│   └── terrain_semantic.yaml       # 默认参数配置
├── launch/
│   └── terrain_semantic.launch.py  # 启动文件
├── package.xml                     # ROS 2 包描述
├── CMakeLists.txt                  # 构建配置
└── README.md                       # 本文档
```

## 7. 依赖项 (Dependencies)
*   `rclcpp`: ROS 2 C++ 客户端库
*   `sensor_msgs`: 点云消息类型
*   `nav_msgs`: 里程计消息类型
*   `diagnostic_msgs`: 诊断消息类型
*   `tf2_ros`: TF2 变换库
*   `pcl_ros`: PCL 点云处理库
