# rc26_terrain 调试指南

本文档提供 `rc26_terrain` 模块（地形感知与语义分割）的详细调试步骤。主要目标是验证点云数据输入、地形高度估计、障碍物检测、悬崖/坑洼检测以及速度限制发布等功能是否正常运行。

## 1. 编译模块

首先需要编译相关的 ROS 2 模块，确保所有的代码更改都已经生效。

打开终端，在工作空间根目录执行：

```bash
cd ~/RC_2026
colcon build --parallel-workers 1 --packages-select rc26_interfaces rc26_terrain rc26_merge_odom
source install/setup.bash
```

## 2. 启动基础环境

建议在启动地形模块前，先准备好点云数据源（如播放 bag 包或启动雷达驱动）以及 TF 变换。

### 播放测试数据 (Bag 包)

如果您有包含点云数据和 TF 信息的 bag 包，可以循环播放：

```bash
ros2 bag play <你的bag包路径> --loop
```

*(如果没有 bag 包，请确保真实机器人的雷达驱动和 TF 发布节点正常运行。)*

## 3. 启动 Terrain 节点

启动 `rc26_terrain` 节点，开始处理点云并发布地形信息。

```bash
ros2 launch rc26_terrain terrain_semantic.launch.py
```

## 4. 验证节点状态与参数

打开一个新终端，检查节点是否成功启动以及参数是否正确加载。

```bash
source install/setup.bash
ros2 node info /terrain_semantic_node
ros2 param dump /terrain_semantic_node
```

## 5. 验证输入话题

确认节点是否接收到了所需的输入数据。

**检查 TF 变换是否正常：**

```bash
ros2 run tf2_ros tf2_echo base_link mid360
```
*(注意替换 `mid360` 为您实际配置的雷达 frame_id)*

**检查点云输入：**

```bash
ros2 topic hz /mid360/points
ros2 topic echo /mid360/points | head -n 20
```

## 6. 验证输出话题 (关键步骤)

通过检查 `rc26_terrain` 节点发布的各个话题，确认算法执行效果。

### 6.1 检查障碍物输出

检查发布的障碍物栅格消息：

```bash
ros2 topic hz /terrain_obstacles
ros2 topic echo /terrain_obstacles --once
```

### 6.2 检查悬崖/坑洼 (Drop) 输出

检查发布的 drop 栅格消息：

```bash
ros2 topic hz /terrain_drop
ros2 topic echo /terrain_drop --once
```

### 6.3 检查速度限制输出

基于地形粗糙度、坡度和悬崖计算出的速度限制：

```bash
ros2 topic hz /terrain_speed_limit
ros2 topic echo /terrain_speed_limit
```

如果在平坦地面，`data` 值应该接近设置的最大速度（例如 `2.0`）。如果遇到障碍物或坑洼，值应该减小或降至 `0.0`。

### 6.4 检查地形特征总线 (可选)

如果启用了 `enable_terrain_features_pub` 参数，可以查看更详细的地形特征：

```bash
ros2 topic hz /terrain_features
ros2 topic echo /terrain_features --once
```

## 7. 动态调参验证 (RQT Reconfigure)

可以在运行时修改参数，观察算法输出的变化。这对于调节障碍物阈值或滤波参数非常有用。

打开 RQT 参数配置工具：

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

在界面中选择 `terrain_semantic_node`。

**尝试修改以下关键参数观察变化：**
- `ground_ema_alpha`: 调整地面高度指数平滑系数（0.0 ~ 1.0）。
- `obstacle_height_thresh_m`: 调整判断为障碍物的高度阈值（例如从 `0.1` 改为 `0.2`）。
- `drop_depth_thresh_m`: 调整判断为坑洼的深度阈值。
- `min_drop_area_cells`: 调整悬崖检测的连通区域面积阈值（过滤小噪点）。

## 8. 闭环测试验证 (PoseSender 限速)

验证 `rc26_terrain` 产生的限速是否成功作用于机器人的速度控制节点（如 `PoseSender` / `rc26_merge_odom`）。

**首先，启动 merge_odom 节点：**

```bash
ros2 launch rc26_merge_odom merge_odom.launch.py
```

**模拟发布一个极低的速度限制：**

打开新终端，向限速话题发布数据：

```bash
ros2 topic pub -r 10 /terrain_speed_limit std_msgs/msg/Float32 "{data: 0.2}"
```

**观察底层控制或里程计发送的最终速度限制：**

（这取决于 `rc26_merge_odom` 的具体输出，通常会反映在下发给底盘的指令中）。同时可以观察 `/terrain_speed_limit` 是否如预期地被 `PoseSender` 接收并未超时。

可以通过终止 `ros2 topic pub` 命令，等待 `terrain_speed_limit_timeout_ms` (默认 500ms) 后，观察系统是否恢复默认最大速度，以验证超时保护机制。

## 9. RViz 可视化调试

强烈建议使用 RViz2 直观地查看点云和生成的代价地图。

1. 启动 RViz2：
   ```bash
   rviz2 -d src/rc26_terrain/rviz/terrain.rviz
   ```
   *(如果没有预设的 rviz 文件，只需直接运行 `rviz2`)*

2. 在 RViz 中添加以下显示项：
   - **TF**: 显示坐标系。
   - **PointCloud2**: 订阅 `/mid360/points` 查看原始点云。
   - **Map / OccupancyGrid**: 订阅 `/terrain_obstacles` 查看障碍物栅格。
   - **Map / OccupancyGrid**: 订阅 `/terrain_drop` 查看悬崖/坑洼栅格。
   - 设置 `Fixed Frame` 为 `base_link` 或是您的全局基准坐标系。

## 10. 常见问题排查

- **节点运行但无障碍物或悬崖输出**：优先检查点云输入话题、TF 变换和 Bag 回放时间是否正常；若输入存在但输出为空，重点核对地形阈值是否设置过严。
- **障碍物/坑洼误检较多**：建议结合离线 Bag 回放逐步调节坡度、粗糙度和高度差阈值，并观察 `/terrain_obstacles` 与 `/terrain_drop` 是否只在真实风险区域稳定出现。
- **`/terrain_speed_limit` 长时间过低或不恢复**：检查是否持续存在高风险地形输入，以及 `terrain_speed_limit_timeout_ms` 是否配置正确；联调时还需确认下游 `rc26_merge_odom` 是否按预期消费该限速话题。
