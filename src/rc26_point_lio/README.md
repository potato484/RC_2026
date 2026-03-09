# Point-LIO (RC2026 定制版)

## 简介
Point-LIO 是一个鲁棒、高带宽的 LiDAR 惯性里程计，基于逐点更新的扩展卡尔曼滤波（IEKF）框架。本项目是 Point-LIO 的定制版本，专为 RoboMaster 2026 赛季机器人（R2，四驱麦克纳姆轮底盘）进行了深度优化。

## 核心特性
本项目在原版 Point-LIO 基础上进行了以下关键改进：

1. **IVox 加速与精度优化**  
   调整了网格分辨率和近邻搜索策略，适配 Mid360 在 12x12m 比赛场地的高精度定位需求。通过优化参数，在保证实时性的同时显著提升了点云匹配的精度。

2. **低延迟控制输出**  
   通过消除帧末等待的机制，实现了低延迟的状态估计输出。这使得里程计能够支持高频控制回路，减少了控制系统的相位滞后。

3. **鲁棒性增强**  
   - **Huber 核函数**：启用并自适应调整测量权重，有效抑制了动态障碍物和噪点对定位的影响。
   - **自适应二次迭代**：针对剧烈运动（如快速自旋）场景，引入了自适应二次迭代机制。当检测到残差过大时，算法会自动触发额外迭代，以保证位姿估计的稳定性。

4. **状态监测与全链路贯通**  
   - **退化检测**：实时计算 Hessian 矩阵特征值比率，输出退化分数，便于下游策略识别风险。
   - **协方差传递**：完整计算并填充里程计消息协方差，供下游融合节点使用。

5. **动态点云密度与累计地图发布**  
   - **`point_keep_ratio`**：使用百分比语义控制输入点保留比例，支持运行时 `ros2 param set` 动态调整。
   - **`laser_map_full`**：持续发布累计地图，方便在 RViz / Foxglove 中观察“已建好的内容是否持续留存显示”。
   - **PCD 保存**：建图时可将累计点云保存为 `PCD/scans.pcd` 或分段 `PCD/scans_*.pcd`，供后续定位复用。

## 依赖项说明
本项目依赖于 ROS 2 (Humble/Iron 版本)、PCL 点云库、Eigen3 线性代数库以及 Livox SDK2。请确保系统环境中已正确安装这些依赖库。

## 编译与安装
请使用标准的 ROS 2 编译工具（如 colcon）进行编译。建议与下游链路一起编译验证：

```bash
colcon build --symlink-install --parallel-workers 3 --packages-select rc26_point_lio rc26_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 配置文件说明
当前仓库内和 Point-LIO 相关的常用配置如下：

- `src/rc26_point_lio/config/mid360.yaml`：通用默认配置，适合直接调参与兼容旧流程。
- `src/rc26_point_lio/config/mid360_cruise_light.yaml`：轻量巡航配置，默认约保留 50% 输入点，并关闭累计地图持续发布。
- `src/rc26_point_lio/config/mid360_mapping_dense.yaml`：高密建图配置，保留全部输入点、降低体素滤波尺寸，并开启累计地图与 PCD 保存。
- `src/rc26_point_lio/config/mid360_mapping_save.yaml`：保留的建图保存配置，适合兼容旧脚本或对比测试。

## 推荐启动方式
推荐通过 `rc26_bringup` 统一启动，让 `slam` 与 `point_lio_profile` 一起决定 Point-LIO 配置。

### 1. 自动选择 profile

```bash
# slam:=true 时自动选择 mapping_dense
# slam:=false 时自动选择 cruise_light
ros2 launch rc26_bringup bringup.launch.py slam:=true visualization_backend:=rviz use_decision:=false
```

### 2. 显式指定 profile

```bash
# 强制高密建图
ros2 launch rc26_bringup bringup.launch.py slam:=true point_lio_profile:=mapping_dense use_decision:=false

# 强制轻量巡航
ros2 launch rc26_bringup bringup.launch.py slam:=false point_lio_profile:=cruise_light use_decision:=false
```

### 3. 显式指定自定义 YAML

```bash
ros2 launch rc26_bringup bringup.launch.py \
  slam:=true \
  point_lio_config_file:=${RC26_WS:-$HOME/RC_2026}/src/rc26_point_lio/config/mid360_mapping_dense.yaml \
  use_decision:=false
```

当 `point_lio_config_file` 非空时，其优先级高于 `point_lio_profile`。

## 运行时动态调参
以下参数支持运行时动态调整：

```bash
# 将输入点保留比例调成约 30%
ros2 param set /point_lio point_keep_ratio 30.0

# 打开累计地图持续发布
ros2 param set /point_lio publish.map_full_publish_en true

# 将累计地图发布周期调成 0.5 秒
ros2 param set /point_lio publish.map_full_publish_interval_sec 0.5

# 调小单帧点云体素滤波尺寸，让 registered_scan 更稠密
ros2 param set /point_lio filter_size_surf 0.1
```

说明：
- `point_keep_ratio` 取值区间建议为 `1.0 ~ 100.0`，其中 `100.0` 表示尽可能保留全部输入点。
- `point_keep_ratio` 是“百分比语义”，最终显示密度还会受 `filter_size_surf` / `filter_size_map` 影响，因此不是严格数学百分比。
- 若希望回退到旧的“每 N 个点取 1 个”语义，可将 `point_keep_ratio` 设为负值，然后直接使用 `point_filter_num`。

## 地图保存与复用
若启用建图保存，Point-LIO 会将累计地图写入：

- `${RC26_WS:-$HOME/RC_2026}/src/rc26_point_lio/PCD/scans.pcd`
- 或 `${RC26_WS:-$HOME/RC_2026}/src/rc26_point_lio/PCD/scans_<N>.pcd`（当 `pcd_save.interval > 0` 时）

推荐流程：
1. 使用 `mapping_dense` 或 `mid360_mapping_save.yaml` 建图；
2. 完成后正常 `Ctrl+C` 退出，让节点将剩余累计点写盘；
3. 将生成的 `PCD` 作为 `prior_pcd_file` 提供给定位链路；
4. 也可以复制到 `src/rc26_bringup/pcd/` 目录集中管理。

## 可视化说明
- `/registered_scan`：当前帧配准点云，适合观察实时局部效果。
- `/laser_map_full`：累计地图点云，适合观察历史建图内容是否持续保留。
- `slam.rviz` 已默认加入 `LaserMapFull` 显示项。

## 调试与测试
为了帮助开发者快速上手和排查问题，我们在 `docs` 目录下提供了详细的调试指南。该指南包含：
- 如何使用 bag 回放进行基础功能测试
- 如何验证控制延迟、退化检测等进阶性能
- 如何检查动态点密度与累计地图发布
- 如何保存 PCD 并复用到定位链路

请查阅 `docs/debug_guide.md` 获取详细信息。

## 维护者
- 原始作者: Dongjiao He (HKU-MARS)
- RC2026 适配: Potato
