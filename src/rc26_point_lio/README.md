# Point-LIO (RC2026 ROS2/Mid-360)

## 简介

`rc26_point_lio` 是 R2 当前的 LiDAR-Inertial Odometry 主链路。当前实现以 `Point-LIO-point-lio-with-grid-map` 的主算法链为行为基准，保留 ROS 2 包形态、Mid-360 `PointCloud2` 输入、车身 ROI 过滤、frame/TF 装配能力和基础 PCD 保存能力。

本包不再承担先验地图注入、输出侧高度裁剪、退化评分或控制降级判断；完整累计地图只通过低频可视化 topic 对外发布。全局先验地图和重定位职责属于 localization 链路；控制/导航默认直接消费 `rc26_odom_interface` 发布的 `/odom`。

## 当前保留能力

- ROS 2 `rclcpp` 节点：`pointlio_mapping`
- Mid-360 `sensor_msgs/msg/PointCloud2` 输入预处理
- Point-LIO IEKF/配准主流程与协方差输出
- `point_filter_num` 整数抽样入口，语义与旧 Point-LIO 一致
- `filter_car_body` 与 `body_*` 车身 ROI 输入侧过滤
- 车身 ROI 运行时热更新，依赖 `base_link <- livox_frame` TF
- `/state_estimation`、`/cloud_registered`、`/cloud_registered_body`、`/Laser_map`、`/point_lio/map_cloud`、`/path`
- 低频完整累计点云发布：默认向 `/point_lio/map_cloud` 发布可视化用完整地图
- 基础 PCD 保存：默认开启，正常退出后写入包内 `PCD/`
- PCD 地图检查脚本：`scripts/pcd_map_inspector.py` 可解析 PCD 边界，并只读校验 Nav2 map YAML/image 覆盖范围

## 已删除链路

- Point-LIO 内部先验 PCD 加载、初始位姿注入和增量建图延迟
- IVox 全量遍历后的高频/无界累计地图持续发布
- 发布/保存输出侧世界系高度裁剪
- 输入点百分比密度参数与相关动态换算
- 退化评分发布、残差统计、Huber 权重和自适应额外迭代
- 除车身 ROI 外的运行时动态调参入口
- launch profile 覆盖；Point-LIO 启动只选择一份完整 YAML

## 编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_point_lio rc26_bringup
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 配置

默认配置文件：

- `src/rc26_point_lio/config/mid360.yaml`

关键参数：

- `point_filter_num`：输入点抽样间隔，`1` 表示尽量保留全部输入点，`2` 表示约二分之一。
- `filter_car_body`、`body_x_min/max`、`body_y_min/max`、`body_z_min/max`：Mid-360 输入侧车身 ROI，单位米，坐标系为 `base_link`。
- `odometry.publish_odometry_without_downsample`：默认 `False`，保持 `/state_estimation` 与 `/cloud_registered` 的时间戳同源。
- `publish.scan_bodyframe_pub_en`：控制 `/cloud_registered_body` 是否发布。
- `publish.full_map_publish_en`、`publish.full_map_topic`、`publish.full_map_interval_sec`、`publish.full_map_voxel_size`、`publish.full_map_max_points`：控制完整累计地图可视化发布。默认开启，低频、降采样并限制单次发布点数；该 topic 只用于观察，不作为定位或导航权威。
- `pcd_save.pcd_save_en`、`pcd_save.interval`：控制 PCD 保存。默认开启且 `interval=-1`，建图正常退出后保存单个 `scans.pcd`。

`mapping.extrinsic_T/R` 是 Point-LIO 内部 LiDAR/IMU 外参，不表示雷达相对 `base_link` 的整机安装位姿。雷达安装外参由 `rc26_sensor_extrinsics` 管理，并经 `rc26_bringup` 发布静态 TF。

## 启动

推荐通过 bringup 装配：

```bash
ros2 launch rc26_bringup bringup.launch.py slam:=true use_decision:=false
```

纯建图启动入口：

```bash
ros2 launch rc26_bringup test_mapping.launch.py
```

只启动 Point-LIO 节点：

```bash
ros2 launch rc26_point_lio point_lio.launch.py
```

需要替换完整配置时，通过上层 bringup 传入完整 YAML：

```bash
ros2 launch rc26_bringup bringup.launch.py \
  slam:=true \
  point_lio_config_file:=/abs/path/to/custom_point_lio.yaml \
  use_decision:=false
```

`point_lio_config_file` 只选择完整配置文件，不叠加预设覆盖。

## 运行时动态调参

当前只支持车身 ROI 热更新：

```bash
ros2 param set /point_lio filter_car_body false
ros2 param set /point_lio filter_car_body true
ros2 param set /point_lio body_x_min -0.45
ros2 param set /point_lio body_z_max 0.75
```

说明：

- `body_x/y/z_min/max` 必须是有限数，且每个轴满足 `min <= max`。
- 从 `filter_car_body=false` 切回 `true` 时，节点必须能查到 `base_link <- livox_frame` TF，否则参数更新会被拒绝。
- 点密度、体素尺寸、量程、平面阈值、测量协方差、iVox 分辨率、发布选项和 PCD 保存开关都不是运行时热更新入口；需要改完整 YAML 后重启。

## 话题

Point-LIO 原生发布：

- `/state_estimation`：LIO 里程计输出。
- `/cloud_registered`：odom/world 系当前帧配准点云。
- `/cloud_registered_body`：body/IMU 系当前帧点云，受 `publish.scan_bodyframe_pub_en` 控制。
- `/Laser_map`：初始地图点云，只在初始化建图完成后发布。
- `/point_lio/map_cloud`：低频发布的完整累计点云地图，默认 2 秒一次，按 `publish.full_map_voxel_size` 降采样后供 RViz/Foxglove 等下游只读观察。
- `/path`：Point-LIO 原生路径，受 `publish.path_en` 控制。

经过 `rc26_odom_interface` 后，下游通常消费 `/odom` 与 `/registered_scan`；`/registered_scan` 由 `/cloud_registered` 转换到统一 odom 坐标系后发布。

## PCD 保存与定位复用

默认 `pcd_save.pcd_save_en=true` 且 `pcd_save.interval=-1`。节点正常退出时会将累计点云写入：

- `${RC26_WS:-$HOME/RC_2026}/src/rc26_point_lio/PCD/scans.pcd`
- 或 `${RC26_WS:-$HOME/RC_2026}/src/rc26_point_lio/PCD/scans_<N>.pcd`，当 `pcd_save.interval > 0` 时分段保存。

`interval=-1` 会把待保存点云累计到正常退出时再写单个文件；长时间建图时需注意单文件大小和内存压力。

生成的 PCD 可作为 localization 链路的 `prior_pcd_file` 使用。Point-LIO 自身不会再读取这份先验地图。

### PCD 边界与 Nav2 map 校验

`scripts/pcd_map_inspector.py` 是只读诊断工具，用于把 PCD 的 x/y/z 边界、推荐 Nav2 `origin`/图片尺寸和已有 map YAML 覆盖情况打印出来。它支持 `DATA ascii`、`DATA binary` 和 PCL `DATA binary_compressed`，不会生成栅格图，也不会修改 `test.yaml`。

源码树直接运行：

```bash
python3 src/rc26_point_lio/scripts/pcd_map_inspector.py \
  src/rc26_point_lio/PCD/scan.pcd \
  --map-yaml src/rc26_bringup/map/test.yaml
```

安装后运行：

```bash
ros2 run rc26_point_lio pcd_map_inspector.py \
  src/rc26_point_lio/PCD/scan.pcd \
  --map-yaml src/rc26_bringup/map/test.yaml
```

常用参数：

- `--resolution 0.05`：按指定分辨率计算推荐图片宽高和 `origin`。
- `--padding-m 0.2`：推荐地图边界时额外留白。
- `--z-min / --z-max`：只用指定高度范围内的点计算 2D 边界。
- `--json`：输出结构化 JSON，供后续脚本消费。

## 地面点云说明

- 建图时看到地面点通常是正常现象，Mid-360 本身具备向下观测能力。
- 对 LIO 而言，地面通常能提供 `z / roll / pitch` 平面约束，不建议从内部里程计地图中粗暴删除。
- 车身 ROI 只用于删除车体自遮挡/自反射盒内点，不应用作大范围地面过滤。
- 如果地面明显发厚、倾斜、上下漂移或分层，优先检查 `extrinsic_T/R`、`gravity`、静态 TF、安装角和驱动侧时间同步。

## 可视化

Point-LIO 和 bringup 主入口保持 headless。需要观察时手工运行：

```bash
rviz2 -d "${RC26_WS:-$HOME/RC_2026}/src/rc26_bringup/rviz/slam.rviz"
```

该 RViz 预设观察 `/point_lio/map_cloud` 完整累计地图、`/registered_scan` 实时点云和 `/Laser_map` 初始地图。完整累计地图是现场可视化输出，不替代 PCD 文件，也不作为 localization 或 Nav2 输入。

## 运行排查

根目录集中式调试文档已删除。Point-LIO 主链、预处理、IMU 初始化、PCD 保存、车身 ROI 热更新和运行耗时统计等用户可见提示按中文输出；topic、frame、参数名、路径和返回码仍保留原值，方便继续排查。

若需要分析 LiDAR/IMU 时间偏移，可运行 `scripts/time_sync_analyzer.py`。该脚本输出中文说明和外部时间偏移建议，Point-LIO 当前不直接消费该结果。若需要排查 PCD 与 Nav2 map YAML 的尺寸、origin 和覆盖关系，使用 `scripts/pcd_map_inspector.py`。
