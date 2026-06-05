# rc26_point_lio

## 模块定位

`rc26_point_lio` 是 R2 当前的 LiDAR-Inertial Odometry 主链路，实现 ROS 2/Mid-360 形态的 Point-LIO 建图/里程计。

## 当前实现

- 主可执行文件：`pointlio_mapping`
- 启动文件：`launch/point_lio.launch.py`
- 关键配置：`config/mid360.yaml`
- 自定义消息：`msg/LocalSensorExternalTrigger.msg`
- 运维脚本：`scripts/time_sync_analyzer.py`

核心源码包含：

- `Estimator.cpp/.hpp`
- `IMU_Processing.cpp/.hpp`
- `laserMapping.cpp`
- `li_initialization.cpp/.hpp`
- `parameters.cpp/.hpp`
- `preprocess.cpp/.hpp`

运行建图时可能在包内生成 `PCD/` 点云输出目录；该目录属于本地运行产物，不作为版本库资产维护。

从当前代码和 README 看，这个包保留：

- Mid-360 适配后的 Point-LIO 主流程
- IEKF/配准估计与协方差输出
- `point_filter_num` 整数抽样入口
- Mid-360 输入侧车身 ROI 裁剪，运行时只允许 `filter_car_body` 与 `body_*` 热更新
- `/state_estimation`、`/cloud_registered`、`/cloud_registered_body`、`/Laser_map`、`/point_lio/map_cloud`、`/path`
- 默认低频完整累计地图可视化发布，供 RViz/Foxglove 等下游只读观察
- 默认基础 PCD 保存能力，`interval=-1` 时正常退出后写入单个 `scans.pcd`

当前已按旧 Point-LIO 主链收口，不再保留先验地图注入、输出侧高度裁剪、退化评分、自适应额外迭代或 profile 覆盖。累计全图只通过低频、降采样、限点数的可视化 topic 发布，不作为定位或导航权威。

## 源码入口与阅读顺序

- 先看 `launch/point_lio.launch.py` 和 `README.md`，确认单配置启动方式。
- 再看 `src/laserMapping.cpp`，这里是主循环和绝大多数发布逻辑。
- 然后看 `parameters.cpp`、`preprocess.cpp`、`IMU_Processing.cpp`、`Estimator.cpp`、`li_initialization.cpp`。
- 最后看 `config/mid360.yaml` 和 `scripts/time_sync_analyzer.py`。

## 目录解剖

- `laserMapping.cpp`：Point-LIO 主程序、地图增量、点云/odom/path 发布和 PCD 保存。
- `parameters.cpp`：参数读取与协方差初始化。
- `preprocess.cpp`：不同雷达输入预处理和特征提取；当前 Mid-360 路径还会按 `base_link` 车身 ROI 丢弃自遮挡点。
- `IMU_Processing.cpp`：IMU 初始化和状态推进。
- `Estimator.cpp`：误差状态模型和旧版风格观测模型。
- `li_initialization.cpp`：传感器初始化回调。

## 关键文件体量

- `src/laserMapping.cpp`：1429 行。
- `src/preprocess.cpp`：901 行。
- `src/parameters.cpp`：425 行。
- `src/Estimator.cpp`：359 行。
- `README.md`：148 行。

## 关键源码行段速览

- `src/rc26_point_lio/src/laserMapping.cpp:60-147`：车身 ROI 参数校验、TF 查询和运行时应用。
- `src/rc26_point_lio/src/laserMapping.cpp:150-246`：完整累计地图可视化缓存、降采样、限点数和低频发布。
- `src/rc26_point_lio/src/laserMapping.cpp:304-424`：地图增量、初始地图发布和 PCD 保存。
- `src/rc26_point_lio/src/laserMapping.cpp:614-715`：运行时参数回调，只允许车身 ROI 热更新。
- `src/rc26_point_lio/src/laserMapping.cpp:768-780`：当前发布 topic 创建。
- `src/rc26_point_lio/src/laserMapping.cpp:1350-1377`：下采样后发布 odom、path、点云和增量地图。
- `src/rc26_point_lio/src/parameters.cpp:134-159`：`point_filter_num`、点云和地图滤波参数读取。
- `src/rc26_point_lio/src/parameters.cpp:242-267`：普通点云、TF 和完整地图可视化发布参数读取。
- `src/rc26_point_lio/src/Estimator.cpp:108-322`：观测模型与特征平面约束。

## 模块边界

- 它是里程计/建图包，不做全局先验地图重定位。
- 它不消费 localization 使用的 `prior_pcd_file`；先验地图只属于定位/重定位链路。
- `/point_lio/map_cloud` 只服务现场观察完整累计地图，不作为 localization、Nav2 或外部控制链路的输入权威。
- 它不负责把传感器结果转换成下游统一里程计接口，那个职责在 `rc26_odom_interface`。
- 控制/导航默认直接消费 `rc26_odom_interface` 发布的 `/odom`；Point-LIO 不再通过独立预测包提供控制态。
- 它不负责车身到雷达的整机安装外参；`config/mid360.yaml` 中的 `mapping.extrinsic_T/R` 只表示 Point-LIO 内部 LiDAR/IMU 外参，车身安装位置和 yaw 归 `rc26_sensor_extrinsics` 管理。
- 它会消费 `rc26_bringup` 发布的 `base_link -> livox_frame` 静态 TF，用于 Mid-360 输入侧车身 ROI 裁剪；该 TF 仍以 `rc26_sensor_extrinsics` 为配置真源。
- 它不负责控制和决策。
- 它的 launch 只启动 headless Point-LIO；如需观察，请手工运行外部 RViz/Foxglove 等工具只读订阅当前 topic。

## 配置注释口径

- `config/mid360.yaml` 保留常用/高影响参数的中文注释，重点说明 Point-LIO 常用预处理、IMU/点云时间、滤波、发布和建图保存相关字段。
- `point_filter_num` 是公开输入点云密度入口，按旧 Point-LIO 整数抽样语义工作。
- `filter_car_body` 和 `body_x/y/z_min/max` 是 Mid-360 输入侧车身 ROI 参数，单位米，坐标系为 `base_link`。这些参数支持热更新；初始配置或运行时更新若出现非有限数或 `min > max`，应直接失败。
- `publish.full_map_*` 控制完整累计地图可视化发布，默认开启、2 秒一次、0.1m 体素降采样、最多 150 万点。
- `pcd_save.pcd_save_en` 当前默认开启，`pcd_save.interval=-1` 表示正常退出后写入单个 `scans.pcd`；长时间建图需注意单文件和内存压力。
- 点密度、滤波尺寸、量程、建图参数、发布开关和保存开关均需通过完整 YAML 修改后重启，不再作为热更新入口。
