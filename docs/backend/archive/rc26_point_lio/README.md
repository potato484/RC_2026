# rc26_point_lio

## 模块定位

`rc26_point_lio` 是 R2 当前的 LiDAR-Inertial Odometry 主链路，实现 ROS 2/Mid-360 形态的 Point-LIO 建图/里程计。

## 当前实现

- 主可执行文件：`pointlio_mapping`
- 启动文件：`launch/point_lio.launch.py`
- 关键配置：`config/mid360.yaml`
- 自定义消息：`msg/LocalSensorExternalTrigger.msg`
- 运维脚本：`scripts/time_sync_analyzer.py`、`scripts/pcd_map_inspector.py`、`scripts/pcd_to_nav2_map.py`

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
- `scripts/pcd_map_inspector.py` 提供只读 PCD 边界分析和 Nav2 map YAML 覆盖校验；支持 ASCII、binary 和 PCL binary_compressed PCD，只输出报告，不生成或修改栅格地图
- `scripts/pcd_to_nav2_map.py` 复用同一套 PCD 解析能力，把高度过滤后的 PCD 投影成 Nav2 `PNG + YAML` 黑白静态地图；默认 `resolution=0.05`、`z_min=0.05`、`z_max=2.0`、`min_points_per_cell=3`、`image_format=png`，也可显式生成 PGM
- 面向现场操作者的建图/里程计控制台提示按通俗中文输出；参数名、topic、frame、路径、返回码等机器信息仍保留原值，方便继续定位问题
- 实验性全局闭环能力，默认 `experimental_loop_closure.enable=false`；关闭时不创建闭环线程、不保存闭环关键帧、不运行 GTSAM，现有 `/state_estimation`、`/path`、`/cloud_registered`、TF 和 PCD 保存行为不变

当前已按旧 Point-LIO 主链收口，不再保留先验地图注入、输出侧高度裁剪、退化评分、自适应额外迭代或 profile 覆盖。累计全图只通过低频、降采样、限点数的可视化 topic 发布，不作为定位或导航权威。新增闭环只属于 Point-LIO odom 系实验优化，不读取 localization 的 `prior_pcd_file`，不接管 `map -> odom`。

## 源码入口与阅读顺序

- 先看 `launch/point_lio.launch.py` 和 `README.md`，确认单配置启动方式。
- 再看 `src/laserMapping.cpp`，这里是主循环和绝大多数发布逻辑。
- 然后看 `experimental_loop_closure.cpp`、`parameters.cpp`、`preprocess.cpp`、`IMU_Processing.cpp`、`Estimator.cpp`、`li_initialization.cpp`。
- 最后看 `config/mid360.yaml` 和 `scripts/time_sync_analyzer.py`。

## 目录解剖

- `laserMapping.cpp`：Point-LIO 主程序、地图增量、点云/odom/path 发布和 PCD 保存。
- `experimental_loop_closure.cpp/.hpp`：实验性关键帧、Scan Context/ICP 闭环检测和 GTSAM/iSAM2 位姿图优化后台。
- `parameters.cpp`：参数读取与协方差初始化。
- `preprocess.cpp`：不同雷达输入预处理和特征提取；当前 Mid-360 路径还会按 `base_link` 车身 ROI 丢弃自遮挡点。
- `IMU_Processing.cpp`：IMU 初始化和状态推进。
- `Estimator.cpp`：误差状态模型和旧版风格观测模型。
- `li_initialization.cpp`：传感器初始化回调。

## 关键文件体量

- `src/laserMapping.cpp`：约 1600 行。
- `src/experimental_loop_closure.cpp`：约 570 行。
- `src/preprocess.cpp`：901 行。
- `src/parameters.cpp`：约 540 行。
- `src/Estimator.cpp`：359 行。
- `README.md`：148 行。

## 关键源码行段速览

- `src/rc26_point_lio/src/laserMapping.cpp:60-162`：车身 ROI 参数校验、PCD 保存原因中文说明、TF 查询和运行时应用。
- `src/rc26_point_lio/src/laserMapping.cpp:170-260`：完整累计地图可视化缓存、降采样、限点数和低频发布。
- `src/rc26_point_lio/src/laserMapping.cpp:318-439`：地图增量、初始地图发布和 PCD 保存。
- `src/rc26_point_lio/src/laserMapping.cpp:629-739`：运行时参数回调，只允许车身 ROI 热更新，拒绝原因使用中文人类说明并保留参数名。
- `src/rc26_point_lio/src/laserMapping.cpp:780-795`：当前发布 topic 创建与完整累计地图可视化启动提示。
- `src/rc26_point_lio/src/laserMapping.cpp:1365-1413`：下采样后发布 odom、path、点云、外参估计提示、增量地图和运行耗时统计。
- `src/rc26_point_lio/src/experimental_loop_closure.cpp`：实验性闭环 backend；只有 YAML 开关开启后才由 `laserMapping.cpp` 创建和启动。
- `src/rc26_point_lio/src/parameters.cpp:134-159`：`point_filter_num`、点云和地图滤波参数读取。
- `src/rc26_point_lio/src/parameters.cpp:242-267`：普通点云、TF 和完整地图可视化发布参数读取。
- `src/rc26_point_lio/src/Estimator.cpp:108-322`：观测模型与特征平面约束。

## 模块边界

- 它是里程计/建图包，不做全局先验地图重定位。
- 它不消费 localization 使用的 `prior_pcd_file`；先验地图只属于定位/重定位链路。
- 实验性全局闭环只优化 Point-LIO 自身轨迹和 iVox 地图；它不是先验地图定位，也不改变 `rc26_localization` 的 `map -> odom` 权威。
- `/point_lio/map_cloud` 只服务现场观察完整累计地图，不作为 localization、Nav2 或外部控制链路的输入权威。
- 它不负责把传感器结果转换成下游统一里程计接口，那个职责在 `rc26_odom_interface`。
- 控制/导航默认直接消费 `rc26_odom_interface` 发布的 `/odom`；Point-LIO 不再通过独立预测包提供控制态。
- 它不负责车身到雷达的整机安装外参；`config/mid360.yaml` 中的 `mapping.extrinsic_T/R` 只表示 Point-LIO 内部 LiDAR/IMU 外参，车身安装位置和 yaw 归 `rc26_sensor_extrinsics` 管理。
- `point_lio.body_frame` 继续只是 Point-LIO 内部输出 frame 名；它会出现在 `/state_estimation.child_frame_id` 与可选 `/cloud_registered_body` 中，但当前不再作为对外 TF 边发布。
- 它会消费 `rc26_bringup` 发布的 `base_link -> livox_frame` 静态 TF，用于 Mid-360 输入侧车身 ROI 裁剪；该 TF 仍以 `rc26_sensor_extrinsics` 为配置真源。
- 它不负责控制和决策。
- 它的 launch 只启动 headless Point-LIO；如需观察，请手工运行外部 RViz/Foxglove 等工具只读订阅当前 topic。

## 配置注释口径

- `config/mid360.yaml` 保留常用/高影响参数的中文注释，重点说明 Point-LIO 常用预处理、IMU/点云时间、滤波、发布和建图保存相关字段。
- `point_filter_num` 是公开输入点云密度入口，按旧 Point-LIO 整数抽样语义工作。
- `filter_car_body` 和 `body_x/y/z_min/max` 是 Mid-360 输入侧车身 ROI 参数，单位米，坐标系为 `base_link`。这些参数支持热更新；初始配置或运行时更新若出现非有限数或 `min > max`，应直接失败。
- `publish.full_map_*` 控制完整累计地图可视化发布，默认开启、2 秒一次、0.1m 体素降采样、最多 150 万点。
- `experimental_loop_closure.*` 是实验性全局闭环参数组，默认关闭。开启后会保存关键帧、执行 Scan Context/ICP 候选验证和 GTSAM/iSAM2 优化；闭环成功后按优化关键帧相对原关键帧的位姿差回写当前 `kf_output` 或 `kf_input` 的 `pos/rot`，并重建 iVox 局部地图。
- `pcd_save.pcd_save_en` 当前默认开启，`pcd_save.interval=-1` 表示正常退出后写入单个 `scans.pcd`；长时间建图需注意单文件和内存压力。
- 点密度、滤波尺寸、量程、建图参数、发布开关和保存开关均需通过完整 YAML 修改后重启，不再作为热更新入口。

## 调试信息口径

- 建图主链的参数异常、IMU 初始化、LiDAR/IMU 时间回退、点云为空、车身 ROI 裁剪、完整地图可视化、PCD 保存、外参估计和耗时统计等用户可见提示已经改为中文。
- 实验性闭环开启时会输出中文告警和闭环成功日志；默认关闭时只记录 `experimental_loop_closure.enable=false`，不会改变主链运行行为。
- 中文提示不改变运行逻辑；`/state_estimation`、`/cloud_registered`、`/point_lio/map_cloud` 等 topic 名，自动导航链里的 `map/odom/base_footprint/base_link/livox_frame` 等 frame 名，以及 `filter_car_body`、`pcd_save.interval` 等参数名继续作为机器契约保留。
- `scripts/time_sync_analyzer.py` 的现场输出也按中文说明展示，推荐时间偏移仍保留 LiDAR/IMU 名称和数值单位，便于直接用于调试记录。
- `scripts/pcd_map_inspector.py` 用于现场核对 PCD bounds、推荐 Nav2 map `origin`/尺寸，以及已有 map YAML/image 是否覆盖点云范围；默认只读，不修改地图。
- `scripts/pcd_to_nav2_map.py` 是当前 PCD 后处理到 Nav2 2D 静态地图的生成入口，默认剔除地板/天花板点云并生成黑白 PNG 二值图；localization 的 `prior_pcd_file` 仍使用 PCD，Nav2 `map_server` 使用生成的 `PNG + YAML`。
