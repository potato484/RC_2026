# point_lio 模块需求总结

## 1. 模块定位与背景
`point_lio` 的核心职能是提供 LiDAR-IMU 里程计（LIO）与增量建图能力。
它的目标非常明确：在比赛场景中实时输出连续、低延迟的位姿估计，供导航和上层策略使用。同时，它还需要输出配准后的点云，方便后续的地形分析、定位以及调试工作。
本工程的实现重点在于处理点云内部的时间戳、保证 IMU 数据的时间覆盖、实时的点云下采样以及地图的增量维护。

## 2. 接口需求（ROS Topic / TF）
根据现有实现，本模块需要提供以下接口：

### 输入（需满足传感器链路 QoS）
*   **点云**：通过参数 `common.lid_topic` 指定（默认为 `livox/lidar`），类型为 `sensor_msgs/PointCloud2`。
*   **IMU**：通过参数 `common.imu_topic` 指定（默认为 `livox/imu`），类型为 `sensor_msgs/Imu`。

### 核心输出
*   **里程计 (`state_estimation`)**：类型为 `nav_msgs/Odometry`。坐标系定义为 `header.frame_id=frame.odom_frame`，`child_frame_id=frame.body_frame`。
*   **配准点云 (`cloud_registered`)**：类型为 `sensor_msgs/PointCloud2`。这是配准到 `frame.odom_frame` 下的点云。
*   **机体系点云 (`cloud_registered_body`)**：类型为 `sensor_msgs/PointCloud2`。这是在 `frame.body_frame` 下的点云，主要用于机体系下的数据消费或调试。
*   **轨迹 (`path`)**：类型为 `nav_msgs/Path`。在 `frame.odom_frame` 下的轨迹记录。
*   **初始地图 (`Laser_map`)**：类型为 `sensor_msgs/PointCloud2`。仅在初始化阶段发布一次。

### TF 变换（可选）
*   当参数 `publish.tf_send_en` 设为 `true` 时，模块会发布 `frame.odom_frame` 到 `frame.body_frame` 的 TF 变换。
*   **注意**：在 `rc26_bringup` 的总集成中，默认将其关闭，以避免与后续的接口节点发生 TF 冲突。

## 3. 点云预处理与时间同步
为了保证实时性和精度，预处理环节有以下硬性要求：

1.  **统一的时间基准**：
    *   必须支持不同雷达的点云格式，并将其统一为“点内相对时间”。
    *   对于 MID-360，点云数据必须包含 `timestamp(double)` 字段。
    *   通过 `preprocess.timestamp_unit` 参数指定时间单位（秒/毫秒/微秒/纳秒），程序会自动将其换算为“相对于帧头的毫秒偏移量”，用于后续的排序和去畸变。

2.  **实时过滤与下采样**：
    *   **视场控制**：使用 `preprocess.blind` 剔除近距离盲区，使用 `mapping.det_range` 限制最大探测距离。
    *   **数据量控制**：通过 `point_filter_num`（采样间隔）、`space_down_sample`（体素降采样）以及 `filter_size_surf`/`filter_size_map` 等参数严格控制点云规模。这是防止在算力受限平台上出现掉帧的关键。

## 4. LiDAR-IMU 同步与鲁棒性
1.  **IMU 数据必须覆盖点云时域**：
    *   在数据组包阶段，逻辑上要求 `last_timestamp_imu >= lidar_end_time`。如果 IMU 数据滞后，必须等待，绝不能使用不完整的 IMU 序列进行状态传播，否则会导致状态发散。

2.  **处理时间回绕**：
    *   必须具备检测“时间倒流”的能力（常见于播放 rosbag 或系统时间重置）。
    *   一旦检测到 LiDAR 或 IMU 时间戳回绕，系统必须立即清空缓存并重置相关状态，防止旧数据污染地图或滤波器。

## 5. 初始化与地图加载
1.  **IMU 初始化与重力对齐**：
    *   启动时，需要累计一定数量的 IMU 数据（当前实现为 100 帧）来估算平均加速度方向，完成初始姿态（重力方向）的对齐。
    *   如果设置 `mapping.imu_en=false`，则跳过此步骤，直接使用 `mapping.gravity_init` 作为已知的重力方向。

2.  **地图初始化**：
    *   **默认模式**：累计足够的点云帧（由 `init_map_size` 控制）来构建初始地图。
    *   **先验地图模式**：若 `prior_pcd.enable=true`，则优先尝试加载 `prior_pcd.prior_pcd_map_path` 指定的 PCD 文件。如果加载失败或文件为空，则自动回退到默认的累积模式。

3.  **首次位姿粗对齐**：
    *   在使用先验地图时，`prior_pcd.init_pose` 参数仅在第一帧输出时生效，用于给出一个粗略的初始位姿，帮助系统快速进入有效的配准区域。

## 6. 输出配置与调试能力
1.  **里程计时间戳的可配置性**：
    *   通过 `odometry.publish_odometry_without_downsample` 参数，可以选择里程计的时间戳是使用“内部传播时刻”还是“点云扫描结束时刻”。这需要在下游模块（如控制、导航、数据记录）的不同需求之间做权衡。

2.  **日志与离线分析**：
    *   提供开关将状态数据写入 `ROOT_DIR/Log` 目录，方便赛后复盘。
    *   如果日志文件创建失败，系统应仅输出告警，决不能阻塞主流程。
    *   允许通过 `pcd_save.*` 参数保存点云文件，但这属于可选的调试功能，开启后可能会影响实时性。

## 7. 现状说明
*   虽然配置文件中存在 `common.time_diff_lidar_to_imu` 等关于时间偏移的字段，但目前的参数读取逻辑并未实际使用该字段。
*   **结论**：当前版本主要依赖 ROS 消息头的时间戳和点云内部的时间字段来完成同步，不会额外施加固定的时间偏移量。

## 8. 模块间契约
为了保证系统正常运行，本模块与其他模块有以下约定：
1.  **`mid360_driver`**：输出的点云必须包含 `timestamp(double)` 字段，且其时间单位必须与 `point_lio` 的 `preprocess.timestamp_unit` 配置完全一致。
2.  **`rc26_odom_interface`**：负责承接 `state_estimation` 和 `cloud_registered`，将其转换为工程统一标准的 `odom` 和 `registered_scan` 格式，并负责补齐或统一 TF 的发布策略。