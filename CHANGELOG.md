# 更新日志（Changelog）

本仓库以 Git Tag 的形式进行版本发布（例如 `v2.0.0`），遵循语义化版本（SemVer）的仓库级版本号管理方式。

## v2.0.0

发布时间：2026-01-17  
发布方式：Git Tag `v2.0.0`

### 重大变更（Breaking Changes）

1. `point_lio` 的 MID360 默认配置行为发生变化（可能影响现有 launch/参数依赖）
   - 文件：`RC_2026_1/point_lio/config/mid360.yaml`
   - 主要变化：
     - 默认话题名改为不带前导 `/` 的形式（例如 `livox/lidar`、`livox/imu`），与部分 ROS2 工程中常见的相对命名方式保持一致。
     - `timestamp_unit` 配置调整（该参数决定点云时间戳字段的单位解释），需要与你的 MID360 驱动输出保持匹配，否则可能导致时间同步/里程计漂移等问题。
   - 升级建议：
     - 如果你的系统其他节点/launch 强依赖 `/livox/lidar`、`/livox/imu` 这类绝对话题名，请在 `mid360.yaml` 中改回原值，或在 launch 中做 remap。
     - 如果出现时间相关异常（例如里程计抖动、时间不同步），优先检查 `timestamp_unit` 与点云字段单位是否一致。

### 功能更新（Features）

1. 新增底盘离地估计模块 `rc26_base_ground`
   - 新增包目录：`RC_2026_1/rc26_base_ground/`
   - 内容包括：
     - 参数：`RC_2026_1/rc26_base_ground/config/base_ground_estimator.yaml`
     - 启动文件：`RC_2026_1/rc26_base_ground/launch/base_ground_estimator.launch.py`
     - 节点实现：`RC_2026_1/rc26_base_ground/src/base_ground_estimator_node.cpp`
     - 头文件接口：`RC_2026_1/rc26_base_ground/include/rc26_base_ground/base_ground_estimator.hpp`

2. 新增手柄遥控模块 `rc26_telecontrol`
   - 新增包目录：`RC_2026_1/rc26_telecontrol/`
   - 内容包括：
     - 参数：`RC_2026_1/rc26_telecontrol/config/joy_params.yaml`、`RC_2026_1/rc26_telecontrol/config/joy_params_dpad.yaml`
     - 启动文件：`RC_2026_1/rc26_telecontrol/launch/wheeltec_joy.launch.py`
     - 节点实现：`RC_2026_1/rc26_telecontrol/src/wheeltec_joy.cpp`、`RC_2026_1/rc26_telecontrol/src/wheeltec_joy_dpad.cpp`

3. `rc26_merge_odom` 串口相关脚本补充
   - 新增：`RC_2026_1/rc26_merge_odom/src/scripts/serial_frame_parser.py`

### 优化与修复（Improvements & Fixes）

1. MID360 驱动发布 QoS 调整，提升与下游建图/里程计节点的兼容性
   - 文件：`RC_2026_1/mid360_driver/src/mid360_driver_node.cpp`
   - 变化：PointCloud2 与 IMU publisher 改用 `rclcpp::SensorDataQoS()`（常见为 BEST_EFFORT 语义），以匹配传感器类数据流的典型消费端设置，减少 QoS 不匹配导致的“收不到消息”等问题。

2. MID360 IMU frame 配置修正
   - 文件：`RC_2026_1/mid360_driver/config/param.yaml`
   - 变化：`imu_frame` 调整为 `livox_frame`，使 IMU 与 LiDAR frame 更一致（具体以你的 TF 树设计为准）。

3. `rc26_bringup` 启动流程与 RViz 配置更新
   - 文件：
     - `RC_2026_1/rc26_bringup/launch/bringup.launch.py`
     - `RC_2026_1/rc26_bringup/launch/odometry.launch.py`
     - `RC_2026_1/rc26_bringup/rviz/slam.rviz`
   - 说明：适配新的模块/参数组织与里程计链路。

4. 决策相关逻辑与构建更新
   - 文件：
     - `RC_2026_1/rc26_decision/CMakeLists.txt`
     - `RC_2026_1/rc26_decision/include/rc26_decision/mf/mf_area.hpp`
     - `RC_2026_1/rc26_decision/src/decision_node.cpp`
     - `RC_2026_1/rc26_decision/src/mc/mc_area.cpp`
     - `RC_2026_1/rc26_decision/src/mf/mf_area.cpp`

5. 里程计融合/接口/串口协议等更新
   - 文件：
     - `RC_2026_1/rc26_merge_odom/...`
     - `RC_2026_1/rc26_odom_interface/...`
     - `RC_2026_1/rc26_serial/...`

### 工程与仓库卫生（Repo Hygiene）

1. 避免将运行时产物/缓存纳入版本控制
   - `.gitignore` 新增忽略：
     - Python：`__pycache__/`、`*.pyc`
     - 运行日志：`RC_2026_1/point_lio/Log/`
     - 大体积 PDF：`RC_2026_1/rc26_pdf/`（建议作为 Release 附件或文档外链管理）
   - 同时将之前误加入仓库的 `__pycache__/*.pyc` 从 Git 追踪中移除。

