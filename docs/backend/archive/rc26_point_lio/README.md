# rc26_point_lio

## 模块定位

`rc26_point_lio` 是 R2 当前的 LiDAR-Inertial Odometry 主链路，实现定制版 Point-LIO 建图/里程计。

## 当前实现

- 主可执行文件：`pointlio_mapping`
- 启动文件：`launch/point_lio.launch.py`（当前已收敛为纯 headless Point-LIO 入口）
- 关键配置：
  - `config/mid360.yaml`
  - `config/mid360_mapping_save.yaml`
- 自定义消息：`msg/LocalSensorExternalTrigger.msg`
- 运维脚本：`scripts/time_sync_analyzer.py`

核心源码包含：

- `Estimator.cpp/.hpp`
- `IMU_Processing.cpp/.hpp`
- `laserMapping.cpp`
- `li_initialization.cpp/.hpp`
- `parameters.cpp/.hpp`
- `preprocess.cpp/.hpp`

从当前代码和 README 看，这个包已经实现：

- Mid-360 适配后的 Point-LIO 主流程
- IEKF/配准估计与协方差输出
- 退化检测
- 点云保留比例与累计地图发布
- 面向 R2 场地的参数定制

## 源码入口与阅读顺序
- 先看 `launch/point_lio.launch.py` 和 `README.md`，确认 Mid-360 配置和运行方式。
- 再看 `src/laserMapping.cpp`，这里是主循环和绝大多数发布逻辑。
- 然后看 `parameters.cpp`、`preprocess.cpp`、`IMU_Processing.cpp`、`Estimator.cpp`、`li_initialization.cpp`。
- 最后看 `config/*.yaml` 和 `scripts/time_sync_analyzer.py`。

## 目录解剖
- `laserMapping.cpp`：Point-LIO 主程序、地图增量、点云/odom/path 发布。
- `parameters.cpp`：参数读取与协方差初始化。
- `preprocess.cpp`：不同雷达输入预处理和特征提取。
- `IMU_Processing.cpp`：IMU 初始化和状态推进。
- `Estimator.cpp`：误差状态模型和观测模型。
- `li_initialization.cpp`：传感器初始化回调。

## 关键文件体量
- `src/laserMapping.cpp`：1529 行，主循环极重。
- `src/preprocess.cpp`：849 行。
- `src/parameters.cpp`：418 行。
- `src/Estimator.cpp`：405 行。
- `README.md`：155 行。

## 关键源码行段速览
- `src/rc26_point_lio/src/laserMapping.cpp:73-515`：地图增量、点云/odom/path 发布等全局辅助函数；`516-1529`：`main()` 主循环和整个运行链。
- `src/rc26_point_lio/src/parameters.cpp:84-396`：参数读取和有效点过滤数设置；`397-418`：文件与协方差初始化辅助。
- `src/rc26_point_lio/src/preprocess.cpp:45-513`：点云输入预处理与不同雷达 handler；`514-848`：平面/边缘特征判断。
- `src/rc26_point_lio/src/IMU_Processing.cpp:16-101`：IMU 初始化和状态推进。
- `src/rc26_point_lio/src/Estimator.cpp:112-379`：观测模型和点到世界坐标变换。

## 模块边界

- 它是里程计/建图包，不做全局先验地图重定位
- 它不负责把传感器结果转换成下游统一里程计接口，那个职责在 `rc26_odom_interface`
- 它也不负责控制和决策
- 它的 `launch/point_lio.launch.py` 不再直接拉起 `rviz2`；可视化应通过 `rc26_bringup/launch/odometry.launch.py` 或 `rc26_xhu_viewer/launch/viewer.launch.py` 外部装配
