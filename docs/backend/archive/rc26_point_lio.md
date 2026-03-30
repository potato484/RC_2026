# rc26_point_lio

## 模块定位

`rc26_point_lio` 是 R2 当前的 LiDAR-Inertial Odometry 主链路，实现定制版 Point-LIO 建图/里程计。

## 当前实现

- 主可执行文件：`pointlio_mapping`
- 启动文件：`launch/point_lio.launch.py`
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

## 模块边界

- 它是里程计/建图包，不做全局先验地图重定位
- 它不负责把传感器结果转换成 Nav2 标准输出，那个职责在 `rc26_odom_interface`
- 它也不负责控制和决策
