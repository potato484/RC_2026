# rc26_mid360_driver

## 模块定位

`rc26_mid360_driver` 是 R2 面向 Livox Mid-360 的专用雷达驱动包，负责直接接收雷达 UDP 数据并发布标准 ROS 2 点云与 IMU。

## 当前实现

- 构建方式：组件库 + 可执行节点
- 导出节点：`rc26_mid360_driver_node`
- 核心源码：
  - `src/mid360_driver.cpp`
  - `src/mid360_driver_node.cpp`
  - 以及对应头文件 `mid360_driver.hpp`、`mid360_driver_node.hpp`
- 参数文件：`config/param.yaml`
- 启动文件：`launch/mid360_driver.launch.py`
- 运维脚本：`scripts/recover_mid360_stream.py`

当前实现的工程重点是：

- 直接处理 Mid-360 网络数据
- 做事件驱动式点云分帧，而不是只靠粗糙定时器
- 处理时间戳、缓冲与吞吐保护
- 为 `rc26_point_lio` 和其他感知模块提供稳定原始输入

## 模块边界

- 这是驱动层，不做建图、定位和地形理解
- 它只负责把传感器数据可靠送入 ROS 2 图中
- 下游的姿态估计、地图配准和风险感知都不在这个包里实现
