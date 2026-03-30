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

## 源码入口与阅读顺序
- 先看 `launch/mid360_driver.launch.py`，确认参数文件和节点拉起方式。
- 再看 `src/mid360_driver_node.cpp`，这是 ROS 2 封装层。
- 然后看 `src/mid360_driver.cpp`，这里才是 Mid-360 UDP 接收和分帧核心。
- 最后看 `scripts/recover_mid360_stream.py`，了解流恢复运维路径。

## 目录解剖
- `mid360_driver.cpp`：底层 UDP 接收、点云/IMU 分流、帧聚合。
- `mid360_driver_node.cpp`：把底层驱动封装成 ROS 2 组件和发布器。
- `config/param.yaml`：驱动参数入口。
- `launch/mid360_driver.launch.py`：单节点启动。
- `scripts/recover_mid360_stream.py`：异常恢复脚本。

## 关键文件体量
- `src/mid360_driver.cpp`：424 行，底层接收核心。
- `src/mid360_driver_node.cpp`：280 行，ROS 2 封装层。
- `scripts/recover_mid360_stream.py`：289 行，运维脚本不算轻。

## 关键源码行段速览
- `src/rc26_mid360_driver/src/mid360_driver.cpp:40-59`：帧状态初始化和新帧切换辅助函数。
- `src/rc26_mid360_driver/src/mid360_driver.cpp:188-227`：驱动构造和停止逻辑。
- `src/rc26_mid360_driver/src/mid360_driver.cpp:228-372`：点云接收协程，负责分帧和组包。
- `src/rc26_mid360_driver/src/mid360_driver.cpp:373-424`：IMU 接收协程。
- `src/rc26_mid360_driver/src/mid360_driver_node.cpp:62-119`：`LidarPublisher` 把底层消息转换成 ROS 2 点云/IMU。
- `src/rc26_mid360_driver/src/mid360_driver_node.cpp:139-280`：ROS 2 节点构造、参数读取和驱动实例装配。

## 模块边界

- 这是驱动层，不做建图、定位和地形理解
- 它只负责把传感器数据可靠送入 ROS 2 图中
- 下游的姿态估计、地图配准和风险感知都不在这个包里实现
