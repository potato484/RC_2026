# rc26_sensor_scan

## 模块定位

`rc26_sensor_scan` 是点云与里程计的时空对齐模块，用来给 Nav2 obstacle layer、导航链调试和其它下游局部感知模块提供“已经同步并投影到传感器视角”的干净输入。

## 当前实现

- 构建方式：组件库 + 可执行节点
- 导出节点：`rc26_sensor_scan_node`
- 核心源码：`src/rc26_sensor_scan/src/sensor_scan.cpp`

当前实现聚焦于：

- 对点云帧和里程计帧做时间同步
- 根据 TF 做逆向投影，把全局/底盘视角点云转换回传感器局部视角
- 同步分发局部视角点云和对应姿态
- 减少重复计算，尽量透传上游已存在的状态量
- 当前 `sensor_scan` 话题默认使用 `livox_frame` 作为 `frame_id`；`rc26_bringup/config/nav2_params.yaml` 的 local/global obstacle layer 默认消费该 PointCloud2，并在 obstacle layer 侧过滤 `0.05m` 以下低矮点后叠加 inflation layer
- `/odom` 与 `/registered_scan` 输入订阅 QoS 深度由 `input_qos_depth` 控制，ExactTime 同步缓存由 `sync_queue_size` 控制，默认均为 20，用于吸收短时回调拥塞
- 当前自动导航链要求输入 `/odom.child_frame_id=base_footprint`
- 由于 `base_link` 现在保留 roll/pitch，`base_footprint -> livox_frame` 对导航链不再是纯静态量；模块已改为按每帧时间戳实时查询组合 TF

## 源码入口与阅读顺序
- 先看 `src/sensor_scan.cpp`，该包核心就是一个组件节点。
- 再回到 `README.md` 和 `src/rc26_bringup/config/sensor_scan_generation.yaml`，确认输入输出和部署参数。

## 目录解剖
- `src/sensor_scan.cpp`：时间同步、TF 查询、局部视角点云与 odom 再发布。
- `README.md`：模块定位说明。
- `src/rc26_bringup/config/sensor_scan_generation.yaml`：参数入口。

## 关键文件体量
- `src/sensor_scan.cpp`：238 行。
- `README.md`：26 行。

## 关键源码行段速览
- `src/rc26_sensor_scan/src/sensor_scan.cpp:66-139`：构造函数，创建 message_filters、发布器和 TF 监听。
- `src/rc26_sensor_scan/src/sensor_scan.cpp:141-198`：同步后的点云/里程计处理主路径。
- `src/rc26_sensor_scan/src/sensor_scan.cpp:200-215`：按时间戳查询组合 TF。
- `src/rc26_sensor_scan/src/sensor_scan.cpp:217-238`：整理后的 odom 发布与组件注册。

## 模块边界

- 它不是点云配准算法，不代替 `rc26_point_lio`
- 它不做地形语义分割，也不承担已删除旧地形链路的语义栅格职责
- 它的职责是“整理输入给别人用”，不是直接产出高层决策结果
- 它当前不负责生成 `/scan` LaserScan 兼容话题；导航主链默认直接把 `sensor_scan` (`PointCloud2`) 接入 Nav2 obstacle layer，具体障碍高度过滤由 `rc26_bringup/config/nav2_params.yaml` 维护
