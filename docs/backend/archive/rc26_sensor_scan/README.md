# rc26_sensor_scan

## 模块定位

`rc26_sensor_scan` 是点云与里程计的时空对齐模块，用来给下游局部感知提供“已经同步并投影到传感器视角”的干净输入。

## 当前实现

- 构建方式：组件库 + 可执行节点
- 导出节点：`rc26_sensor_scan_node`
- 核心源码：`src/rc26_sensor_scan/src/sensor_scan.cpp`

当前实现聚焦于：

- 对点云帧和里程计帧做时间同步
- 根据 TF 做逆向投影，把全局/底盘视角点云转换回传感器局部视角
- 同步分发局部视角点云和对应姿态
- 减少重复计算，尽量透传上游已存在的状态量

## 源码入口与阅读顺序
- 先看 `src/sensor_scan.cpp`，该包核心就是一个组件节点。
- 再回到 `README.md` 和 `src/rc26_bringup/config/sensor_scan_generation.yaml`，确认输入输出和部署参数。

## 目录解剖
- `src/sensor_scan.cpp`：时间同步、TF 查询、局部视角点云与 odom 再发布。
- `README.md`：模块定位说明。
- `src/rc26_bringup/config/sensor_scan_generation.yaml`：参数入口。

## 关键文件体量
- `src/sensor_scan.cpp`：242 行。
- `README.md`：23 行。

## 关键源码行段速览
- `src/rc26_sensor_scan/src/sensor_scan.cpp:66-141`：构造函数，创建 message_filters、发布器和 TF 监听。
- `src/rc26_sensor_scan/src/sensor_scan.cpp:142-205`：同步后的点云/里程计处理主路径。
- `src/rc26_sensor_scan/src/sensor_scan.cpp:206-220`：静态 TF 查询。
- `src/rc26_sensor_scan/src/sensor_scan.cpp:221-242`：整理后的 odom 发布。

## 模块边界

- 它不是点云配准算法，不代替 `rc26_point_lio`
- 它不做地形语义分割，不代替 `rc26_terrain`
- 它的职责是“整理输入给别人用”，不是直接产出高层决策结果
