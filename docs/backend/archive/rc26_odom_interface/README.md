# rc26_odom_interface

## 模块定位

`rc26_odom_interface` 是 Point-LIO 与 Nav2/底盘坐标系之间的标准化接口层，负责把上游里程计结果转换成下游统一消费的底盘里程计和 TF。

## 当前实现

- 构建方式：组件库 + 可执行节点
- 导出节点：`rc26_odom_interface_node`
- 核心源码：`src/rc26_odom_interface/src/odom_interface.cpp`

当前实现主要承担：

- 将上游传感器坐标系结果映射到底盘坐标系
- 发布系统中权威的底盘动态 TF
- 规范化输出 Nav2 需要的里程计消息
- 在必要时提供速度解算回退路径

## 源码入口与阅读顺序
- 先看 `src/odom_interface.cpp`，该包几乎就是一个大组件节点。
- 再看 `README.md` 和 bringup 里的 `config/odom_interface.yaml`，确认 TF 权威和参数输入。

## 目录解剖
- `src/odom_interface.cpp`：点云时间戳缓存、底盘/雷达坐标对齐、TF 和 odom 输出。
- `README.md`：解释为什么它是 TF 权威边界。
- `src/rc26_bringup/config/odom_interface.yaml`：部署参数。

## 关键文件体量
- `src/odom_interface.cpp`：752 行，核心逻辑集中。
- `README.md`：23 行。

## 关键源码行段速览
- `src/rc26_odom_interface/src/odom_interface.cpp:147-258`：构造函数，参数、订阅、发布器和缓存初始化。
- `src/rc26_odom_interface/src/odom_interface.cpp:259-318`：时间戳历史缓存和查找。
- `src/rc26_odom_interface/src/odom_interface.cpp:319-435`：点云回调和已配准点云发布。
- `src/rc26_odom_interface/src/odom_interface.cpp:436-752`：里程计回调，完成坐标映射、权威 TF 和规范化 odom 输出。

## 模块边界

- 它不做里程计估计算法，真正的估计来自 `rc26_point_lio` 或其他来源
- 它不做控制器求解，也不做地图匹配
- 它属于坐标与消息接口层，而不是完整感知算法层
