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

## 模块边界

- 它不做里程计估计算法，真正的估计来自 `rc26_point_lio` 或其他来源
- 它不做控制器求解，也不做地图匹配
- 它属于坐标与消息接口层，而不是完整感知算法层
