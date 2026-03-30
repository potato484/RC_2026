# rc26_lio_state_predictor

## 模块定位

`rc26_lio_state_predictor` 用来解决 LIO 输出相对控制回路存在延迟的问题，把上游里程计前向预测到更接近“当前时刻”的状态。

## 当前实现

- 构建方式：组件库 + 独立可执行
- 导出节点：`rc26_lio_state_predictor_node`
- 核心源码：`src/rc26_lio_state_predictor/src/lio_state_predictor.cpp`

当前实现主要做三件事：

- 基于最近一帧里程计和 IMU 角速度做短时前向预测
- 以更高频率发布面向控制器的预测状态
- 按预测跨度膨胀协方差，并输出退化/降级相关语义

## 模块边界

- 它不生成原始里程计，原始来源仍是 `rc26_point_lio` 等上游
- 它不做全局重定位，只做控制侧的时间对齐和预测补偿
- 它输出的是控制可用状态，不直接做导航决策
