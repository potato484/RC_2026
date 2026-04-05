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

## 源码入口与阅读顺序
- 先看 `src/lio_state_predictor.cpp`，该包核心逻辑基本都在这一份组件实现里。
- 再看 `README.md` 和 `docs/debug_guide.md`，确认输入输出和调试关注点。

## 目录解剖
- `src/lio_state_predictor.cpp`：参数、订阅、预测发布定时器、协方差膨胀和退化状态输出。
- `README.md`：解释为什么要把 LIO 估计前推到控制当前时刻。
- `docs/debug_guide.md`：离线 bag 联调步骤。

## 关键文件体量
- `src/lio_state_predictor.cpp`：404 行，预测器实现集中。
- `README.md`：23 行，说明较短。

## 关键源码行段速览
- `src/rc26_lio_state_predictor/src/lio_state_predictor.cpp:99-194`：构造函数，声明参数、创建订阅/发布器与高频定时器。
- `src/rc26_lio_state_predictor/src/lio_state_predictor.cpp:195-221`：里程计、IMU、退化分数回调以及协方差膨胀辅助函数。
- `src/rc26_lio_state_predictor/src/lio_state_predictor.cpp:222-404`：`publishPredictedState()`，真正完成时间对齐预测和控制侧状态输出。

## 模块边界

- 它不生成原始里程计，原始来源仍是 `rc26_point_lio` 等上游
- 它不做全局重定位，只做控制侧的时间对齐和预测补偿
- 它输出的是控制可用状态，不直接做导航决策
