# rc26_visualization

## 模块定位

`rc26_visualization` 是 R2 的状态聚合与运维诊断包，负责把定位、控制、地形、机构、导航安全等多路状态压成统一的可视化语义输出。

## 当前实现

- 构建产物：
  - 库 `visualization_status_core`
  - 可执行文件 `rc26_visualization_status_node`
- 核心源码：
  - `src/visualization_status_core.cpp`
  - `src/visualization_status_node.cpp`
- 配置文件：`config/visualization_status.yaml`
- 已有测试：`test/test_visualization_status.cpp`

当前实现的关键能力包括：

- 聚合定位健康度、控制退化、导航安全、机构状态等输入
- 输出操作员可读的 `GREEN / YELLOW / ORANGE / RED` 等级语义
- 输出结构化事件列表，便于 Foxglove、RViz 或值守界面使用
- 通过 `rc26_bringup` 统一接入 RViz/Foxglove/none 三种可视化后端

## 模块边界

- 它不生产底层状态，只消费其他模块输出后做语义汇总
- 它不负责前端页面渲染，Foxglove 布局和前端页面都在别处
- 它的价值在于“汇总和诊断”，不是控制或决策本体
