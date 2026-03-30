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

## 源码入口与阅读顺序
- 先看 `config/visualization_status.yaml` 和 `README.md`，确认聚合目标和等级语义。
- 再看 `src/visualization_status_node.cpp`，这里负责把 ROS 2 输入映射成 core 所需状态。
- 然后看 `src/visualization_status_core.cpp`，理解真正的等级判定和事件生成。
- 最后看 `test/test_visualization_status.cpp`。

## 目录解剖
- `visualization_status_node.cpp`：参数、topic 订阅、输入整形和对 core 的调用。
- `visualization_status_core.cpp`：等级判定、事件生成、topic timeout 跟踪。
- `config/visualization_status.yaml`：聚合阈值和诊断配置。
- `test/test_visualization_status.cpp`：用例化验证聚合规则。

## 关键文件体量
- `src/visualization_status_core.cpp`：876 行。
- `src/visualization_status_node.cpp`：824 行。
- `test/test_visualization_status.cpp`：460 行。
- `README.md`：181 行。

## 关键源码行段速览
- `src/rc26_visualization/src/visualization_status_core.cpp:238-273`：core 构造、配置和 topic timeout tracker。
- `src/rc26_visualization/src/visualization_status_core.cpp:274-876`：`evaluate()`，完成等级判定、事件拼接和输出结构构建。
- `src/rc26_visualization/src/visualization_status_node.cpp:1-817`：节点侧参数声明、订阅、输入转换和 core 调用。
- `src/rc26_visualization/src/visualization_status_node.cpp:818-824`：`main()`。

## 模块边界

- 它不生产底层状态，只消费其他模块输出后做语义汇总
- 它不负责前端页面渲染，Foxglove 布局和前端页面都在别处
- 它的价值在于“汇总和诊断”，不是控制或决策本体
