# rc26_terrain

## 模块定位

`rc26_terrain` 是 R2 当前的地形感知与语义栅格生成包，用来把点云转换成导航可消费的地形风险结果。

## 当前实现

- 构建方式：组件库 + 两个可执行节点
- 导出节点：
  - `rc26_terrain_node`
  - `terrain_grid_map_bridge_node`
- 关键配置：
  - `config/terrain_semantic.yaml`
  - `config/terrain_filter_chain.yaml`
  - `config/terrain_risk_model.yaml`
  - `config/terrain_grid_map_bridge.yaml`
- 启动文件：`launch/terrain_semantic.launch.py`

内部实现已经拆成多个子模块：

- `terrain_semantic_node.cpp`
  - 地形语义主节点
- `point_cloud_preprocessor.cpp`
  - 点云预处理
- `terrain_risk_model.cpp`
  - 风险建模
- `terrain_grid_map_bridge.cpp`
  - GridMap 桥接输出
- `safety_guard.cpp`
  - 安全守护逻辑
- `tf_chain_validator.cpp`
  - TF 链校验

此外还带有训练/数据集辅助脚本：

- `scripts/export_terrain_training_dataset.py`
- `scripts/fit_terrain_logistic.py`

测试也比较完整，已经覆盖：

- 风险模型
- GridMap 桥
- 安全守护
- TF 链
- Costmap 集成

## 源码入口与阅读顺序
- 先看 `launch/terrain_semantic.launch.py` 和 `README.md`，理解这个包如何单独运行。
- 再看 `src/terrain_semantic_node.cpp`，这是地形主节点。
- 然后看 `safety_guard.cpp`、`terrain_risk_model.cpp`、`tf_chain_validator.cpp`、`terrain_grid_map_bridge.cpp`，按“判风险 -> 守安全 -> 对外桥接”顺序回读。
- 最后看 `config/*.yaml` 和 `test/`，这里的测试覆盖是当前文档的重要依据。

## 目录解剖
- `terrain_semantic_node.cpp`：主节点，做点云接入、网格更新、地形分类、虚拟围栏、诊断和速度限制输出。
- `terrain_risk_model.cpp`：风险模型。
- `safety_guard.cpp`：安全守护策略与 fail-safe 判决。
- `tf_chain_validator.cpp`：TF 链完整性检查。
- `terrain_grid_map_bridge.cpp`：把地形结果和 keepout/KFS 状态转换给 GridMap/Nav2。
- `test/`：桥接、risk model、TF、安全守护、costmap 集成测试。

## 关键文件体量
- `src/terrain_semantic_node.cpp`：1978 行，是整个包的主战场。
- `src/terrain_grid_map_bridge.cpp`：1808 行。
- `test/test_terrain_grid_map_bridge.cpp`：832 行。
- `test/test_costmap_integration.py`：694 行。
- `src/safety_guard.cpp`：367 行。

## 关键源码行段速览
- `src/rc26_terrain/src/terrain_semantic_node.cpp:67-594`：构造函数和全部参数/pub/sub/缓存初始化。
- `src/rc26_terrain/src/terrain_semantic_node.cpp:595-877`：odom、安全守护、TF 验证、诊断和速度限制辅助。
- `src/rc26_terrain/src/terrain_semantic_node.cpp:878-1387`：网格初始化、梅林布局加载、KFS 占用更新、高度估计和分类。
- `src/rc26_terrain/src/terrain_semantic_node.cpp:1388-1978`：输出发布、点云回调、紧急停车和点云合法性检查。
- `src/rc26_terrain/src/terrain_grid_map_bridge.cpp:206-533`：桥节点构造和 keepout/KFS/feature 输入接线。
- `src/rc26_terrain/src/terrain_grid_map_bridge.cpp:534-1798`：GridMap 层映射、布局解析、采样和 diagnostics。
- `src/rc26_terrain/src/safety_guard.cpp:87-226`：安全守护决策主逻辑。

## 模块边界

- 它负责地形语义，不负责 Nav2 插件集成，Nav2 侧桥接在 `rc26_terrain_nav2`
- 它不做全局定位
- 它输出的是地形风险和限速依据，不直接做决策编排
