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

## 模块边界

- 它负责地形语义，不负责 Nav2 插件集成，Nav2 侧桥接在 `rc26_terrain_nav2`
- 它不做全局定位
- 它输出的是地形风险和限速依据，不直接做决策编排
