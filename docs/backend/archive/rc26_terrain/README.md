# rc26_terrain

## 模块定位

`rc26_terrain` 是 R2 当前的地形感知与语义栅格生成包，用来把点云转换成 topo/xhu 自研导航链可消费的地形风险结果。

## 当前实现

- 导出节点:
  - `rc26_terrain_node`
  - `terrain_grid_map_bridge_node`
- 关键配置:
  - `config/terrain_semantic.yaml`
  - `config/terrain_filter_chain.yaml`
  - `config/terrain_risk_model.yaml`
  - `config/terrain_grid_map_bridge.yaml`

## 当前边界

- 输出地形风险和 GridMap 语义层
- 不再发布 `terrain_speed_limit` 或任何面向执行器的限速建议话题
- 不再存在额外兼容桥接包
- 不直接做决策编排
