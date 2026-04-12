# rc26_terrain

## 模块定位

`rc26_terrain` 是 R2 当前的地形感知与语义栅格生成包，用来把点云转换成 topo/xhu 自研导航链可消费的地形风险结果。

## 当前实现

- 导出节点:
  - `rc26_terrain_node`
  - `terrain_grid_map_bridge_node`
- 关键输出:
  - `terrain_features`
  - `/xhu_nav/semantic_layer_summary`
- 关键配置:
  - `config/terrain_semantic.yaml`
  - `config/terrain_filter_chain.yaml`
  - `config/terrain_risk_model.yaml`
  - `config/terrain_grid_map_bridge.yaml`

## 当前语义摘要口径

- `rc26_terrain_node` 当前会额外订阅 `/mf_block_overlay`，把 terrain obstacle/drop 风险和 keepout blocked/slow 摘要合并后发布到 `/xhu_nav/semantic_layer_summary`。
- 该摘要只暴露轻量级语义统计：
  - `obstacle_cells / drop_cells`
  - `blocked_cells / slow_cells`
  - `max_obstacle_probability / max_drop_probability`
  - `active_sources / active_reasons / revision`
- 这条摘要 topic 采用 transient-local QoS，主要服务 `rc26_xhu_nav` 内的 local planner / runtime 执行链快速状态判断，不替代原始 `terrain_features` 栅格真源。

## 当前边界

- 输出地形风险和 GridMap 语义层
- 允许输出执行链可消费的轻量级 semantic summary，但不直接输出 `cmd_vel` 级决策
- 不再发布 `terrain_speed_limit` 或任何面向执行器的限速建议话题
- 不再存在额外兼容桥接包
- 不直接做决策编排
