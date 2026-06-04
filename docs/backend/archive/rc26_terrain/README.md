# rc26_terrain

## 模块定位

`rc26_terrain` 是 R2 当前的地形感知与语义栅格生成包，用来把点云转换成障碍、跌落和 GridMap 语义结果。

## 当前实现

- 导出节点:
  - `rc26_terrain_node`
  - `terrain_grid_map_bridge_node`
- 关键输出:
  - `terrain_features`
  - `/terrain_obstacles`
  - `/terrain_drop`
  - `/terrain_grid_map`
- 关键配置:
  - `config/terrain_semantic.yaml`
  - `config/terrain_filter_chain.yaml`
  - `config/terrain_risk_model.yaml`
  - `config/terrain_grid_map_bridge.yaml`

## 当前边界

- 输出地形风险和 GridMap 语义层
- 不直接输出速度命令或控制建议
- 本轮不再发布旧导航语义摘要
- 本轮不把 terrain 输出接入 Nav2 costmap filter 或 obstacle layer
- 不直接做决策编排

## 本轮收口

- 移除旧导航语义摘要 publisher、参数和成员变量
- `terrain_features` 与 GridMap 输出保持不变
- `/mf_block_overlay` 仍可由 keepout/decision 侧独立观察，但 terrain 不再合并它生成旧执行链摘要
