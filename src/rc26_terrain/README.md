# rc26_terrain

`rc26_terrain` 是 R2 的地形感知与语义栅格生成模块，负责把点云转换成障碍、跌落、栅格语义和限速建议。

## 主要输出

- `/terrain_obstacles`
- `/terrain_drop`
- `/terrain_speed_limit`
- `/terrain_grid_map_local`
- `TerrainFeatureGrid` 相关语义总线

## 当前定位

- 作为 topo/xhu 自研导航链的前置感知输入
- 为执行器和可视化提供障碍、跌落和限速依据
- 不负责控制求解，也不承担任何旧兼容桥接职责
- 当前测试入口只保留 `test_tf_chain`、`test_safety_guard`、`test_terrain_grid_map_bridge*` 和 `test_terrain_risk_model`，不再注册已删除的旧集成测试脚本
