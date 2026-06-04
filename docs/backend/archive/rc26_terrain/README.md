# rc26_terrain

## 模块定位

`rc26_terrain` 是已归档的 R2 地形感知与语义栅格生成源码包。当前主运行时不再编译、启动或消费它；默认 CMake 只完成包配置，不生成节点、组件、库、测试或安装目标。

## 当前实现

- 归档源码保留的历史节点:
  - `rc26_terrain_node`
  - `terrain_grid_map_bridge_node`
- 历史输出:
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

- 不参与 `rc26_bringup`、`rc26_decision`、Nav2、smoke CI 的默认运行时链路
- 默认不发布任何 ROS 数据；只有显式以 `RC26_ENABLE_ARCHIVED_RUNTIME_TARGETS=ON` 恢复本地调试构建后，历史节点才可能被手工启动
- 当前主链没有任何模块订阅 terrain 输出；如未来恢复，必须先重新定义接口契约、启动入口、验证范围和文档边界

## 本轮收口

- 在 CMake 中加入默认关闭的归档构建开关
- 从 bringup、odometry、mapping 调试、RViz、验收探针、decision 和 CI 默认闭包中移除 terrain 链路
- 保留源码和配置作为历史参考，不再把历史输出描述为当前运行时契约
