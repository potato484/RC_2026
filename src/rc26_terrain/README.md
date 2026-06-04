# rc26_terrain

`rc26_terrain` 是已归档的 R2 地形感知与语义栅格生成源码包。当前主运行时不再编译、启动或消费它；默认 CMake 只完成包配置，不生成节点、组件、库、测试或安装目标。

## 归档历史输出

- `/terrain_obstacles`
- `/terrain_drop`
- `/terrain_grid_map_local`
- `TerrainFeatureGrid` 相关语义总线

## 当前定位

- 不参与 `rc26_bringup`、`rc26_decision`、Nav2 的默认运行时链路。
- 默认不发布任何 ROS 数据；只有显式以 `RC26_ENABLE_ARCHIVED_RUNTIME_TARGETS=ON` 恢复本地调试构建后，历史节点才可能被手工启动。
- 当前主链没有任何模块订阅 terrain 输出；如未来恢复，必须先重新定义接口契约、启动入口、验证范围和文档边界。
- 历史源码、launch、config 和测试文件保留为恢复参考，不再描述为当前运行时契约。
