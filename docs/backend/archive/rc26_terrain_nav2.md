# rc26_terrain_nav2

## 模块定位

`rc26_terrain_nav2` 是 `rc26_terrain` 与 Nav2 之间的适配层，负责把地形结果接到 costmap 和速度限制链路中。

## 当前实现

这个包当前由两部分组成：

- Nav2 costmap 插件 `TerrainTraversabilityLayer`
- 速度限制桥节点 `terrain_speed_limit_bridge_node`

对应源码与构建产物如下：

- `src/terrain_traversability_layer.cpp`
  - 构建为共享库 `terrain_traversability_layer`
  - 通过 `rc26_terrain_nav2.xml` 导出为 `nav2_costmap_2d::Layer` 插件
- `src/terrain_speed_limit_bridge.cpp`
  - 构建为共享库 `terrain_speed_limit_bridge_lib`
- `src/terrain_speed_limit_bridge_node.cpp`
  - 构建为可执行文件 `terrain_speed_limit_bridge_node`

当前实现能力包括：

- 订阅地形 GridMap 并按 traversability、drop、rule legality、keepout 等层写入 master costmap
- 把 `Float32` 类型的地形限速结果转换成 `nav2_msgs/msg/SpeedLimit`
- 处理限速消息的重发、超时和 stale policy

## 模块边界

- 这个包不做地形语义理解，原始风险计算在 `rc26_terrain`
- 它也不做控制器求解，只是把地形结果正确接进 Nav2
- 它的职责是“桥接和插件化”，不是地形算法本体
