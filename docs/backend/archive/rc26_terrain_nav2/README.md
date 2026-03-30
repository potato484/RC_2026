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

## 源码入口与阅读顺序
- 先看 `rc26_terrain_nav2.xml`，确认它给 Nav2 暴露了哪一个 layer 插件。
- 再看 `src/terrain_traversability_layer.cpp`，这是 costmap layer 主体。
- 然后看 `src/terrain_speed_limit_bridge.cpp` 和对应测试。

## 目录解剖
- `terrain_traversability_layer.cpp`：把 terrain grid 映射成 Nav2 costmap layer。
- `terrain_speed_limit_bridge.cpp`：把地形限速话题桥到 Nav2 可消费接口，并带 watchdog。
- `terrain_speed_limit_bridge_node.cpp`：桥节点可执行入口。
- `test/`：桥接和 layer 映射测试。

## 关键文件体量
- `src/terrain_traversability_layer.cpp`：402 行。
- `src/terrain_speed_limit_bridge.cpp`：189 行。
- `test/test_terrain_speed_limit_bridge.cpp`：232 行。
- `test/test_terrain_traversability_layer_mapping.cpp`：27 行。

## 关键源码行段速览
- `src/rc26_terrain_nav2/src/terrain_traversability_layer.cpp:81-212`：插件初始化、订阅和 bounds 更新。
- `src/rc26_terrain_nav2/src/terrain_traversability_layer.cpp:213-396`：`updateCosts()`、layer 读取、reset 和 clearable 语义。
- `src/rc26_terrain_nav2/src/terrain_speed_limit_bridge.cpp:25-89`：构造、参数和 pub/sub。
- `src/rc26_terrain_nav2/src/terrain_speed_limit_bridge.cpp:90-176`：输入回调和 watchdog。
- `src/rc26_terrain_nav2/src/terrain_speed_limit_bridge.cpp:177-189`：限速发布。

## 模块边界

- 这个包不做地形语义理解，原始风险计算在 `rc26_terrain`
- 它也不做控制器求解，只是把地形结果正确接进 Nav2
- 它的职责是“桥接和插件化”，不是地形算法本体
