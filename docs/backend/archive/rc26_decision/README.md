# rc26_decision

## 模块定位

`rc26_decision` 是 R2 的主决策包，采用 BehaviorTree.CPP 组织比赛流程。

## 当前实现

- 构建产物:
  - `rc26_decision_nodes`
  - `decision_node`
- 关键行为树:
  - `behavior_trees/main_tree.xml`
  - `behavior_trees/mf_tree.xml`
  - `behavior_trees/mf_tree_topo.xml`
  - `behavior_trees/mc_tree.xml`
  - `behavior_trees/combat_tree.xml`
- 关键源码:
  - `src/decision_node.cpp`
  - `src/navigation/bt_topo_nav.cpp`
  - `src/mf/mf_area.cpp`
  - `src/bt/bt_runtime_publisher.cpp`

## 当前导航调用口径

- 只保留 topo/xhu 自研导航节点
- MF 导航通过 `NavigateTopoTarget` action 对接 `rc26_topo_nav`
- `main_tree.xml` 已作为唯一主树入口

## 当前清理状态

- 旧 waypoint 导航桥接实现已移除
- 独立 waypoint 配置已移除
- 旧分模式主树已移除

## 当前边界

- 负责流程编排和策略切换
- 不直接做底层控制求解
- 不再直接持有底层导航目标协议或独立 waypoint 文件
