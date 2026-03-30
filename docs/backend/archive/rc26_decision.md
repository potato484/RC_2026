# rc26_decision

## 模块定位

`rc26_decision` 是 R2 自动机器人当前的主决策包，采用 BehaviorTree.CPP 实现比赛流程级决策。

## 当前实现

- 构建产物：
  - 共享库 `rc26_decision_nodes`
  - 可执行文件 `decision_node`
- 行为树文件：`behavior_trees/main_tree.xml`、`mc_tree.xml`、`mf_tree.xml`、`combat_tree.xml`
- 参数文件：`config/decision_params.yaml`
- 本地化配置：`config/bt_localization.yaml`

共享库当前拆成几个明确子域：

- `src/mc/mc_area.cpp`：武馆区相关行为节点
- `src/mf/mf_area.cpp`、`src/mf/merlin_rule_world_model.cpp`：梅林区流程和规则世界模型
- `src/combat/battle_grid_state.cpp`、`src/combat/combat_area.cpp`：对抗区状态与行为
- `src/navigation/waypoint_manager.cpp`、`smart_waypoint_navigator.cpp`、`bt_nav_to_smart_point.cpp`：智能航点、Nav2 目标下发、导航档位切换
- `src/vision/bt_nodes.cpp`：`VisionStart`、`VisionStop`、`VisionSetModel`、`WaitVisionTarget` 等 BT 节点

`decision_node` 当前除了装树和 tick 行为树，还实现了两条重要辅助链路：

- `src/bt/bt_runtime_publisher.cpp`
  - 发布行为树模型、本地化信息、运行时快照、黑板、Trace、事件流、调试状态
- `src/bt/chinese_localization_module.cpp`
  - 基于 `bt_localization.yaml` 产出中文节点名、黑板键说明和子树解释

另外，当前决策包已经直接和以下模块发生业务耦合：

- `rc26_interfaces`：行为树运行时消息、导航服务、机构状态等接口
- `rc26_vision`：通过 `VisionInferenceManager` 提供视觉黑板输入
- Nav2：通过 `SmartWaypointNavigator` 下发 `navigate_to_pose`
- `rc26_nav_mode_manager`：通过 `SetNavMode` 服务切档

## 模块边界

- 这个包负责流程编排和策略切换，不自己做底层控制求解
- 它不实现视觉推理本体，只消费 `rc26_vision` 的结果
- 它不代替机构控制，只通过接口驱动 `rc26_mechanism` 或相关执行链
- 前端行为树查看器读取的是这里的 XML 和运行时消息，但前端不属于本包
