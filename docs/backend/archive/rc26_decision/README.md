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

## 源码入口与阅读顺序
- 先看 `behavior_trees/main_tree.xml` 以及各区域 XML，理解“树上声明了什么阶段顺序”。
- 再看 `src/decision_node.cpp`，这里负责把 XML、BT factory、服务和运行时发布器装起来。
- 然后看 `src/bt/bt_runtime_publisher.cpp` 和 `src/bt/chinese_localization_module.cpp`，这两块决定前端和操作员能看到哪些运行时语义。
- 最后按域回到 `src/mf/`、`src/mc/`、`src/combat/`、`src/navigation/`、`src/vision/` 看具体 BT 节点实现。

## 目录解剖
- `behavior_trees/*.xml`：比赛流程的静态编排真源。
- `src/decision_node.cpp`：节点启动入口、行为树装配、主循环与服务接线。
- `src/bt/`：运行时快照、黑板、Trace、本地化描述输出。
- `src/mf/`、`src/mc/`、`src/combat/`：各区域业务节点和规则模型。
- `src/navigation/`：智能航点和 Nav2 行为节点桥接。
- `src/vision/`：视觉相关 BT 节点。

## 关键文件体量
- `src/decision_node.cpp`：662 行，启动与主循环装配入口。
- `src/bt/bt_runtime_publisher.cpp`：1006 行，运行时导出是一个完整子系统。
- `src/mf/mf_area.cpp`：1150 行，梅林区节点最多。
- `src/navigation/smart_waypoint_navigator.cpp`：821 行，导航桥接逻辑很重。
- `src/bt/chinese_localization_module.cpp`：520 行，中文解释层并不薄。
- `behavior_trees/mf_tree.xml`：120 行，当前前端默认读取的区域树。

## 关键源码行段速览
- `src/rc26_decision/behavior_trees/mf_tree.xml:1-12`：梅林区主树，只做初始化和三阶段子树串联。
- `src/rc26_decision/behavior_trees/mf_tree.xml:15-45`：`MF_Entry`，进门、让路和入场。
- `src/rc26_decision/behavior_trees/mf_tree.xml:47-86`：`MF_Loop`，扫描、选格、分支执行。
- `src/rc26_decision/behavior_trees/mf_tree.xml:88-109`：抓取子树和移动子树。
- `src/rc26_decision/behavior_trees/mf_tree.xml:111-120`：`MF_Exit`，出门和下台阶。
- `src/rc26_decision/src/decision_node.cpp:1-655`：几乎整文件都在做节点启动、工厂注册、XML 装载、服务和 tick 循环装配；`656-662` 只是 `main()` 外壳。
- `src/rc26_decision/src/bt/bt_runtime_publisher.cpp:70-229`：发布器构造和初始缓存；`230-564`：tick 生命周期、模型/快照/黑板/本地化输出；`565-1006`：Trace callback、节点摘要和格式化。
- `src/rc26_decision/src/bt/chinese_localization_module.cpp:37-181`：模块初始化与消息拼装；`182-334`：YAML 载入、解析和 merge；`420-512`：运行时查找与消息转换。
- `src/rc26_decision/src/navigation/smart_waypoint_navigator.cpp:36-221`：导航器构造、profile 和 keepout gate 配置；`257-379`：启动、停稳检查、模式请求；`532-767`：Nav2 goal 下发、tick 与取消。
- `src/rc26_decision/src/mf/mf_area.cpp:192-326`：梅林地图模型；`327-1120`：梅林区 BT Action/Condition；`1131-1150`：节点注册。
- `src/rc26_decision/src/vision/bt_nodes.cpp:89-198`：`VisionStart/Stop/SetModel`；`199-334`：`WaitVisionTarget` 与节点注册。

## 模块边界

- 这个包负责流程编排和策略切换，不自己做底层控制求解
- 它不实现视觉推理本体，只消费 `rc26_vision` 的结果
- 它不代替机构控制，只通过接口驱动 `rc26_mechanism` 或相关执行链
- 前端行为树查看器读取的是这里的 XML 和运行时消息，但前端不属于本包
