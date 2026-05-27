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
  - `src/mf/keepout_runtime.cpp`
  - `src/bt/bt_runtime_publisher.cpp`

## 当前导航调用口径

- 只保留 topo/xhu 自研导航节点
- MF 导航通过 `NavigateTopoTarget` action 对接 `rc26_xhu_nav`
- `rc26_decision` 仍然拥有 MF 目标格选择；`NavToTaskPose(grid_id)` 只负责把已选格映射成 topo/xhu 导航目标
- `main_tree.xml` 已作为唯一主树入口
- `MFAreaTree` 当前被自定义 Decorator `WithKeepoutRuntime` 包裹；进入 MF 子树前会调用 `/kfs_keepout/set_runtime activate=true`，离开或 halt 时会调用 `activate=false`
- 当前导航 BT 节点已经按 action feedback/result 口径消费 `rc26_xhu_nav`，不再依赖局部规划器内部状态猜测执行进度
- topo action feedback 当前会持续回写以下黑板键，供 Groot2 / 诊断观察：
  - `nav_last_exec_state`
  - `nav_last_active_node_id`
  - `nav_last_active_edge_id`
  - `nav_last_replan_count`
- topo action result 当前会回写：
  - `nav_last_failure_code`
  - `nav_last_failure_reason`
  - 并把 topo failure code 映射成稳定的 BT `error_code`
- `bt_runtime_publisher` 当前已经显式放行上述导航观测键，避免行为树可视化侧看不到真实执行上下文

## 当前边界

- 负责流程编排和策略切换
- 不直接做底层控制求解
- 通过统一 topo 目标协议驱动导航执行链
- MF keepout 的装载时机由决策显式控制，但 keepout 本体仍属于 `rc26_kfs_keepout`，本包不反向拥有其融合算法

## 近期实现说明

- 当前 `BtActionNode` 已支持 action `feedback_callback`，导航节点可以在 action 运行期间持续刷新黑板状态。
- `bt_topo_nav.cpp` 当前会在 goal 开始、反馈推进、成功结束、失败结束四个阶段统一维护导航观测键，避免现场只在失败时才看到零散信息。
- `WithKeepoutRuntime` 当前把 keepout 运行时错误分成两层：
  - 进入 MF 激活失败：Decorator 直接返回 `FAILURE`，阻止进入梅林区流程
  - 退出 MF 清空失败：Decorator 返回 `FAILURE` 并回写 `merlin_last_transition_reason`
  - 退出 MF 仅卸载失败：Decorator 透传子树原结果，只记录告警
- 机构侧当前只再向决策黑板回写 `mechanism_hal_open`、`mechanism_current_cmd_id` 与 `last_action_error_code` 三个最小观测键；旧的 `mechanism_tip_state / mechanism_comm_health_level / mechanism_locked_tip_slot` 已随 `rc26_mechanism` 减法移除。
- 对抗区 `PlaceKFSGrid` 节点当前不再依赖专用 `PlaceKFSGrid.action`，而是继续保留 BT 节点名，并在内部改为通过 `/mechanism/run_command` 下发 `PLACE_KFS_GRID + payload`。

## 配置注释口径

- `config/decision_params.yaml` 与 `config/bt_localization.yaml` 已保留常用/高影响字段的中文注释，分别重点说明行为树运行参数、服务/topic 和定位守护 profile；`bt_localization.yaml` 的中文解释表不再逐项机械注释；本次只改变注释，不改变决策流程或黑板契约。
