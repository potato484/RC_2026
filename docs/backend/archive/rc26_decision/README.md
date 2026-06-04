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
  - `behavior_trees/mc_tree.xml`
  - `behavior_trees/combat_tree.xml`
- 关键源码:
  - `src/decision_node.cpp`
  - `src/navigation/bt_nav2_pose.cpp`
  - `src/mf/mf_area.cpp`
  - `src/mf/keepout_runtime.cpp`
  - `src/bt/bt_runtime_publisher.cpp`

## 当前导航调用口径

- 梅林区导航统一使用 `NavToPose` BT 节点
- `NavToPose` 调用 Nav2 `/navigate_to_pose`，action 类型为 `nav2_msgs/action/NavigateToPose`
- BT XML 中显式写入 `frame_id / x / y / yaw / behavior_tree / timeout_sec`，不再通过动态字符串拼接目标
- `SelectNextGrid` 仍负责写入 `target_grid`；动态格位导航通过显式分支选择对应 `NavToPose`
- `current_grid:=target_grid` 等脚本在对应 pose 成功后继续保持原有语义
- `WithKeepoutRuntime` 仍包裹 `MFAreaTree`，进入 MF 前调用 `/kfs_keepout/set_runtime activate=true`，离开或 halt 时调用 `activate=false`

`NavToPose` 会维护以下黑板观测键：

- `nav_last_exec_state`: `PENDING | RUNNING | SUCCEEDED | FAILED`
- `nav_last_failure_code`
- `nav_last_failure_reason`
- `nav_last_distance_remaining`
- `nav_last_recovery_count`

Nav2 action result 映射规则：

- `SUCCEEDED` -> BT `SUCCESS`
- `ABORTED` -> BT `FAILURE`，`error_code=120`
- `CANCELED` -> BT `FAILURE`，`error_code=121`
- action server missing、invalid goal、timeout 继续沿用 `BtActionNode` 的通用错误码

## 当前边界

- 负责流程编排、目标选择、keepout 启停和策略切换
- 不直接做底层控制求解
- 不拥有 Nav2 planner/controller 的内部配置
- MF keepout 的装载时机由决策显式控制，但 keepout 本体仍属于 `rc26_kfs_keepout`

## 本轮收口

- 删除旧导航 BT 节点源码，新增 `bt_nav2_pose.cpp/.hpp`
- `rc26_decision` 增加 `nav2_msgs` 依赖
- `main_tree.xml` 改为 include `mf_tree.xml`
- `mf_tree.xml` 中梅林区目标点固化为 Nav2 pose，并为 `target_grid` 建立显式分支
- 行为树运行时发布白名单加入新的 `nav_last_*` 观测键
