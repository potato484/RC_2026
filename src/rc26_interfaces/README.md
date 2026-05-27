# rc26_interfaces

`rc26_interfaces` 是 R2 当前运行时的接口真源。这个包只定义跨包消息、服务和动作，不包含任何业务逻辑。

## 当前接口范围

- 行为树运行时:
  - `BehaviorTreeModel.msg`
  - `BehaviorTreeSnapshot.msg`
  - `BehaviorTreeTrace.msg`
  - `BehaviorTreeEvent*.msg`
  - `BehaviorTreeBlackboard*.msg`
  - `BehaviorTreeLocalization*.msg`
- 定位:
  - `LocalizationHealth.msg`
  - `LocalizationBackendStatus.msg`
  - `RouteObservability.msg`
- 自研导航:
  - `NavigateTopoTarget.action`
  - `NavigateSurfaceRoute.action`
  - `SetXhuMotionMode.srv`
  - `MfBlockOverlay.msg`
  - `MfBlockOverlayCell.msg`
  - `XhuSemanticCorridor.msg`
  - `XhuMotionModeState.msg`
  - `XhuTrackingState.msg`
- 机构与任务:
  - `MechanismState.msg`
  - `MechanismTransportFeedback.msg`
  - `ExecuteMechanism.action`
  - `GrabTip.action`
  - `AssembleWeapon.action`
  - `SendMechanismTransportCommand.srv`

当前机构契约已经收口为最小执行面：

- 保留专用动作 `GrabTip.action` 与 `AssembleWeapon.action`
- 保留通用动作 `ExecuteMechanism.action`
- 删除 `PlaceKFSGrid.action`；KFS 放置改为通过 `ExecuteMechanism` 下发 `PLACE_KFS_GRID + payload`
- `MechanismState.msg` 只保留 `hal_open`、`last_error_code`、`current_cmd_id` 三个最小观测字段
- 感知与规则:
  - `MfKfsState.msg`
  - `MfKfsCell.msg`
  - `TerrainFeatureGrid.msg`
  - `TipDetection.msg`
  - `TipDetectionArray.msg`

## 视觉端头契约

- `TipDetection.msg`、`TipDetectionArray.msg` 与 `/vision/tip_detections` 是当前视觉端头检测稳定契约。
- 字段 `tip_index` 继续表示端头编号；本轮只统一 `rc26_vision` 内部实验链命名，不改变这些 ROS 消息、topic 或 action 名。

## 当前清理状态

旧兼容导航契约已从接口清单中移除，当前只保留 topo/xhu 主链与决策运行时实际使用的消息、服务和动作。

随 `src/rc26_xhu_viewer` 一起退役的诊断可视化消息也已从接口生成清单中移除：

- `OperatorStatus.msg`
- `VisualizationEvent.msg`
- `VisualizationEventArray.msg`

## 维护原则

- 任何跨包字段语义变更，都必须同步更新 [docs/middle/modules/navigation.yaml](/home/potato/RC_2026/docs/middle/modules/navigation.yaml) 或对应模块契约文档。
- 新接口优先围绕 `rc26_xhu_nav` 这条自研链设计，不再为历史兼容链增加冗余字段。
- 判断“接口是否真实存在”时，以 [CMakeLists.txt](/home/potato/RC_2026/src/rc26_interfaces/CMakeLists.txt) 中 `rosidl_generate_interfaces()` 的清单为准。
