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
  - `PlaceKFSGrid.action`
  - `SendMechanismTransportCommand.srv`
- 感知与规则:
  - `MfKfsState.msg`
  - `MfKfsCell.msg`
  - `TerrainFeatureGrid.msg`
  - `TipDetection.msg`
  - `TipDetectionArray.msg`
- 可视化与诊断:
  - `OperatorStatus.msg`
  - `VisualizationEvent.msg`
  - `VisualizationEventArray.msg`

## 当前清理状态

旧兼容导航契约已从接口清单中移除，当前只保留 topo/xhu 主链与决策运行时实际使用的消息、服务和动作。

## 维护原则

- 任何跨包字段语义变更，都必须同步更新 [docs/middle/modules/navigation.yaml](/home/potato/RC_2026/docs/middle/modules/navigation.yaml) 或对应模块契约文档。
- 新接口优先围绕 `rc26_xhu_nav` 这条自研链设计，不再为历史兼容链增加冗余字段。
- 判断“接口是否真实存在”时，以 [CMakeLists.txt](/home/potato/RC_2026/src/rc26_interfaces/CMakeLists.txt) 中 `rosidl_generate_interfaces()` 的清单为准。
