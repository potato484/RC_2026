# rc26_interfaces

## 模块定位

`rc26_interfaces` 是整个 R2 仓库的自定义 ROS 2 接口契约包，统一承载消息、服务和动作定义。

## 当前实现

当前接口大致分成几类：

- 行为树运行时接口
  - `BehaviorTreeModel.msg`
  - `BehaviorTreeSnapshot.msg`
  - `BehaviorTreeTrace.msg`
  - `BehaviorTreeEvent*.msg`
  - `BehaviorTreeBlackboard*.msg`
  - `BehaviorTreeLocalization*.msg`
- 定位相关接口
  - `LocalizationHealth.msg`
  - `LocalizationBackendStatus.msg`
  - `RouteObservability.msg`
- 机构与执行接口
  - `MechanismState.msg`
  - `ExecuteMechanism.action`
  - `GrabTip.action`
  - `AssembleWeapon.action`
  - `PlaceKFSGrid.action`
- 导航安全接口
  - `NavSafetyState.msg`
  - `NavTolerance.msg`
  - `SmartWaypoint.msg`
  - `SetNavMode.srv`
- 感知与规则接口
  - `MfKfsState.msg`
  - `MfKfsCell.msg`
  - `TerrainFeatureGrid.msg`
  - `TipDetection.msg`
  - `TipDetectionArray.msg`
- 可视化接口
  - `VisualizationEvent.msg`
  - `VisualizationEventArray.msg`

服务接口当前包括：

- `ControlBehaviorTree.srv`
- `SetNavMode.srv`

## 模块边界

- 这个包只定义接口格式，不包含业务逻辑
- 它不启动节点，也不做算法计算
- 所有上层文档和前端若要描述跨包数据结构，原则上都应以这里的定义为准
