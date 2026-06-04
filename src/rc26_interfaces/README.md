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
  - `ControlBehaviorTree.srv`
- 定位:
  - `LocalizationHealth.msg`
  - `LocalizationBackendStatus.msg`
  - `RouteObservability.msg`
  - `LocalizationKeyframe.msg`
  - `LocalizationLoopClosure.msg`
  - `LocalizationRelocState.msg`
  - `RegistrationDebug.msg`
- Nav2 / MF keepout 支撑:
  - `SetKeepoutRuntime.srv`
  - `MfBlockOverlay.msg`
  - `MfBlockOverlayCell.msg`
  - `MfKfsState.msg`
  - `MfKfsCell.msg`
  - `TerrainFeatureGrid.msg`
- 机构与任务:
  - `MechanismState.msg`
  - `MechanismTransportFeedback.msg`
  - `MechanismActionHistory*.msg`
  - `ExecuteMechanism.action`
  - `GrabTip.action`
  - `AssembleWeapon.action`
  - `SendMechanismTransportCommand.srv`
- 感知:
  - `TipDetection.msg`
  - `TipDetectionArray.msg`
  - `DynamicPrediction*.msg`

Nav2 的 `/navigate_to_pose` action 使用外部包 `nav2_msgs/action/NavigateToPose`，不在本包重复定义导航 action。

## 当前清理状态

旧导航 action、运动模式服务和导航状态消息已经从接口生成清单中移除。当前导航运行权威由 Nav2 提供，本包只保留决策、keepout、地形、定位、机构和行为树运行时仍真实使用的自定义契约。

随旧版第一方诊断 viewer 一起退役的诊断可视化消息也已从接口生成清单中移除：

- `OperatorStatus.msg`
- `VisualizationEvent.msg`
- `VisualizationEventArray.msg`

## 视觉端头契约

- `TipDetection.msg`、`TipDetectionArray.msg` 与 `/vision/tip_detections` 是当前视觉端头检测稳定契约。
- 字段 `tip_index` 继续表示端头编号；本轮只统一 `rc26_vision` 内部实验链命名，不改变这些 ROS 消息、topic 或 action 名。

## 机构契约

- 保留专用动作 `GrabTip.action` 与 `AssembleWeapon.action`。
- 保留通用动作 `ExecuteMechanism.action`。
- 删除 `PlaceKFSGrid.action`；KFS 放置改为通过 `ExecuteMechanism` 下发 `PLACE_KFS_GRID + payload`。
- `MechanismState.msg` 只保留 `hal_open`、`last_error_code`、`current_cmd_id` 三个最小观测字段。

## Keepout Runtime 契约

- `SetKeepoutRuntime.srv` 是 MF 阶段 keepout 运行时控制契约，当前固定服务名为 `/kfs_keepout/set_runtime`，由 `rc26_decision` 调用 `rc26_kfs_keepout` 运行时管理器。
- `activate=true` 表示进入 `MFAreaTree` 前请求装载并激活 keepout；`activate=false` 表示退出 MF 前请求先清空输出再卸载。
- 响应字段 `outputs_cleared` 表示 `/mf_block_overlay` 与 `/kfs_filter_mask` 是否已经被安全清空；后续流程只依赖这个字段判断是否会残留陈旧 keepout。
- 响应字段 `component_loaded` 表示 keepout 组件当前是否仍留在容器内；它允许在 `outputs_cleared=true` 时仍为 `true`，用于表达“已安全退出但卸载失败”的资源告警状态。

## 维护原则

- 任何跨包字段语义变更，都必须同步更新 [docs/middle/modules/navigation.yaml](/home/potato/RC_2026/docs/middle/modules/navigation.yaml) 或对应模块契约文档。
- 判断“接口是否真实存在”时，以 [CMakeLists.txt](/home/potato/RC_2026/src/rc26_interfaces/CMakeLists.txt) 中 `rosidl_generate_interfaces()` 的清单为准。
