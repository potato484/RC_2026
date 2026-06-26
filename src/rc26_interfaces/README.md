# rc26_interfaces

`rc26_interfaces` 是 R2 当前运行时的接口真源。这个包只定义跨包消息和服务，不包含任何业务逻辑；旧机构 action 已经下线。

## 当前接口范围

- 归档兼容接口:
  - `SetKeepoutRuntime.srv`
  - `MfBlockOverlay.msg`
  - `MfBlockOverlayCell.msg`
  - `MfKfsState.msg`
  - `MfKfsCell.msg`
  - `TerrainFeatureGrid.msg`
- 机构与任务:
  - `MechanismTransportFeedback.msg`
  - `MechanismActionHistory*.msg`
  - `SendMechanismTransportCommand.srv`
- 感知:
  - `TipDetection.msg`
  - `TipDetectionArray.msg`
  - `DynamicPrediction*.msg`

Nav2 的 `/navigate_to_pose` action 使用外部包 `nav2_msgs/action/NavigateToPose`，不在本包重复定义导航 action。

`rc26_decision` 当前仍以内置 BehaviorTree.CPP 组织流程，但不再通过本包发布任何第一方 BT 运行时调试或控制契约。

## 当前清理状态

旧导航 action、运动模式服务和导航状态消息已经从接口生成清单中移除。当前导航运行权威由 Nav2 提供；定位主链只使用标准 ROS 消息和 TF，不再由本包生成定位自定义消息。

BT 运行时调试与控制接口也已从接口生成清单中移除；行为树调试和控制面当前不属于公开 ROS 契约。

`SetKeepoutRuntime`、`MfBlockOverlay*`、`MfKfs*`、`TerrainFeatureGrid` 仍可生成，用作历史兼容和后续恢复参考；它们不再代表当前主链运行时契约。

随旧版第一方诊断 viewer 一起退役的诊断可视化消息也已从接口生成清单中移除：

- `OperatorStatus.msg`
- `VisualizationEvent.msg`
- `VisualizationEventArray.msg`

## 视觉端头契约

- `TipDetection.msg`、`TipDetectionArray.msg` 与 `/vision/tip_detections` 是当前视觉端头检测稳定契约。
- 字段 `tip_index` 继续表示端头编号；本轮只统一 `rc26_vision` 内部实验链命名，不改变这些 ROS 消息、topic 或 action 名。

## 机构契约

- 旧抓端头、旧组装、旧通用机构执行与旧九宫格放置 action 均不再生成。
- 当前机构执行只保留 raw transport：`SendMechanismTransportCommand.srv` 与 `MechanismTransportFeedback.msg`。
- `MechanismActionHistory*.msg` 暂作归档兼容消息保留，当前主链不发布 `/mechanism/action_history`。

## 归档 Keepout / Terrain 接口

- `SetKeepoutRuntime.srv`、`MfBlockOverlay.msg`、`MfBlockOverlayCell.msg`、`MfKfsState.msg`、`MfKfsCell.msg`、`TerrainFeatureGrid.msg` 仅作为归档兼容接口保留。
- 当前主链不提供 `/kfs_keepout/set_runtime`，不发布或订阅 `/mf_block_overlay`、`/kfs_filter_mask`、`/kfs_keepout_heartbeat`、`/mf_kfs_state` 或 terrain grid/feature 话题。
- 如未来恢复这些接口，必须先恢复对应包的归档构建开关、重新接入 bringup/decision，并同步更新 [docs/middle/modules/navigation.yaml](/home/potato/RC_2026/docs/middle/modules/navigation.yaml)。

## 维护原则

- 任何跨包字段语义变更，都必须同步更新 [docs/middle/modules/navigation.yaml](/home/potato/RC_2026/docs/middle/modules/navigation.yaml) 或对应模块契约文档。
- 判断“接口是否真实存在”时，以 [CMakeLists.txt](/home/potato/RC_2026/src/rc26_interfaces/CMakeLists.txt) 中 `rosidl_generate_interfaces()` 的清单为准。
