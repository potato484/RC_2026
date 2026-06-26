# rc26_interfaces

## 模块定位

`rc26_interfaces` 是 R2 仓库的跨包接口真源，只定义消息和服务，不包含业务逻辑。当前机构高层 action 已经下线，接口生成清单以 `src/rc26_interfaces/CMakeLists.txt` 为准。

## 当前导航相关契约

本包不再定义自定义导航 action。运行时导航 action 使用外部 Nav2 契约：

- `/navigate_to_pose`
- `nav2_msgs/action/NavigateToPose`

## 当前定位相关契约

定位主链已经收口为标准 ROS 消息与 TF：

- `map -> odom` 动态 TF
- `/localization/pose_with_cov` (`geometry_msgs/msg/PoseWithCovarianceStamped`)
- `/localization/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`)

本包不再生成定位健康度、后端状态、路线可观测性、关键帧、闭环、重定位状态或配准调试自定义消息。

## 当前视觉契约

- `TipDetection.msg`
- `TipDetectionArray.msg`
- `/vision/tip_detections`

这些接口仍是视觉端头检测的稳定契约。字段 `tip_index` 继续表示端头编号。

## 当前机构契约

当前机构执行只保留 raw transport：

- `SendMechanismTransportCommand.srv`
- `MechanismTransportFeedback.msg`

旧机构 action 已从 interface generation 中移除，不再作为当前接口生成或文档化：

- 旧抓取端头 action
- 旧武馆组装 action
- 旧通用机构执行 action
- 旧九宫格放置 action

`/mechanism/send_command` 与 `/mechanism/command_feedback` 的 provider 是 `rc26_mcu_transport`。`rc26_mechanism` 当前只是轻量 lifecycle 占位，不再暴露旧高层 action。

`MechanismActionHistory*.msg` 暂作归档兼容消息保留，当前主链不发布 `/mechanism/action_history`，也不把它列入中间层活跃契约。

## 归档 Keepout / Terrain 接口

下列接口仍可生成，用作历史兼容和后续恢复参考，但它们不属于当前主链运行时契约：

- `SetKeepoutRuntime.srv`
- `MfBlockOverlay.msg`
- `MfBlockOverlayCell.msg`
- `MfKfsState.msg`
- `MfKfsCell.msg`
- `TerrainFeatureGrid.msg`

当前 `rc26_bringup` 不启动 `rc26_kfs_keepout`、`rc26_terrain` 或 `rc26_base_ground`，`rc26_decision` 也不调用 `/kfs_keepout/set_runtime`、不发布 `/mf_kfs_state`，不订阅 terrain/base-ground/keepout 输出。

## 当前边界

- 接口是否存在以 `src/rc26_interfaces/CMakeLists.txt` 的 `rosidl_generate_interfaces()` 清单为准。
- 跨模块 ROS 契约同步维护在 `docs/middle/modules/*.yaml`。
- 新增或移除跨包接口时，必须同步更新本 README、对应包 README 和中间层模块契约。

## 本轮同步

2026-06-26 同步：移除旧机构 action 生成项和 `action_msgs` 依赖。机构侧当前只生成 raw transport message/service；旧高层动作能力已经从公开契约中删除。
