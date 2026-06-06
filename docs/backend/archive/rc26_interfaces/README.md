# rc26_interfaces

## 模块定位

`rc26_interfaces` 是整个 R2 仓库的跨包接口真源，只定义消息、服务和动作，不包含业务逻辑。

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

`rc26_decision` 的行为树仍作为包内实现存在，但本包当前不再生成任何第一方 BT 运行时消息或控制接口；行为树调试面不再属于公开 ROS 契约。

本包仍生成下列归档兼容接口，但它们不属于当前主链运行时契约：

- `SetKeepoutRuntime.srv`
- `MfBlockOverlay.msg`
- `MfBlockOverlayCell.msg`
- `MfKfsState.msg`
- `MfKfsCell.msg`
- `TerrainFeatureGrid.msg`

## 当前视觉与机构端头契约

- `TipDetection.msg`、`TipDetectionArray.msg` 与 `/vision/tip_detections` 是视觉端头检测稳定契约。
- `GrabTip.action` 与 `/mechanism/grab_tip` 继续作为机构抓取端头的动作契约。
- `AssembleWeapon.action` 与 `/mechanism/assemble_weapon` 继续作为武馆组装动作契约。
- `ExecuteMechanism.action` 继续作为机构通用命令入口；当前调用路径已经收口到 action `/mechanism/run_command`。
- `PlaceKFSGrid.action` 已移除；若需要放置 KFS，调用侧应改为通过 `ExecuteMechanism` 下发 `PLACE_KFS_GRID + payload{grid_position, layer}`。
- `MechanismState.msg` 当前只保留 `hal_open`、`last_error_code`、`current_cmd_id` 三个最小运行时观测字段。

## 归档 Keepout / Terrain 接口

- `SetKeepoutRuntime.srv`、`MfBlockOverlay.msg`、`MfBlockOverlayCell.msg`、`MfKfsState.msg`、`MfKfsCell.msg`、`TerrainFeatureGrid.msg` 仅作为历史兼容和后续恢复参考保留。
- 当前 `rc26_bringup` 不启动 `rc26_kfs_keepout`、`rc26_terrain` 或 `rc26_base_ground`，`rc26_decision` 也不调用 `/kfs_keepout/set_runtime`、不发布 `/mf_kfs_state`，不订阅 terrain/base-ground/keepout 输出。
- 当前主链没有活跃的 `/mf_block_overlay`、`/kfs_filter_mask`、`/kfs_keepout_heartbeat` 或 `/kfs_keepout/set_runtime` 契约。

## 当前边界

- 接口是否存在以 [src/rc26_interfaces/CMakeLists.txt](/home/potato/RC_2026/src/rc26_interfaces/CMakeLists.txt) 为准
- 跨模块 ROS 契约同步维护在 [docs/middle/modules/navigation.yaml](/home/potato/RC_2026/docs/middle/modules/navigation.yaml) 等模块契约文档中

## 本轮收口

- 删除旧导航 action、运动模式 service 和相关状态消息生成
- 删除定位自定义消息生成，定位接口改为标准 ROS 消息与 TF
- 删除全部 BT 运行时消息与控制接口，明确行为树仅保留为 `rc26_decision` 包内实现细节
- 保留 `MfBlockOverlay`、`MfBlockOverlayCell`、`MfKfs*`、`TerrainFeatureGrid` 与 `SetKeepoutRuntime` 的生成文件，但明确标记为归档兼容接口；默认运行时不再消费这些契约
