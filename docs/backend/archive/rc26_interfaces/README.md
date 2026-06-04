# rc26_interfaces

## 模块定位

`rc26_interfaces` 是整个 R2 仓库的跨包接口真源，只定义消息、服务和动作，不包含业务逻辑。

## 当前导航相关契约

本包不再定义自定义导航 action。运行时导航 action 使用外部 Nav2 契约：

- `/navigate_to_pose`
- `nav2_msgs/action/NavigateToPose`

本包当前只保留 MF keepout / decision 支撑契约：

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

## Keepout Runtime 契约

- `/kfs_keepout/set_runtime` 当前是 `rc26_decision -> rc26_kfs_keepout` 的 MF keepout 运行时控制服务，接口类型为 `SetKeepoutRuntime.srv`
- `activate=true` 表示进入 `MFAreaTree` 前请求装载并激活 keepout；`activate=false` 表示离开 MF 时先清空输出再卸载
- 响应字段 `outputs_cleared` 表示下游是否已经收到安全清空后的 overlay/mask 结果
- 响应字段 `component_loaded` 表示组件是否仍留在容器内；允许在 `outputs_cleared=true` 时仍为 `true`，用来表达“已安全退出但卸载失败”

## 当前边界

- 接口是否存在以 [src/rc26_interfaces/CMakeLists.txt](/home/potato/RC_2026/src/rc26_interfaces/CMakeLists.txt) 为准
- 跨模块 ROS 契约同步维护在 [docs/middle/modules/navigation.yaml](/home/potato/RC_2026/docs/middle/modules/navigation.yaml) 等模块契约文档中

## 本轮收口

- 删除旧导航 action、运动模式 service 和相关状态消息生成
- 保留 `MfBlockOverlay`、`MfBlockOverlayCell` 与 `SetKeepoutRuntime`，因为它们仍属于 MF keepout / decision runtime 支撑接口
