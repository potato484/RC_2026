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
  - `SetXhuMotionMode.srv`
  - `NavigateTopoTarget.action`：拓扑导航 action，支持 node / task / route 三类目标
  - `MfBlockOverlay.msg` / `MfBlockOverlayCell.msg`：MF 格阻塞离散状态，rc26_kfs_keepout → rc26_topo_nav
  - `XhuSemanticCorridor.msg`：`rc26_topo_nav` 发布给 `xhu_motion_follower` 的 corridor 执行契约
  - `XhuMotionModeState.msg`：`xhu_motion_mode_manager` 发布的模式状态与速度/加速度约束
  - `XhuTrackingState.msg`：`xhu_motion_follower` 回传给 `rc26_topo_nav` 的执行状态语义（PASS/HOLD/REPLAN/ABORT）
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
- `SetXhuMotionMode.srv`

## 源码入口与阅读顺序
- 先看 `CMakeLists.txt`，这里定义了哪些 `.msg/.srv/.action` 会被 rosidl 生成。
- 再按主题读 `msg/`、`srv/`、`action/`，确认跨包字段真源。
- 最后回到 `docs/middle/openapi.yaml` 和对应模块 YAML，把接口文档和生成接口一一对齐。

## 目录解剖
- `msg/`：跨包消息契约，按行为树、定位、导航、机构、地形、可视化等主题展开。
- `srv/`：短时请求/响应契约，目前主要是行为树控制和导航模式切换。
- `action/`：长任务契约，给机构执行和高层动作使用。
- `CMakeLists.txt`：真正决定哪些接口会被构建和安装。

## 关键文件体量
- `msg/`：当前有 36 份消息定义。
- `action/`：当前有 5 份动作定义。
- `srv/`：当前有 3 份服务定义。
- `README.md`：77 行，是接口主题入口说明。

## 关键源码行段速览
- `src/rc26_interfaces/CMakeLists.txt:15-46`：`rosidl_generate_interfaces()` 列出所有被生成的消息、服务和动作，是“接口是否存在”的构建真源。
- `msg/BehaviorTree*.msg`：决策运行时快照、黑板、Trace、本地化说明契约。
- `msg/LocalizationHealth.msg`、`msg/LocalizationBackendStatus.msg`、`msg/RouteObservability.msg`：定位健康度和图后端状态契约。
- `action/ExecuteMechanism.action`、`GrabTip.action`、`AssembleWeapon.action`、`PlaceKFSGrid.action`、`NavigateTopoTarget.action`：机构执行与拓扑导航异步契约。
- `msg/MfBlockOverlay.msg`、`msg/MfBlockOverlayCell.msg`：`rc26_kfs_keepout` 发布给 `rc26_topo_nav` 的离散阻塞 overlay。
- `srv/SetNavMode.srv`：导航模式切换服务，是 `rc26_nav_mode_manager` 的核心跨包接口；topo 模式下调用值收敛为 `plane_move` / `mf_traverse` / `mf_exit` / `ramp_up` / `ramp_down` / `hold`。
- `msg/XhuSemanticCorridor.msg`、`msg/XhuMotionModeState.msg`、`msg/XhuTrackingState.msg`、`srv/SetXhuMotionMode.srv`：`xhu_direct` 模式的新执行契约，覆盖 corridor 下发、模式发布和执行反馈。

## 模块边界

- 这个包只定义接口格式，不包含业务逻辑
- 它不启动节点，也不做算法计算
- 所有上层文档和前端若要描述跨包数据结构，原则上都应以这里的定义为准
