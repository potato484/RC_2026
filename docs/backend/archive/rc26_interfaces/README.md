# rc26_interfaces

## 模块定位

`rc26_interfaces` 是整个 R2 仓库的跨包接口真源。

## 当前导航相关契约

- `NavigateTopoTarget.action`
- `SetXhuMotionMode.srv`
- `MfBlockOverlay.msg`
- `MfBlockOverlayCell.msg`
- `XhuSemanticCorridor.msg`
- `XhuMotionModeState.msg`
- `XhuTrackingState.msg`

## 当前清理状态

旧兼容导航契约已从该包移除，当前清单只覆盖 topo/xhu 主链和决策运行时所需接口。

## 当前边界

- 只定义消息、服务、动作
- 接口是否存在以 [src/rc26_interfaces/CMakeLists.txt](/home/potato/RC_2026/src/rc26_interfaces/CMakeLists.txt) 为准
