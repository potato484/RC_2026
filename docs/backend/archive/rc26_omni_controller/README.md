# rc26_omni_controller

## 模块定位

`rc26_omni_controller` 是 R2 自研导航链里的局部走廊跟踪执行器宿主包。

## 当前实现

- 构建产物:
  - `xhu_motion_follower_node`
- 关键源码:
  - [src/xhu_motion_follower.cpp](/home/potato/RC_2026/src/rc26_omni_controller/src/xhu_motion_follower.cpp)
  - [include/rc26_omni_controller/xhu_motion_follower.hpp](/home/potato/RC_2026/src/rc26_omni_controller/include/rc26_omni_controller/xhu_motion_follower.hpp)

## 当前接口

- 订阅:
  - `/xhu_nav/corridor_cmd`
  - `/xhu_nav/motion_mode_state`
  - `/xhu_nav/tracking_state`
  - `/localization/health`
  - `control_state`
- 发布:
  - `cmd_vel`
  - `/xhu_nav/lookahead_path`
  - `/xhu_nav/tracking_state`
  - `/xhu_nav/semantic_gate`

## 当前清理状态

旧插件化执行器资产已移除，当前只保留 corridor 跟踪与状态回传所需实现。

## 当前边界

- 负责 corridor 跟踪与执行反馈
- 不负责 topo 图搜索和模式决策
