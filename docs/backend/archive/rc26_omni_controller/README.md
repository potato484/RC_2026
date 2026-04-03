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

## 当前边界

- 负责 corridor 跟踪与执行反馈
- 不负责 topo 图搜索和模式决策

## 近期实现说明

- 当前控制器新增 `chassis_model` 参数，支持 `mecanum_4wheel | tracked_diff` 两种底盘模式。
- 当前默认底盘模式已切到 `tracked_diff`，bringup 不再默认按全向控制律运行。
- 四轮模式继续按全向控制律输出 `cmd_vel.linear.y`。
- 履带模式改为单车体跟踪：横向误差通过曲率项和朝向误差项转成 `linear.x + angular.z`，运行时固定 `cmd_vel.linear.y=0`。
