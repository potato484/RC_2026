# rc26_omni_controller

## 模块定位

`rc26_omni_controller` 是 R2 自研导航链里的局部走廊跟踪执行器宿主包。

## 当前实现

- 构建产物:
  - `xhu_motion_follower_node`
- 关键源码:
  - [src/xhu_motion_follower.cpp](/home/aidlux/RC_2026/src/rc26_omni_controller/src/xhu_motion_follower.cpp)
  - [include/rc26_omni_controller/xhu_motion_follower.hpp](/home/aidlux/RC_2026/src/rc26_omni_controller/include/rc26_omni_controller/xhu_motion_follower.hpp)

## 当前接口

- 订阅:
  - `/xhu_nav/corridor_cmd`
  - `/xhu_nav/motion_mode_state`
  - `/localization/health`
  - `terrain_features`
  - `control_state`
- 发布:
  - `cmd_vel`
  - `/xhu_nav/lookahead_path`
  - `/xhu_nav/tracking_state`
  - `/xhu_nav/semantic_gate`

## 当前边界

- 负责 corridor 跟踪与执行反馈
- 负责基于地形风险和 stop envelope 的局部保守安全检查
- 不负责 topo 图搜索和模式决策
- 不负责完整局部避障器或全域 full-body collision planner

## 近期实现说明

- 当前控制器新增 `chassis_model` 参数，支持 `mecanum_4wheel | tracked_diff` 两种底盘模式。
- 当前 R2 实机口径已经切到履带式底盘，bringup 默认使用 `tracked_diff`，不再按全向控制律运行。
- 四轮模式继续按全向控制律输出 `cmd_vel.linear.y`。
- 履带模式改为单车体跟踪：横向误差通过曲率项和朝向误差项转成 `linear.x + angular.z`，运行时固定 `cmd_vel.linear.y=0`。
- 当前新增 `robot_geometry_file` / `robot_geometry_profile` 参数；`xhu_motion_follower` 会从共享几何真源读取 `stop_envelope_half_width_m`，并把本地 `stop_envelope_half_width_m` 参数与几何 profile 取较大值，用于地形风险 ahead sampling。

## 当前口径说明

- R2 当前真实底盘形态是履带式，`tracked_diff` 应视为整车运行和联调时的默认运动学口径。
- `mecanum_4wheel` 仍然保留在控制器里，主要用于兼容旧链路、回归测试和参数对照，不应再作为当前实机默认假设。
- full-body collision planning 设计仍需把“车体几何”与“底盘运动学模式”解耦，不能假设二者永久绑定。
