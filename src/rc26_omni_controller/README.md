# rc26_omni_controller

`rc26_omni_controller` 现在只保留 `xhu_motion_follower_node`，它是 R2 自研导航链中的局部走廊跟踪执行器。

## 当前职责

- 订阅 `/xhu_nav/corridor_cmd`
- 订阅 `/xhu_nav/motion_mode_state`
- 订阅 `/localization/health`、`terrain_features`、`control_state`
- 输出 `cmd_vel`
- 发布 `/xhu_nav/lookahead_path`
- 发布 `/xhu_nav/tracking_state`
- 发布 `/xhu_nav/semantic_gate`

## 当前实现特点

- 适配麦克纳姆底盘的全向跟踪
- 根据 `XhuMotionModeState` 统一约束线速度/角速度与加速度
- 当 corridor、odom、模式状态或关键语义输入缺失时，优先进入 `HOLD`
- 对 topo 执行器回传 `PASS / HOLD / REPLAN / ABORT`

## 当前清理状态

旧插件化执行器资产已经移除，当前仓库只保留 `xhu_motion_follower_node` 及其调试文档。

## 源码入口

- [src/xhu_motion_follower.cpp](/home/potato/RC_2026/src/rc26_omni_controller/src/xhu_motion_follower.cpp)
- [include/rc26_omni_controller/xhu_motion_follower.hpp](/home/potato/RC_2026/src/rc26_omni_controller/include/rc26_omni_controller/xhu_motion_follower.hpp)

调试命令见 [docs/debug_guide.md](/home/potato/RC_2026/src/rc26_omni_controller/docs/debug_guide.md)。
