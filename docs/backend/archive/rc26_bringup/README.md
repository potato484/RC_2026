# rc26_bringup

## 模块定位

`rc26_bringup` 是 R2 整车链路的统一装配入口，负责决定哪些运行时包被拉起，但不拥有这些包内部算法真源。

## 当前可视化装配口径

`bringup.launch.py` 当前推荐优先使用：

- `visualization_profile:=headless`
- `visualization_profile:=operator_gui`
- `visualization_profile:=engineering_gui`
- `visualization_profile:=diagnostic_gui`

其中：

- `headless`：车端默认口径；不启动 GUI，但继续装配 `rc26_xhu_viewer_status_node`
- `*_gui`：显式按需启动系统级魔改 `rviz2`，并自动选择 RC26 preset

兼容覆盖项当前收口为：

- `visualization_backend:=rviz2 | none`
- `visualization_layout:=operator | engineering | diagnostic`
- `use_rviz:=true | false`

旧里程计调试入口 `launch/odometry.launch.py` 也已对齐：

- `odometry_use_rviz:=true` 直接启动魔改 `rviz2`
- 默认使用 `odometry_visualization_layout:=diagnostic`
- 无 DISPLAY / WAYLAND_DISPLAY 时自动跳过 GUI，只保留 headless 里程计链

Foxglove 和 `local_web` 都不再是 bringup 的主链路后端。

## 当前关键文件

- [launch/bringup.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/bringup.launch.py)
- [launch/odometry.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/odometry.launch.py)
- `launch/localization.launch.py`
- `config/localization.yaml`
- `config/xhu_motion_follower.yaml`
- `config/xhu_motion_runtime.yaml`

## 当前边界

- 负责装配，不承载 planner、控制器或可视化平台的实现本体
- `rviz2` 只是被装配进总启动链，不意味着 bringup 自己变成 GUI 实现包
- `rc26_xhu_viewer_status` 仍独立负责 `r2/xhu_viewer/*` 的 headless 聚合

## 当前真实变化

- bringup 默认口径仍是 `visualization_profile:=headless`
- `visualization_profile:=operator_gui | engineering_gui | diagnostic_gui` 现在解析到 `rviz2 + RC26 参数`
- `visualization_status_enable` 继续装配 `rc26_xhu_viewer_status/rc26_xhu_viewer_status_node`
- `visualization_backend` 的正式取值已切到 `rviz2|none`
- `odometry.launch.py` 已改为直接 `Node(package='rviz2', executable='rviz2', ...)`
- 旧独立 GUI 包 `rc26_xhu_viewer` 已退役，不再作为 bringup 依赖
