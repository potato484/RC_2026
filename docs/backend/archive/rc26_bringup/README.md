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
- `*_gui`：显式按需启动 RC26 自己维护的 RViz 定制入口，并自动选择对应 preset

兼容覆盖项仍保留：

- `visualization_backend:=rc26_xhu_viewer | none`
- `visualization_layout:=operator | engineering | diagnostic`
- `use_rviz:=true | false`

旧里程计调试入口 `launch/odometry.launch.py` 也已对齐这套口径：

- `odometry_use_rviz:=true` 不再直接启动 `rviz2`
- 现在会转而 include `rc26_xhu_viewer/launch/viewer.launch.py`
- 默认使用 `odometry_visualization_layout:=diagnostic`
- 无 DISPLAY / WAYLAND_DISPLAY 时会自动跳过 GUI，只保留 headless 里程计链

Foxglove 和 `local_web` 都不再是 bringup 的主链路后端。

## 当前关键文件

- [launch/bringup.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/bringup.launch.py)
- `launch/localization.launch.py`
- `launch/odometry.launch.py`
- `config/localization.yaml`
- `config/xhu_motion_follower.yaml`
- `config/xhu_motion_runtime.yaml`

## 当前边界

- 负责装配，不承载 planner、控制器或可视化平台的实现本体
- `rc26_xhu_viewer` 只是被装配进总启动链，不意味着 bringup 自己变成可视化实现包
- `src/rc26_bringup/foxglove/*.json` 与 `src/rc26_xhu_viewer/rc26_xhu_viewer/viewer` 都不再是 bringup 默认启动的主可视化后端

## 本次瘦身后的真实变化

- bringup 默认口径已切到 `visualization_profile:=headless`，不再默认常驻 GUI
- `visualization_profile:=operator_gui | engineering_gui | diagnostic_gui` 会自动解析到 `rc26_xhu_viewer + 对应 layout`
- `visualization_status_enable` 现在装配的是 `rc26_xhu_viewer_status/rc26_xhu_viewer_status_node`
- `odometry.launch.py` 的历史 `odometry_use_rviz` 开关已迁移到 `rc26_xhu_viewer`，不再直接依赖 `rviz2`
- `visualization_backend / visualization_layout / use_rviz` 继续保留为兼容覆盖项
- headless 环境下不再推荐 `local_web`；没有图形环境时应优先保持 `visualization_profile:=headless`
- `visualization_backend:=rviz` 仅保留为兼容别名，实际仍会转发到 `rc26_xhu_viewer`
