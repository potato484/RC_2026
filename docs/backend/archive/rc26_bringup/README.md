# rc26_bringup

## 模块定位

`rc26_bringup` 是 R2 整车链路的统一装配入口，负责决定哪些运行时包被拉起，但不拥有这些包内部算法真源。

## 当前可视化装配口径

`bringup.launch.py` 里的 `visualization_backend` 已经收口为：

- `rc26_xhu_viewer`
- `none`

其中：

- `rc26_xhu_viewer`：启动 RC26 自己维护的 RViz 定制入口
- `none`：不启动可视化进程

同时新增：

- `visualization_layout:=operator | engineering | diagnostic`

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

## 本次迁移后的真实变化

- bringup 默认可视化后端从通用 `rviz2` 切到 `rc26_xhu_viewer`
- `visualization_layout` 用来显式选择 `operator / engineering / diagnostic` 三套 preset
- `visualization_status_enable` 现在装配的是 `rc26_xhu_viewer_status_node`
- headless 环境下不再推荐 `local_web`；没有图形环境时应直接使用 `visualization_backend:=none`
- `visualization_backend:=rviz` 仅保留为兼容别名，实际仍会转发到 `rc26_xhu_viewer`
