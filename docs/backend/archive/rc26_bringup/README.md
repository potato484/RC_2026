# rc26_bringup

## 模块定位

`rc26_bringup` 是 R2 整车链路的统一装配入口，负责决定哪些运行时包被拉起，但不拥有这些包内部算法真源。

## 当前可视化装配口径

`bringup.launch.py` 里的 `visualization_backend` 已经收口为：

- `rviz`
- `local_web`
- `none`

其中：

- `rviz`：启动 RViz 观察面
- `local_web`：直接拉起 `python3 <prefix>/lib/rc26_visualization/visualization_server.py`
- `none`：不启动可视化进程

Foxglove 已不再是 bringup 的主链路后端，也不再通过 launch 参数自动生成布局。

## 当前关键文件

- [launch/bringup.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/bringup.launch.py)
- `launch/localization.launch.py`
- `launch/odometry.launch.py`
- `config/localization.yaml`
- `config/xhu_motion_follower.yaml`
- `config/xhu_motion_runtime.yaml`

## 当前边界

- 负责装配，不承载 planner、控制器或可视化平台的实现本体
- `local_web` 只是把 `rc26_visualization` 接进总装配，不意味着 bringup 自己变成 Web 后端
- `src/rc26_bringup/foxglove/*.json` 仅保留为历史参考资产，不再参与默认安装和主链路 launch

## 本次迁移后的真实变化

- Web 观察主入口从 Foxglove 切换为 `rc26_visualization`
- `visualization_host` / `visualization_port` 替代旧的 `foxglove_*` 参数
- headless 环境下推荐 `visualization_backend:=local_web`，而不是再依赖 Foxglove
