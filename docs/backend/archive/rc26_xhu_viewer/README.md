# rc26_xhu_viewer

## 当前状态

`src/rc26_xhu_viewer/` 已于 2026-04-10 整体从工作区删除，不再保留任何实现代码。

当前真实状态是：

- 它不再参与 `colcon` 包发现或运行时装配
- `rc26_bringup` 与 `odometry.launch.py` 已收口为 headless
- 根目录 `start_rviz2_rc26_web.sh` 与相关 preflight / E2E / release 脚本已一并移除
- 本目录下文档现在只保留历史事实，不再描述当前源码入口

## 历史边界

这个目录树曾经承载：

- 独立 GUI 壳层入口
- `.rviz` preset
- 可选插件和 Web 工具链

但这些能力对应的整个源码树现在都已经从仓库删除；这里只保留历史资料事实，不再作为真实实现入口。

## 现在应该看哪里

- 当前 ROS2 工作区边界看 [../../fitness/architecture_fitness_ros2_workspace/README.md](/home/potato/RC_2026/docs/fitness/architecture_fitness_ros2_workspace/README.md)
- 当前整车装配入口看 [../rc26_bringup/README.md](/home/potato/RC_2026/docs/backend/archive/rc26_bringup/README.md)
- 历史瘦身过程看 [瘦身执行方案.md](/home/potato/RC_2026/docs/backend/archive/rc26_xhu_viewer/瘦身执行方案.md)
