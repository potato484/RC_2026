# 前端当前实现与边界

当前仓库里有两套成型的本地前端工具：

- `merlin-bt-visualizer`
- `src/rc26_xhu_viewer/rviz2/viewer`

它们都属于工程工具，不是机器人运行时权威后端。

## 当前前端入口

- `merlin-bt-visualizer`
  - 读取行为树 XML，提供查看、编辑和本地模拟执行能力
- `src/rc26_xhu_viewer/rviz2/viewer`
  - 读取 `rviz2_rc26_server.py` 输出的 scene manifest、surface-route 回放、局部规划案例和 live 运行态
  - 统一展示场地 mesh、路线、阶段区、keepout、定位健康、机构状态和 BT 快照
  - 当前保留为仓库内的本地 Web 可视化工具，不是 bringup 默认启动的车端可视化后端

## 当前准确定位

- `merlin-bt-visualizer` 是行为树本地工作台
- `src/rc26_xhu_viewer/rviz2/viewer` 是 R2 的本地 Web 可视化平台前端
- `src/rc26_xhu_viewer/rviz2/scripts/rviz2_rc26_server.py` 是浏览器 adapter，不是运行时权威
- 两者都不直接拥有 ROS2 运行时控制权

## 推荐阅读

- [overview/README.md](overview/README.md)
- [visualization_viewer/README.md](visualization_viewer/README.md)
- [boundaries/README.md](boundaries/README.md)

如果关注行为树编辑链，再进入 `viewer_mode / editor_mode / local_simulator` 系列文档。

## 文档同步规则

- 改 `src/rc26_xhu_viewer/rviz2/viewer/*` 或 `src/rc26_xhu_viewer/rviz2/scripts/rviz2_rc26_server.py` 时，优先同步 [visualization_viewer/README.md](visualization_viewer/README.md)
- 改前端能力边界或对外表述时，必须同步 [boundaries/README.md](boundaries/README.md)
- 改全局入口或工程骨架时，优先同步 [overview/README.md](overview/README.md)
