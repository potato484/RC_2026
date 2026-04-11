# 前端当前实现与边界

当前仓库里仍在维护的本地前端工具是：

- `merlin-bt-visualizer`

## 当前前端入口

- `merlin-bt-visualizer`
  - 读取行为树 XML，提供查看、编辑和本地模拟执行能力

## 当前准确定位

- `merlin-bt-visualizer` 是行为树本地工作台
- 它不直接拥有 ROS2 运行时控制权
- 它也不承载机器人在线可视化或诊断聚合后端

## 推荐阅读

- [overview/README.md](overview/README.md)
- [boundaries/README.md](boundaries/README.md)

如果关注行为树编辑链，再进入 `viewer_mode / editor_mode / local_simulator` 系列文档。

## 文档同步规则

- 改前端能力边界或对外表述时，必须同步 [boundaries/README.md](boundaries/README.md)
- 改全局入口或工程骨架时，优先同步 [overview/README.md](overview/README.md)
