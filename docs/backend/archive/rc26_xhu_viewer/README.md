# rc26_xhu_viewer

## 当前定位

`rc26_xhu_viewer` 已经退役为独立 ROS2 GUI 包。

当前真实状态是：

- 它不再参与 `colcon` 包发现
- 它不再是 bringup 或 odometry 的启动目标
- 系统级 GUI 主入口已经切到 `rviz2`
- `r2/xhu_viewer/*` 的 headless 聚合继续留在 `rc26_xhu_viewer_status`

## 仍保留这个目录的原因

`src/rc26_xhu_viewer/rc26_xhu_viewer/` 目录当前仍保留，原因不是继续把它当成包维护，而是把其中已有的 RC26 资产继续作为 `rviz2` 的源码 staging 区复用，包括：

- `RC26XhuViewerFrame`
- Display allowlist
- 6 份 `.rviz` preset
- `resources/terminology.yaml`
- 可选 Panel/Display 插件骨架
- 可选 Web adapter 与前端资源

为避免继续被误认为独立包，该目录已添加 `COLCON_IGNORE`。

## 历史边界

这个目录树曾经承载：

- 独立 GUI 壳层入口
- `.rviz` preset
- 可选插件和 Web 工具链

但当前这些 GUI 能力已经并入 `rviz2`；这里只保留历史资料和源码 staging 事实，不再作为真实实现入口。

## 现在应该看哪里

- 当前 GUI 真入口看 [../rviz2/README.md](/home/potato/RC_2026/docs/backend/archive/rviz2/README.md)
- 当前 headless 语义聚合看 [../rc26_xhu_viewer_status/README.md](/home/potato/RC_2026/docs/backend/archive/rc26_xhu_viewer_status/README.md)
- 历史瘦身过程和当前收口结果看 [瘦身执行方案.md](/home/potato/RC_2026/docs/backend/archive/rc26_xhu_viewer/瘦身执行方案.md)
