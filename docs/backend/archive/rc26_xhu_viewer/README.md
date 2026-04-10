# rc26_xhu_viewer

## 当前定位

`rc26_xhu_viewer` 已经完全退役为独立 ROS2 GUI 包。

当前真实状态是：

- 它不再参与 `colcon` 包发现
- 它不再是 bringup、odometry 或 Web 调试的启动目标
- 系统级 GUI 主入口已经切到 `rviz2`
- Web adapter 与前端源码也已经迁入 `src/rc26_xhu_viewer/rviz2/`
- `r2/xhu_viewer/*` 的 headless 聚合继续留在 `rc26_xhu_viewer_status`

## 仍保留这个目录的原因

`src/rc26_xhu_viewer/rc26_xhu_viewer/` 当前保留的原因，只是为了保留历史资料和退役包 tombstone，而不是继续把它当成源码 staging 区维护。

这意味着：

- 这里不再保留可构建包骨架；当前只保留 `COLCON_IGNORE` 与一份说明 README
- 当前真实 GUI / Web 源码不再从这里拷贝或安装
- 如果你在这里看到“文件已被移走”或“路径为空”，这属于预期结果，不是工作区损坏

为避免继续被误认为独立包，该目录已添加 `COLCON_IGNORE`。

## 已迁出的真实实现

以下内容现在都应直接看 `src/rc26_xhu_viewer/rviz2/`，不要再回头看本目录：

- `RViz2Rc26ViewerFrame`
- Display allowlist
- 6 份 `.rviz` preset
- `resources/terminology.yaml`
- 可选 Panel/Display 插件
- `rviz2_rc26_server.py` 与 `viewer/` 前端工程

对应地，中文术语真源也已经固定在：

- `src/rc26_xhu_viewer/rviz2/resources/terminology.yaml`

它继续承载：

- 面板、菜单、工具栏、对话框、帮助页、About 文案
- 内建 display/tool/view/panel 名称与说明
- 属性树字段名、枚举值、常见状态文本
- RC26 默认验收链路里的 topic/frame/技术缩写中文别名

## 历史边界

这个目录树曾经承载：

- 独立 GUI 壳层入口
- `.rviz` preset
- 可选插件和 Web 工具链

但当前这些能力已经并入 `rviz2`；这里只保留历史资料事实，不再作为真实实现入口。

## 现在应该看哪里

- 当前 GUI 真入口看 [../rviz2/README.md](/home/potato/RC_2026/docs/backend/archive/rviz2/README.md)
- 当前 headless 语义聚合看 [../rc26_xhu_viewer_status/README.md](/home/potato/RC_2026/docs/backend/archive/rc26_xhu_viewer_status/README.md)
- 历史瘦身过程和最终收口结果看 [瘦身执行方案.md](/home/potato/RC_2026/docs/backend/archive/rc26_xhu_viewer/瘦身执行方案.md)
