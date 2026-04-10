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

其中 `resources/terminology.yaml` 的角色在当前实现里已经进一步固化：它不只是 RC26 中文词表备份，而是 `rviz2/rviz_common` 共用的中文术语真源，当前已经集中承载：

- 面板、菜单、工具栏、对话框、帮助页、About 文案
- 内建 display/tool/view/panel 名称与说明
- 属性树字段名、枚举值、常见状态文本
- RC26 默认验收链路里的 topic/frame/技术缩写中文别名

对应的显示规则也已经固定：

- 主界面统一“中文主显”，属性提示层可以保留原始英文值用于排障
- `.rviz` 配置保存和加载继续使用英文原值，不把中文显示文案写回协议层
- 与面板恢复相关的稳定 key 不再直接复用可见标题，避免中文化后破坏窗口状态恢复
- 为满足当前零英文验收口径，RC26 壳层菜单和 `rviz_common` 对话框标题里的可见助记符已经去掉，不再在 `文件(F)`、`帮助(H)` 这类位置保留英文括号提示
- Help 入口现在会在创建/定位帮助面板后显式把 floating dock 拉到前台，避免帮助页对象已创建但窗口没有真正弹出的情况

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
