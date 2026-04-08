# rc26_xhu_viewer

## 模块定位

`rc26_xhu_viewer` 是 RC26 当前保留的 RViz GUI/工具包，源码位于 `src/rc26_xhu_viewer/rc26_xhu_viewer/`。

它负责：

- RViz GUI 壳层入口
- `navigation/slam × operator/engineering/diagnostic` 六份 `.rviz` preset
- 中文菜单/工具栏/状态栏与 Display 白名单
- 可选 Display/Panel 插件与可选 Web 工具链

它不再拥有默认常驻的状态聚合运行时；`r2/xhu_viewer/*` 当前由 sibling 包 `rc26_xhu_viewer_status` 发布。

## 源码来源

- upstream 仓库：`https://github.com/ros2/rviz.git`
- upstream 分支：`humble`
- 导入提交：`5ba3d8ea8ebe5ceec5f008d35f005e02939dac5a`
- 本地路径：`src/rc26_xhu_viewer/rc26_xhu_viewer`

## 包含内容

### 当前包 (`rc26_xhu_viewer`)

| 目录/文件 | 说明 |
|---|---|
| `src/main.cpp` | 自定义入口，直接创建 `RC26XhuViewerFrame` |
| `src/rc26_xhu_viewer_frame.hpp/cpp` | 子类化 `VisualizationFrame`，覆写中文菜单/工具栏，注入白名单 Factory |
| `src/rc26_display_factory.hpp` | Display 白名单过滤器 |
| `src/displays/` | 5 个自定义 Display 插件骨架（默认 `BUILD_XHU_VIEWER_PLUGINS=OFF` 不构建） |
| `src/panels/` | 14 个 Panel 插件（1 门禁 + 12 观测 + 1 历史查询；默认 `BUILD_XHU_VIEWER_PLUGINS=OFF` 不构建） |
| `src/panels/web_panel_base.hpp/cpp` | QWebEngine 嵌入基类（构建期可选） |
| `src/panels/web_bridge.hpp/cpp` | QWebChannel JS 桥接 |
| `src/history/` | SQLite 环缓冲历史库（2h 保留，tmpfs；默认随 `BUILD_XHU_VIEWER_PLUGINS=OFF` 一起不构建） |
| `scripts/rc26_xhu_viewer_server.py` | 本地 Web adapter，给 `viewer/` 提供 scene manifest、路线预览和 live bridge（仅 `BUILD_XHU_VIEWER_WEB_TOOLS=ON` 时安装） |
| `scripts/visualization_algorithms.py` | 几何与规划辅助算法（仅 `BUILD_XHU_VIEWER_WEB_TOOLS=ON` 时安装） |
| `scripts/render_graph_sim_html.py` | 离线场景 HTML 渲染辅助脚本（仅 `BUILD_XHU_VIEWER_WEB_TOOLS=ON` 时安装） |
| `viewer/` | `rc26-xhu-viewer-web` 前端（仅 `BUILD_XHU_VIEWER_WEB_TOOLS=ON` 时安装） |
| `config/*.rviz` | 6 份分组预设（navigation/slam × operator/engineering/diagnostic） |
| `config/field_scene_manifest.yaml` | Web viewer 场景真源 |
| `web/` | 3 份 Web 面板 HTML（BT 时间线/趋势图/事件日志；仅 `BUILD_XHU_VIEWER_PLUGINS=ON` 时安装） |
| `resources/terminology.yaml` | 中文术语单一真源 |
| `plugins_description.xml` | pluginlib 注册（5 Display + 14 Panel；仅 `BUILD_XHU_VIEWER_PLUGINS=ON` 时导出） |
| `launch/viewer.launch.py` | bringup 接入点 |
| `scripts/validate_viewer_configs.py` | 配置验收脚本（默认始终安装） |

### 插件清单

**Display (5):** LocalizationDisplay, RegistrationDebugDisplay, NavCandidatesDisplay, DynamicPredictionDisplay, TerrainSemanticDisplay

**Panel (14):** RC26ConsoleGatePanel, LocalizationPanel, RegistrationPanel, LioPanel, NavigationPanel, TerrainPanel, BtAreaPanel, BtTreePanel, BtTimelinePanel, MechanismPanel, VisionPanel, TelecontrolPanel, SerialPanel, OperatorStatusPanel, HistoryQueryPanel

## 当前边界

- 菜单/工具栏/状态栏已中文化
- Display 白名单限制"添加显示"对话框只展示 RC26 使用的插件
- 6 份 .rviz 预设已升级为 Group 分组结构
- 门禁面板实现操作态/工程态/诊断态三级权限控制
- Display/Panel 当前均为骨架状态，显示"等待数据..."作为优雅降级
- `r2/xhu_viewer/*` 默认由 sibling 包 `rc26_xhu_viewer_status` 提供，本包不再承载 headless 常驻节点
- bringup 默认已切到 `visualization_profile:=headless`，GUI 不再随整车启动链默认常驻
- 本包只在 `visualization_profile:=*_gui` 或显式 `visualization_backend:=rc26_xhu_viewer` 时被 bringup 拉起
- `rc26_bringup/launch/odometry.launch.py` 的旧调试入口也已改为 include `launch/viewer.launch.py`，默认走 `diagnostic` 布局
- `BUILD_XHU_VIEWER_PLUGINS` 默认 `OFF`，当前 6 份 `.rviz` 预设不依赖 `rc26_xhu_viewer_plugins`
- `BUILD_XHU_VIEWER_WEB_TOOLS` 默认 `OFF`，机器人最小部署不再默认安装 Web adapter 和前端资源
- QWebEngine 为可选构建依赖，缺失时 Web 面板回退为"不可用"占位
- 本地历史库基础设施已就绪，ROS 订阅接入待完成

## Vendor 改动清单

| 文件 | 改动 |
|---|---|
| `rviz_common/.../visualization_frame.hpp` | `initMenus()` / `initToolbars()` 加 `virtual` |
| `rviz_common/.../visualization_manager.hpp` | 新增 `setDisplayFactory(DisplayFactory*)` |
| `rviz_common/.../visualization_manager.cpp` | 实现 `setDisplayFactory` |
| `rviz_common/include/.../display_factory.hpp` | 从 `src/` 复制到 `include/` 公开 |
| `rviz_common/.../visualization_frame.cpp` | FPS 标签中文化 |

## 注意点

- 导入后已移除 `src/rc26_xhu_viewer/.git` 避免嵌套
- bringup 默认 `visualization_profile:=headless`；带屏调试可显式选择 `operator_gui|engineering_gui|diagnostic_gui`
- 兼容覆盖项 `visualization_backend:=rc26_xhu_viewer|none`、`visualization_layout:=operator|engineering|diagnostic` 仍可用
- `rc26_xhu_viewer_status_node` 与 `config/xhu_viewer_status.yaml` 已迁入 `rc26_xhu_viewer_status`
- 兼容别名 `visualization_backend:=rviz` 仍可用但转发到 `rc26_xhu_viewer`
- `odometry.launch.py` 里的 `odometry_use_rviz:=true` 现在同样只会拉起 `rc26_xhu_viewer`，不再直接引用 `rviz2`
- `BUILD_XHU_VIEWER_PLUGINS=ON` 时才构建/安装 `rc26_xhu_viewer_plugins` 和 `web/`
- `BUILD_XHU_VIEWER_WEB_TOOLS=ON` 时才安装 `viewer/` 和 Web adapter 脚本
- 扩展 Display/Panel 应在 `rc26_xhu_viewer/` 内演进

## 延伸文档

- [思考.md](/home/potato/RC_2026/docs/backend/archive/rc26_xhu_viewer/思考.md)：围绕 vendor 瘦身、运行时减负和当前链路适配的阶段性思考。当前结论是优先做“默认 headless + status 常驻 + GUI/Web 工具链可选化”，而不是直接硬删所有 RViz vendor 包。
- [瘦身执行方案.md](/home/potato/RC_2026/docs/backend/archive/rc26_xhu_viewer/瘦身执行方案.md)：把瘦身方向拆成 `P0/P1/P2` 的可执行路线，明确先做默认 headless 和可选构建，再做 status/gui/web 拆分，最后才评估是否退出本地 RViz fork。
- [../rc26_xhu_viewer_status/README.md](/home/potato/RC_2026/docs/backend/archive/rc26_xhu_viewer_status/README.md)：当前默认常驻的状态聚合运行时包说明。

## M3 新增消息（rc26_interfaces）

LocalizationKeyframe, LocalizationLoopClosure, LocalizationRelocState, RegistrationDebug, MechanismActionHistoryEntry, MechanismActionHistory, DynamicPrediction, DynamicPredictionArray
