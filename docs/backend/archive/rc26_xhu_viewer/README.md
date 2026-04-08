# rc26_xhu_viewer

## 模块定位

`rc26_xhu_viewer` 是 RC26 专用工程操作台所在的目录树，当前由两部分组成：

- 一组保留在 `src/rc26_xhu_viewer/` 下的 RViz 上游 vendor 包
- 一个 RC26 运行时包 `src/rc26_xhu_viewer/rc26_xhu_viewer/`

当前 bringup 默认启动的是运行时包提供的 `rc26_xhu_viewer` 可执行入口；状态聚合与本地 Web adapter 也已经并入同一运行时包，对外统一发布 `r2/xhu_viewer/*`。

## 源码来源

- upstream 仓库：`https://github.com/ros2/rviz.git`
- upstream 分支：`humble`
- 导入提交：`5ba3d8ea8ebe5ceec5f008d35f005e02939dac5a`
- 本地路径：`src/rc26_xhu_viewer`

## 包含内容

### Vendor 包

`rviz2`, `rviz_common`, `rviz_default_plugins`, `rviz_rendering`, `rviz_ogre_vendor`, `rviz_assimp_vendor`, `rviz_visual_testing_framework`, `rviz_rendering_tests`

### RC26 运行时包 (`rc26_xhu_viewer`)

| 目录/文件 | 说明 |
|---|---|
| `src/main.cpp` | 自定义入口，直接创建 `RC26XhuViewerFrame` |
| `src/rc26_xhu_viewer_frame.hpp/cpp` | 子类化 `VisualizationFrame`，覆写中文菜单/工具栏，注入白名单 Factory |
| `src/rc26_display_factory.hpp` | Display 白名单过滤器 |
| `src/xhu_viewer_status_core.cpp` | 状态聚合核心，汇总定位、控制、keepout、terrain、机构与导航态 |
| `src/xhu_viewer_status_node.cpp` | `rc26_xhu_viewer_status_node`，对外发布 `r2/xhu_viewer/*` |
| `src/displays/` | 5 个自定义 Display 插件骨架 |
| `src/panels/` | 14 个 Panel 插件（1 门禁 + 12 观测 + 1 历史查询） |
| `src/panels/web_panel_base.hpp/cpp` | QWebEngine 嵌入基类（构建期可选） |
| `src/panels/web_bridge.hpp/cpp` | QWebChannel JS 桥接 |
| `src/history/` | SQLite 环缓冲历史库（2h 保留，tmpfs） |
| `scripts/rc26_xhu_viewer_server.py` | 本地 Web adapter，给 `viewer/` 提供 scene manifest、路线预览和 live bridge |
| `scripts/visualization_algorithms.py` | 几何与规划辅助算法 |
| `scripts/render_graph_sim_html.py` | 离线场景 HTML 渲染辅助脚本 |
| `viewer/` | `rc26-xhu-viewer-web` 前端 |
| `config/*.rviz` | 6 份分组预设（navigation/slam × operator/engineering/diagnostic） |
| `config/xhu_viewer_status.yaml` | `rc26_xhu_viewer_status_node` 参数 |
| `config/field_scene_manifest.yaml` | Web viewer 场景真源 |
| `web/` | 3 份 Web 面板 HTML（BT 时间线/趋势图/事件日志） |
| `resources/terminology.yaml` | 中文术语单一真源 |
| `plugins_description.xml` | pluginlib 注册（5 Display + 14 Panel） |
| `launch/viewer.launch.py` | bringup 接入点 |
| `scripts/validate_viewer_configs.py` | 配置验收脚本 |

### 插件清单

**Display (5):** LocalizationDisplay, RegistrationDebugDisplay, NavCandidatesDisplay, DynamicPredictionDisplay, TerrainSemanticDisplay

**Panel (14):** RC26ConsoleGatePanel, LocalizationPanel, RegistrationPanel, LioPanel, NavigationPanel, TerrainPanel, BtAreaPanel, BtTreePanel, BtTimelinePanel, MechanismPanel, VisionPanel, TelecontrolPanel, SerialPanel, OperatorStatusPanel, HistoryQueryPanel

## 当前边界

- 菜单/工具栏/状态栏已中文化
- Display 白名单限制"添加显示"对话框只展示 RC26 使用的插件
- 6 份 .rviz 预设已升级为 Group 分组结构
- 门禁面板实现操作态/工程态/诊断态三级权限控制
- Display/Panel 当前均为骨架状态，显示"等待数据..."作为优雅降级
- 状态聚合、Web adapter 和本地前端都已经并入 `rc26_xhu_viewer`
- 对外公共诊断接口已切到 `r2/xhu_viewer/summary`、`r2/xhu_viewer/operator_status`、`r2/xhu_viewer/events`
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
- bringup 默认 `visualization_backend:=rc26_xhu_viewer`，`visualization_layout:=operator|engineering|diagnostic`
- bringup 默认同步装配 `rc26_xhu_viewer_status_node`
- 兼容别名 `visualization_backend:=rviz` 仍可用但转发到 `rc26_xhu_viewer`
- 扩展 Display/Panel 应在 `rc26_xhu_viewer/` 内演进

## M3 新增消息（rc26_interfaces）

LocalizationKeyframe, LocalizationLoopClosure, LocalizationRelocState, RegistrationDebug, MechanismActionHistoryEntry, MechanismActionHistory, DynamicPrediction, DynamicPredictionArray
