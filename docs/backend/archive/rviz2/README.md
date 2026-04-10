# rviz2

## 模块定位

`rviz2` 现在是 R2 当前系统级覆盖安装的可视化 GUI 主入口，源码位于 `src/rc26_xhu_viewer/rviz2/`。

它同时承载两种模式：

- 默认 RC26 模式
  - 直接进入 RC26 中文壳层
  - 加载 `navigation/slam × operator/engineering/diagnostic` 六份 preset
  - 启用 Display 白名单
- `--classic` 通用模式
  - 保留完整原生 RViz 使用方式
  - 用于加载任意通用 `.rviz` 配置和常规插件

`rviz2` 只负责 GUI 壳层，不拥有 `r2/xhu_viewer/*` 的 headless 状态聚合；该职责继续留在 `rc26_xhu_viewer_status`。

## 当前实现

- 主入口：`ros2 run rviz2 rviz2`
- RC26 参数：
  - `--rc26-mode navigation|slam`
  - `--rc26-layout operator|engineering|diagnostic`
- 通用回切参数：
  - `--classic`

当前 `rviz2` 包内安装的 RC26 资产包括：

- 中文菜单/工具栏壳层与 Display allowlist
- 6 份 `.rviz` preset
- `resources/terminology.yaml`
- 可选 `rc26_xhu_viewer_plugins`
- 可选 Web adapter 与前端资源

这些 RC26 资产的源码当前仍暂放在 `src/rc26_xhu_viewer/rc26_xhu_viewer/` 目录树中，但该目录已经不再作为独立 ROS2 包参与 `colcon`；它只作为 `rviz2` 的源码资产 staging 区。

启动期实现上还有一个已经踩过的约束：

- `RC26XhuViewerFrame` 可以自定义菜单和工具栏外观，但必须继续保留 upstream `VisualizationFrame` 的工具管理骨架，尤其是 add/remove tool action 与 `remove_tool_menu_`；否则默认工具在 load config 阶段调用 `VisualizationFrame::addTool()` 时会直接崩溃。
- `RC26DisplayFactory` 只能在 `VisualizationManager` 创建后、`loadDisplayConfig()` 之前注入。不能在 config 已经加载、display 实例已经创建之后再替换 factory；否则旧 factory 被销毁时会把已创建 display 依赖的插件宿主一并卸掉，运行期会直接崩溃。
- Displays 面板的 property tree 在恢复展开状态时，不能再假定模型是静态的。当前实现已经额外做了两层防护：
  - `Expanded` 为空时不再递归遍历整棵属性树
  - 遍历和绘制阶段统一通过 `PropertyTreeModel::getProp()` 校验 index 对应属性是否仍然有效

这条防护是为了解决 `地形安全` 一类动态 display/status 子树在启动期或交互期发生增删时触发的悬空索引崩溃。

退出链路上当前还有一条实现约束也已经固化：

- `rviz2` 不再使用 `rclcpp` 默认安装的进程级 `SIGINT/SIGTERM` handler，而是自己把信号记录成“请求关闭”，交给 Qt 定时器去关窗和退出事件循环。
- `qapp.exec()` 返回后，必须先停掉 `continue_timer`，再销毁 `VisualizationFrame`，并显式冲刷一次 Qt 的 `DeferredDelete` 队列；否则定时器 lambda 可能在 frame 已释放后继续访问窗口对象，`Ctrl+C` 退出会直接崩。
- 在 Humble 下，若进程是被 `SIGINT/SIGTERM` 拉起退出链，`rclcpp::Node` 的 `CallbackGroup` 析构仍可能在 ROS context 收尾阶段崩溃。当前实现对此采用了有意识的“仅信号退出路径泄漏 node 到进程结束”的规避，避免终端 `Ctrl+C` 再次把 `rviz2` 打成 `Segmentation fault`。常规窗口关闭路径仍保持正常析构。

当前 6 份 RC26 preset 还补充了一条显式约束：

- `Visualization Manager` 根节点和每个 `rviz_common/Group` 容器节点都必须写出 `Value: true`

原因不是格式洁癖，而是当前 preset 采用“Group 包 Display”的稀疏写法；如果 group 容器缺少显式启用值，运行时会把整组 display 当成默认关闭，表现为配置文件存在但界面里看不到完整信息。

本轮中文化补充后，`rviz2` 还有三条已经固化的新实现口径：

- classic 和 RC26 两种模式下，菜单、工具栏、面板标题、属性树、Add Display、Help、About 以及常见状态文本都统一经过 `rviz_common::DisplayStringManager` 做中文显示；术语真源是随包安装的 `resources/terminology.yaml`。
- 为满足“界面主显不出现英文”的验收口径，菜单和对话框标题里的 `(&F)` / `(&H)` 这类可见助记符已一并去掉；真实快捷键仍继续由 `Ctrl+O`、`Ctrl+S`、`F11` 等 `QKeySequence` 提供，不靠标题里的英文括号提示。
- 兼容性仍以 upstream `.rviz` 协议为准：配置文件里的 `Class`、`Name`、枚举原值、topic/frame 等内部保存值仍保留英文原值，不因为界面中文化而改写序列化格式。
- dock/panel 标题已经拆成“内部稳定 key + 可见中文标题”两层；窗口恢复和 `QMainWindow::saveState()` 依赖稳定 key，界面主显示再走中文标题，避免中文化后破坏布局恢复。
- Help 菜单入口现在改为“找到或创建帮助 dock 后显式 `show()/raise()/activateWindow()`”；这样即使帮助面板以 floating dock 方式创建，也不会只生成对象而不把窗口拉到前台。

## 依赖与边界

- 当前系统级替换包面是：
  - `rviz2`
  - `rviz_common`
  - `rviz_default_plugins`
- 继续消费系统已有：
  - `rviz_rendering`
  - `rviz_ogre_vendor`
  - `rviz_assimp_vendor`
- 当前 6 份 preset 仍显式依赖：
  - `rviz_default_plugins/*`
  - `grid_map_rviz_plugin/GridMap`

边界要求：

- RC26 模式可以定制菜单、工具栏、白名单和 preset，但不能反向成为运行时状态权威。
- `--classic` 模式必须保留完整原生 RViz 体验，不得把 RC26 白名单或菜单约束强塞给通用使用场景。
- `r2/xhu_viewer/*` 仍由 `rc26_xhu_viewer_status` 负责发布，`rviz2` 只消费这些状态。
- 根工作区一旦覆盖安装了本地 `rviz_common` 或 `rviz2`，就必须同步覆盖安装本地 `rviz_default_plugins`。默认插件直接链接 `rviz_common`；如果 `source install/setup.bash` 后形成“本地 `rviz_common/rviz2` + `/opt/ros/humble` 的 `rviz_default_plugins`”混装，启动期会因为插件宿主 ABI 不一致再次触发崩溃。

## 当前启动口径

- `rc26_bringup` 的 GUI 装配已经直接切到 `rviz2`
- `visualization_profile:=operator_gui|engineering_gui|diagnostic_gui` 会转发为 `rviz2 --rc26-mode ... --rc26-layout ...`
- `visualization_backend` 的当前正式取值为 `rviz2|none`
- `odometry.launch.py` 的 `odometry_use_rviz:=true` 现在同样直接启动魔改 `rviz2`

## 注意点

- 默认执行 `rviz2` 不再是原生 RViz，而是 RC26 模式；需要通用行为时必须显式加 `--classic`
- 当前 `rviz_common` 仍保留少量本地 patch，用于支撑 RC26 壳层覆写与 Display allowlist 注入
- 只要改动涉及 `rviz_common`、`rviz2` 或共享插件 ABI，根工作区验收构建至少应包含 `rviz_common rviz_default_plugins rviz2` 三包；不要只重编前两者后继续让根环境从 `/opt/ros/humble` 取 `rviz_default_plugins`
- `Add Display` 的 topic 树和 suffix 下拉框现在显示中文别名，但内部回填 topic 时仍强制使用原始英文 topic/suffix，不能把显示文案当成 ROS 契约值继续回写
- `src/rc26_xhu_viewer/rc26_xhu_viewer/` 已添加 `COLCON_IGNORE`，不应再把它当成独立可启动包使用
