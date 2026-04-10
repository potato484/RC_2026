# rviz2

## 模块定位

`rviz2` 是 R2 当前系统级覆盖安装的可视化 GUI 主入口，源码位于 `src/rc26_xhu_viewer/rviz2/`。

它同时承载两种模式：

- 默认 RC26 模式
  - 直接进入 RC26 中文壳层
  - 加载 `navigation/slam × operator/engineering/diagnostic` 六份 preset
  - 启用 Display 白名单
- `--classic` 通用模式
  - 保留完整原生 RViz 使用方式
  - 用于加载任意通用 `.rviz` 配置和常规插件

`rviz2` 只负责 GUI 壳层与工程工具入口，不拥有 `r2/xhu_viewer/*` 的 headless 状态聚合；该职责继续留在 `rc26_xhu_viewer_status`。

## 当前实现

- 主入口：`ros2 run rviz2 rviz2`
- RC26 参数：
  - `--rc26-mode navigation|slam`
  - `--rc26-layout operator|engineering|diagnostic`
- 通用回切参数：
  - `--classic`
- 本地 Web 调试入口：
  - 根目录脚本 `start_rviz2_rc26_web.sh`
  - adapter `src/rc26_xhu_viewer/rviz2/scripts/rviz2_rc26_server.py`
  - 前端 `src/rc26_xhu_viewer/rviz2/viewer`

当前 `rviz2` 包内的 RC26 资产已经全部收口在同一目录树中，不再通过退役的 `src/rc26_xhu_viewer/rc26_xhu_viewer/` 做 staging。当前真实源码分层是：

- `config/`
  - 6 份 `.rviz` preset
  - `field_scene_manifest.yaml`
- `resources/`
  - `terminology.yaml`
- `web/`
  - 嵌入式 Web panel 静态页面
- `viewer/`
  - `rviz2-rc26-web` 前端工程
- `scripts/`
  - `rviz2_rc26_server.py`
  - `visualization_algorithms.py`
  - `validate_viewer_configs.py`
- `src/`
  - `launch_mode.*`
  - `classic_mode.*`
  - `mode_utils.*`
  - `rc26/` 下的 RC26 壳层、显示工厂、插件与历史模块

这一轮模块化后，前端内部也已从“单文件堆逻辑”改成职责拆分：

- `viewer/src/components/SceneCanvas.tsx` 只保留 React 包装层
- Babylon 场景细节收进 `viewer/src/components/scene/BabylonSceneManager.ts`
- trace 结果转换收进 `viewer/src/features/trace/traceModel.ts`
- live 状态桥接收进 `viewer/src/features/live/liveBridge.ts`
- 文案与格式化收进 `viewer/src/features/viewer/formatting.ts`

Web adapter 还有一条已经固定的实现口径：

- `rviz2_rc26_server.py` 不再保留自己的 `render_graph_sim_html.py` 副本
- surface-route / graph trace 的真源脚本固定来自 `src/rc26_topo_nav/scripts/render_graph_sim_html.py`
- 这保证浏览器回放继续消费 `rc26_topo_nav` 的真实算法口径，而不是在 GUI 包里复制一份旁支实现

## 已固化的实现约束

- `RViz2Rc26ViewerFrame` 可以自定义菜单和工具栏外观，但必须继续保留 upstream `VisualizationFrame` 的工具管理骨架，尤其是 add/remove tool action 与 `remove_tool_menu_`；否则默认工具在 load config 阶段调用 `VisualizationFrame::addTool()` 时会直接崩溃。
- `RViz2Rc26DisplayFactory` 只能在 `VisualizationManager` 创建后、`loadDisplayConfig()` 之前注入。不能在 config 已经加载、display 实例已经创建之后再替换 factory；否则旧 factory 被销毁时会把已创建 display 依赖的插件宿主一并卸掉，运行期会直接崩溃。
- Displays 面板的 property tree 在恢复展开状态时，不能再假定模型是静态的。当前实现已经额外做了两层防护：
  - `Expanded` 为空时不再递归遍历整棵属性树
  - 遍历和绘制阶段统一通过 `PropertyTreeModel::getProp()` 校验 index 对应属性是否仍然有效

退出链路上当前还有一条实现约束也已经固化：

- `rviz2` 不再使用 `rclcpp` 默认安装的进程级 `SIGINT/SIGTERM` handler，而是自己把信号记录成“请求关闭”，交给 Qt 定时器去关窗和退出事件循环。
- `qapp.exec()` 返回后，必须先停掉 `continue_timer`，再销毁 `VisualizationFrame`，并显式冲刷一次 Qt 的 `DeferredDelete` 队列；否则定时器 lambda 可能在 frame 已释放后继续访问窗口对象，`Ctrl+C` 退出会直接崩。

## 中文化与 preset 约束

- classic 和 RC26 两种模式下，菜单、工具栏、面板标题、属性树、Add Display、Help、About 以及常见状态文本都统一经过 `rviz_common::DisplayStringManager` 做中文显示；术语真源是随包安装的 `resources/terminology.yaml`。
- 为满足“界面主显不出现英文”的验收口径，菜单和对话框标题里的 `(&F)` / `(&H)` 这类可见助记符已一并去掉；真实快捷键仍继续由 `Ctrl+O`、`Ctrl+S`、`F11` 等 `QKeySequence` 提供，不靠标题里的英文括号提示。
- 兼容性仍以 upstream `.rviz` 协议为准：配置文件里的 `Class`、`Name`、枚举原值、topic/frame 等内部保存值仍保留英文原值，不因为界面中文化而改写序列化格式。
- dock/panel 标题已经拆成“内部稳定 key + 可见中文标题”两层；窗口恢复和 `QMainWindow::saveState()` 依赖稳定 key，界面主显示再走中文标题，避免中文化后破坏布局恢复。
- 当前 6 份 RC26 preset 继续要求 `Visualization Manager` 根节点和每个 `rviz_common/Group` 容器节点都显式写出 `Value: true`，否则运行期会把整组 display 当成默认关闭。

## 依赖与边界

- 当前系统级替换包面是：
  - `rviz2`
  - `rviz_common`
  - `rviz_default_plugins`
- 继续消费系统已有：
  - `rviz_rendering`
  - `rviz_ogre_vendor`
  - `rviz_assimp_vendor`
- RC26 插件库当前名称是：
  - `rviz2_rc26_plugins`

边界要求：

- RC26 模式可以定制菜单、工具栏、白名单和 preset，但不能反向成为运行时状态权威。
- `--classic` 模式必须保留完整原生 RViz 体验，不得把 RC26 白名单或菜单约束强塞给通用使用场景。
- `r2/xhu_viewer/*` 仍由 `rc26_xhu_viewer_status` 负责发布，`rviz2` 只消费这些状态。
- 只要改动涉及 `rviz_common`、`rviz2` 或共享插件 ABI，根工作区验收构建至少应包含 `rviz_common rviz_default_plugins rviz2` 三包；不要只重编前两者后继续让根环境从 `/opt/ros/humble` 取 `rviz_default_plugins`。

## 当前启动口径

- `rc26_bringup` 的 GUI 装配已经直接切到 `rviz2`
- `visualization_profile:=operator_gui|engineering_gui|diagnostic_gui` 会转发为 `rviz2 --rc26-mode ... --rc26-layout ...`
- `visualization_backend` 的当前正式取值为 `rviz2|none`
- `odometry.launch.py` 的 `odometry_use_rviz:=true` 现在同样直接启动魔改 `rviz2`

## 注意点

- 默认执行 `rviz2` 不再是原生 RViz，而是 RC26 模式；需要通用行为时必须显式加 `--classic`
- `start_rviz2_rc26_web.sh` 只是本地工程工具入口，不改变 `rviz2` 与 `rc26_topo_nav` 的权威边界
- `src/rc26_xhu_viewer/rc26_xhu_viewer/` 已添加 `COLCON_IGNORE`，当前只保留历史资料，不应再把它当成独立包或源码真源使用
