# rc26_xhu_viewer

## 模块定位

`rc26_xhu_viewer` 现在已经不只是 RViz 上游源码 fork 底座，而是：

- 一组保留在 `src/rc26_xhu_viewer/` 下的 RViz 上游 vendor 包
- 一个新增的 RC26 运行时包 `src/rc26_xhu_viewer/rc26_xhu_viewer/`

当前 bringup 默认启动的是这个运行时包提供的 `rc26_xhu_viewer` 可执行入口，而不是直接拉起上游 `rviz2`。

## 当前源码来源

- upstream 仓库：`https://github.com/ros2/rviz.git`
- upstream 分支：`humble`
- 导入提交：`5ba3d8ea8ebe5ceec5f008d35f005e02939dac5a`
- 本地路径：`src/rc26_xhu_viewer`

## 当前包含内容

当前目录直接保留了上游仓库的主要包：

- `rviz2`
- `rviz_common`
- `rviz_default_plugins`
- `rviz_rendering`
- `rviz_ogre_vendor`
- `rviz_assimp_vendor`
- `rviz_visual_testing_framework`
- `rviz_rendering_tests`

同时新增了一个 RC26 自己维护的运行时包：

- `rc26_xhu_viewer`
  - `src/main.cpp`: 自定义入口，默认解析 `mode/layout` 并装载 RC26 预设
  - `launch/viewer.launch.py`: bringup 接入点
  - `config/navigation_*.rviz`: 导航模式下的 `operator / engineering / diagnostic` 三套预设
  - `config/slam_*.rviz`: 建图模式下的 `operator / engineering / diagnostic` 三套预设
  - `scripts/validate_viewer_configs.py`: 代码级配置验收脚本

## 当前边界

- `rc26_xhu_viewer` 当前只接管 RViz 运行时入口和预设布局，不改写定位、导航、地形、BT、机构等模块的权威数据源
- `r2/diag/*` 仍由 `rc26_visualization` 发布，viewer 只是消费者
- 当前仍保留 upstream vendor 包，尚未做到“可视化域单包、完全裁剪后的单发行物”
- 进程内嵌 Web、2 小时本地历史库、专用 Panel/Display 白名单和全量中文化目前还未落地，本次只是先把默认可视化后端切到 RC26 自己的 RViz 入口

## 当前注意点

- 为避免父仓库内出现嵌套 git 仓库，导入后已移除 `src/rc26_xhu_viewer/.git`
- bringup 默认参数已切到 `visualization_backend:=rc26_xhu_viewer`，并新增 `visualization_layout:=operator|engineering|diagnostic`
- 兼容别名 `visualization_backend:=rviz` 仍可用，但只会转发到 `rc26_xhu_viewer`；文档口径不再把它当正式后端
- 本次预设主要把旧 `bringup/rviz/*.rviz` 的显示能力固化成 6 份 RC26 preset，后续如果要继续扩专用 Display/Panel，应优先在 `src/rc26_xhu_viewer/rc26_xhu_viewer/` 内演进，而不是再改回 `rc26_bringup/rviz/*.rviz`
