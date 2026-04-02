# rc26_topo_nav 3D 仿真 Viewer

## 1. 工程定位

`src/rc26_topo_nav/sim_viewer` 是围绕 `rc26_topo_nav` 建的本地三维观测工具，用来把 topo 图搜索过程、Gazebo 场地几何和只读运行时状态放到同一个 WebGL 页面里观察。

- 它是本地工具，不是机器人运行时后端。
- 它的实时模式只读消费 adapter 输出，不拥有任何控制权。
- A* 的真逻辑仍然来自 `rc26_topo_nav` C++ planner，不在前端里复制一套规划真源。

## 2. 入口与链路

- 前端入口
  - [src/main.tsx](/home/potato/RC_2026/src/rc26_topo_nav/sim_viewer/src/main.tsx)
  - [src/App.tsx](/home/potato/RC_2026/src/rc26_topo_nav/sim_viewer/src/App.tsx)
  - [src/components/SceneCanvas.tsx](/home/potato/RC_2026/src/rc26_topo_nav/sim_viewer/src/components/SceneCanvas.tsx)
  - [src/store.ts](/home/potato/RC_2026/src/rc26_topo_nav/sim_viewer/src/store.ts)
  - [src/api.ts](/home/potato/RC_2026/src/rc26_topo_nav/sim_viewer/src/api.ts)
- 本地 adapter
  - [scripts/topo_sim_server.py](/home/potato/RC_2026/src/rc26_topo_nav/scripts/topo_sim_server.py)
  - [scripts/topo_sim_algorithms.py](/home/potato/RC_2026/src/rc26_topo_nav/scripts/topo_sim_algorithms.py)
- 运行时 / 几何真源
  - [src/planner.cpp](/home/potato/RC_2026/src/rc26_topo_nav/src/planner.cpp)
  - [src/planner_trace_cli.cpp](/home/potato/RC_2026/src/rc26_topo_nav/src/planner_trace_cli.cpp)
  - [r2_field_graph_blue.yaml](/home/potato/RC_2026/src/rc26_topo_nav/config/r2_field_graph_blue.yaml)
  - [robocon2026_v2_aligned.world](/home/potato/RC_2026/RC_Sim_001_github/src/rc01_world/worlds/robocon2026_v2_aligned.world)
  - [kfs_config_v2_aligned.yaml](/home/potato/RC_2026/RC_Sim_001_github/src/rc01_kfs_manager/config/kfs_config_v2_aligned.yaml)

## 3. 当前能力

- 完整三维场景
  - 通过 `render_graph_sim_html.py` 里的世界解析链抽取 world / dae 面片，在页面中以 mesh 面而不是线框渲染。
  - Three.js 场景包含环境背景、方向光、阴影和 surface 材质。
- 路径与算法过程
  - 起点绿色、目标点红色、规划路径蓝色。
  - 可选显示关键节点、open set、已扩展节点、RRT 树段和 DWA 候选轨迹。
  - A* 复用 C++ runtime planner trace；RRT / DWA 是本地仿真算法，但输出同一套帧结构。
- 交互相机
  - 支持 `orbit`、`follow`、`first_person`、`top_ortho`、`side_ortho`。
  - 支持鼠标旋转、平移、滚轮缩放和视角 gizmo。
- 只读 live 模式
  - 通过 `topo_sim_server.py` 订阅 `/topo_nav/route`、`/topo_nav/corridor`、`/xhu_nav/active_edge`、`/xhu_nav/semantic_gate`、`/mf_block_overlay`、`/xhu_nav/tracking_state`。
  - 页面只做观察，不回写 ROS2，不修改规划状态。

## 4. 启动方式

- 构建 ROS 包
  - `MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 --packages-select rc26_topo_nav`
- 启动 adapter
  - `python3 src/rc26_topo_nav/scripts/topo_sim_server.py`
- 前端开发态
  - `cd src/rc26_topo_nav/sim_viewer && npm run dev`
- 前端构建态
  - `cd src/rc26_topo_nav/sim_viewer && npm run build`

开发态由 Vite 把 `/api` 和 `/ws` 代理到 `127.0.0.1:8796`。构建态下，如果 `sim_viewer/dist` 已存在，`topo_sim_server.py` 会直接返回该静态页面。

## 5. 当前边界

- 它不是运行时导航权威，`rc26_topo_nav` 节点和相关 topic 才是。
- 它不是浏览器直接控车入口，live 模式只读。
- 它不改写 topo 图、world 或 KFS 真源。
- “显示完整 3D 几何”不等于“所有 mesh 面都自动转成规划障碍”。

当前实现里，完整 world mesh 用于渲染；RRT / DWA 的 keep-out 只取围栏等竖向障碍和 block overlay，避免把可通行平台表面错误地当成二维障碍。
