# rc26_topo_nav 3D 仿真 Viewer

## 1. 工程定位

`src/rc26_topo_nav/sim_viewer` 是围绕 `rc26_topo_nav` 建的本地三维观测工具，用来把 topo 图搜索过程、Gazebo 场地几何和只读运行时状态放到同一个 WebGL / WebGPU 页面里观察。

- 它是本地工具，不是机器人运行时后端。
- 它的实时模式只读消费 adapter 输出，不拥有任何控制权。
- A* 的真逻辑仍然来自 `rc26_topo_nav` C++ planner，不在前端里复制一套规划真源。
- **视觉风格**：采用工业战术沙盘风格，UI 和图形元素均已中文化，强化“观察台”的层级与质感。

## 2. 入口与链路

- 前端入口
  - [src/main.tsx](/home/potato/RC_2026/src/rc26_topo_nav/sim_viewer/src/main.tsx)
  - [src/App.tsx](/home/potato/RC_2026/src/rc26_topo_nav/sim_viewer/src/App.tsx)
  - [src/components/SceneCanvas.tsx](/home/potato/RC_2026/src/rc26_topo_nav/sim_viewer/src/components/SceneCanvas.tsx) (Babylon.js 渲染核心)
  - [src/store.ts](/home/potato/RC_2026/src/rc26_topo_nav/sim_viewer/src/store.ts)
  - [src/api.ts](/home/potato/RC_2026/src/rc26_topo_nav/sim_viewer/src/api.ts)
  - [src/labels.ts](/home/potato/RC_2026/src/rc26_topo_nav/sim_viewer/src/labels.ts) (中文映射集)
- 本地 adapter
  - [scripts/topo_sim_server.py](/home/potato/RC_2026/src/rc26_topo_nav/scripts/topo_sim_server.py)
  - [scripts/topo_sim_algorithms.py](/home/potato/RC_2026/src/rc26_topo_nav/scripts/topo_sim_algorithms.py)
  - [docs/test/e2e/topo_sim_stub_server.py](/home/potato/RC_2026/docs/test/e2e/topo_sim_stub_server.py) (浏览器 E2E 使用的 stub backend)
- 运行时 / 几何真源
  - [src/planner.cpp](/home/potato/RC_2026/src/rc26_topo_nav/src/planner.cpp)
  - [src/planner_trace_cli.cpp](/home/potato/RC_2026/src/rc26_topo_nav/src/planner_trace_cli.cpp)
  - [r2_field_graph_blue.yaml](/home/potato/RC_2026/src/rc26_topo_nav/config/r2_field_graph_blue.yaml)
  - [robocon2026_v2_aligned.world](/home/potato/RC_2026/src/rc26_topo_nav/sim_assets/worlds/robocon2026_v2_aligned.world)
  - [kfs_config_v2_aligned.yaml](/home/potato/RC_2026/src/rc26_topo_nav/sim_assets/config/kfs_config_v2_aligned.yaml)
  - [robocon2026_world/model.sdf](/home/potato/RC_2026/src/rc26_topo_nav/sim_assets/models/robocon2026_world/model.sdf)

## 3. 当前能力

- 完整三维场景
  - 通过 `render_graph_sim_html.py` 里的世界解析链抽取 world / dae 面片，在页面中以 mesh 面而不是线框渲染。
  - 使用 Babylon.js 引擎，优先尝试 WebGPU 渲染，自动回退到 WebGL。场景包含环境背景、方向光、阴影和 PBR 材质。
- 路径与算法过程
  - 起点绿色、目标点红色、规划路径蓝色。
  - 可选显示关键节点、open set、已扩展节点、RRT 树段和 DWA 候选轨迹。
  - A* 复用 C++ runtime planner trace；RRT / DWA 是本地仿真算法，但输出同一套帧结构。
  - 页面现在默认采用“场景优先”首屏：初始会先展示场地与基础路径语义，`graph / keyNodes / openSet / expanded / tree / candidates` 图层默认关闭，避免未生成运行时被 topo 节点球体淹没。
  - 页面默认不会自动生成或自动播放离线运行；只有用户手动设置起点/目标后点击“生成手动离线运行”，播放/单步/重置才会对该离线回放生效。
  - 场景点击不是把浏览器点击点直接下发为任意坐标目标；前端会把点击到的场地位置吸附到最近 topo 节点，再回写到 `start_node` / `goal_node` 表单字段，保持后端契约仍然是 topo-native 目标。
- 交互相机
  - 支持 `orbit / follow / first_person / top_ortho / side_perspective` 多视角；`side_perspective` 现在使用更低、更近的斜侧机位，不再像高空投影图。
  - 支持鼠标旋转、平移、滚轮缩放。
- 立体感增强
  - topo 节点、起点、目标点和选点预览都使用带立柱的 pin marker，而不是贴地圆点；其中静态 topo 图节点已改成更低、更细的弱化标记，避免抢占场地主体视觉。
  - 图边改为细 tube，而不是单纯线段，侧视更容易看出高度和前后关系。
- 只读 live 模式
  - 通过 `topo_sim_server.py` 订阅 `/topo_nav/route`、`/topo_nav/corridor`、`/xhu_nav/active_edge`、`/xhu_nav/semantic_gate`、`/mf_block_overlay`、`/xhu_nav/tracking_state`。
  - 页面只做观察，不回写 ROS2，不修改规划状态。
- 包内资产
  - viewer 默认依赖 `src/rc26_topo_nav/sim_assets` 里的最小保留资产，不再要求外部 `RC_Sim_001_github` 目录继续存在。
  - 当前保留集只包含 viewer/server 所需的 `.world`、KFS 对齐配置，以及 `robocon2026_world` 模型。
- 测试与交付链路
  - 前端 API 现在支持通过 `VITE_API_BASE_URL` 和 `VITE_WS_BASE_URL` 切换 HTTP / WebSocket 后端来源，让浏览器 E2E 可以稳定改连 stub backend，而不再强依赖 Vite dev proxy。
  - 离线运行创建后，页面会先使用 `POST /api/runs` 的返回值写入 `runId / state / frameCount / summary`；后续 `播放 / 暂停 / 单步 / 重置` 也会先使用 `POST /api/runs/{id}/control` 的返回值同步 `state / cursor`，随后再由 run WebSocket 补齐首帧与后续状态；这样浏览器控制区不会被首条 `meta` 或首条 `frame` 的时序抖动卡住。
  - 根仓库新增 [docs/test/README.md](/home/potato/RC_2026/docs/test/README.md) 作为测试入口，收口 `npm run preflight`、`npm run test:e2e`、`npm run cd:package` 以及 `.github/workflows/ci.yml`、`.github/workflows/cd.yml`。

## 4. 启动方式

- 根目录一键启动
  - `./start_r2_topo_nav_sim.sh`
  - 这条入口会先增量构建 `rc26_topo_nav`，并按需补齐 `sim_viewer/dist`，启动 `topo_sim_server.py` 后自动打开浏览器，适合本地直接联调 viewer
- 构建 ROS 包
  - `MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 --packages-select rc26_topo_nav`
- source 运行时环境
  - `source install/setup.bash`
- 启动 adapter
  - `python3 src/rc26_topo_nav/scripts/topo_sim_server.py`
- 默认资源路径
  - `src/rc26_topo_nav/sim_assets/worlds/robocon2026_v2_aligned.world`
  - `src/rc26_topo_nav/sim_assets/config/kfs_config_v2_aligned.yaml`
- 前端开发态
  - `cd src/rc26_topo_nav/sim_viewer && npm run dev`
- 前端构建态
  - `cd src/rc26_topo_nav/sim_viewer && npm run build`
- 浏览器 E2E
  - `npm run test:e2e`
- 本地预演
  - `npm run preflight`
  - `npm run preflight:strict`

开发态由 Vite 把 `/api` 和 `/ws` 代理到 `127.0.0.1:8796`。构建态下，如果 `sim_viewer/dist` 已存在，`topo_sim_server.py` 会直接返回该静态页面。

浏览器 E2E 默认不直接连接真实 `topo_sim_server.py`，而是通过 `docs/test/e2e/topo_sim_stub_server.py` 提供最小 HTTP / WebSocket 契约面，再以 `VITE_API_BASE_URL` 和 `VITE_WS_BASE_URL` 重建 `sim_viewer` preview。这么做的目的不是替代真实 planner 联调，而是让 CI 能稳定覆盖“页面能否加载、离线运行能否创建、单步是否推进、live 状态是否能显示”这条浏览器真链路。

当前 `SceneCanvas` 会在 `scene-manifest` 真正返回、`canvas` 已经挂载后再初始化 Babylon 引擎；开发态下也会忽略 React `StrictMode` 触发的过期异步初始化，避免首屏停留在“场景已加载”但中间画布仍为空白的状态。为了让比赛场地在浏览器里既保留颜色也保留立体感，当前 Babylon 画布使用不透明背景清屏，world 面片改为保留受光材质与阴影层次，而不是继续做成偏平的纯色贴图面。

当前 viewer 的手动交互口径也已明确：

- 离线模式默认空闲，不再自动播“演示 run”。
- 用户既可以在左侧表单选 `start_node / goal_node / goal_task / goal_route`，也可以显式开启“在场景中设起点 / 设目标”模式。
- 场景选点只负责吸附到最近 topo 节点，并把结果同步回表单；这让前端看起来像“点场地”，但后端 API 仍然只接收 topo 图里的节点/任务/路线语义。
- `topo_sim_server.py` 的 live 线程现在会在 `rclpy` 外部关闭时安静退出，不再因为 `ExternalShutdownException` 在服务关闭时打出一段无意义 traceback。

## 5. 当前边界

- 它不是运行时导航权威，`rc26_topo_nav` 节点和相关 topic 才是。
- 它不是浏览器直接控车入口，live 模式只读。
- 它不改写 topo 图、world 或 KFS 真源。
- “显示完整 3D 几何”不等于“所有 mesh 面都自动转成规划障碍”。

当前实现里，完整 world mesh 用于渲染；RRT / DWA 的 keep-out 只取围栏等竖向障碍和 block overlay，避免把可通行平台表面错误地当成二维障碍。
