# rc26_topo_nav 3D 路线观察台

## 1. 工程定位

`src/rc26_topo_nav/sim_viewer` 当前已经从“多模式拓扑仿真沙盘”收口成单用途工具：只观察比赛场地里任意一点到任意一点的 3D 路线，以及这条路线在当前 A* 算法里的具体推导过程。

- 它是本地观测工具，不是机器人运行时权威后端。
- 它不再暴露 `Topo 节点 / 任意点 3D 路线` 双模式，也不再暴露离线 `RRT / DWA`、live ROS 只读桥接和 run WebSocket 播放控制。
- 浏览器当前只保留三条主链：
  - 在地面、坡面、阶梯表面直接点起点和终点
  - 生成完整 3D 路线
  - 用滑块回看 `surface_graph + planner_trace_cli` 导出的逐帧搜索过程
- 页面仍保留“执行当前路线”按钮，但它只是把当前起终点对应的 surface route 下发给 `navigate_surface_route`；机器人必须已经在起点附近，不会自动补一段接驳路线。

## 2. 入口与链路

- 前端入口
  - [src/main.tsx](/home/potato/RC_2026/src/rc26_topo_nav/sim_viewer/src/main.tsx)
  - [src/App.tsx](/home/potato/RC_2026/src/rc26_topo_nav/sim_viewer/src/App.tsx)
  - [src/components/SceneCanvas.tsx](/home/potato/RC_2026/src/rc26_topo_nav/sim_viewer/src/components/SceneCanvas.tsx)
  - [src/store.ts](/home/potato/RC_2026/src/rc26_topo_nav/sim_viewer/src/store.ts)
  - [src/api.ts](/home/potato/RC_2026/src/rc26_topo_nav/sim_viewer/src/api.ts)
- 本地 adapter
  - [scripts/topo_sim_server.py](/home/potato/RC_2026/src/rc26_topo_nav/scripts/topo_sim_server.py)
- 运行时真源
  - [src/planner_trace_cli.cpp](/home/potato/RC_2026/src/rc26_topo_nav/src/planner_trace_cli.cpp)
  - [src/surface_route_cli.cpp](/home/potato/RC_2026/src/rc26_topo_nav/src/surface_route_cli.cpp)
  - [config/r2_surface_graph_blue.yaml](/home/potato/RC_2026/src/rc26_topo_nav/config/r2_surface_graph_blue.yaml)
  - [config/r2_surface_graph_red.yaml](/home/potato/RC_2026/src/rc26_topo_nav/config/r2_surface_graph_red.yaml)

## 3. 当前真实能力

- 完整三维场景
  - Babylon.js 继续负责渲染比赛场地 mesh、材质、光照与相机。
  - `scene-manifest` 仍保留结构性竖直面，保证侧视能看出梅林阶梯、平台侧壁和坡面体积。
  - 阴影策略维持“结构竖面保留体积阴影，地面/坡面/平台表面不统一接收动态阴影”，避免大面积阴影压暗可通行区域。
- 任意点 3D 路线
  - 页面只允许在场地 mesh 上直接选取世界坐标起点和终点。
  - `POST /api/surface-route/trace` 会先复用 `surface_route_cli` 把点击点投影到最近可通行 surface sample，再复用 `planner_trace_cli` 基于投影后的起终点节点生成 A* 逐帧 trace。
  - 返回结果同时包含：
    - 投影后的起点/终点 pose
    - 投影后的起点/终点 node id
    - 完整 3D 路径点序列
    - 语义分段列表
    - 本次 `surface_route_cli / planner_trace_cli` 的结构化规划日志与耗时
    - 逐帧 `PlannerFrame`
  - 页面右侧当前额外提供“规划日志”卡片，按调用顺序展示浏览器请求、surface 图投影与路径规划、A* 搜索回放导出，以及整条 trace pipeline 的总耗时。
- 搜索回放
  - 页面不再提供 play/pause/step/reset，只保留一个滑块控制当前帧。
  - 当前帧会显示 `frontier / expanded / current best path`，用于观察搜索如何逐步逼近最终路线。
  - 展示层默认按中文显示回放语义；`planner_trace_cli` 原始 message 如 `goal reached`、`better path discovered` 会在前端映射成中文，不再直接透传英文提示。
  - `surface-route` trace 现在继续复用真实 `surface_route_cli -> planner_trace_cli`，但回放传输层已经压缩成：
    - 采样后的 `frames`
    - 本次回放实际涉及到的 `node_poses` 字典
    - 浏览器本地按 `nodeId` 复原 open-set / expanded / best-path 的可视化点位
  - adapter 不再在每次 trace 请求里重新解析整份 dense `surface_graph` YAML，从而避免页面长时间停留在“生成中...”。
  - 顶部只保留 `scene / frontier / expanded / shadows` 四个有意义的图层开关。
- 相机与读图
  - 当前只保留 `orbit / top_ortho / side_perspective` 三个真正有用的观察视角。
  - surface trace 路线现在沿用更细的 tube 半径，并保持轻微抬高和淡 halo，减少侧视时被地表边缘吞没的问题。

## 4. API 口径

`topo_sim_server.py` 当前与这个页面直接相关的接口已经收口为：

- `GET /api/scene-manifest`
- `POST /api/surface-route/preview`
- `POST /api/surface-route/trace`
- `POST /api/surface-route/execute`

其中真正驱动页面主流程的是 `POST /api/surface-route/trace`。

当前这个接口的实现要点是：

- `surface_route_cli` 继续负责把浏览器点击点投影到真实可通行 `surface_graph`
- `planner_trace_cli --max-frames` 继续负责导出真实 A* 搜索过程，但只输出采样后的帧
- `planner_trace_cli` 额外输出当前回放涉及节点的 `node_poses`
- `topo_sim_server.py` 直接复用 preview 阶段已经得到的 `path_points / segments`，不再为 trace 额外重读 surface graph YAML
- `topo_sim_server.py` 现在还会把 `surface_route_cli` 和 `planner_trace_cli` 的耗时、成功/失败状态、起终点投影节点等信息整理成 `planning_logs`，供浏览器直接展示

## 5. 启动与验证

- 构建 ROS 包
  - `MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 --packages-select rc26_interfaces rc26_topo_nav`
- 启动 adapter
  - `source install/setup.bash`
  - `python3 src/rc26_topo_nav/scripts/topo_sim_server.py`
- 前端开发态
  - `cd src/rc26_topo_nav/sim_viewer && npm run dev`
- 前端构建态
  - `cd src/rc26_topo_nav/sim_viewer && npm run build`
- 前端单元测试
  - `cd src/rc26_topo_nav/sim_viewer && npm run test`
- Python 回归测试
  - `python3 -m unittest src/rc26_topo_nav/test/test_topo_sim_server.py`

## 6. 这次收口后的维护备注

- 现在如果再给页面加回 `Topo 节点模式`、`RRT / DWA`、live ROS 观察或 run 控制，就不再是普通 UI 补丁，而是一次功能边界扩张。
- 如果未来需要继续观察“任意点 3D 路线”的算法演绎，优先复用 `surface_route_cli -> planner_trace_cli -> /api/surface-route/trace` 这条链，不要在前端复制第二套规划逻辑。
- 如果以后再出现“生成中...”长期不返回，优先先看 `surface-route` trace 的总 payload、大图 YAML 是否被重复解析，以及 CLI 是否仍然在输出未采样的超大 trace，而不是先怀疑 React 状态机。
- 当前文档反映的真实现状是：`sim_viewer` 已经是一个单用途 3D 路线观察台，而不是通用导航实验场。
