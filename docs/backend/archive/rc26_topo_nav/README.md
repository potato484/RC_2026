# rc26_topo_nav

## 模块定位

`rc26_topo_nav` 是 R2 当前唯一的导航表达层，负责把 node/task/route 目标转换成 topo route 与语义 corridor，并驱动自研执行链完成单边执行。

## 当前实现

- Action Server:
  - `navigate_topo_target`
  - `navigate_surface_route`
- 运行时仍通过静态 YAML 建图，不在节点启动时动态从点云或地图自动建图：
  - `graph_file` 负责 topo 节点 / route / task 规划
  - `surface_graph_file` 负责任意点 3D 路线规划与执行
- `topo_nav_node` 当前会在 ROS 日志里输出每次 topo / surface 规划的耗时、目标和成功/失败结果，便于现场判断“发布目标后到底卡在规划还是执行”
- 发布:
  - `/topo_nav/route`
  - `/topo_nav/corridor`
  - `/xhu_nav/corridor_cmd`
  - `/xhu_nav/active_edge`
  - `/xhu_nav/semantic_gate`
  - `/xhu_nav/diagnostics`
- 订阅:
  - `/localization/health`
  - `/localization/backend_status`
  - `/localization/route_observability`
  - `/mf_block_overlay`
  - `/xhu_nav/tracking_state`
  - 地形与 base_ground 相关输入
- 服务客户端:
  - `set_xhu_motion_mode`

## 任意点 3D 路线当前口径

- `navigate_surface_route` 面向浏览器或上游模块传入的世界坐标起点/终点。
- 当前实现不是直接在 mesh 上插临时锚点，而是把点击点投影到最近可通行 `surface_graph` sample node，再复用现有 `FieldGraph + A* + XhuSemanticCorridor` 执行主链。
- 运行时要求机器人已经足够接近投影后的起点；当前不会从机器人当前位置自动补一段“接驳到起点”的前置路径。
- 规划成功后会发布完整 `planned_path` 到 `/topo_nav/route`，并按 surface segment 逐段发布 corridor 到 `/topo_nav/corridor` 与 `/xhu_nav/corridor_cmd`。

## 执行链当前口径

- `edge_executor` 负责两件事：
  - 切换 xhu 运动模式
  - 发布 `XhuSemanticCorridor` 并等待 `XhuTrackingState`

## 源码入口

- [src/topo_nav_node.cpp](/home/potato/RC_2026/src/rc26_topo_nav/src/topo_nav_node.cpp)
- [src/edge_executor.cpp](/home/potato/RC_2026/src/rc26_topo_nav/src/edge_executor.cpp)
- [src/planner.cpp](/home/potato/RC_2026/src/rc26_topo_nav/src/planner.cpp)
- [src/planner_trace_cli.cpp](/home/potato/RC_2026/src/rc26_topo_nav/src/planner_trace_cli.cpp)
- [scripts/topo_sim_server.py](/home/potato/RC_2026/src/rc26_topo_nav/scripts/topo_sim_server.py)
- [scripts/topo_sim_algorithms.py](/home/potato/RC_2026/src/rc26_topo_nav/scripts/topo_sim_algorithms.py)
- [sim_viewer/src/App.tsx](/home/potato/RC_2026/src/rc26_topo_nav/sim_viewer/src/App.tsx)
- [sim_assets/worlds/robocon2026_v2_aligned.world](/home/potato/RC_2026/src/rc26_topo_nav/sim_assets/worlds/robocon2026_v2_aligned.world)
- [sim_assets/config/kfs_config_v2_aligned.yaml](/home/potato/RC_2026/src/rc26_topo_nav/sim_assets/config/kfs_config_v2_aligned.yaml)
- [sim_assets/models/robocon2026_world/model.sdf](/home/potato/RC_2026/src/rc26_topo_nav/sim_assets/models/robocon2026_world/model.sdf)

## 图配置口径

- `config/r2_field_graph_blue.yaml` 和 `config/r2_field_graph_red.yaml` 现在是离线生成产物
- `config/r2_surface_graph_blue.yaml` 和 `config/r2_surface_graph_red.yaml` 是 dense 可通行表面图生成产物，覆盖比赛场地中的地面、坡面与阶梯可行走表面 sample
- 共享几何真源来自 [r2_mf_world.yaml](/home/potato/RC_2026/src/rc26_kfs_keepout/config/r2_mf_world.yaml)，只负责 MF 块位姿、高度和基础场地区域事实
- topo 语义补充来自 [r2_field_graph_overlay.yaml](/home/potato/RC_2026/src/rc26_topo_nav/config/r2_field_graph_overlay.yaml)，只负责入口/出口 staging、坡道点、任务/路线，以及无法从几何稳定推导的节点和边成本
- surface graph 语义补充来自 [r2_surface_graph_overlay.yaml](/home/potato/RC_2026/src/rc26_topo_nav/config/r2_surface_graph_overlay.yaml)，只负责定义哪些 mesh 面可采样、采样密度和跨面连接阈值
- 当前 v1 不负责把任意 `.pcd` 地图自动变成任意 3D 导航点网络；dense surface graph 仍然基于仓库内固定比赛场地 world 生成

## 维护方式

- 生成命令:
  - `python3 src/rc26_topo_nav/scripts/generate_graph.py --world-layout src/rc26_kfs_keepout/config/r2_mf_world.yaml --overlay src/rc26_topo_nav/config/r2_field_graph_overlay.yaml --team blue --out src/rc26_topo_nav/config/r2_field_graph_blue.yaml`
  - `python3 src/rc26_topo_nav/scripts/generate_graph.py --world-layout src/rc26_kfs_keepout/config/r2_mf_world.yaml --overlay src/rc26_topo_nav/config/r2_field_graph_overlay.yaml --team red --out src/rc26_topo_nav/config/r2_field_graph_red.yaml`
- 一致性检查:
  - `python3 src/rc26_topo_nav/scripts/generate_graph.py --world-layout src/rc26_kfs_keepout/config/r2_mf_world.yaml --overlay src/rc26_topo_nav/config/r2_field_graph_overlay.yaml --team blue --out src/rc26_topo_nav/config/r2_field_graph_blue.yaml --check-existing`
  - `python3 src/rc26_topo_nav/scripts/generate_graph.py --world-layout src/rc26_kfs_keepout/config/r2_mf_world.yaml --overlay src/rc26_topo_nav/config/r2_field_graph_overlay.yaml --team red --out src/rc26_topo_nav/config/r2_field_graph_red.yaml --check-existing`
- 图合法性检查:
  - `python3 src/rc26_topo_nav/scripts/validate_graph.py src/rc26_topo_nav/config/r2_field_graph_blue.yaml src/rc26_topo_nav/config/r2_field_graph_red.yaml`
- surface graph 生成命令:
  - `python3 src/rc26_topo_nav/scripts/generate_surface_graph.py --team blue --world src/rc26_topo_nav/sim_assets/worlds/robocon2026_v2_aligned.world --overlay src/rc26_topo_nav/config/r2_surface_graph_overlay.yaml --out src/rc26_topo_nav/config/r2_surface_graph_blue.yaml`
  - `python3 src/rc26_topo_nav/scripts/generate_surface_graph.py --team red --world src/rc26_topo_nav/sim_assets/worlds/robocon2026_v2_aligned.world --overlay src/rc26_topo_nav/config/r2_surface_graph_overlay.yaml --out src/rc26_topo_nav/config/r2_surface_graph_red.yaml`

## 静态可视化

- `render_graph_html.py` 可以把当前 `graph_file` 直接渲染成单文件 HTML，离线查看 topo 算法真正使用的 `nodes / edges / tasks / routes`
- 这个工具不依赖 ROS 节点启动、不依赖实机，也不依赖 RViz 运行态 topic；适合静态检查图是否建对、路线是否串对、任务候选点是否合理
- 生成命令:
  - `python3 src/rc26_topo_nav/scripts/render_graph_html.py --graph src/rc26_topo_nav/config/r2_field_graph_blue.yaml --out /tmp/r2_field_graph_blue.html`
  - `python3 src/rc26_topo_nav/scripts/render_graph_html.py --graph src/rc26_topo_nav/config/r2_field_graph_red.yaml --out /tmp/r2_field_graph_red.html`
- 页面能力:
  - 节点颜色区分 `mf_edge_pose / staging / ramp_entry / ramp_exit`
  - 边颜色区分 `plane_move / ramp_up / ramp_down`
  - 点击 route 可高亮预设链路
  - 点击 task 可高亮候选节点
  - 点击节点或边可直接查看完整字段

## 仿真场地联动可视化

- `render_graph_sim_html.py` 会把 topo 图和 `rc26_topo_nav` 包内保留的 Gazebo 场地资产对齐到同一张离线 HTML 页面
- 对齐方式不是人工写死偏移，而是用 `kfs_config_v2_aligned.yaml` 里的 `meilin.<team>` 坐标自动拟合 `graph_file` 的 MF block 网格，再把 staging / ramp 节点按同一平移落到仿真世界里
- 页面除了顶视图叠图，还会离线复现 planner 当前真实代价逻辑：
  - priority queue 扩展顺序
  - 每次 edge relax / keep-best / blocked 的步骤
  - 最终路径的 `motion_type` 和 `z / height_change` 语义剖面
- Gazebo 场地不是再画抽象框，而是直接投影 `robocon2026.dae` 的真实面片；但页面会主动过滤 `柱体 / 杆架 / 细碎竖向小面` 这类噪声，只保留更容易看懂比赛场地颜色分区、台面和围栏轮廓的部分
- 页面的人机可读层现在优先用中文说明：
  - `motion_type / node_type / goal_kind / frame type` 都会显示中文解释
  - `node_id / edge_id / task_tag / route_tag` 不再只裸露英文变量，而是先显示中文含义，再附原始 ID 供开发排查
  - `高度变化 dZ / 本段代价 / 累计代价` 等字段会直接在页面上解释是什么意思
- 这个工具仍然是离线观察工具，不改变 `rc26_topo_nav` 运行时只加载静态 `graph_file` 的边界
- 当前最小可保留资产已经内置在 [sim_assets](/home/potato/RC_2026/src/rc26_topo_nav/sim_assets) 下，默认路径不再依赖外部 `RC_Sim_001_github`
- 生成命令:
  - `python3 src/rc26_topo_nav/scripts/render_graph_sim_html.py --graph src/rc26_topo_nav/config/r2_field_graph_blue.yaml --world src/rc26_topo_nav/sim_assets/worlds/robocon2026_v2_aligned.world --kfs-config src/rc26_topo_nav/sim_assets/config/kfs_config_v2_aligned.yaml --out /tmp/r2_field_graph_blue_sim.html`
  - `python3 src/rc26_topo_nav/scripts/render_graph_sim_html.py --graph src/rc26_topo_nav/config/r2_field_graph_red.yaml --world src/rc26_topo_nav/sim_assets/worlds/robocon2026_v2_aligned.world --kfs-config src/rc26_topo_nav/sim_assets/config/kfs_config_v2_aligned.yaml --out /tmp/r2_field_graph_red_sim.html`
- 常用观察参数:
  - `--start <node_id> --goal-node <node_id>`: 看单点到单点的搜索过程
  - `--goal-task <task_tag>`: 看 task 候选点比较和最终选中结果
  - `--blocked-node <id>` / `--blocked-edge <id>`: 模拟动态阻塞后的规划变化

## 3D 仿真 viewer

- 当前包新增了一个本地 3D 仿真工具链：
  - `planner_trace_cli`：把 C++ planner 当前真实搜索过程导出为 JSON trace，作为 A* 观察真源
  - `surface_route_cli`：把浏览器点击的世界坐标起终点投影到 dense `surface_graph` 上，得到运行时真实会走的起终点节点和完整路径
  - `topo_sim_server.py`：把 `surface_route_cli + planner_trace_cli + sim_assets` 组合成 HTTP adapter，提供场景、路线预览、路线 trace 和可选执行接口
  - `sim_viewer`：基于 Babylon.js / React 的单用途 3D 路线观察台，只显示任意点路线和它的搜索推导过程
- viewer 当前支持：
  - 浏览器直接在地面、坡面、阶梯表面设置起点和终点
  - `POST /api/surface-route/preview` 现在先返回：
    - 投影后的起点/终点 pose
    - 投影后的起点/终点 node id
    - 完整 3D 路径点
    - 路径语义分段
    - `surface_route_cli` 的结构化规划日志与耗时
    - 其中前端主指标 `完整规划时间` 当前固定对应 `surface_route_cli.timing_ms.completePlanning`
  - `POST /api/surface-route/trace-from-nodes` 再基于 preview 已得到的投影 node id 后台补充：
    - `planner_trace_cli` 的结构化回放日志与耗时
    - 逐帧 `PlannerFrame` 搜索过程
    - 回放涉及节点的 `node_poses`
  - `POST /api/surface-route/trace` 仍保留为兼容接口，但内部已经改成 `preview -> trace-from-nodes` 组合调用；它返回：
    - 投影后的起点/终点 pose
    - 投影后的起点/终点 node id
    - 完整 3D 路径点
    - 路径语义分段
    - `surface_route_cli / planner_trace_cli` 的结构化规划日志与耗时
    - 其中精确 timing 现在进一步拆成：
      - `surface_route_cli.timing_ms.projection`: 点击点投影到 `surface_graph` 的耗时
      - `surface_route_cli.timing_ms.routePlanning`: 图上路径搜索的耗时
      - `surface_route_cli.timing_ms.pathExpand`: 从 node/edge 路径展开成完整三维路径点的耗时
      - `surface_route_cli.timing_ms.segmentBuild`: 根据 motion_type / required_mode 生成语义分段的耗时
      - `surface_route_cli.timing_ms.completePlanning`: 从请求世界坐标到完整可执行路径与分段就绪的总耗时
      - `planner_trace_cli.timing_ms.planning`: 生成搜索回放时 A* 真正执行的耗时
    - 逐帧 `PlannerFrame` 搜索过程
  - 当前 dense `surface_graph` 的真实运行时规划已经从 trace 捕获路径里拆出：
    - `planRoute / planToTask / planRouteTag` 不再通过 `planRouteTrace(...).result` 间接执行
    - `surface_route_cli` 会按当前 `surface_graph` 的最小边代价密度自动估算可采纳启发式，避免真实 surface 路径规划继续退化成纯 Dijkstra
    - 以同一组点击点 spot check，`surface_route_cli.timing_ms.routePlanning` 已从约 `717 ms` 降到约 `17.6 ms`
  - 页面只保留一个滑块回放 trace，不再提供 play/pause/step/reset
  - 页面只保留 `scene / frontier / expanded / shadows` 四个图层开关，以及 `orbit / top_ortho / side_perspective` 三个视角
  - 页面现在改成“画布优先 + 摘要条 + 下方双列检视器”结构：
    - 摘要条和画布右下优先显示 `完整规划时间 / 投影耗时 / 路径搜索耗时 / 路径展开耗时 / 分段生成耗时`
    - `网页预览链路 / 回放链路` 继续保留，但只作为 Web 侧诊断耗时
    - 左列固定放 `搜索回放 + 规划日志`
    - 右列固定放 `当前路线 + 规划时间拆解 + 路线分段`
    - 这样宽屏下不会再出现左下大块空白，同时浏览器会先显示 preview 路线，再后台补回放
    - 画布右上还会固定显示颜色图例，明确说明蓝色圆点是 `前沿点`、黄色方块是 `已探查点`
  - 当前 viewer 用户可见文案已经统一中文化：
    - `N/A` 改成 `暂无`
    - `surface_route_cli / planner_trace_cli / A*` 不再直接暴露给现场用户
    - 回放 metrics 的 `gCost / fCost / stepCost` 会在前端映射成中文
    - preview/trace 返回的 `projected_start_node_id / projected_goal_node_id / segment.from_node_id / segment.to_node_id` 现在会在前端映射成中文节点名显示
    - preview/trace 的失败码、失败原因，以及浏览器侧请求异常，前端都会优先转成中文提示，不再直接向用户暴露英文异常文本
  - `POST /api/surface-route/execute` 仍可选下发 `navigate_surface_route`，但执行前提不变：机器人已经接近被点击的起点
- 当前实现特别注意把“完整渲染几何”和“规划碰撞 keep-out”分开：
  - viewer 会尽量显示完整 world mesh
  - world 面片现在保留受光和阴影层次，减少“只有颜色分区、没有体积感”的平面观感；`scene-manifest` 还会为 viewer 额外保留结构性竖直面，恢复梅林阶梯和平台侧壁的连接体积感
  - 这条“保留结构性竖直面”的逻辑只服务 Babylon 3D viewer；`render_graph_sim_html.py` 的离线 2D HTML 仍维持投影简化，不把这些竖直面重新变成难读的 SVG 线束
  - 当前 surface trace 仍然严格复用现有 planner，而不是在前端复制一套规划逻辑

## 本地启动方式

- 根目录便捷脚本:
  - `./start_r2_topo_nav_sim.sh`
  - 脚本会先增量构建 `rc26_topo_nav`，并按需补齐 `sim_viewer/dist`，拉起 `topo_sim_server.py`，等待 `/api/health` 就绪后自动打开浏览器；默认前台保活，`Ctrl+C` 一次性关闭
- 包构建:
  - `MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 --packages-select rc26_topo_nav`
- 启动 adapter:
  - `source install/setup.bash`
  - `python3 src/rc26_topo_nav/scripts/topo_sim_server.py`
- 默认 asset:
  - `topo_sim_server.py` 默认读取 `src/rc26_topo_nav/sim_assets/worlds/robocon2026_v2_aligned.world`
  - `topo_sim_server.py` 默认读取 `src/rc26_topo_nav/sim_assets/config/kfs_config_v2_aligned.yaml`
- 前端开发态:
  - `cd src/rc26_topo_nav/sim_viewer && npm run dev`
- 前端静态构建:
  - `cd src/rc26_topo_nav/sim_viewer && npm run build`
- 当前 `dist/` 构建完成后，`topo_sim_server.py` 会优先直接返回该静态页面；开发态仍然推荐走 Vite 代理到 `127.0.0.1:8796`
- 当前根仓库还新增了 `docs/test/` 下的浏览器 E2E / preflight / release 收口脚本；其中 E2E 默认通过 stub backend 覆盖浏览器真交互，不直接依赖真实 ROS2 / planner binary。

## 当前边界

- 负责 topo 图搜索与单边执行调度
- 负责 dense `surface_graph` 上的任意点 3D 路线规划与分段执行
- 不负责底层速度控制求解
- 只对接 topo/xhu 自研执行接口
- 不拥有场地几何真源；图几何事实由 `rc26_kfs_keepout/config/r2_mf_world.yaml` 提供
- `sim_viewer` 和 `topo_sim_server.py` 是当前包附带的本地观测与受控下发工具，不是运行时导航权威；真正执行仍以 `rc26_topo_nav` action server 为准
