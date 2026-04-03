# rc26_topo_nav

## 模块定位

`rc26_topo_nav` 是 R2 当前唯一的导航表达层，负责把 node/task/route 目标转换成 topo route 与语义 corridor，并驱动自研执行链完成单边执行。

## 当前实现

- Action Server: `navigate_topo_target`
- 运行时仍通过 `graph_file` 加载静态 topo YAML，不在节点启动时动态从点云或地图自动建图
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
- 共享几何真源来自 [r2_mf_world.yaml](/home/potato/RC_2026/src/rc26_kfs_keepout/config/r2_mf_world.yaml)，只负责 MF 块位姿、高度和基础场地区域事实
- topo 语义补充来自 [r2_field_graph_overlay.yaml](/home/potato/RC_2026/src/rc26_topo_nav/config/r2_field_graph_overlay.yaml)，只负责入口/出口 staging、坡道点、任务/路线，以及无法从几何稳定推导的节点和边成本
- 当前 v1 只覆盖 MF 主区与入口/出口坡道链路，不负责把任意 `.pcd` 地图自动变成任意 3D 导航点网络

## 维护方式

- 生成命令:
  - `python3 src/rc26_topo_nav/scripts/generate_graph.py --world-layout src/rc26_kfs_keepout/config/r2_mf_world.yaml --overlay src/rc26_topo_nav/config/r2_field_graph_overlay.yaml --team blue --out src/rc26_topo_nav/config/r2_field_graph_blue.yaml`
  - `python3 src/rc26_topo_nav/scripts/generate_graph.py --world-layout src/rc26_kfs_keepout/config/r2_mf_world.yaml --overlay src/rc26_topo_nav/config/r2_field_graph_overlay.yaml --team red --out src/rc26_topo_nav/config/r2_field_graph_red.yaml`
- 一致性检查:
  - `python3 src/rc26_topo_nav/scripts/generate_graph.py --world-layout src/rc26_kfs_keepout/config/r2_mf_world.yaml --overlay src/rc26_topo_nav/config/r2_field_graph_overlay.yaml --team blue --out src/rc26_topo_nav/config/r2_field_graph_blue.yaml --check-existing`
  - `python3 src/rc26_topo_nav/scripts/generate_graph.py --world-layout src/rc26_kfs_keepout/config/r2_mf_world.yaml --overlay src/rc26_topo_nav/config/r2_field_graph_overlay.yaml --team red --out src/rc26_topo_nav/config/r2_field_graph_red.yaml --check-existing`
- 图合法性检查:
  - `python3 src/rc26_topo_nav/scripts/validate_graph.py src/rc26_topo_nav/config/r2_field_graph_blue.yaml src/rc26_topo_nav/config/r2_field_graph_red.yaml`

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
  - `topo_sim_server.py`：把 topo 图、包内 `sim_assets` 下的 Gazebo world / KFS 对齐配置，以及只读运行时状态整理成 HTTP / WebSocket adapter
  - `sim_viewer`：基于 Babylon.js / React 将完整 mesh 场景、路径、关键节点、open set、扩展树和候选轨迹渲染成可交互的 3D 战术沙盘页面。
- viewer 当前支持：
  - 离线 A* / RRT / DWA 回放
  - `orbit / follow / first_person / top_ortho / side_perspective` 多视角切换；侧视现在使用更低、更近的透视机位，避免继续像高空投影图
  - 工业战术沙盘风格的高级 UI 界面，以及全中文化的字段与控制图例
  - viewer 首屏默认优先展示场地主体；`graph / keyNodes / openSet / expanded / tree / candidates` 默认关闭，减少未生成运行前的 topo 节点干扰
  - 起点绿色、目标点红色、路径蓝色的三维路径层
  - 显式“在场景中设起点 / 设目标”模式：浏览器点击场地任意位置后，会吸附到最近 topo 节点，再写回 `start_node / goal_node`
  - 只读 live ROS 观察：`/topo_nav/route`、`/topo_nav/corridor`、`/xhu_nav/active_edge`、`/xhu_nav/semantic_gate`、`/mf_block_overlay`、`/xhu_nav/tracking_state`
- 当前实现特别注意把“完整渲染几何”和“规划碰撞 keep-out”分开：
  - viewer 会尽量显示完整 world mesh
  - world 面片现在保留受光和阴影层次，减少“只有颜色分区、没有体积感”的平面观感
  - RRT / DWA 的平面 keep-out 只取围栏等竖向障碍和 block overlay，不把可通行平台表面错误地当成二维障碍物

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
- 不负责底层速度控制求解
- 只对接 topo/xhu 自研执行接口
- 不拥有场地几何真源；图几何事实由 `rc26_kfs_keepout/config/r2_mf_world.yaml` 提供
- `sim_viewer` 和 `topo_sim_server.py` 是当前包附带的本地观测工具，不是运行时导航权威
