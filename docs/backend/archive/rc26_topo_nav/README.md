# rc26_topo_nav

## 模块定位

`rc26_topo_nav` 仍然是 R2 的 topo 图、surface graph、全局路线表达和 `navigate_surface_route` action 真源。

## 当前仍由本包拥有的内容

- topo graph / surface graph 配置
- `planner_trace_cli`
- `surface_route_cli`
- `navigate_surface_route` action 服务端
- `sim_assets/`、`worlds/`、KFS 对齐配置

## 不再由本包拥有的内容

以下 Web 可视化入口已经迁出到 `rc26_xhu_viewer`：

- Babylon.js viewer
- FastAPI / WebSocket adapter
- bringup 的主 Web 可视化入口

因此，`rc26_topo_nav` 当前不再是浏览器入口包，而是被 `rc26_xhu_viewer` 消费的规划与场地数据真源。

## 当前关键文件

- [config/r2_field_graph_blue.yaml](/home/potato/RC_2026/src/rc26_topo_nav/config/r2_field_graph_blue.yaml)
- [config/r2_surface_graph_blue.yaml](/home/potato/RC_2026/src/rc26_topo_nav/config/r2_surface_graph_blue.yaml)
- [sim_assets/worlds/robocon2026_v2_aligned.world](/home/potato/RC_2026/src/rc26_topo_nav/sim_assets/worlds/robocon2026_v2_aligned.world)
- [sim_assets/config/kfs_config_v2_aligned.yaml](/home/potato/RC_2026/src/rc26_topo_nav/sim_assets/config/kfs_config_v2_aligned.yaml)

## 与 rc26_xhu_viewer 的当前关系

- `rc26_xhu_viewer_server.py` 默认仍从本包读取 graph、surface graph、world 和 sim_assets
- Web 端的路线回放仍然依赖本包导出的 `planner_trace_cli` / `surface_route_cli`
- 这次迁移没有改变 planner、action、graph schema 的权威归属，只是把浏览器承载层挪到了 `rc26_xhu_viewer`

## 本次迁移后的注意点

- 本包不再安装 `topo_sim_server.py`，也不再注册原有 `test_topo_sim_server`
- 如果调整 graph/world/surface-route CLI 行为，仍然需要同步检查 `rc26_xhu_viewer/scripts/rc26_xhu_viewer_server.py`
- `test_generate_surface_graph.py` 现在默认只做 checked-in 文件的 manifest 快检、helper 回归和小场景 in-process CLI smoke；
  对两份 60 万行级 dense `surface_graph` 做完整 YAML parse / validate，以及基于 `robocon2026_v2_aligned.world` 的整图再生成回归，都改为显式 opt-in。
- 需要手动设置 `RC26_RUN_FULL_SURFACE_GRAPH_REGEN=1` 后再运行完整重验收，避免普通 `ctest` 每次都重跑多分钟的整图加载与生成。
