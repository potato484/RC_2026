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

以下内容不在 `rc26_topo_nav` 内实现：

- 浏览器前端
- Web adapter
- bringup 的主可视化入口

当前这些仓库内实现也已经随 `src/rc26_xhu_viewer` 一并删除。因此，`rc26_topo_nav` 现在只保留规划与场地数据真源身份，不再默认面向某个仓库内 viewer。

## 当前关键文件

- [config/r2_field_graph_blue.yaml](/home/potato/RC_2026/src/rc26_topo_nav/config/r2_field_graph_blue.yaml)
- [config/r2_surface_graph_blue.yaml](/home/potato/RC_2026/src/rc26_topo_nav/config/r2_surface_graph_blue.yaml)
- [sim_assets/worlds/robocon2026_v2_aligned.world](/home/potato/RC_2026/src/rc26_topo_nav/sim_assets/worlds/robocon2026_v2_aligned.world)
- [sim_assets/config/kfs_config_v2_aligned.yaml](/home/potato/RC_2026/src/rc26_topo_nav/sim_assets/config/kfs_config_v2_aligned.yaml)

## 与外部可视化的当前关系

- 若后续有工作区外部工具需要回放路线或读取场地资产，应直接消费本包的 graph、surface graph、world、sim_assets 与 CLI 输出
- `scripts/render_graph_sim_html.py` 仍是 graph trace 的单一真源
- planner、action、graph schema 的权威归属没有因为删除 viewer 子树而改变

## 当前注意点

- 本包不再安装 `topo_sim_server.py`，也不再注册原有 `test_topo_sim_server`
- `test_generate_surface_graph.py` 现在默认只做 checked-in 文件的 manifest 快检、helper 回归和小场景 in-process CLI smoke；
  对两份 60 万行级 dense `surface_graph` 做完整 YAML parse / validate，以及基于 `robocon2026_v2_aligned.world` 的整图再生成回归，都改为显式 opt-in。
- 需要手动设置 `RC26_RUN_FULL_SURFACE_GRAPH_REGEN=1` 后再运行完整重验收，避免普通 `ctest` 每次都重跑多分钟的整图加载与生成。
