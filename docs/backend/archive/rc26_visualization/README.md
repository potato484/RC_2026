# rc26_visualization

## 模块定位

`rc26_visualization` 现在的主职责重新收口为诊断聚合与迁移期工具链承载。

当前真实职责分成两块：

- 诊断聚合：生成 `r2/diag/operator_status`、`r2/diag/events` 等值守语义
- 迁移期 Web adapter：继续保留 `viewer/`、`visualization_server.py` 和 live bridge，供本地联调、离线验证或后续迁移对照使用

## 当前关键文件

- 状态聚合：
  - [src/visualization_status_core.cpp](/home/potato/RC_2026/src/rc26_visualization/src/visualization_status_core.cpp)
  - [src/visualization_status_node.cpp](/home/potato/RC_2026/src/rc26_visualization/src/visualization_status_node.cpp)
- Web 平台：
  - [scripts/visualization_server.py](/home/potato/RC_2026/src/rc26_visualization/scripts/visualization_server.py)
  - [viewer/src/App.tsx](/home/potato/RC_2026/src/rc26_visualization/viewer/src/App.tsx)
  - [config/field_scene_manifest.yaml](/home/potato/RC_2026/src/rc26_visualization/config/field_scene_manifest.yaml)

## 当前输入口径

- 运行态：`/control_state`、`/topo_nav/route`、`/topo_nav/corridor`、`/xhu_nav/lookahead_path`
- 诊断：`/r2/diag/operator_status`、`/r2/diag/events`
- 行为树：`/r2/bt/snapshot`、`/r2/bt/events`
- 导航局部态：`/xhu_nav/motion_mode_state`、`/xhu_nav/tracking_state`、`/xhu_nav/local_planner_state`、`/xhu_nav/recovery_state`
- 定位与 keepout：`/localization/health`、`/localization/backend_status`、`/mf_block_overlay`

## 当前边界

- topo 图、surface graph、planner CLI、`navigate_surface_route` action 仍以 `rc26_topo_nav` 为真源
- bringup 只负责装配，本包不反向拥有 launch 权威
- `src/rc26_visualization/viewer` 虽然仍可执行 `surface-route execute`，但那是受控 action 下发，不是浏览器直控底盘
- 当前 bringup 默认可视化后端已经切到 `rc26_xhu_viewer`，不再通过 `visualization_backend:=local_web` 启动本包

## 本次迁移后的注意点

- `visualization_server.py` 默认仍然读取 `rc26_topo_nav` 的 graph/world/sim_assets
- `field_scene_manifest.yaml` 是 viewer 自己的场地布局元数据真源，负责 `semanticZones / displayCatalog / layoutPresets`
- `viewer` 已从“单用途路线观察台”扩成“统一比赛场地闭环可视化平台”，主界面同时承载路线回放和 live 运行态
- 但这条 Web viewer 链路现在已经退回到“仓库内保留的本地工具链”定位，不再是 bringup 主入口
- 当操作员在 `diagnostic` 这类隐藏路线图层的布局里生成三维路线时，viewer 现在会自动重新打开 `route` 图层，避免预览结果被布局状态吞掉
- 当 body-aware `surface_route_cli` 返回 `BODY_CONSTRAINT_UNSATISFIED` 时，adapter 会再生成一条 `legacy` 参考路线回给浏览器；这条路线只用于观察，不代表 action 已经接受或可直接执行
- `visualization_server.py` 现在会在 surface-route preview 丢失 `projected_*_node_id` 时，优先复用 fallback 参考预览或按投影点回推 surface graph 最近节点，避免浏览器因为缺节点 ID 而完全拿不到搜索回放
- `semanticZones` 现在按 3 块 coarse phase band 定义在 world frame 中，直接映射 `MCAreaTree / MFAreaTree / CombatAreaTree`；不再把梅林局部入口/出口坡道条带误当成独立行为树边界
