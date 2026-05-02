# rc26_xhu_nav

## 模块定位

`rc26_xhu_nav` 是 R2 当前唯一的 3D 导航实现宿主包。

## 当前拥有的内容

- topo graph / surface graph / corridor 生成
- body-aware surface planner
- local 3D planner core 与 trace CLI
- xhu motion mode manager
- xhu motion runtime 执行器
- graph/surface graph 配置、离线脚本、`sim_assets/`

## 当前运行时节点

- `topo_nav_node`
- `xhu_motion_mode_manager_node`
- `xhu_motion_runtime_node`

## 当前边界

- 对外继续复用 `rc26_interfaces` 中定义的 action / service / topic 契约
- 不拥有 `rc26_bringup` 的装配权
- 不拥有 `rc26_terrain`、`rc26_base_ground`、`rc26_kfs_keepout`、`rc26_localization` 的状态真源
- `cmd_vel` 的唯一权威是 `xhu_motion_runtime_node`

## 当前说明

- `src/rc26_topo_nav`、`src/rc26_surface_body_planner`、`src/rc26_local_3d_planner`、`src/rc26_nav_mode_manager`、`src/rc26_omni_controller` 已于 2026-04-10 从工作区实现层退出，并统一收口到本包。
- 外部 ROS 名称保持不变：`navigate_topo_target`、`navigate_surface_route`、`set_xhu_motion_mode`、`/xhu_nav/*`、`cmd_vel` 继续沿用原契约。

## 当前导航图与规划口径

- `graph_file` 当前同时支持 `routes` 和 `edges.control_points`，分别服务 route 级执行与 corridor 细化
- `surface_graph` 当前已经按 body-aware 口径工作，离线图里包含 node pitch、edge slope、edge lateral clearance 等约束注解
- `NavigateSurfaceRoute.allow_replan` 继续沿用 route-level replan；dynamic surface overlay 归并、TTL 过期与 overlay version 检查都已经并入本包运行时
- `topo`、`body_planner`、`local_planner`、`mode_manager`、`runtime` 虽然都已收口到同一宿主包，但运行时职责仍保持分离

## 文档收口总结

- `docs/backend/archive/` 不再为上述旧导航实现包保留单独 README
- 当前如果要理解 3D 导航实现，应只从本页和 `src/rc26_xhu_nav/` 源码入口继续展开
- 已经落地的架构变化已直接折叠进本页、`rc26_bringup`、`rc26_interfaces`、`rc26_kfs_keepout` 与 `rc26_robot_geometry` 的当前实现说明

## 配置注释口径

- 手写配置 `config/topo_nav.yaml`、`local_3d_planner.yaml`、`xhu_motion_runtime.yaml`、`nav_profiles.yaml`、overlay YAML 和 `sim_assets/config/kfs_config_v2_aligned.yaml` 已保留常用/高影响参数的中文注释，重点说明 topic、frame、profile、规划权重、执行限幅和高影响 overlay 字段，低频内部字段不再逐项注释。
- `r2_field_graph_blue/red.yaml` 与 `r2_surface_graph_blue/red.yaml` 是 `generated: true` 的生成图文件，当前采用文件头 schema 注释说明 `meta/nodes/edges/routes/tasks` 字段，不对数万条重复 node/edge 实例逐项复制注释；需要调整图数据时仍应修改共享几何、overlay 或生成脚本后重新生成。
- 本次只改变注释和少量等价 YAML 展开，不改变导航图数据、规划参数或运行时接口。
