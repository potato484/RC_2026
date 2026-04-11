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

## 文档收口总结

- `docs/backend/archive/` 不再为上述旧导航实现包保留单独 README
- 当前如果要理解 3D 导航实现，应只从本页和 `src/rc26_xhu_nav/` 源码入口继续展开
- 历史演进原因保留在 `docs/fitness/architecture_fitness_ros2_workspace/README.md` 的架构变更记录中
