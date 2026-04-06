# rc26_visualization

`rc26_visualization` 现在同时承担两层职责：

- ROS2 诊断聚合：把定位、控制、keepout、地形、机构和导航运行态收敛成 `r2/diag/*`
- 本地 Web 可视化平台：提供 `viewer/` 前端、`visualization_server.py` adapter 和 bringup 的 `local_web` 入口

## 当前目录真源

- `src/visualization_status_core.cpp`
- `src/visualization_status_node.cpp`
- `scripts/visualization_server.py`
- `scripts/visualization_algorithms.py`
- `scripts/render_graph_sim_html.py`
- `viewer/`
- `config/visualization_status.yaml`
- `config/field_scene_manifest.yaml`

## 当前输入

诊断聚合继续消费：

- `/xhu_nav/motion_mode_state`
- `/xhu_nav/tracking_state`
- `/xhu_nav/local_planner_state`
- `/xhu_nav/recovery_state`
- `/xhu_nav/semantic_layer_summary`
- `/localization/health`
- `/localization/backend_status`
- `/mf_block_overlay`
- `/mechanism/state`

Web adapter 在此基础上还会只读消费：

- `/control_state`
- `/topo_nav/route`
- `/topo_nav/corridor`
- `/xhu_nav/lookahead_path`
- `/r2/diag/operator_status`
- `/r2/diag/events`
- `/r2/bt/snapshot`
- `/r2/bt/events`

## 当前输出

- `r2/diag/operator_status`
- `r2/diag/events`
- `r2/diag/reset_topic_timeout_count`
- `/api/scene-manifest`
- `/api/surface-route/*`
- `/api/local-planner/*`
- `/api/live/*`

## 当前边界

- 不拥有导航 action、planner CLI 或 topo 图真源，这些仍属于 `rc26_topo_nav`
- 不拥有 bringup 装配权，这些仍属于 `rc26_bringup`
- 不直接控制机器人；Web 端只做只读观测和受控的 `navigate_surface_route` 下发

## 本次迁移后的真实实现

- 原 `rc26_topo_nav/sim_viewer` 已迁入 `rc26_visualization/viewer`
- 原 `topo_sim_server.py` 已迁入并泛化为 `visualization_server.py`
- bringup 的 Web 主入口已经切到 `visualization_backend:=local_web`
- Web 端现在统一展示场地、路线、阶段区、keepout、定位健康、机构状态和 BT 快照，不再把 Foxglove 当主入口
- `start_r2_visualization.sh` 在已 source 当前工作区的 shell 中会显式带上 `--allow-overriding rc26_topo_nav rc26_visualization`，避免重复构建时持续出现 colcon override-check 警告
- `visualization_server.py` 在 `BODY_CONSTRAINT_UNSATISFIED` 这类 body-aware 失败场景下，会追加一次 `legacy + --disable-body-planning` 参考预览；浏览器可继续看到参考路线，但执行按钮会保持禁用
