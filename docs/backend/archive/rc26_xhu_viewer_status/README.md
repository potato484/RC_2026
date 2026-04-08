# rc26_xhu_viewer_status

## 模块定位

`rc26_xhu_viewer_status` 是 R2 当前默认常驻的操作员语义聚合运行时包，源码位于 `src/rc26_xhu_viewer/rc26_xhu_viewer_status/`。

它负责把 localization、controller、keepout、terrain、navigation runtime 和 mechanism 的技术状态聚合成统一的：

- `r2/xhu_viewer/summary`
- `r2/xhu_viewer/operator_status`
- `r2/xhu_viewer/events`

`rc26_xhu_viewer_status` 不承载 RViz GUI、不承载 Web adapter，也不拥有 `.rviz` preset；这些能力继续留在 sibling 包 `rc26_xhu_viewer`。

## 当前关键文件

| 目录/文件 | 说明 |
|---|---|
| `include/rc26_xhu_viewer_status/xhu_viewer_status_core.hpp` | 状态聚合 core 对外头文件 |
| `src/xhu_viewer_status_core.cpp` | 汇总定位、控制、keepout、terrain、机构与导航态的语义规则 |
| `src/xhu_viewer_status_node.cpp` | `rc26_xhu_viewer_status_node`，负责订阅运行时 topic 并对外发布 `r2/xhu_viewer/*` |
| `config/xhu_viewer_status.yaml` | topic、阈值、watchdog 与 summary presence 参数真源 |
| `test/test_xhu_viewer_status.cpp` | 聚合规则单测 |

## 输入与输出

输入 topic 主要包括：

- `/localization/health`
- `/localization/backend_status`
- `/control_degraded`
- `/xhu_nav/motion_mode_state`
- `/xhu_nav/tracking_state`
- `/xhu_nav/local_planner_state`
- `/xhu_nav/recovery_state`
- `/xhu_nav/semantic_layer_summary`
- `/mf_block_overlay`
- `/kfs_filter_mask`
- `/terrain_grid_map_local`
- `odom`
- `control_state`

输出与服务：

- topic `r2/xhu_viewer/summary`
- topic `r2/xhu_viewer/operator_status`
- topic `r2/xhu_viewer/events`
- service `r2/xhu_viewer/reset_topic_timeout_count`

## 当前边界

- 默认随 `rc26_bringup` 的 `visualization_status_enable:=true` 装配
- 不依赖 GUI，不要求 DISPLAY/WAYLAND_DISPLAY
- 不拥有 RViz 布局、Display/Panel 插件和 Web 前端资源
- 保留 `rc26_xhu_viewer` 作为同一可视化域的 sibling GUI 包，但运行时职责已经拆开

## 本次拆包后的真实变化

- `xhu_viewer_status_core`、`rc26_xhu_viewer_status_node` 和 `config/xhu_viewer_status.yaml` 已从 `rc26_xhu_viewer` 迁出
- `rc26_bringup` 现在直接装配 `rc26_xhu_viewer_status/rc26_xhu_viewer_status_node`
- `rc26_xhu_viewer` 保留 GUI 壳层、`.rviz` preset 和可选插件/Web 工具链

## 注意点

- `r2/xhu_viewer/*` 的操作员语义边界仍然属于同一可视化域，但最小常驻单元现在是 `rc26_xhu_viewer_status`
- 如果修改 topic 语义、聚合阈值或输出字段，应优先同步本包 README 和 `config/xhu_viewer_status.yaml`
- 如果修改 GUI preset、Panel/Display 或 Web adapter，不应回头把逻辑塞回本包
