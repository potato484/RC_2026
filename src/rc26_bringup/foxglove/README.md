# R2 Foxglove 使用说明

## 启动后端

- RViz 保底：`ros2 launch rc26_bringup bringup.launch.py visualization_backend:=rviz`
- Foxglove 监督层：`ros2 launch rc26_bringup bringup.launch.py visualization_backend:=foxglove`
- 仅状态聚合：`ros2 launch rc26_bringup bringup.launch.py visualization_backend:=none visualization_status_enable:=true`

## 连接地址

- 默认地址：`ws://<机器人IP>:8765`
- 端口可通过 `foxglove_port:=8765` 调整

## 入口分工

- `src/rc26_bringup/foxglove/*.json`：仓库内模板资产
- bringup 在 `visualization_backend:=foxglove` 时会自动生成当前 namespace 对应的有效布局文件

## 布局文件

- 自动生成目录默认是 `/tmp/rc26_foxglove_layouts/current/`
- 可通过 `foxglove_layout_dir:=<path>` 改变输出目录
- `operator.json`：legacy 值守骨架，保留 `r2/diag/operator_status`、`r2/diag/events`、控制趋势视角，并增加 `r2/bt/model`、`r2/bt/snapshot`、`r2/bt/blackboard`、`r2/bt/events` Raw Messages 面板；默认显示 `/terrain_grid_map_markers`、`/registered_scan`、`/laser_map_full`
- `engineering.json`：工程联调，配合轨迹、点云、lookahead 与曲率标记使用；默认显示 `/terrain_grid_map_local_markers`、`/registered_scan`、`/laser_map_full`
- `diagnostic.json`：故障排查，重点关注 `r2/diag/summary`、keepout/terrain/topic freshness；默认显示 `/terrain_grid_map_markers`、`/registered_scan`、`/laser_map_full`

## BT 运行态话题

先通过 Foxglove 验证 `r2/bt/*` 话题，确认数据正确后再导入仓库内布局或现场定制布局继续值守。

- `r2/bt/model`：静态树结构（transient_local，晚连可拿到）
- `r2/bt/snapshot`：每次 tick 后的节点状态快照
- `r2/bt/blackboard`：黑板白名单键值（200ms 周期）
- `r2/bt/events`：节点状态变化事件（按 tick 批量 flush）

若 bringup 使用了非空 `namespace`，自动生成目录中的布局会把 `topicPath` 与 3D 订阅主题重写成 `/<namespace>/...`，不需要再手工改 JSON。

## 常见告警

- `LOCALIZATION_DEGRADED`：定位退化，优先检查 `/localization/health`
- `CONTROL_DEGRADED`：控制退化，优先检查 `/control_degraded` 与 `control_degenerate_score`
- `KEEPOUT_STALE`：keepout 输入陈旧，检查 `/mf_block_overlay`、`/kfs_filter_mask`、`/kfs_keepout_heartbeat`
- `NAV_STOP_REQUIRED` / `NAV_TIMED_OUT`：导航运行时要求停车或 watchdog 超时，检查 `/xhu_nav/motion_mode_state` 与 `/xhu_nav/tracking_state`
- `MECHANISM_COMM_WARN`：机构通信异常，检查 `/mechanism/state`

## 导入布局

- 打开 Foxglove
- 连接机器人 WebSocket
- 通过 Layouts -> Import from file 导入 `/tmp/rc26_foxglove_layouts/current/` 下的生成文件
- 若只做值守，优先使用 `operator.json` 作为基础值守布局
- 若需要自定义输出位置，可在 bringup 时传 `foxglove_layout_dir:=<path>`


## 建图相关话题

- `/registered_scan`：当前帧配准点云，适合看实时局部效果
- `/laser_map_full`：累计地图点云，适合看历史建图内容是否持续保留
- `/terrain_grid_map_markers`：`map` 系下的 2.5D 栅格 marker，可在 `diagnostic.json` / `operator.json` 直接看到
- `/terrain_grid_map_local_markers`：`odom` 系下的 2.5D 栅格 marker，可在 `engineering.json` 直接看到
- `engineering.json` 已默认打开 `/registered_scan` 和 `/laser_map_full`，可直接和 2.5D 栅格一起对照观察
