# R2 Foxglove 使用说明

## 启动后端

- RViz 保底：`ros2 launch rc26_bringup bringup.launch.py visualization_backend:=rviz`
- Foxglove 监督层：`ros2 launch rc26_bringup bringup.launch.py visualization_backend:=foxglove`
- 仅状态聚合：`ros2 launch rc26_bringup bringup.launch.py visualization_backend:=none visualization_status_enable:=true`

## 连接地址

- 默认地址：`ws://<机器人IP>:8765`
- 端口可通过 `foxglove_port:=8765` 调整

## 入口分工

- `operator.json`：legacy 过渡骨架，保留给 Foxglove Studio 导入对照
- `engineering.json`、`diagnostic.json`：继续作为工程联调与故障排查布局资产

## 布局文件

- `operator.json`：legacy 值守骨架，保留 `r2/diag/operator_status`、`r2/diag/events`、控制趋势视角
- `engineering.json`：工程联调，配合轨迹、点云、lookahead 与曲率标记使用
- `diagnostic.json`：故障排查，重点关注 `r2/diag/summary`、keepout/terrain/topic freshness

## 常见告警

- `LOCALIZATION_DEGRADED`：定位退化，优先检查 `/localization/health`
- `CONTROL_DEGRADED`：控制退化，优先检查 `/control_degraded` 与 `control_degenerate_score`
- `KEEPOUT_STALE`：keepout 输入陈旧，检查 `/costmap_filter_info`、`/kfs_filter_mask`、`/kfs_keepout_heartbeat`
- `NAV_STOP_REQUIRED` / `NAV_TIMED_OUT`：导航策略要求停车或 watchdog 超时，检查 `/nav_safety_state`
- `MECHANISM_COMM_WARN`：机构通信异常，检查 `/mechanism/state`

## 导入布局

- 打开 Foxglove
- 连接机器人 WebSocket
- 通过 Layouts -> Import from file 导入本目录 JSON
- 若需要 Foxglove Studio 布局，`operator.json` 仅作为 legacy 过渡骨架


## 建图相关话题

- `/registered_scan`：当前帧配准点云，适合看实时局部效果
- `/laser_map_full`：累计地图点云，适合看历史建图内容是否持续保留
- 若使用 `visualization_backend:=foxglove`，建议把这两个点云一起放进工程布局中对照观察
