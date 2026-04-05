# rc26_visualization 调试指南

## 核心输出

```bash
ros2 topic echo /r2/diag/operator_status
ros2 topic echo /r2/diag/events
ros2 topic echo /r2/diag/summary
```

## 导航链相关输入

```bash
ros2 topic echo /xhu_nav/motion_mode_state --once
ros2 topic echo /xhu_nav/tracking_state --once
ros2 topic echo /xhu_nav/semantic_gate --once
ros2 topic echo /mf_block_overlay --once
ros2 topic echo /kfs_filter_mask --once
ros2 topic hz /kfs_keepout_heartbeat
```

## 常见事件

- `KEEPOUT_STALE`
  - 优先检查 `/mf_block_overlay`、`/kfs_filter_mask`、`/kfs_keepout_heartbeat`
- `NAV_STOP_REQUIRED`
  - 查看 `/xhu_nav/motion_mode_state`
- `NAV_TIMED_OUT`
  - 查看 `/xhu_nav/tracking_state`
- `TOPIC_STALE_*`
  - 检查对应 topic 是否断流或时间戳异常

## 重置 topic timeout 计数

```bash
ros2 service call /r2/diag/reset_topic_timeout_count std_srvs/srv/Trigger "{}"
```
