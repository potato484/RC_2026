# rc26_kfs_keepout

`rc26_kfs_keepout` 负责把梅林区 KFS 状态融合成稳定的禁入/慢行约束输入，供决策侧和外部观察链路消费。

## 当前输出

- `/kfs_filter_mask`：占据栅格掩码
- `/mf_block_overlay`：离散格状态覆盖层
- `/kfs_keepout_heartbeat`：链路心跳
- diagnostics

## 当前职责

- 基于 Log-Odds 维护格位阻挡概率
- 对状态变化做驻留去抖
- 对长时间未更新的状态做软衰减
- 为决策提供稳定的 keepout 约束，而不是直接做路径规划

## 当前边界

- 不识别原始 KFS 感知
- 不直接控制机器人
- 输出同时服务 `rc26_decision`、外部可视化消费者和 keepout gate
- 本轮基础 Nav2 迁移不把 `/mf_block_overlay` 或 `/kfs_filter_mask` 接入 Nav2 costmap
- 当 layout team 与运行时 `MfKfsState.team` 不一致时，只会禁用 keepout 输出并发布诊断，不会在本模块内直接触发底盘安全模式
