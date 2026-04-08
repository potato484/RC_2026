# rc26_kfs_keepout

`rc26_kfs_keepout` 负责把梅林区 KFS 状态融合成稳定的禁入约束输入，供 topo/xhu 自研导航链消费。

## 当前输出

- `/kfs_filter_mask`：占据栅格掩码
- `/mf_block_overlay`：离散格状态覆盖层
- `/kfs_keepout_heartbeat`：链路心跳
- diagnostics

## 当前职责

- 基于 Log-Odds 维护格位阻挡概率
- 对状态变化做驻留去抖
- 对长时间未更新的状态做软衰减
- 为决策和导航提供稳定的 keepout 约束，而不是直接做路径规划

## 当前边界

- 不识别原始 KFS 感知
- 不直接控制机器人
- 输出同时服务 `rc26_topo_nav`、`rc26_xhu_viewer` 和 keepout gate
- 当 layout team 与运行时 `MfKfsState.team` 不一致时，只会禁用 keepout 输出并发布诊断，不会在本模块内直接触发底盘安全模式
