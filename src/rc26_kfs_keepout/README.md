# rc26_kfs_keepout

`rc26_kfs_keepout` 是已归档的梅林区动态 keepout 融合源码包。当前主运行时不再编译、启动或消费它；默认 CMake 只完成包配置，不生成节点、组件、库、测试或安装目标。

## 归档历史输出

- `/kfs_filter_mask`：占据栅格掩码
- `/mf_block_overlay`：离散格状态覆盖层
- `/kfs_keepout_heartbeat`：链路心跳
- diagnostics

## 归档历史职责

- 基于 Log-Odds 维护格位阻挡概率
- 对状态变化做驻留去抖
- 对长时间未更新的状态做软衰减
- 为决策提供稳定的 keepout 约束，而不是直接做路径规划

## 当前边界

- 不参与 `rc26_bringup`、`rc26_decision`、Nav2 的默认运行时链路。
- 当前主链不发布、订阅或消费 `/mf_block_overlay`、`/kfs_filter_mask`、`/kfs_keepout_heartbeat`。
- 当前主链不提供或调用 `/kfs_keepout/set_runtime`。
- `config/r2_mf_world.yaml` 不再是当前 MF 主区共享几何真源；MF 格位逻辑已退回 `rc26_decision` 包内静态表。
- 只有显式以 `RC26_ENABLE_ARCHIVED_RUNTIME_TARGETS=ON` 恢复本地调试构建后，历史运行时才可能被手工启动。
- 如未来恢复，必须先重新定义接口契约、启动入口、验证范围和文档边界。
