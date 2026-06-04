# rc26_kfs_keepout

## 模块定位

`rc26_kfs_keepout` 是梅林区动态 keepout 生成模块，负责把 KFS 状态融合成决策侧可消费的禁入/慢行约束输入。

## 当前实现

- 导出运行时:
  - `kfs_keepout_runtime_manager_node`
  - `rc26_kfs_keepout::KfsBlockFuser` 组件（调试时仍可经 `kfs_block_fuser_node` 直接起单节点）
- 关键输出:
  - `/kfs_filter_mask`
  - `/mf_block_overlay`
  - `/kfs_keepout_heartbeat`
- 关键服务:
  - `/kfs_keepout/set_runtime`
- 关键配置:
  - `config/r2_mf_world.yaml`
  - `config/mf_grid_layout.yaml`

当前导航 bringup 不直接常驻拉起 `kfs_block_fuser_node`。`slam=false` 时只常驻一个空组件容器和 `kfs_keepout_runtime_manager`，真正的 keepout 组件只在 `rc26_decision` 进入 `MFAreaTree` 前被 load，并在离开 MF 子树时先清空输出再 unload。

## 当前运行时语义

- `kfs_keepout_runtime_manager` 负责 `UNLOADED / LOADING / ACTIVE / CLEARING / ERROR` 这组幂等状态。
- `KfsBlockFuser` 组件被 load 后默认是非激活态；非激活态不会继续融合新的 `MfKfsState`，`/kfs_keepout_heartbeat` 固定为 `false`。
- 离开 MF 时，组件会先发布空 overlay、全零 mask 和 `false` 心跳。
- 只有 `outputs_cleared=true` 之后管理器才会尝试 `UnloadNode`；若清空成功但卸载失败，会保留 `component_loaded=true` 作为资源告警，但允许后续流程继续。

## 当前 overlay 语义

- `/mf_block_overlay` 当前除了 `FREE / BLOCKED / UNKNOWN` 外，还支持 `SLOW` 单元状态。
- `SLOW` 表达“可通行但需要保守对待”的区域；本模块不直接切换机器人控制状态。

## 当前边界

- 不做路径规划
- 不直接控制机器人
- 本轮基础 Nav2 迁移不把 `/mf_block_overlay` 或 `/kfs_filter_mask` 接入 Nav2 costmap
- “什么时候启用 keepout” 由 `rc26_decision` 的 `MFAreaTree` 子树边界决定
- team mismatch 只会关闭 keepout 输出并通过 diagnostics 暴露降级状态
