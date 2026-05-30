# rc26_kfs_keepout

## 模块定位

`rc26_kfs_keepout` 是梅林区动态 keepout 生成模块，负责把 KFS 状态融合成 `rc26_xhu_nav` 可消费的约束输入。

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

当前导航 bringup 不再直接常驻拉起 `kfs_block_fuser_node`。`slam=false` 时只常驻一个空组件容器和 `kfs_keepout_runtime_manager`，真正的 keepout 组件只在 `rc26_decision` 进入 `MFAreaTree` 前被 load，并在离开 MF 子树时先清空输出再 unload。

## 当前运行时语义

- `kfs_keepout_runtime_manager` 负责 `UNLOADED / LOADING / ACTIVE / CLEARING / ERROR` 这组幂等状态。
- `KfsBlockFuser` 组件被 load 后默认是非激活态；非激活态不会继续融合新的 `MfKfsState`，`/kfs_keepout_heartbeat` 固定为 `false`。
- 当前已经移除人工强制释放某格的运行时入口；keepout 状态只再由 `MfKfsState` 输入、时间衰减和 `MFAreaTree` 的显式启停共同驱动。
- 离开 MF 时，组件会先发布：
  - 空 `cells=[]` 的 `/mf_block_overlay`
  - 与当前布局一致、但全零的 `/kfs_filter_mask`
  - `false` 的 `/kfs_keepout_heartbeat`
- 只有 `outputs_cleared=true` 之后管理器才会尝试 `UnloadNode`；若清空成功但卸载失败，会保留 `component_loaded=true` 作为资源告警，但允许后续流程继续。

## 当前 overlay 语义

- `/mf_block_overlay` 当前除了 `FREE / BLOCKED / UNKNOWN` 外，还支持 `SLOW` 单元状态。
- `kfs_block_fuser_node` 新增 `slow_grid_ids` 参数后，会把指定格子发布为 `SLOW`，并保持 `keepout_active=true`，用于提示上游执行链降速而不是直接判死阻塞。
- 该扩展没有改变原始 mask/占据计算方式；它只补充 overlay 的语义层，供 `rc26_terrain` 摘要和局部执行链消费。

## 共享几何口径

- [r2_mf_world.yaml](/home/potato/RC_2026/src/rc26_kfs_keepout/config/r2_mf_world.yaml) 现在同时作为 `rc26_kfs_keepout` 和 `rc26_xhu_nav` 的 MF 主区共享几何真源
- 发布到 `/mf_block_overlay` 的 `team` 当前来自运行时 KFS 输入，而不是 `r2_mf_world.yaml` 内的静态写死阵营
- 这个文件只描述块位置、高度、入口/出口 block 集合和场地区域事实，不直接定义 topo 任务、staging 点、坡道边和导航代价
- `rc26_xhu_nav` 会在离线生成阶段读取这里的几何，再叠加自己的 topo overlay 产出运行时 `graph_file`

## 当前边界

- 不做路径规划
- 只面向 topo/xhu 约束输入
- 当前主要服务 `rc26_xhu_nav`、`rc26_decision` 和下游可视化消费者
- “什么时候启用 keepout” 现在由 `rc26_decision` 的 `MFAreaTree` 子树边界决定，不由本包自行根据位姿或 corridor 判断
- `SLOW` 语义当前只表达保守通行区域，不在本模块内直接把机器人切到 recovery 或 stop
- team mismatch 只会关闭 keepout 输出并通过 diagnostics 暴露降级状态，不在本模块内直接接管机器人控制

## 配置注释口径

- `config/r2_mf_world.yaml` 与 `config/mf_grid_layout.yaml` 已保留常用/高影响字段的中文注释，重点说明共享几何、入口/出口集合和兼容 shim；重复 block 实例不再逐字段机械注释；本次只改变注释和等价 YAML 展开，不改变共享几何数据。
