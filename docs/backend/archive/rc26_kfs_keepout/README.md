# rc26_kfs_keepout

## 模块定位

`rc26_kfs_keepout` 是已归档的梅林区动态 keepout 融合源码包。当前主运行时不再编译、启动或消费它；默认 CMake 只完成包配置，不生成节点、组件、库、测试或安装目标。

## 当前实现

- 归档源码保留的历史运行时:
  - `kfs_keepout_runtime_manager_node`
  - `rc26_kfs_keepout::KfsBlockFuser` 组件（调试时仍可经 `kfs_block_fuser_node` 直接起单节点）
- 历史输出:
  - `/kfs_filter_mask`
  - `/mf_block_overlay`
  - `/kfs_keepout_heartbeat`
- 历史服务:
  - `/kfs_keepout/set_runtime`
- 关键配置:
  - `config/r2_mf_world.yaml`
  - `config/mf_grid_layout.yaml`

当前导航 bringup 不再拉起 keepout 容器或 runtime manager，`rc26_decision` 也不再调用 `/kfs_keepout/set_runtime` 或发布 `/mf_kfs_state`。`config/r2_mf_world.yaml` 不再是当前 MF 主区共享几何真源；MF 格位逻辑已退回 `rc26_decision` 包内静态表。

## 当前运行时语义

- 默认没有运行时语义：包不生成可执行目标，不发布 overlay/mask/heartbeat，不提供 runtime service。
- 只有显式以 `RC26_ENABLE_ARCHIVED_RUNTIME_TARGETS=ON` 恢复本地调试构建后，历史运行时才可能被手工启动。

## 当前 overlay 语义

- `/mf_block_overlay` 当前除了 `FREE / BLOCKED / UNKNOWN` 外，还支持 `SLOW` 单元状态。
- `SLOW` 表达“可通行但需要保守对待”的区域；本模块不直接切换机器人控制状态。

## 当前边界

- 不参与 `rc26_bringup`、`rc26_decision`、Nav2 的默认运行时链路
- 当前主链没有任何模块订阅 `/mf_block_overlay`、`/kfs_filter_mask` 或 `/kfs_keepout_heartbeat`
- 当前主链没有任何模块调用 `/kfs_keepout/set_runtime`
- 如未来恢复，必须先重新定义接口契约、启动入口、验证范围和文档边界
