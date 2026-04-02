# rc26_kfs_keepout

## 模块定位

`rc26_kfs_keepout` 是梅林区动态 keepout 生成模块，负责把 KFS 状态融合成自研导航链可消费的约束输入。

## 当前实现

- 导出节点: `kfs_block_fuser_node`
- 关键输出:
  - `/kfs_filter_mask`
  - `/mf_block_overlay`
  - `/kfs_keepout_heartbeat`
- 关键配置:
  - `config/r2_mf_world.yaml`
  - `config/mf_grid_layout.yaml`

## 当前边界

- 不做路径规划
- 只面向 topo/xhu 约束输入
- 当前主要服务 `rc26_topo_nav`、`rc26_decision` 和 `rc26_visualization`
- team mismatch 只会关闭 keepout 输出并通过 diagnostics 暴露降级状态，不在本模块内直接接管机器人控制
