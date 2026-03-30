# rc26_kfs_keepout

## 模块定位

`rc26_kfs_keepout` 是 R2 在梅林区等场景下的动态 Keepout 生成模块，负责把 KFS 状态融合成 Nav2 可消费的禁行掩码。

## 当前实现

- 构建方式：组件库 + 可执行节点
- 导出节点：`kfs_block_fuser_node`
- 核心源码：`src/rc26_kfs_keepout/src/kfs_block_fuser.cpp`
- 关键配置：`config/mf_grid_layout.yaml`、`config/r2_mf_world.yaml`

当前实现围绕 `KfsBlockFuser` 展开，重点包括：

- 接收 `MfKfsState` 等离散网格状态输入
- 维护 Log-Odds 概率状态，而不是直接信任单帧观测
- 对状态变化做去抖和按需发布
- 生成 Nav2 KeepoutFilter 能直接使用的 `OccupancyGrid`
- 为导航安全门控提供更稳定的“当前哪些格子不可过”的结果

## 模块边界

- 它不做原始感知，不负责识别 KFS
- 它不做路径规划，只是给 Nav2 提供约束输入
- 它不替代 `rc26_terrain` 的地形风险图，两者属于不同来源的安全信息
