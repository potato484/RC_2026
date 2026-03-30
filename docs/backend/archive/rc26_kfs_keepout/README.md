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

## 源码入口与阅读顺序
- 先看 `src/kfs_block_fuser.cpp`，这个包的业务逻辑几乎都在单文件里。
- 再看 `config/r2_mf_world.yaml` 和 `config/mf_grid_layout.yaml`，确认世界格布局和坐标映射。
- 最后看 `src/rc26_kfs_keepout/docs/debug_guide.md`，理解心跳、mask 和强制释放的调试方法。

## 目录解剖
- `src/kfs_block_fuser.cpp`：Log-Odds 状态估计、去抖、软衰减、mask 发布、心跳和 diagnostics。
- `config/r2_mf_world.yaml`：梅林区世界布局与格位语义。
- `config/mf_grid_layout.yaml`：格子映射与几何配置。
- `docs/debug_guide.md`：链路联调指南。

## 关键文件体量
- `src/kfs_block_fuser.cpp`：654 行，核心实现高度集中。
- `README.md`：52 行，功能与边界说明。
- `config/r2_mf_world.yaml`：34 行，世界布局输入。

## 关键源码行段速览
- `src/rc26_kfs_keepout/src/kfs_block_fuser.cpp:34-193`：构造函数，声明参数、订阅/发布接口、初始化计时器和内部网格状态。
- `src/rc26_kfs_keepout/src/kfs_block_fuser.cpp:194-311`：网格布局载入和网格间距校验。
- `src/rc26_kfs_keepout/src/kfs_block_fuser.cpp:312-428`：`onKfsState()` 和强制释放处理，更新每格证据。
- `src/rc26_kfs_keepout/src/kfs_block_fuser.cpp:429-579`：软衰减、mask 生成和发布。
- `src/rc26_kfs_keepout/src/kfs_block_fuser.cpp:580-654`：心跳、诊断和 safe mode 触发。

## 模块边界

- 它不做原始感知，不负责识别 KFS
- 它不做路径规划，只是给 Nav2 提供约束输入
- 它不替代 `rc26_terrain` 的地形风险图，两者属于不同来源的安全信息
