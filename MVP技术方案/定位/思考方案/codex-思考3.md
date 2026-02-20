# `/src/rc26_terrain` 地形感知模块技术评估报告

> 审计日期：2026-02-18  
> 审计范围：`src/rc26_terrain`（`terrain_semantic_node.cpp/.hpp` + `terrain_semantic.yaml`）  
> 规则基线：`MVP技术方案/决策/主赛规则.md`  
> 外部检索：arXiv / GitHub（2024-2026 地形识别与可通行性方向）

## 现状分析

### 1. 规则对标结论（核心条款）

| 规则条款 | 规则要求 | 当前实现覆盖度 | 结论 |
|---|---|---|---|
| `主赛规则.md:33-61` | 梅林为 12 块 200/400/600mm 方块，镜像分布 | 几何高度差可处理（`h_climb_m=0.30`、`h_obstacle_m=0.33`） | 对固定地形可用 |
| `主赛规则.md:194-201` | R2 仅按“公共边相邻”策略行动 | `classifyAndUpdate` 使用 8 邻域（含对角）`terrain_semantic_node.cpp:574-587` | 与规则“边相邻”不一致 |
| `主赛规则.md:201` + `主赛规则.md:290` | R2 不得踩踏“有 KFS 的方块顶面” | 模块无 KFS 占位语义输入，仅做几何分割 | 关键缺口（高风险） |
| `主赛规则.md:202` + `主赛规则.md:289` | 不得接触/移动假 KFS | 模块无法区分真/假 KFS | 关键缺口（高风险） |
| `主赛规则.md:56-61` + `主赛规则.md:118-124` | 需在树林方块（梅花桩）与干扰木码（假 KFS）共存场景下安全作业 | 当前算法仅按几何高度分类，无法语义区分“梅花桩/真KFS/假KFS” | 对抗规则层面能力不足 |
| `主赛规则.md:91` | R2 必须自动 | 本模块可持续输出 costmap 语义点云 | 满足自动化基础感知要求 |

### 2. 代码实现合理性（按链路）

1. 点云预处理  
- `VoxelGrid` 下采样：`terrain_semantic_node.cpp:864-870`，叶子 `0.05m`（参数 `voxel_leaf_size_m`）。  
- 局部投影与高度门限：`terrain_semantic_node.cpp:874-903`，按 `min_rel_z/max_rel_z/dis_ratio_z` 过滤。  
- 评价：对 12 块规则地形是“足够快、足够稳”的工程取舍，但无材质/反射强度建模。

2. 地面分割（Ground Segmentation）  
- 分位数地面 + 顶部估计：`terrain_semantic_node.cpp:502-520`。  
- EMA 平滑：`terrain_semantic_node.cpp:515-516`。  
- 评价：对静态台阶有效；对高反射白胶带、边缘稀疏点、快速跨台阶存在迟滞和偏置风险。

3. 障碍与跌落判定  
- 邻域高差与顶部高度：`terrain_semantic_node.cpp:590-595`。  
- 跌落扇区约束：`terrain_semantic_node.cpp:661-670`，默认前向 `180°`。  
- 评价：几何规则清晰，但“高台与低台标注归属”与“对角邻接”会引入策略噪声。

4. 聚类算法维度  
- 当前实现无显式聚类（无 Euclidean Cluster/DBSCAN/Connected Components），仅栅格逐单元判定。  
- 评价：实现简单高效，但对离散噪点、稀疏伪高点缺少对象级抑制。

5. 与 MVP 决策链响应关系  
- Nav2 本地 costmap 使用 `terrain_obstacles/terrain_drop`：`src/rc26_bringup/config/nav2_params.yaml:86-118`。  
- `nav_mode_manager` 用 `terrain_obstacles` 时间戳做清图后重建确认：`src/rc26_nav_mode_manager/README.md:29-33`。  
- 影响：该模块延迟/丢帧会直接放大为局部规划抖动与模式切换超时风险。

### 3. 距离与精度是否支撑 R2 高速穿梭

- 感知半径 `3.2m`（`terrain_semantic.yaml:32`），分辨率 `0.1m`（`terrain_semantic.yaml:33`）。  
- 若 R2 速度约 `1.5m/s`，前视时间窗约 `2.1s`；若接近 `2.5m/s`，仅约 `1.3s`。  
- 结论：对中速可用；对“高速+急转+密集台阶边缘”场景，安全裕量偏紧，需要更保守的风险输出策略。

### 4. 2024-2026 技术领先性对比（联网检索）

已检索并核验可访问条目（标题/仓库元信息）：
- TE-NeXt（2024）：LiDAR 稀疏卷积可通行性估计，`https://arxiv.org/abs/2406.01395`  
- RoadRunner（2024）：越野可通行性学习，`https://arxiv.org/abs/2402.19341`  
- STATE-NAV（2025）：稳定性感知可通行性（含 Transformer），`https://arxiv.org/abs/2506.01046`  
- Online Adaptive Traversability（2025）：交互式在线自适应，`https://arxiv.org/abs/2502.01987`  
- I Move Therefore I Learn（2025）：经验驱动可通行性，`https://arxiv.org/abs/2507.00882`  
- elevation_mapping_cupy（持续维护）：GPU 多模态高程与 traversability，`https://github.com/leggedrobotics/elevation_mapping_cupy`

对比结论：  
- 当前 `rc26_terrain` 不算“过时到不可用”，在结构化赛场有工程优势（简单、可控、可解释）。  
- 但相对 2024-2026 主流方案，缺少三项能力：语义融合、不确定度建模、在线自适应。  
- 决赛前不建议整包替换为深度模型；建议“几何主链 + 规则补丁 + 轻量语义注入”渐进升级。

## 风险识别

### R1（致命）：无法区分“可通行方块”与“规则禁踩方块（有 KFS）”

- 证据：当前输入仅 `registered_scan + odom + TF`，无 KFS 占位/真假标签输入（`terrain_semantic_node.hpp:46-53`，`terrain_semantic_node.cpp:65-70`）。  
- 规则冲突：`主赛规则.md:201`、`主赛规则.md:290`。  
- 后果：规划可能走上“几何可通行但规则禁止”的方块，触发强制重试。

### R2（高）：邻域定义与规则“公共边相邻”不一致

- 证据：8 邻域遍历 `dx,dy=-1..1`，包含对角邻接（`terrain_semantic_node.cpp:574-587`）。  
- 规则口径：`主赛规则.md:196` 明确“公共边相邻，不含对角”。  
- 后果：对角高差可能误触发障碍/跌落，影响 R2 路径选择与取件策略。

### R3（高）：障碍标注归属偏“低台单元”，导致靠边可达性受损

- 证据：`diff = ground[nidx]-ground[idx]` 后直接将 `idx` 判为障碍（`terrain_semantic_node.cpp:584-593`）。  
- 后果：靠近高台边界时，低台邻近区可能被提前封死，影响接近动作。

### R4（高）：无对象级聚类，稀疏噪点通过 `h_above` 触发误障碍

- 证据：`h_above = top-ground` 直接参与障碍判定（`terrain_semantic_node.cpp:590-593`），无连通域面积阈值。  
- 后果：反光/偶发高点可能触发局部误阻塞，导致局部规划绕行抖动。

### R5（中）：默认 `unknown_policy=aggressive` 在材质不稳定区存在漏检

- 证据：`terrain_semantic.yaml:58-63`。  
- 场景：2026 赛场白胶带、木面反射差异、入射角变化导致局部稀疏时，unknown 被忽略。  
- 后果：高速穿梭时安全边界变薄，尤其在边缘遮挡区域。

### R6（中）：跌落仅前向扇区，转向期可能出现侧向瞬时盲区

- 证据：`drop_forward_sector_deg=180`（`terrain_semantic.yaml:109`），判定逻辑 `terrain_semantic_node.cpp:661-670`。  
- 后果：原地转向或侧移阶段，边缘保护一致性下降。

### R7（中）：性能热点来自多次点云拷贝与全量栅格遍历

- 证据：`transformPointCloud -> fromROSMsg -> voxel.filter`（`terrain_semantic_node.cpp:856-870`）+ 两次全量 `num_cells` 循环（`terrain_semantic_node.cpp:524`、`terrain_semantic_node.cpp:632`）。  
- 影响：在高点数输入下容易侵占控制周期，放大到 costmap 与模式管理链路。

### R8（中）：与模式管理耦合存在“刷新超时”风险

- 证据：`nav_mode_manager` 依赖 `terrain_obstacles` 时间戳更新确认重建（`src/rc26_nav_mode_manager/README.md:32`），默认重建等待 `0.5s`（`src/rc26_nav_mode_manager/README.md:79`）。  
- 后果：地形节点抖动会触发误回退，影响 MVP 流程节拍。

## 优化清单

### P0（赛前必须做，规则闭环）

1. 增加 KFS 占位禁行融合（优先 keepout filter）  
- 建议位置：`terrain_semantic_node.hpp` 新增 KFS 占位订阅与栅格掩码；`terrain_semantic_node.cpp:523-623` 强制把占位方块标为 obstacle。  
- 结果：对齐 `主赛规则.md:201/290`，避免“几何可走、规则不可走”。

2. 将邻域从 8 邻域改为 4 邻域（或可配置）  
- 修改点：`terrain_semantic_node.cpp:574-587`。  
- 建议：仅保留 `(±1,0)/(0,±1)`，并提供 `neighbor_mode: edge8/edge4` 参数。  
- 结果：与“公共边相邻”规则一致，减少对角误判。

3. 修正障碍归属策略  
- 修改点：`terrain_semantic_node.cpp:590-593`。  
- 建议：把“高台不可攀”优先标记到高侧单元 `nidx` 或边界单元，不直接封死低侧通道单元。  
- 结果：提升接近边缘时的可达性与稳定性。

### P1（稳定性增强）

1. 引入对象级最小连通域过滤  
- 修改点：`terrain_semantic_node.cpp:625-724` 发布前增加 2D 连通域面积阈值。  
- 结果：抑制孤立噪点障碍。

2. MF 模式下切换 `unknown_policy=conservative`  
- 修改点：参数配置 `terrain_semantic.yaml:58-63`，配合模式管理动态设参。  
- 结果：高速期优先安全，不以漏检换速度。

3. 转向/台阶顶面阶段启用全向跌落保护  
- 修改点：`terrain_semantic.yaml:109` 或运行时动态调参。  
- 建议：`drop_forward_sector_deg=360`（至少在 `MF_TRAVERSE` 高风险段）。

4. 为地面估计加入“跳变快速收敛”  
- 修改点：`terrain_semantic_node.cpp:515-516`。  
- 建议：当 `|ground_z - ground_z_filtered_| > jump_thresh` 时临时 `alpha=1.0`。  
- 结果：减轻跨台阶瞬态漂移。

### P2（性能与工程质量）

1. 减少点云拷贝链  
- 修改点：`terrain_semantic_node.cpp:856-870`。  
- 建议：优先直接在 PCL 结构上变换/滤波，减少中间 ROSMsg↔PCL 转换。

2. 全量循环改“活跃单元循环”  
- 修改点：`terrain_semantic_node.cpp:524`、`terrain_semantic_node.cpp:632`。  
- 建议：维护 `active_cells`（最近 `stale_time_sec` 内观测到的单元）并增量衰减。

3. 输出诊断增加延迟指标  
- 修改点：`publishDiagnostics`（`terrain_semantic_node.cpp:413-454`）。  
- 建议：新增 `cloud_to_publish_latency_ms`、`frame_points_before/after_voxel`、`active_cells_count`。

### P3（技术演进，不阻塞比赛）

1. 保留当前几何主链，外接轻量高程/可通行层  
- 可评估：`elevation_mapping_cupy` 思路（多层高程 + traversability）。  

2. 深度模型仅做“辅助风险层”，不替换主链  
- 候选方向：TE-NeXt / RoadRunner / STATE-NAV 类方法离线蒸馏后输出附加风险分数。  
- 原则：不影响主链实时性与可解释性，先 A/B 验证再接入实车。

---

### 总结

`rc26_terrain` 作为 R2 的几何地形感知底座是可用的，但当前最大问题不是“算得慢”，而是“规则语义缺口”：不能识别并规避“有 KFS 的禁踩方块”。  
建议按 `P0 -> P1 -> P2` 顺序推进，先完成规则闭环，再做鲁棒性与性能打磨。
