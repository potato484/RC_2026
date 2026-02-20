# rc26_terrain 地形感知模块技术评估报告

> **评审日期**: 2026-02-18
> **评审范围**: `src/rc26_terrain/` 全模块
> **核心文件**: `terrain_semantic_node.cpp` / `terrain_semantic_node.hpp` / `terrain_semantic.yaml`
> **对标规则**: `MVP技术方案/决策/主赛规则.md`

---

## 一、现状分析

### 1.1 模块定位

`TerrainSemanticNode` 是一个以机器人为中心的**局部滚动栅格高程语义引擎**。输入：LiDAR 点云（`registered_scan`） + TF + 里程计；输出：三类语义点云供 Nav2 Costmap 消费：

| 输出话题 | 含义 |
|---|---|
| `terrain_obstacles` | 不可通行障碍（含可攀爬方块边沿外的障碍） |
| `terrain_drop` | 跌落/悬崖（方块侧面向下边沿） |
| `terrain_climbable` | 可越障台阶（调试用，不接入 costmap） |

核心算法链：体素下采样 → 局部栅格投影 → 分位数地面/顶部估计 → EMA 平滑 → 差分邻域 → 迟滞积分状态机 → 语义发布。

### 1.2 赛场地形约束还原

根据规则 §1.2.2，梅林由 12 块方块构成，以**红方**为例：

```
行 1 (1-2-3):   400H - 200H - 400H   ← 与地面/R2入口相邻
行 2 (4-5-6):   200H - 400H - 600H   ← 高度差最大的行
行 3 (7-8-9):   400H - 600H - 400H
行 4 (10-11-12):200H - 400H - 200H   ← 靠近对抗区
```

关键边界台阶（仅相邻方块，`规则§4.4.13` 定义相邻为公共边）：

| 相邻对 | 高度差 |
|---|---|
| 地面 → 200H 方块 | 200mm |
| 200H ↔ 400H | 200mm |
| 400H ↔ 600H | 200mm |
| 同行最大非相邻跨越（地面→600H） | 600mm（R2 不需要直接跨越） |

**所有相邻方块高度差均为 200mm**，而 `h_climb_m: 0.30m` 将 200mm 台阶正确分类为可攀爬。该核心参数匹配规则需求。

### 1.3 算法流程概要

```
cloudCallback()
 ├─ TF 查询 (target←base, target←cloud_frame)
 ├─ pcl_ros::transformPointCloud → target_frame
 ├─ VoxelGrid 下采样 (leaf=0.05m)
 ├─ 点迭代 → 局部栅格投影 → cell_z_samples_[cell].push_back(p.z)
 ├─ estimateCellHeights()
 │    └─ per touched_cell: quantile(ground_q=0.25), quantile(top_q=0.95), EMA 更新
 ├─ classifyAndUpdate()
 │    └─ per ALL cells: 邻域差分 → obstacle/drop 候选 → 迟滞积分状态机
 └─ publishOutputs()
      └─ per ALL cells: 生成语义点云 → toROSMsg → publish
```

---

## 二、风险识别

### 风险 R1 ⚠️ 关键缺口：无 KFS-on-block 感知集成

**规则约束**（`§4.4.12 / 罚则 §8.9`）：R2 底盘不得接触**有 KFS 的方块顶面**，否则强制重试。

**当前实现**：`terrain_semantic_node.cpp` 完全基于高度统计，无任何 KFS 占用信息。模块仅能输出"物理可/不可通行"，不能标识"物理可通行但规则禁止"的 KFS 占用方块。

**风险等级**：**致命**。Nav2 在规划时不会回避有 KFS 的方块，R2 可能直接踏上被占用方块导致强制重试。

**所需补充**：上游 KFS 检测节点必须将检测到的 KFS 位置（世界坐标）转换为 Nav2 代价地图的静态禁区（`keepout_filter` 或自定义代价图层），或通过向 `terrain_obstacles` 注入虚拟障碍点的方式标记禁行区域。

---

### 风险 R2 ⚠️ 障碍标注位置歧义

**代码位置**：`terrain_semantic_node.cpp:570-593` (`classifyAndUpdate`)

```cpp
// Line 570-588: 遍历 8 邻域计算 dz_up
const float diff = ground_z_filtered_[nidx] - ground_z_filtered_[idx];
dz_up = std::max(dz_up, diff);

// Line 591-593: 判定并标注"当前单元格"为障碍
const bool obstacle_candidate =
    (dz_up > static_cast<float>(h_obstacle_m_)) ||
    (h_above > static_cast<float>(h_obstacle_m_));
```

当前逻辑：从单元格 `(ix, iy)` 看到邻域有高台阶时，将 `(ix, iy)` 标为障碍。这意味着 R2 **当前所在格子**会被标为障碍，而非阻挡它的**高台格子**。

实际后果：
- R2 在地面朝 400H 方块（0.4m > h_obstacle_m=0.33m）行驶时，地面格子被标为障碍，可能造成 Nav2 认为当前位置不可进入，干扰局部轨迹规划。
- 正确语义应为：高台格子本身（或其边界）标为障碍，低处格子保持可通行。

---

### 风险 R3 ⚠️ 400mm 方块对 R2 的障碍误标问题

**数值分析**：

- R2 从地面（0mm）看 400H 方块：台阶高度 400mm = 0.4m > `h_obstacle_m=0.33m` → 标为障碍 ✗
- 但从 200H 方块（200mm）看 400H 方块：台阶高度 200mm = 0.2m < `h_climb_m=0.30m` → 标为可攀爬 ✓

这意味着：**R2 必须经由 200H 中转方块才能进入 400H/600H 方块**，否则直接从入口区面对 400H+ 方块时地形模块会拦截导航。

**前提确认**：这是否符合实际场地设计？若 R2 入口区紧邻 200H 方块（行 1），路径合理；若入口直面 400H+ 方块，则 `h_obstacle_m` 需上调至 ≥ 0.42m 以允许该入口台阶被识别为可攀爬。**需实地测量确认入口区与第一行方块的高度关系**。

---

### 风险 R4 ⚠️ 侧向跌落盲区

**代码位置**：`terrain_semantic_node.cpp:661-670` (`publishOutputs`)

```cpp
if (drop_forward_sector_deg_ < 360.0) {
    if (x_rel < drop_forward_min_x_m_) {
        in_sector = false;
    } else {
        double angle_rad = std::abs(std::atan2(y_rel, x_rel));
        double sector_half_rad = drop_forward_sector_deg_ * 0.5 * M_PI / 180.0;
        if (angle_rad > sector_half_rad) { in_sector = false; }
    }
}
```

**配置值**：`drop_forward_sector_deg: 180.0`，即只检测前向 180° 扇区的跌落风险。

当 R2 在 600H 方块顶部**横向移动**时，左右侧方向（±90°附近）的跌落边沿也被 180° 扇区覆盖；但当 R2 开始**旋转原地调头**时，原本的"前方"跌落点会短暂落入扇区外，造成 `terrain_drop` 清空，Nav2 暂时解除禁行，存在侧溜风险。

---

### 风险 R5 ☕ EMA 在滚动栅格中的语义漂移

**代码位置**：`terrain_semantic_node.cpp:514-516`

```cpp
ground_z_filtered_[idx] = static_cast<float>(ground_ema_alpha_) * ground_z +
                          static_cast<float>(1.0 - ground_ema_alpha_) * ground_z_filtered_[idx];
```

滚动栅格以机器人为中心，`cell[idx]` 的世界坐标随机器人运动而改变。当 R2 跨越台阶后，格子 (5,5) 的历史 EMA 值（原属于旧地形）会被新观测稀释，`alpha=0.6` 下约需 2~3 帧才能收敛。在 10Hz LiDAR 刷新率下，这导致台阶过渡后约 200~300ms 的地面高度误差，可能触发一过性虚假 drop/obstacle 告警。

---

### 风险 R6 📊 性能瓶颈：全量遍历 vs 触碰遍历

**代码位置**：`terrain_semantic_node.cpp:524` / `terrain_semantic_node.cpp:632`

```cpp
// classifyAndUpdate: 遍历所有 num_cells_
for (int cell = 0; cell < num_cells_; cell++) { ... }

// publishOutputs: 再次遍历所有 num_cells_
for (int cell = 0; cell < num_cells_; cell++) { ... }
```

栅格规模：`perception_radius=3.2m`，`resolution=0.1m` → `width≈65`，`num_cells≈4225`。两次全量遍历 + 每格 8 邻域查询，每帧约 **4225×2×8 = 67,600 次内存访问**。在 QCS8550 平台（ARM，cache 敏感）上，`std::vector<double> last_seen_sec_` 与 `std::vector<float> ground_z_filtered_` 的非连续访问模式会导致 cache miss 放大。

`publishOutputs` 中的 climbable 检测（line 699-713）重复了 `classifyAndUpdate` 中已完成的邻域差分计算，存在冗余计算。

---

### 风险 R7 📋 `h_above` 条件可能将 KFS 本身标为障碍

**代码位置**：`terrain_semantic_node.cpp:590-592`

```cpp
const float h_above = top_z_[idx] - ground_z_filtered_[idx];
const bool obstacle_candidate =
    (dz_up > static_cast<float>(h_obstacle_m_)) ||
    (h_above > static_cast<float>(h_obstacle_m_));   // ← h_above 条件
```

KFS 体积：350mm × 350mm × 350mm。当 KFS 放在方块顶面，LiDAR 会同时采到方块顶面点和 KFS 上表面点，导致该格子 `h_above ≈ 0.35m > h_obstacle_m=0.33m`，被标为障碍。

- **正面效果**：KFS 位置在 `terrain_obstacles` 中有点，Nav2 会在其周围生成代价膨胀，自然引导 R2 绕行（不踩）。
- **隐患**：方块侧面格子的 `top_z` 采到 KFS 顶部点时，也可能触发 `h_above` 条件，使方块侧面被误标为障碍，阻碍 R2 接近该方块进行 KFS 采集（规则允许与方块侧面接触）。

---

## 三、优化清单

### 优先级 P0：功能正确性

| # | 问题 | 建议 |
|---|---|---|
| O1 | KFS 占用方块无法标记为禁区 | 在 Nav2 代价地图中增加 `keepout_filter` 图层，由 KFS 检测节点发布禁行区域多边形；或创建 `KfsBlockMarkingNode` 订阅 KFS 检测结果，将检测到的 KFS 方块中心+1200mm 边界区域注入 `terrain_obstacles` 虚拟点 |
| O2 | 障碍标注位置应在高台格子，非低台格子 | `classifyAndUpdate` 中，当 `ground_z_filtered_[nidx] - ground_z_filtered_[idx] > h_obstacle_m_` 时，将邻域格子 `nidx` 标记为 obstacle candidate，而非 idx 本身 |
| O3 | 确认 R2 入口区高度 | 实地测量入口区与行1方块（200H/400H）的高度关系；若入口对面直接是 400H，则调整 `h_obstacle_m` 至 0.42m 以上，或在入口区使用 `conservative` unknown 策略临时降低进入门槛 |

### 优先级 P1：安全可靠性

| # | 问题 | 建议 |
|---|---|---|
| O4 | 侧向跌落盲区 | 将 `drop_forward_sector_deg` 改为 `270.0` 或 `360.0`（全向检测）；或在 R2 方块顶部导航状态激活时，由状态机临时切换为全向检测模式 |
| O5 | EMA 台阶过渡抖动 | 在 `estimateCellHeights` 中检测 `|ground_z - ground_z_filtered_| > 0.15f`（跨台阶跳变）时，强制覆盖 EMA（`alpha=1.0`），跳过历史平滑；避免过渡期虚假告警 |
| O6 | KFS 侧面格子误标障碍 | 增加 `h_above` 条件的约束：仅当 `h_above > h_obstacle_m_ && dz_up < h_obstacle_m_`（即纯顶部物体且无台阶跳变）时，考虑附近是否真的是方块侧面；或将 `h_obstacle_m` 上调至 0.40m 以上（350mm KFS 需要同时满足 ground_quantile + top_quantile 才能达到 0.40m h_above）|

### 优先级 P2：性能优化

| # | 问题 | 代码位置 | 建议 |
|---|---|---|---|
| O7 | `classifyAndUpdate` 全量遍历 | `cpp:524` | 维护 `active_cells_` 集合（有历史数据的格子），仅遍历 active_cells_；radius 内空闲格子按时间衰减延迟处理 |
| O8 | `publishOutputs` 重复邻域计算 | `cpp:699-713` | 在 `classifyAndUpdate` 中计算 climbable 状态并存入 `climbable_state_[]` 向量，`publishOutputs` 直接读取，消除重复遍历 |
| O9 | `cell_z_samples_` 堆碎片 | `cpp:487` / `hpp:139` | 将 `vector<vector<float>>` 改为 `vector<float> cell_z_flat_` + `vector<int> cell_z_offset_`（连续内存），减少堆分配压力；或改用预分配的 `array<float, MAX_SAMPLES_PER_CELL>` + `uint8_t cell_z_count_[]` |

### 优先级 P3：技术现代化

| 方向 | 当前状态 | 现代替代方案 |
|---|---|---|
| 地面估计 | 分位数 + EMA | ANYbotics `elevation_mapping` (GPM Kalman) —— 针对非平坦地面提供方差估计，更鲁棒 |
| 障碍分类 | 几何规则差分 | **VertiFormer** (2025) 提供数据高效的多任务 Transformer，可在 QCS8550 INT8 推理下 < 20ms 完成可通行性分类；但需标注数据集，比赛周期内**风险较高** |
| 实时遍历性 | 单层高程栅格 | SEL Map (Semantic Elevation Layer) —— 结合语义与几何的双层表示，无需改动 Nav2 接口 |

> **评估结论**：对于 Robocon 4 分钟竞技场景，已知结构化地形（12 块固定方块），**当前几何规则方案是合理选择**。引入 Transformer 的收益（精度）远低于风险（调试、算力、训练数据），不建议在决赛前替换核心算法。应聚焦 P0/P1 修复。

---

## 四、规则对标总结

| 规则条款 | 当前实现覆盖？ | 备注 |
|---|---|---|
| §8.9 R2 不踩有 KFS 方块 | **否** | 模块无 KFS 感知能力（R1 最高风险） |
| §4.4.12 相邻方块 200mm 台阶可攀爬 | 是（h_climb=0.30m） | 参数匹配 |
| §4.4.13 R2 不得运动到有 KFS 方块 | **否** | 同 §8.9 |
| §1.2.2 方块高度 200/400/600mm | 部分覆盖 | 200mm step ok；直面 400mm step 需确认 |
| 规则无约束：方块侧面接触允许 | 存在误标风险 | O6 优化项 |
| 失效保护（TF 丢失/点云中断） | 是 | virtual_fence 策略完整 |
| 迟滞防闪烁 | 是 | 积分状态机实现正确 |

---

## 五、核心结论

```
最高优先级修复项（影响比赛结果）：
  O1: KFS 占用方块禁区标注 → 与 KFS 检测模块联动
  O2: 障碍标注位置纠正 → classifyAndUpdate 邻域标注逻辑修正
  O3: 入口高度实地确认 → h_obstacle_m 参数校准

次优先级（影响稳定性）：
  O4: 全向跌落检测 → drop_forward_sector_deg 配置调整
  O5: 台阶过渡 EMA 跳变保护 → 单次代码修改

性能项（当前平台可接受，赛前余力时处理）：
  O7-O9: 内存与遍历优化
```

> 整体评估：地形感知引擎架构清晰、失效保护完善、参数体系合理。**最大风险不在算法精度，而在与 KFS 检测的集成缺口**。修复 O1-O3 后，该模块可支撑 R2 梅林穿越的基础导航需求。
