# RC_2026 `small_gicp` 赛季胜任性深度评估（2026-02-18）

## 1. 结论先行

- `small_gicp` **适合继续作为 R2 的后端地图匹配/重定位核心**（`map->odom` 修正、绑架恢复）。
- `small_gicp` **不适合作为单一主定位前端去承担 50Hz+ 全链路定位**；它在当前仓库本来就是低频后端。
- 2026 赛季最稳妥选型仍是：**高频 LIO 前端（Point-LIO 系）+ `small_gicp` 后端**，而不是用 `small_gicp` 单独替换整条定位链。

## 2. 证据分级（避免拍脑袋）

- 高置信（官方/代码/论文原文）：
  - `small_gicp` 官方 `BENCHMARK.md`、`fast_gicp` 官方 README、`ndt_omp` 官方 README。
  - 本仓库 `rc26_localization` 与 `localization.yaml` 实现。
  - arXiv 2025 论文：`Super-LIO`（`arxiv:2509.05723v1`）、`Adaptive-LIO`（`arxiv:2503.05077v1`）。
  - 规则文件：`MVP技术方案/决策/主赛规则.md`。
- 中置信：学术门户/仓库镜像摘要（用于交叉印证，不作为唯一依据）。
- 低置信：二次转载博客、未给可复现实验配置的结论（本报告未作为核心结论依据）。

## 3. 本仓库现状（关键事实）

- `rc26_localization` 中 `small_gicp` 配准定时器是 **2Hz**（500ms），TF 发布是 20Hz，不是 50Hz 主定位前端：
  - `src/rc26_localization/src/localization.cpp:213`
  - `src/rc26_localization/src/localization.cpp:218`
- 当前默认参数偏“稳态后端”：`num_threads:4`、`gicp_max_iterations:20`、`global_leaf_size:0.25`：
  - `src/rc26_bringup/config/localization.yaml:41`
  - `src/rc26_bringup/config/localization.yaml:51`
  - `src/rc26_bringup/config/localization.yaml:45`
- `small_gicp` 官方代码明确提示多线程下存在轻微非确定性：
  - `num_threads>=2` 时有 run-by-run non-determinism：
    - `src/small_gicp/include/small_gicp/registration/registration_helper.hpp:13`
  - 并行降采样点数相对单线程可能出现最多约 10% 偏差：
    - `src/small_gicp/include/small_gicp/util/downsampling_omp.hpp:17`

## 4. 联网性能对标（CPU 非 GPU）

> 说明：`small_gicp` 官方 2024/2025 公开 benchmark 以“倍率/曲线”为主；`fast_gicp` 与 `ndt_omp` 给出了具体 ms。2025-2026 新文献里，直接把四者放同一硬件同一数据集对跑的公开表很少，需做“可比性标注”。

| 算法 | 公开 CPU 指标（可核查） | 线程扩展性 | 非GPU下 50Hz 可行性 | 备注 |
|---|---|---|---|---|
| `small_gicp` | 官方 `BENCHMARK.md`：单线程 GICP 约比 `pcl::GICP` 快 2.4x、比 `fast_gicp::GICP` 快 1.9x；并行扩展优于 `fast_gicp` | 明确强调 OMP/TBB 多线程可扩展，TBB flow 在多核场景扩展更强 | **作为后端可行**；作为“单前端 50Hz”需实机复测点数和参数 | 官方结果偏“相对速度”而非固定 ms |
| `Fast-GICP` | README 基准：`fgicp_mt` 单次约 20.16ms（约 49.6Hz）；`vgicp_mt` 18.11ms（约 55.2Hz） | 多线程强，但官方也提示线程数过大不一定更快 | **边缘可达 50Hz**（取决于点数/线程/是否复用） | CPU i9-9900K、样例点云约 17k |
| `NDT_OMP` | README 基准（i7-6700K）：`DIRECT1,8线程` 17.24ms（约 58.0Hz）；`DIRECT7,8线程` 63.14ms（约 15.8Hz） | 对模式敏感（DIRECT1 快但稳定性风险更高） | **模式相关**：DIRECT1 可达，DIRECT7/KDTREE 难达 50Hz | 官方推荐 DIRECT7 兼顾稳定 |
| Point-LIO（CPU优化路线，2025） | `Adaptive-LIO` 2025 文中对照：Point-LIO 在其 Robot Town 实验中约 7.98/11.31/10.54ms 每帧；`Super-LIO` 2025 报告对 SOTA 在 CPU 侧有更高效率 | Point-LIO 家族整体偏高频前端设计 | **高频前端可行性强** | Point-LIO 原论文/README主打 4k-8kHz 输出带宽（点级更新） |

### 4.1 对“small_gicp 多线程是否优于传统方案”的回答

- 结论：**在官方公开证据里是“更优或不弱”**。
- 依据：`small_gicp` 官方 benchmark 明确给出相对 `fast_gicp`/PCL 的优势与更好并行扩展；代码层面确实全链路 OMP/TBB 并行化（降采样、协方差、KDTree、reduction）。
- 限定：同一硬件、同一点云分布、同参数下的“2026 最新第三方横评”公开数据不足，建议你队内实机 A/B 收敛确认。

## 5. 规则与赛场环境对标

### 5.1 规则抽取到的定位风险源

- 室内场地 + 12 个规则方块 + 镜像高度布局，存在重复几何与对称歧义：
  - `MVP技术方案/决策/主赛规则.md:34`
  - `MVP技术方案/决策/主赛规则.md:45`
- 对抗区存在亚克力护栏/亚克力九宫格，透明材质会引入激光异常回波/幽灵点风险：
  - `MVP技术方案/决策/主赛规则.md:16`
  - `MVP技术方案/决策/主赛规则.md:67`
- 假 KFS、对手与本队机器人都构成高动态干扰源：
  - `MVP技术方案/决策/主赛规则.md:124`
- R2 必须自动、强制重试 15s，要求“可重复恢复”而不是单次最好结果：
  - `MVP技术方案/决策/主赛规则.md:91`
  - `MVP技术方案/决策/主赛规则.md:300`

### 5.2 `small_gicp` 在该赛场的漂移风险判断

- 在 MF 方块区：几何特征丰富，`small_gicp` 通常可稳定收敛。
- 在 MC/BZ 平整区 + 亚克力附近：
  - 点云有效几何约束下降，易出现 yaw/平移弱可观；
  - 动态点和透明件异常回波会污染配准内点。
- 多线程轻微非确定性会放大“重复定位精度”抖动（尤其在特征边缘场景）。

## 6. 规则适配性得分（0-100）

> 打分对象是 `small_gicp` 在 **RC_2026 R2 自动任务**中的适配度。分别看“单独使用”和“当前双层架构使用”。

| 维度 | 权重 | `small_gicp` 单独承担主定位 | `Point-LIO前端 + small_gicp后端` |
|---|---:|---:|---:|
| 50Hz 实时性 | 25 | 45 | 85 |
| 重复定位一致性 | 20 | 62 | 76 |
| 对重复几何抗退化 | 20 | 58 | 72 |
| 动态/透明干扰鲁棒 | 20 | 52 | 68 |
| 强制重试后恢复能力 | 15 | 60 | 88 |
| **总分** | 100 | **55.3** | **77.3** |

解读：
- `small_gicp` **不建议单独担任 R2 全栈主定位**。
- 在你们当前双层架构中，`small_gicp` **是胜任且关键**的后端组件。

## 7. 潜在风险点（针对赛场特征）

1. 重复几何 + 镜像布局导致初值不佳时局部极小值风险上升。
2. 亚克力护栏/九宫格导致异常回波，配准内点质量下降。
3. 对手与本队机械臂/KFS 动态遮挡造成瞬时错误对应。
4. 多线程并行降采样非确定性，影响“重启后同点位重复精度”。
5. 当前后端 2Hz 设计天然不承担高频控制闭环，若误当主定位会出现“频率错配”。
6. 本仓库 `small_gicp` 为裁剪版（benchmark 目标所需源码不完整），本地难以直接复现官方 benchmark。

## 8. 最终技术选型建议

### 8.1 主建议（2026赛季）

- 维持并强化：**`Point-LIO（或同级高频LIO前端） + small_gicp 后端`**。
- 不建议把 `small_gicp` 改成单前端主定位。

### 8.2 如果你要进一步提高“重复定位精度”

1. 把 `small_gicp` 关键重定位通道做“确定性优先”配置（例如重定位阶段限制线程或固定调度）。
2. 对 BZ 亚克力区域做 map/ROI 剔除与动态点降权，减少幽灵点进入配准。
3. 维持当前 L1/L2 重定位状态机，但把候选初值更强绑定“重试区先验”。
4. 评估 NDT_OMP 作为粗配准候选（DIRECT7 稳定、DIRECT1 仅在算力紧张时谨慎启用），再交给 `small_gicp` 精配准。

## 9. 参考来源（联网）

1. 主赛规则：`/home/potato/RC_2026/MVP技术方案/决策/主赛规则.md`
2. small_gicp benchmark（官方 raw）：https://raw.githubusercontent.com/koide3/small_gicp/master/BENCHMARK.md
3. fast_gicp README（官方 raw）：https://raw.githubusercontent.com/koide3/fast_gicp/master/README.md
4. ndt_omp README（官方 raw）：https://raw.githubusercontent.com/koide3/ndt_omp/master/README.md
5. Point-LIO README（官方 raw）：https://raw.githubusercontent.com/hku-mars/Point-LIO/point-lio-with-grid-map/README.md
6. Point-LIO 条目信息（HKUST）：https://repository.hkust.edu.hk/ir/Record/1783.1-156320
7. Super-LIO（2025）：https://arxiv.org/html/2509.05723v1
8. Adaptive-LIO（2025）：https://arxiv.org/html/2503.05077v1

