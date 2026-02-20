# RC_2026 全局重定位方案深度评估（`rc26_localization`）

## 1. 评估依据

### 1.1 代码证据（本仓库）
- 全局链路实现：`src/rc26_localization/src/localization.cpp:663`、`src/rc26_localization/src/localization.cpp:755`、`src/rc26_localization/src/localization.cpp:848`、`src/rc26_localization/src/localization.cpp:875`、`src/rc26_localization/src/localization.cpp:893`
- 绑架检测与超时触发：`src/rc26_localization/src/localization.cpp:490`、`src/rc26_localization/src/localization.cpp:561`、`src/rc26_localization/src/localization.cpp:609`
- 多假设策略：`src/rc26_localization/src/localization.cpp:631`
- 重试区快速通道：`src/rc26_localization/src/localization.cpp:744`、`src/rc26_localization/src/localization.cpp:1072`
- 实际参数（当前配置）：`src/rc26_bringup/config/localization.yaml:67`-`src/rc26_bringup/config/localization.yaml:156`

### 1.2 规则证据（你指定文档）
- 场地动态与重复结构：`MVP技术方案/决策/主赛规则.md:33`-`MVP技术方案/决策/主赛规则.md:61`
- R2 必须自动：`MVP技术方案/决策/主赛规则.md:91`
- 强制重试与重试区机制：`MVP技术方案/决策/主赛规则.md:283`-`MVP技术方案/决策/主赛规则.md:309`
- “动态定位”难点：`MVP技术方案/决策/主赛规则.md:340`-`MVP技术方案/决策/主赛规则.md:343`

### 1.3 不确定性声明
- 你补充的算力平台为：QCS8550（1+4+3 Kryo CPU、Adreno 740、~48 TOPS NPU、16GB LPDDR5x、AidLux Ubuntu22.04）。
- 当前 `rc26_localization` 代码路径是 PCL + OpenMP + small_gicp（`src/rc26_localization/CMakeLists.txt`），未接入 GPU/NPU 推理或加速算子，因此重定位瓶颈主要在 CPU 大核和内存带宽，而不是 48 TOPS NPU。

---

## 2. 核心问题诊断

## 2.1 链路冗余性：当前五级链路偏长，且在实战中存在“串行放大”风险

当前全局链路是：
`ISS -> FPFH -> SAC-IA -> 多假设(yaw N份) -> NDT(可选) -> ICP`

关键问题：
- `num_yaw_hypotheses` 当前配置为 8（`src/rc26_bringup/config/localization.yaml:131`），且每个候选都可能跑 NDT+ICP（`src/rc26_localization/src/localization.cpp:871`-`src/rc26_localization/src/localization.cpp:905`）。
- NDT 与 ICP 都属于密集配准精化，目标函数高度重合；在“初值已较好”时双跑通常收益有限、耗时显著上升。
- 局部配准频率是 2Hz（`src/rc26_localization/src/localization.cpp:229`），当全局重定位运行时常规配准直接跳过（`global_reloc_running_` 早退），会放大重定位耗时的系统影响。

结论：
- 在 RoboMaster 的实时对抗场景下，这条链路更像“离线鲁棒链路”而不是“比赛在线链路”。
- `NDT + ICP` 不应长期同时开启；建议改为“条件启用 NDT，常态仅 ICP/GICP 精化”。

---

## 2.1.1 QCS8550 约束下的耗时风险复盘（新增）

基于当前代码实现方式（同一 `num_threads` 同时驱动 ISS/FPFH/small_gicp OMP 计算），在 QCS8550 上的关键风险是：
- 线程数开太高会把任务推到 Silver 核，反而增加长尾延迟与抖动。
- 全局重定位的候选循环是串行的（`for candidates`），即使单候选并行，也会出现总时延线性累加。
- NDT 与 ICP 叠加在每个候选上，易造成“恢复成功率提升有限，但耗时翻倍”。

结合你当前配置（`num_threads:4`、`num_yaw_hypotheses:8`、`global_icp_max_iterations:100`、`use_ndt_refinement:true`），保守判断：
- 局部跟踪（2Hz）通常可守住，但全局恢复长尾风险明显。
- 比赛中若碰到连续重试/动态遮挡，可能出现秒级到多秒级恢复窗口，影响 R2 连续作业节奏。

---

## 2.2 逻辑重合度：绑架检测、自动恢复、多假设存在决策层重复

当前有 4 类入口同时影响重定位：
- 连续高误差触发（`detectKidnapping`）：`src/rc26_localization/src/localization.cpp:609`
- 长时间无成功配准超时触发：`src/rc26_localization/src/localization.cpp:561`
- 重试区先验快速通道：`src/rc26_localization/src/localization.cpp:744`
- 外部 `initialpose` 强制重置：`src/rc26_localization/src/localization.cpp:966`

重复性表现：
- “何时进入恢复态”由多个分支独立判定，缺少统一状态机，导致策略和参数调优耦合。
- “多假设”既在全局链路内部处理，又在重试区快速通道中以另一套候选机制处理，逻辑语义接近但分散。

结论：
- 建议合并为统一恢复状态机（而不是继续堆 if-else 分支）。
- 粒子滤波可以做，但对当前项目更高性价比的是“有限状态机 + TopK候选打分”，工程风险更低。

---

## 2.3 算法合理性：`ISS+FPFH` 在该赛场的区分度存在天然上限

结合规则中的场地特性：
- MF 区由大量尺寸一致方块构成，顶部白胶带区域高度重复（`MVP技术方案/决策/主赛规则.md:34`-`MVP技术方案/决策/主赛规则.md:57`）。
- 红蓝半场存在镜像结构（`MVP技术方案/决策/主赛规则.md:45`-`MVP技术方案/决策/主赛规则.md:55`）。
- 比赛中 KFS、对手、己方机械臂都会引入动态遮挡与局部几何变化（`MVP技术方案/决策/主赛规则.md:283`-`MVP技术方案/决策/主赛规则.md:309`）。

对 `ISS+FPFH+SAC-IA` 的影响：
- ISS 关键点在规则化台阶和边缘上会出现“可重复但不够唯一”的特征点。
- FPFH 对局部法向统计敏感，在重复平面/阶梯结构里容易出现特征混淆。
- 代码中若 ISS 点过少会回退全点云 FPFH（`src/rc26_localization/src/localization.cpp:795`），进一步增加计算负担并不必然提升判别性。

结论：
- `ISS+FPFH` 更适合作为“兜底全局检索”而非“每次恢复主通道”。
- 主通道应优先使用更强全局场景描述（如 Scan Context/Intensity SC）+ 小规模候选精配准。

---

## 2.4 联网检索（2024-2026）结论

## A. 赛事侧趋势
- ICRA 2024/2026 官方竞赛列表中未检索到名为 “RoboMap” 的正式赛项；更接近的是导航与地图相关挑战（如 BARN、GPR2024）。
- ICRA 2024 竞赛页明确包含 BARN 与 GPR2024（Crowdsourced Map Association），强调在受限资源下的可靠导航/地图关联。
- ICRA 2026 BARN 描述明确“标准化车体 + 2D LiDAR + 指定 onboard 算力资源”，体现“轻量高效”导向。

## B. 工程实现侧趋势（开源实践）
- `FAST-LIO + Scan Context` 已形成稳定组合：
  - `gisbi-kim/FAST_LIO_SLAM`：项目描述直接给出该组合。
  - `aserbremen/scancontext_ros2`：ROS2 化全局描述子用于 place recognition / long-term localization。
  - `engcang/FAST-LIO-Localization-SC-QN`：ScanContext 候选检索 + 精配准（Quatro/Nano-GICP）。
- `hku-mars/FAST_LIO` 官方 README 里明确列出重定位扩展项目，并强调 ARM 平台支持与高频实时性。

## C. 学术侧（2024-2025）
- Sensors 2024（`A Real-Time Global Re-Localization Framework for a 3D LiDAR-Based Navigation System`）给出模板库+粗到细检索，报告 7-11Hz 级全局重定位。
- arXiv 2025（`Sparse Feasible Hypothesis Sampling`）针对 kidnapped robot 提出“稀疏可行假设+分阶段推理”，本质也是“减少盲目全局搜索”。

结论：
- 2024-2026 的主流方向不是“继续加深 ISS/FPFH 级联”，而是“全局描述子检索 + 小规模候选精配准 + 明确状态机”。

---

## 3. 优化建议方案（面向 RC_2026 R2）

## 3.1 重构为三层恢复架构（替代当前五级串行常驻）

### L0 正常跟踪层（保留）
- 继续使用现有 small_gicp 局部配准跟踪。
- 维持冻结门控逻辑（当前已有 `freeze_update_err` / `min_inliers`）。

### L1 快速恢复层（主路径，优先）
- 触发场景：强制重试后回重试区、或高置信度先验已知。
- 方法：`先验位姿(2~4个yaw) + ICP/GICP`，默认不跑 NDT。
- 你们代码已具备此能力框架（`tryRetryZoneFastChannel`），但当前默认关闭（`retry_zone_enable: false`）。

### L2 全局恢复层（兜底路径）
- 首先做 Scan Context（或 Intensity SC）全局检索，取 TopK 候选（建议 K=3~5）。
- 对 TopK 逐个做快速精配准（ICP/GICP），只在“初值差 + 收敛差”时启用 NDT。
- 当前 `ISS+FPFH+SAC-IA` 链路保留为最后兜底，不再作为常规主路径。

---

## 3.2 NDT 与 ICP 的取舍建议（直接回答“是否都保留”）

- 建议默认策略：保留 ICP，NDT 改为条件触发。
- 条件触发 NDT 的典型门槛：
  - 粗匹配残差高于阈值。
  - yaw 不确定度大于预设（如 >30°）。
  - 候选间得分接近，需扩大收敛域。
- 这样可避免“每个候选都跑 NDT+ICP”的串行耗时爆炸。

---

## 3.3 统一状态机（替代分散判定）

建议状态：
- `TRACKING`
- `SUSPECT`（误差升高但未确认）
- `FAST_RECOVERY`（重试区先验恢复）
- `GLOBAL_RECOVERY`（全局检索恢复）
- `STABLE_CONFIRM`（短时间确认，避免抖动）

统一触发指标（输入）：
- `normalized_error`
- `num_inliers`
- `time_since_last_success`
- `referee_retry_event`（如果可接入裁判/流程事件）

收益：
- 把绑架检测、超时恢复、多假设从“代码分支”升级为“状态行为”，减少冗余与调参复杂度。

---

## 3.4 近期可执行参数动作（不改大架构先提效）

1. 先启用并标定 `retry_zone_*`，让 L1 快速通道可用。  
2. 将 `num_yaw_hypotheses` 从 8 降到 4（先保守减半）。  
3. 将 `use_ndt_refinement` 改为默认 false，仅在恢复失败后重试时开启。  
4. 给全局恢复增加耗时与候选统计日志（每次恢复记录候选数、每候选耗时、最终得分）。  
5. 以“重定位总耗时”而非“单算法收敛率”作为主 KPI（R2 比赛目标是任务连续性，不是单次最优配准误差）。  

---

## 3.5 QCS8550 平台参数基线（新增）

建议先用下面这组“算力约束优先”基线，再做 bag 回放微调：

| 参数 | 当前 | 建议基线 | 说明 |
|---|---:|---:|---|
| `num_threads` | 4 | 4（可试 5） | 先锁定大核预算，避免调度到小核导致抖动 |
| `use_ndt_refinement` | true | false（默认） | 仅在兜底重试时开启 |
| `num_yaw_hypotheses` | 8 | 4 | 候选减半，直接压缩全局恢复串行耗时 |
| `global_icp_max_iterations` | 100 | 40~60 | 在恢复时延与精度间取平衡 |
| `ndt_max_iterations` | 50 | 20~30（仅启用时） | 限制 NDT 长尾 |
| `sac_ia_fpfh_ksearch` | 50 | 30~40 | 降低 FPFH 计算负担 |
| `sac_ia_correspondence_randomness` | 50 | 20~30 | 控制 SAC-IA 随机匹配开销 |

工程建议（同平台）：
- 若能在 launch 层做亲和性控制，优先把 `localization` 和 `point_lio` 固定到大核集合，避免和视觉线程抢核。
- 视觉尽量走 NPU/GPU（AidLite），把 CPU 大核留给定位链路。
- 比赛连续运行要监控热降频，重定位抖动往往先表现为“尾延迟突然增大”。

---

## 4. 验收指标（建议）

- 局部配准（2Hz）单次耗时：`P95 < 250ms`
- 快速恢复（L1，重试区先验）：`P95 < 0.6s`
- 全局恢复（L2，NDT关闭时）：`P95 < 2.0s`
- 全局恢复（L2，NDT开启兜底时）：`P95 < 3.0s`
- 恢复成功后 2s 内 `map->odom` 不出现明显跳变
- 连续动态干扰场景下，误触发率可控（按 bag 回放统计）

---

## 5. 联网参考（本次检索使用）

- ICRA 2024 Competitions: `https://2024.ieee-icra.org/program/competitions/`
- ICRA 2026 Competitions: `https://2026.ieee-icra.org/program/competitions`
- GPR2024 (ICRA map association): `https://metaslam.github.io/competitions/icra2024/`
- FAST_LIO 官方仓库: `https://github.com/hku-mars/FAST_LIO`
- FAST_LIO_SLAM（FAST-LIO + Scan Context）: `https://github.com/gisbi-kim/FAST_LIO_SLAM`
- scancontext_ros2: `https://github.com/aserbremen/scancontext_ros2`
- FAST-LIO-Localization-SC-QN: `https://github.com/engcang/FAST-LIO-Localization-SC-QN`
- FAST_LIO_LOCALIZATION: `https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION`
- Sensors 2024 重定位论文: `https://www.mdpi.com/1424-8220/24/19/6288`
- arXiv 2025 Kidnapped Robot: `https://arxiv.org/abs/2511.01219`
