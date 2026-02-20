# codex-思考4：执行方案4的现实因素深度审计（硬件/传感器/赛规约束）

## 0. 前置说明（联网核验结果与不确定性）

本报告先完成联网参数核验，再对比：
- 参考思考 1：`MVP技术方案/定位/思考方案/claude-思考3.md`
- 参考思考 2：`MVP技术方案/定位/思考方案/codex-思考3.md`
- 参考规则：`MVP技术方案/决策/主赛规则.md`
- 目标执行：`MVP技术方案/定位/执行方案/执行方案4.md`

### 0.1 已核验的硬件/传感器/中间件数据

1. QCS8550 + AidLux（官方可得）
- Rhino X1 官方文档可核验：QCS8550、CPU 1x3.2GHz + 2x2.8GHz + 3x2.0GHz、AI 48 TOPS(INT8)、Adreno 740、16GB LPDDR5X、12V/5A 供电、-20~60℃工作温度。  
  来源：`https://rhinopi.docs.aidlux.com/rhino-x1-ubuntu/`
- AidLux 官网可核验其“Android+Linux 融合系统、CPU+GPU+NPU 综合调度”定位，以及其对“双系统并行、非虚拟机方式、内核级互调 API”的公开表述（但未给出 ROS2 专项 IPC 延迟实测值）。  
  来源：`https://www.aidlux.com/`、`https://aidlux.com/platform`

2. Livox Mid-360（官方规格页）
- 近场盲区：0.1m；0.1~0.2m 可检测但精度不保证。
- 强光指标：100 klx 条件下，40m@10%反射率、70m@80%反射率；噪声误报率 <0.01%@100 klx。
- 点云特性：200,000 points/s，10Hz；测距精度 ≤2cm@10m，≤3cm@0.2m。 
- FOV：水平 360°，垂直 -7°~52°；时间同步：PTPv2(IEEE 1588-2008) / GPS；功耗：平均 6.5W（低温自加热模式峰值可达 14W）；工作温度 -20~55℃，并明确提示高温会触发保护机制且建议外壳温度不超过 80℃。  
  来源：`https://www.livoxtech.com/mid-360/specs`

3. Mid-360 内置 IMU（ICM-40609-D 数据）
- Gyro noise 4.5 mdps/√Hz；gyro offset 温漂 ±10 mdps/°C；accel noise 100 μg/√Hz。  
  来源：`https://invensense.tdk.com/wp-content/uploads/2022/07/DS-000330-ICM-40609-D-v1.2.pdf`

4. ROS 2 / Nav2 资源与通信（公开实测）
- Nav2 官方调参指南明确建议开启 `use_composition` 以降低进程开销，并在“Performance in ROS 2: RMW, Node Composition, Intra-process Communication, and QoS”章节引用了不同 RMW + IPC 设置下的 CPU 使用对比（TurtleBot4 simulation 测试口径）。  
  来源：`https://docs.nav2.org/tuning/index.html`
- ROS2/ Nav2 性能讨论中给出了不同 RMW 与 intra-process 设置的 CPU 使用数值（包含 Zenoh / CycloneDDS / FastDDS 等组合），可用于说明“中间件选择 + IPC 形态”对 CPU 占用的敏感性。  
  来源：`https://discourse.openrobotics.org/t/performance-characteristics-subscription-callback-signatures-rmw-implementation-intra-process-communication-ipc/51454`
- ROS 2 官方 TSC 原始 CSV（two-process）可得到跨进程延迟量级（构建农场环境，非 AidLux/QCS8550 专项）：以 `median latency_mean (ms)` 计，`Array1k/Array4k` 级别可到约 0.05~0.07ms；`Array2m` 级别可到约 0.6~1.6ms；更大消息可上升到多毫秒至十余毫秒量级并伴随丢包。  
  来源（示例值可在 CSV 内直接核对）：`https://raw.githubusercontent.com/osrf/TSC-RMW-Reports/main/humble/notebooks/data/build_farm/two_process_perf_network_results.csv`、`https://raw.githubusercontent.com/osrf/TSC-RMW-Reports/main/humble/notebooks/data/build_farm/node_perf.csv`
- Nav2 costmap 参数（`update_frequency/publish_frequency/always_send_full_costmap/width/height/resolution` 等）与 KeepoutFilter 接入方式（filters 机制）可直接核验：`https://docs.nav2.org/configuration/packages/configuring-costmaps.html`

5. 嵌入式 ARM（Raspberry Pi 4）Nav2 经验基线
- 公开案例：planner_server、bt_navigator 单节点各>30% CPU，4 核总体 90~100%，并出现行为失败；启用 composition 后显著改善。  
  来源：`https://robotics.stackexchange.com/questions/114197/nav2-overloading-raspberry-pi-cpu`
- Nav2 组合化讨论报告：相对普通多进程，手动 composition 测得 CPU 约降 20%、内存约降 70%（嵌入式优化场景）。  
  来源：`https://discourse.openrobotics.org/t/nav2-composition/22175`

### 0.2 关键不确定性（必须显式标注）

当前公开资料**未检索到 AidLux 官方给出的 Android↔Ubuntu 跨系统 ROS2 IPC 延迟实测值**；AidLux 平台页虽强调“双系统并行、内核级互调 API、非虚拟机方式”，但未给出 ROS2 DDS/IPC 的端到端时延曲线或在 SoC 高负载下的抖动上界。  
因此本报告把 ROS2 官方 two-process CSV 作为“跨进程量级下限参考”，不把它当作 AidLux 专项实测值；并把“Android 并发负载对 Linux 侧 ROS2 调度抖动的影响”视为关键现实风险源之一。

---

## 1. 三份方案对照后的主结论

### 1.1 两份“思考方案”的共识

- `claude-思考3` 与 `codex-思考3` 的核心共识一致：
1. 规则致命项不是几何阈值，而是“KFS 禁踩语义缺口”。
2. 8 邻域与“公共边相邻”赛规不一致（规则 `主赛规则.md:196`）。
3. EMA 过渡、侧向跌落扇区、噪点误标是稳定性主风险。

### 1.2 执行方案4对共识的覆盖

执行方案4确实覆盖了主要补洞：
- KFS keepout 主链路（`执行方案4.md:303-392`）
- 新消息契约（`执行方案4.md:307-325`）
- 2Hz+事件发布（`执行方案4.md:333`）
- 邻域/BFS/EMA/延迟监控（`执行方案4.md:69-173`）

但在“现实因素”上仍存在若干高概率冲突，下面按改动逐项审计。

---

## 2. 现实因素审计（仅聚焦现实冲突，不做代码补全）

## 2.1 改动A：KFS禁区导航（新包 + Nav2 Costmap）

### A-1 赛场几何与禁区膨胀的刚性耦合风险

- 赛规中梅林方块是 `1200mm x 1200mm`（`主赛规则.md:35`），执行方案 keepout 膨胀半径固定 `0.60m`（`执行方案4.md:353`）。
- 这意味着禁区半径与“半块边长”刚好重合：只要定位或格点中心误差达到 5~10cm，禁区就会出现“误侵相邻可通行边”或“漏封本块边缘”的二选一问题。
- 在4分钟比赛节奏下，这种几何刚性会直接表现为：
1. 误封：路径被无谓切断，绕行耗时增加。
2. 漏封：底盘进入规则禁区（`主赛规则.md:201` / `:290`）。

### A-2 概率衰减规则与释放阈值存在现实逻辑冲突

- 执行方案定义：block阈值 `P=0.70`、free阈值 `P=0.35`、衰减“向 P=0.5”（`执行方案4.md:348-352`）。
- 若系统只具备正向命中（hit）+向0.5衰减，则概率可能长期停在 `[0.35,0.70)`，出现“既不继续确认，也不释放”的灰区粘滞。
- 对 4 分钟比赛而言，灰区粘滞会放大为路线锁死或幽灵禁区，直接损失任务节拍。

### A-3 启动时序风险

- bringup 顺序中 `nav2_launch` 在前、`kfs_block_fuser` / `costmap_filter_info_server` 在后（`bringup.launch.py:328,332,333`）。
- 这会形成“导航先激活、禁区后到位”的窗口期：若此时决策已驱动动作，R2 可能先进入违规方块再收到 keepout。

### A-4 标定缺省值与赛场真实尺度冲突风险

- `mf_grid_layout.yaml` 当前示例间距为 0.5m 级（`mf_grid_layout.yaml:7-18`），而赛规真实块间中心距由 1.2m 方块几何决定。
- 文件虽注明“需场地标定后填写”（`mf_grid_layout.yaml:2`），但竞赛现场最常见失败模式就是“带着示例值上场”。
- 一旦标定未完成或误配，keepout 将在地图上系统性错位，风险等级高于普通噪声误报。

### A-5 Keepout 掩码的现实“体量-时延”耦合

- KeepoutFilter 的输入本质是栅格化掩码（OccupancyGrid），其消息体量与 `width/height/resolution` 线性相关（Nav2 costmap 参数口径可核验：`https://docs.nav2.org/configuration/packages/configuring-costmaps.html`）。
- 在跨进程传输上，ROS2 官方 two-process 数据显示：消息从 KB 级增长到 MB 级后，跨进程 `median latency_mean` 会从 0.05ms 量级上升到 0.6~1.6ms（甚至更高）量级（`two_process_perf_network_results.csv`）。  
  现实含义：若临场为了“更精细禁区边界”把掩码分辨率调得过细或覆盖范围过大，则 keepout 更新本身会挤占 CPU/内存带宽预算，反过来拉长“禁区生效时间”，与“规则硬约束”发生对抗。

---

## 2.2 改动B：KFS 自定义消息（Cell/State）

### B-1 契约过于轻量导致语义歧义风险

- `MfKfsCell` 仅有 `grid_id/kfs_type/confidence`（`MfKfsCell.msg:1-3`）。
- `MfKfsState` 仅有 `header/team/cells`（`MfKfsState.msg:1-3`）。
- 现实问题：该契约默认“格子离散正确+地图静态正确+队伍映射正确”，但赛场镜像与重试恢复中，任何一项误配都会把禁区投射到错误格。

### B-2 “team”字段的现实价值依赖外部一致性

- 决策端会发布 `team`（`decision_node.cpp:368`），但禁区融合若不把队伍-布局映射做成强校验，`team` 只会成为“写在消息里但不参与防错”的信息噪声。
- 在蓝红镜像赛场（`主赛规则.md:45-55`）中，这类“有字段无强约束”最容易在临场换边时触发错误。

---

## 2.3 改动C：启动链与决策节点（2Hz/事件驱动）

### C-1 2Hz保底频率在动态禁区切换下的时间预算偏紧

- 执行方案指定 `/mf_kfs_state` 为“2Hz 周期 + 状态变化立即发布”（`执行方案4.md:333`）。
- 真实链路最短路径是：黑板变化 -> decision发布 -> fuser融合 -> keepout mask -> local costmap更新。
- 本仓库 local costmap 更新频率是 20Hz（50ms 周期，`nav2_params.yaml:77`），global 是 2Hz（`nav2_params.yaml:135`）。
- 理想低负载下该链路可在百毫秒级完成；但在嵌入式 ARM 实战中，公开案例已显示 Nav2 很容易进入高CPU占用区间（RPi4 90~100%）。负载上升后，2Hz保底会明显拉长“禁区生效到达时间”。

补充现实量化：如果按 `mf_approach` 的线速度上限 0.6m/s（`执行方案4.md:34-55`）估算，2Hz 的最差“采样等待”窗口为 0.5s，机器人在禁区生效前理论上可前进约 0.3m。这一尺度已经与“方块边缘到机器人足迹边界”的安全裕量同量级，尤其在入口区与方块边界距离本就紧张时，风险会被放大。

### C-2 QCS8550 算力强不等于该链路天然安全

- QCS8550 的 48 TOPS 主要是 NPU 峰值能力；本链路主体（DDS + costmap +规划）仍是 CPU 调度与内存带宽敏感任务。
- AidLux 的 Android+Linux 融合架构本质是共享硬件资源，若 Android 侧并发负载上来，会出现 Linux 侧控制链路抖动。
- 由于缺少 AidLux 官方跨系统 IPC 延迟实测，不能把“峰值算力”直接换算成“竞赛级实时性保障”。

---

## 2.4 改动D：地形语义优化（邻域/BFS/EMA/延迟监控）

### D-1 邻域修正是正确方向，但会暴露传感器物理边界

- 从8邻域走向 obstacle边4/drop边8，本质上更贴近“公共边相邻”赛规（`主赛规则.md:196`），方向正确。
- 但 Mid-360 在 0.1~0.2m 近场精度不保证；并明确声明在 0.1~1m 区间内对“低反射/薄小/细线/水面/抛光/哑光”等目标检测效果不保证（官方脚注）。现实含义是：越靠近边缘/越需要“刚好看见”的危险点，越可能因为物理回波条件而稀疏甚至缺失。
- 当 `min_obstacle_area_cells=2`（`terrain_semantic.yaml:115`）叠加 BFS 去噪时，容易把“真实但稀疏的边缘危险”与“噪声孤点”一起抹掉。

### D-2 EMA 跳变保护在高动态工况下有误触发风险

- `jump_thresh_m=0.15`（`terrain_semantic.yaml:116`），而赛场合法台阶增量是 0.2m（200/400/600 结构）。
- 现实中的急加减速会引入姿态快速变化与时间对齐误差；当前后帧有效高度差接近 0.15m 时，系统更容易把“真实台阶切换”当成“异常跳变”，从而触发保护分支。
- Mid-360 内置 IMU（ICM-40609）虽噪声指标不差（4.5 mdps/√Hz），但温漂项（±10 mdps/°C）和车体振动仍会把边界工况变成“阈值抖动放大器”。

### D-3 延迟监控阈值与比赛现场负载的耦合风险

- 监控阈值设置为 warn 12ms / error 20ms（`terrain_semantic.yaml:119-120`）。
- Mid-360 点频 200k pts/s、10Hz 输入下，地形语义链路在“视觉并发+导航并发+日志并发”时很容易逼近阈值边界。
- 若只报警不联动降载策略，诊断会变成“已知超时但行为不变”，比赛价值有限。

### D-4 “强光鲁棒”不等于“语义输入稳定”

- Mid-360 的强光指标在 100 klx 下给出了明确量化（探测距离与噪声误报率），这意味着它在“太阳直射级别环境光”下仍能工作；但这并不直接等价于“赛场语义输入稳定”：
1. 误报率 <0.01% 看似很低，但在 200,000 points/s 下，理论上仍可能出现数量级为“几十点/秒”的杂散光噪点。对于栅格语义而言，这类噪点往往是“单格孤立高点”的典型形态，恰好与 BFS 去噪的清除目标重合，导致系统在强光场景下的行为更依赖 `min_obstacle_area_cells` 与点云落格稳定性。
2. 赛场存在白色胶带标记区（`主赛规则.md:56-61`，355mm×355mm）与亚克力护栏等高反射/透明材料，Livox 官方也提示“不同反射率目标检测时极少数位置精度可能略降”。这会把“边缘/角点”变成最敏感区域，进一步放大 D-1 的稀疏风险。

---

## 3. 对“动态禁区切换速度 vs 赛规响应时间”的结论

赛规对 R2 的约束是硬约束：不能踩有 KFS 的方块顶面（`主赛规则.md:201`/`:290`）。

执行方案4把语义链路补齐了，但在现实中仍受三类时间抖动约束：
1. 发布抖动：2Hz保底意味着最差500ms级离散采样窗口。
2. 融合抖动：概率衰减与释放机制存在灰区粘滞，可能形成“禁区释放不及时”。
3. 系统抖动：AidLux 融合系统下缺少公开 IPC 实测，无法事先证明“Android并发负载对ROS链路无影响”。

因此，执行方案4在“静态可行”层面成立，但在“极端赛场鲁棒性”层面仍存在以下现实漏洞：
- 几何标定容错不足（0.60m膨胀与1.2m方块尺寸刚性耦合）。
- 概率机理与释放目标不完全一致（向0.5衰减但要求0.35释放）。
- 启动顺序存在禁区晚到位窗口。
- 传感器物理边界（近场盲区/反光/稀疏）会与BFS去噪、跳变阈值形成复合误判。

这四点并非代码细节问题，而是“现实物理 + 系统时序 + 赛规硬约束”之间的结构性冲突。
