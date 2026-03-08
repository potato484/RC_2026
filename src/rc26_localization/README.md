# rc26_localization 模块说明

## 模块简介
`rc26_localization` 是 R2 机器人在复杂比赛场地上实现高精度、高频率自定位的核心模块。基于 `small_gicp` 提供的基础配准能力，该模块结合 R2 底盘的轮式里程计（Odom）和高精度陀螺仪，实现了 LiDAR + Odom + IMU 的多源异构数据融合定位。

## 核心功能
1. **多层级初值融合**：以高频、低延迟的 IMU/Odom 融合位姿作为点云配准的初值，极大缩小搜索范围，降低匹配失败概率。
2. **场景自适应退化处理**：通过实时监控配准的残差分布和协方差矩阵特征值（Hessian 矩阵的谱分析），自动识别长走廊等特征匮乏的退化场景。
3. **动态信赖域与惩罚机制**：引入 Huber 核函数以及动态过滤策略，屏蔽场上移动目标（如其他机器人、动态障碍物）对点云配准的恶性干扰。
4. **不确定性感知控制接口**：将定位置信度（协方差大小）实时反馈给运动控制层（Controller），在不确定性升高时触发降速或保守策略，保障系统安全。
5. **智能优化器调度**：根据上一帧位姿变化幅度和环境复杂度，动态在 GN（Gauss-Newton）和 LM（Levenberg-Marquardt）优化器间切换，优化 CPU/GPU 算力分配。

## P0 新增输出（前端定位 + 后端闭环执行方案）
- `/localization/health`：语义化定位健康度（LHI），给控制器和决策层做限速/停车判据。
- `/localization/backend_status`：定位后端状态与诊断（P0 为占位版，P1 启用图后端后切换为真实值）。

## P1 在线后端（关键帧 + Pose2 图 + 平滑 map->odom）
- 内嵌五个子模块：`KeyframeManager`、`OnlineScanContextDB`、`ConstraintValidator`、`PoseGraphBackend`、`MapToOdomSmoother`。
- `enable_graph_backend=true` 时，局部配准成功后会触发关键帧入图、回环候选验证、iSAM2 增量更新。
- `publishTransform()` 保持 20Hz，不新增发布线程；图后端修正通过 `MapToOdomSmoother` 以限斜率方式输出。
- `markRelocalizationSuccess()` 在图后端模式默认走“锚点入图 + 平滑输出”，仅在失败时回退到 legacy 硬切。
- `/localization/backend_status` 会输出真实 `optimizer_ready`、`graph_health`、`last_loop_age_sec`、`last_anchor_age_sec`。

## P2 路径可观测性输出（RouteObservability）
- 新增订阅 `plan_topic`（默认 `local_plan`），按前方 `2m~5m` 路径窗口评估风险。
- 新增发布 `/localization/route_observability`，包含：
  - `score` / `risk_level`
  - `repeat_structure_risk` / `dynamic_risk`
  - `loop_opportunity_score` / `anchor_opportunity_score`
  - `recommended_nav_profile`

## 关键开关参数
- `enable_graph_backend`：是否启用图后端（P0 默认 `false`）。
- `legacy_hard_reloc_enable`：是否允许旧版重定位硬切（仅紧急回退用，默认 `false`）。
