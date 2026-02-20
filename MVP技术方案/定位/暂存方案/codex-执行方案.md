
  # 执行方案：定位链路改进方案 v5.0（按 改进方案5.md 落地）

  ## 摘要（交付目标与分层）

  - 目标（比赛 4 分钟节奏）：达成「KFS 禁踩 0 违规」+「不规划死锁」+「可证明实时性」+「可控降级」；仅面向 R2（R1 仅
    作为外部动态干扰源，不做适配）。
  - P0（赛前必须）：
      1. 修复 kfs_block_fuser “概率释放永不发生”死锁；
      2. 修复 terrain_semantic 台阶场景 EMA 冻结（含冻结上限/台阶门控）；
      3. 消灭 bringup「Nav2 先动 / Keepout 后到」窗口；
      4. 运动放行门控：禁区链路未就绪 → 必须停（不发 Nav2 goal / stop_required）。
  - P1（强烈建议）：/mf_kfs_state 保底 5Hz；Keepout 掩码分辨率与 costmap 对齐（0.1m）；TerrainModeAdapter 参数下发
    重试+回读校验 并扩展 profile 映射；补充角点/切角守门。
  - P2（必须可执行）：把 latency_error_ms=20ms 从“只报警”升级为“触发降载/限速/停机”，并提供 QCS8550 降频下 fallback
    配置。

  ———

  ## 影响的公开接口/行为（必须明确）

  ### 1) Keepout 链路（rc26_kfs_keepout / Nav2）

  - 新增/调整 kfs_block_fuser 参数（建议全部可在 launch/yaml 配置）：
      - decay_target_prob（默认 0.05）：log-odds 衰减目标从 P=0.5 改为低概率，保证能穿越 free_thresh。
      - decay_rate（语义改为“log-odds 每秒衰减量”）：P0 设为 2.0（使无新证据释放约 2–3s 量级）。
      - ttl_sec（默认 0=关闭；P0 建议 10.0）：超过 TTL 无命中强制释放（兜底通信/上游中断）。
      - keepout_shape：circle / square（P0 配置为 square）。
      - block_half_size_m（默认 0.60）+ keepout_margin_m（默认 0.03）：用于 square keepout。
      - map_resolution 默认改为 0.10（与 src/rc26_bringup/config/nav2_params.yaml costmap resolution 对齐）。
  - mf_grid_layout.yaml 元信息扩展（配置文件接口变更）：
      - 增加 team、layout_version、validated 字段；validated: false 时 keepout_enabled=false（防错硬约束）。
      - 增加几何自检：相邻格中心距应接近 1.2m ± 0.05m（若失败 → keepout 禁用并报 diagnostics ERROR）。

  ### 2) 运动放行门控（rc26_decision）

  - 在 SmartWaypointNavigator 增加 Keepout Ready Gate（新增订阅 + 状态机阶段）：
      - 订阅：/costmap_filter_info、/kfs_filter_mask。
      - 放行条件（硬约束 I-KFS-1）：两者均收到且 mask_age_ms <= 300（P95 <= 200 作为验收统计目标）。
      - 未就绪：保持/切换 set_nav_mode safe，并 拒绝发送 Nav2 goal（返回失败原因用于日志复盘）。
      - Gate 参数化：keepout_gate_enable=true、keepout_gate_timeout_sec=3.0、keepout_mask_max_age_ms=300、topic 名可
        配。

  ### 3) terrain_semantic（rc26_terrain）

  - EMA 跳变保护改造（P0）：从“跳过更新导致长期冻结”改为“台阶工况慢 EMA + 静止工况冻结有上限”。
  - 新增参数：
      - jump_thresh_m：P0 配置 0.23（从 0.15 上调）。
      - ema_freeze_max_sec：P0 配置 0.30（冻结上限）。
      - stair_gate_speed_mps：P0 配置 0.25（速度门控）。
      - ground_ema_alpha_slow：P0 配置 0.25（台阶工况慢 EMA）。
      - enable_pitch_compensation=true：用 TF(base) 的 pitch 对采样高度做 z_corr = z_raw - x_rel * tan(pitch) 修正，
        降低急停俯仰伪跳变。
  - 延迟超限动作（P2）：
      - 统计 terrain_latency_ms 连续超限计数 latency_error_n=3；触发 fail_safe_strategy=virtual_fence 或
        emergency_stop（二选一：P2 默认 virtual_fence + 同时切 safe profile）。

  ### 4) TerrainModeAdapter（rc26_nav_mode_manager）

  - 参数下发可靠性：wait_for_service 120ms，set_parameters 120ms，失败重试 2 次（间隔 30ms），成功后 get_parameters
    回读校验 50ms；最终失败 → 发布 ERROR 并触发 safe/stop_required（通过 nav_mode_manager 侧策略或直接
    set_nav_mode）。
  - profile 映射扩展（至少覆盖）：
      - unknown_policy
      - drop_forward_sector_deg
      - min_obstacle_area_cells
      - obstacle_neighbor_mode
      - jump_thresh_m

  ———

  ## 逐步实施（按 P0→P1→P2，步骤不可跳）

  ### Phase A — P0（必须完成再进入下一阶段）

  1. Bringup 启动顺序消窗
      - 文件：src/rc26_bringup/launch/bringup.launch.py
      - 调整 LaunchDescription 顺序（导航模式 UnlessCondition(slam)）为：
          1. map_server_node
          2. costmap_filter_info_server
          3. map_server_lifecycle_manager（node_names 必须包含 costmap_filter_info_server）
          4. kfs_block_fuser_node
          5. nav_mode_manager_node
          6. nav2_launch
          7. decision_node
      - 同步检查：autostart: true 保持不变，但 keepout 信息必须先可用。
  2. 修复 kfs_block_fuser 概率释放死锁 + 分辨率对齐 + 几何/标定防错
      - 文件：
          - src/rc26_kfs_keepout/src/kfs_block_fuser.cpp
          - src/rc26_kfs_keepout/include/rc26_kfs_keepout/kfs_block_fuser.hpp
          - src/rc26_kfs_keepout/config/mf_grid_layout.yaml
      - 实施点：
          - 将衰减目标改为 decay_target_prob=0.05（log-odds 目标为负值），保证能跨越 free_thresh=0.35。
          - decay_rate P0 配置为 2.0（释放速度可接受）。
          - 增加 ttl_sec=10.0（超过 TTL 无命中强制释放）。
          - map_resolution P0 配置为 0.10。
          - 增加 keepout_shape=square + block_half_size_m=0.60 + keepout_margin_m=0.03，替代圆形膨胀（circle 模式保
            留作为回滚开关）。
          - mf_grid_layout.yaml 增加 team/layout_version/validated；validated=false 或几何自检失败 → keepout 禁用并
            diagnostics ERROR。
  3. 修复 terrain_semantic 台阶 EMA 冻结 + 俯仰补偿
      - 文件：src/rc26_terrain/src/terrain_semantic_node.cpp + src/rc26_terrain/config/terrain_semantic.yaml
      - 实施点（对应 改进方案5.md §3.1）：
          - jump guard 不再 continue 造成无限冻结：
              - 台阶工况（任一成立：speed > 0.25m/s 或 nav profile 属于 mf_traverse/stair_*）→ 使用
                ground_ema_alpha_slow=0.25 更新；
              - 静止工况 → 允许冻结，但冻结持续超过 ema_freeze_max_sec=0.30 必须强制更新一次。
          - 采样写入 bucket 前做 pitch compensation（使用 cloudCallback 已计算的 pitch，修正量用 x_rel）。
          - YAML 配置将 jump_thresh_m: 0.15 -> 0.23（P0 最低修复）。
  4. TerrainModeAdapter 下发可靠化 + 扩展 profile 映射
      - 文件：src/rc26_nav_mode_manager/src/terrain_mode_adapter.cpp
      - 实施点：
          - 实现“等待服务→set_parameters 重试→get_parameters 回读校验→失败触发 safe/stop”完整闭环。
          - profile_map_ 中为 normal/safe/mf_traverse/mf_exit/mf_approach 写死完整参数集合（与 改进方案5.md §3.2/
            §3.4/§5.4 对齐）。
  5. 决策侧放行门控（禁区未就绪 → 必停）
      - 文件：src/rc26_decision/src/navigation/smart_waypoint_navigator.cpp + src/rc26_decision/include/
        rc26_decision/navigation/smart_waypoint_navigator.hpp
      - 实施点：
          - 新增 ExecState：WaitKeepoutReady，置于 SetMode 之前（保证任何 Nav2 goal 前已门控）。
          - 新增订阅：/costmap_filter_info（nav2_msgs/msg/CostmapFilterInfo）、/kfs_filter_mask（nav_msgs/msg/
            OccupancyGrid）。
          - Gate 通过：两 topic 均收到且 now - mask.header.stamp <= 300ms；否则在 timeout_sec=3.0 内等待，超时 →
            abortWithFailure("keepout not ready") 并请求 set_nav_mode safe（reason="keepout_gate")。

  P0 验收（必须逐项打勾）

  - 复现实验（最小集）：启动 bringup，立即触发一次导航动作：必须先看到 /costmap_filter_info 与 /kfs_filter_mask，否
    则导航动作被拒绝且系统处于 safe/stop。
  - kfs_block_fuser：停止发布某 grid 的命中后，该 grid keepout 在 ≤3s 内释放（以 decay_rate=2.0 目标），且不再出
    现“永久禁区”。
  - terrain_semantic：200mm 台阶跨越时 ground_z_filtered 不出现 >300ms 的持续冻结；急停重复 10 次不出现长期冻结（按
    §6.1 复盘日志/统计）。

  ———

  ### Phase B — P1（P0 稳定后再做）

  6. /mf_kfs_state 提频到 5Hz

  - 文件：src/rc26_decision/src/decision_node.cpp
  - 将 kfs_timer_ 从 500ms 改为 200ms；QoS depth 调为 3（KeepLast(3) reliable），保留“状态变化立即发布”。

  7. 角点/切角守门（禁止对角直连）

  - 文件：src/rc26_decision/src/navigation/waypoint_manager.cpp
  - 在 generateMerlinPoints() 生成的点中为每个格点写入 payload["grid_id"]=id，并在 MF 路径生成/导航策略中只允许 4 邻
    接（现有 adjacency 即 4 邻接，保持）；必要时为相邻移动插入中继点（沿轴向拆分），避免 Nav2 平滑切角。

  8. 拾取后强制释放（如果能拿到 grid_id）

  - 方案固定为 Topic（避免新增自定义 srv/msg）：/kfs_force_release_grid，类型 std_msgs/msg/UInt8
  - kfs_block_fuser 订阅该 topic：收到 grid_id → 将该格 blocked_state=0 且 log_odds=lo_target，立即发布 mask（目标
    P95 ≤0.5s）。
  - 决策侧在“GrabKFS 成功且已知 grid_id”时发布一次该消息。

  P1 验收

  - /mf_kfs_state：ros2 topic hz 显示 >= 5Hz（允许轻微抖动）；KFS 状态变化到 /kfs_filter_mask 更新 P95 ≤100ms（见
    §6.8）。
  - 强制释放：Grab 完成到 mask 解除 P95 ≤0.5s（见 §6.6）。

  ———

  ### Phase C — P2（热降频/高负载下可控降级）

  9. latency_error_ms 触发动作闭环

  - 文件：src/rc26_terrain/src/terrain_semantic_node.cpp + src/rc26_terrain/config/terrain_semantic.yaml
  - 实施点：
      - 连续 N=3 帧 last_latency_ms_ > latency_error_ms_ → 进入降级：
          1. 发布 virtual_fence（阻止移动），并
          2. 触发 set_nav_mode safe（可通过在 decision 侧/单独 watchdog 节点实现；默认实现为 terrain 侧直接“围栏停
             机”，确保不依赖 IPC）。
      - 降级解除条件：M=10 帧均 <= latency_warn_ms_ 且输入健康 OK。

  10. 轻量 fallback 配置（QCS8550 降频）

  - 配置清单固定如下（作为 safe 或 degraded profile 的参数集）：
      - terrain_semantic: voxel_leaf_size_m=0.10, perception_radius_m=2.5, min_points_per_cell=3,
        unknown_policy=conservative
      - local_costmap: update_frequency=15, obstacle_max_range=2.5, raytrace_max_range=3.0
  - 触发条件：latency 超限动作触发后自动切换（或手动切换用于压测）。

  P2 验收

  - 4 分钟压测：出现降频/负载上升时，系统必须进入 fallback 并保持 keepout + 跌落避障可用；超限触发到“停住/限速生
    效”≤350ms（见 §6.10/§6.11）。

  ———

  ## 测试用例与验收场景（对应文档 §6，落到可执行步骤）

  1. 启动时序：bringup 后 10s 内，必须已发布 /costmap_filter_info 和 /kfs_filter_mask（transient_local），且
     SmartWaypointNavigator gate 通过后才允许 goal。
  2. KFS 释放：对单格制造“命中→撤销命中”，验证释放时间；再注入“上游停更”验证 TTL 行为。
  3. 200mm 台阶：过台阶 + 急停 10 次，检查冻结计数/最长冻结时间 ≤300ms。
  4. 近场/反光（若 P0 未做 near-zone 自适应，可放到回归）：近距离靠近方块 60s 不出现大面积伪障碍闪烁导致频繁停走。
  5. 端到端延迟：记录时间戳链路：/mf_kfs_state→/kfs_filter_mask→local costmap 生效，P95 ≤300ms。
  6. IPC 抖动：连续切 profile 100 次，成功 ≥99 次；失败触发 safe/stop_required。

  ———

  ## 构建与验证命令（统一约束）

  - 编译验证：colcon build --parallel-workers 1
    1，以及 /diagnostics。
  ———

  ## 回滚策略（必须预置）

  - kfs_block_fuser：保留 keepout_shape=circle 开关；若 square 行为不符合场地几何，回滚到 circle 但 分辨率对齐与释放
    修复不回滚。
  - terrain_semantic：保留 enable_pitch_compensation 开关；出现异常再关闭，但冻结上限逻辑不回滚。
  - Gate：提供 keepout_gate_enable=false 仅用于调试；比赛配置强制为 true。

  ———

  ## 明确假设/默认值（若与现场不符必须按此清单改配置）

  - 默认队伍：team=blue（若为 red，需同步修改 decision_node 参数与 mf_grid_layout.yaml 的 team）。
  - 赛场格距期望：1.2m；若实际不同，需改 expected_grid_pitch_m（或等价自检参数）并重新标定 layout。
  - Nav2 costmap resolution：0.1m（已在 src/rc26_bringup/config/nav2_params.yaml 中为 0.1）。
