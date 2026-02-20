╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌
 执行方案：改进方案5 落地实施                           

 Context

 基于 MVP技术方案/定位/改进方案/改进方案5.md（v5.0-综合收敛版），该方案已完成评分对齐分析。当前仓库存在三个已确认的
 P0 级 bug 和两个 P1 改进点，目标是在 4 分钟赛场节奏下实现「禁踩 KFS 0 违规 + 不规划死锁 + 可控降级」。

 ---
 已确认 Bug（代码级核实）

 编号: BUG-1
 文件: kfs_block_fuser.hpp:31
 问题: free_thresh_=0.35 < 0.5，衰减目标 P=0.5，永远无法穿越 free_thresh → 永久禁区
 确认位置: 默认值硬编码
 ────────────────────────────────────────
 编号: BUG-2
 文件: terrain_semantic_node.cpp:542-546
 问题: EMA 跳变时直接 continue，无帧数上限 → 200mm 台阶持续冻结
 确认位置: estimateCellHeights()
 ────────────────────────────────────────
 编号: BUG-3
 文件: bringup.launch.py:328 vs 332
 问题: nav2_launch 在第 5 位，kfs_block_fuser_node 在第 9 位 → 窗口期禁区未就绪
 确认位置: LaunchDescription 列表顺序
 ────────────────────────────────────────
 编号: P1-1
 文件: terrain_mode_adapter.cpp:63-66
 问题: wait_for(80ms) 超时静默返回，无重试，仅下发 2 参数
 确认位置: applyConfig()
 ────────────────────────────────────────
 编号: P1-2
 文件: decision_node.cpp:323-324
 问题: KFS 发布 2Hz / QoS depth=10，延迟窗口过大
 确认位置: kfs_timer_ 创建处

 ---
 P0 改动（必须完成）

 1. src/rc26_bringup/launch/bringup.launch.py

 改动：将 LaunchDescription 列表（当前 lines 323-336）重排，把 KFS 链路提到 Nav2 之前。

 # 目标顺序（replace lines 323-336）
 odometry_launch,
 localization_launch,
 base_ground_node,
 terrain_launch,
 map_server_node,              # ↑ 从位置7 移到位置5
 map_server_lifecycle_manager, # ↑ 从位置8 移到位置6
 kfs_block_fuser_node,         # ↑ 从位置9 移到位置7
 costmap_filter_info_server,   # ↑ 从位置10 移到位置8
 nav2_launch,                  # ↓ 从位置5 推后到位置9
 nav_mode_manager_node,
 decision_node,
 realsense_group,
 rviz_group,

 理由：pub_mask_ 已是 transient_local QoS（kfs_block_fuser.cpp:67-70），先启动可保证 Nav2 costmap
 订阅时立即收到缓存掩码。

 ---
 2. src/rc26_kfs_keepout/include/rc26_kfs_keepout/kfs_block_fuser.hpp

 改动 2a：修正 free_thresh_ 默认值（line 31）：
 // Before:
 double free_thresh_{0.35};
 // After:
 double free_thresh_{0.55};   // 衰减目标 P=0.5 < 0.55 → 可穿越 free_thresh

 改动 2b：修正 map_resolution_ 默认值（line 29），与 nav2_params.yaml costmap 0.1m 对齐：
 // Before:
 double map_resolution_{0.05};
 // After:
 double map_resolution_{0.10};

 改动 2c：增加 TTL 成员（line 34 附近），作为兜底防止通信中断造成永久禁区：
 double ttl_sec_{8.0};  // 新增参数声明
 std::array<rclcpp::Time, kGridCount> last_hit_time_{};  // 每格最后命中时间
 bool   ttl_initialized_{false};

 ---
 3. src/rc26_kfs_keepout/src/kfs_block_fuser.cpp

 改动 3a：构造函数中声明 TTL 参数（line 31 decay_rate 之后）：
 this->declare_parameter<double>("ttl_sec", ttl_sec_);
 // get_parameter 同步读取
 this->get_parameter("ttl_sec", ttl_sec_);

 改动 3b：onKfsState() 中记录每格命中时间（line 125-127 循环体内）：
 auto& lo = log_odds_[cell.grid_id];
 lo = std::clamp(lo + lo_hit_, -4.0, 4.0);
 last_hit_time_[cell.grid_id] = this->get_clock()->now();  // 新增
 ttl_initialized_ = true;                                   // 新增

 改动 3c：decayTimer() 中增加 TTL 强制释放（line 136 lo_decay 计算之后，循环之前）：
 // TTL 兜底：超时未命中则强制释放
 if (ttl_initialized_) {
     for (int i = 1; i <= 12; i++) {
         if ((now - last_hit_time_[static_cast<size_t>(i)]).seconds() > ttl_sec_) {
             log_odds_[static_cast<size_t>(i)] = probToLogOdds(0.10); // 远低于 free_thresh=0.55
         }
     }
 }

 效果验证：
 - 一次命中后：lo = 1.099（P=0.75，BLOCKED）
 - 无新命中，decay_rate=0.5/s，约 1.3s 后 lo < probToLogOdds(0.55) → 自动释放
 - TTL=8s 作为极端兜底（通信中断场景）

 ---
 4. src/rc26_terrain/include/rc26_terrain/terrain_semantic_node.hpp

 在 P0.2 参数块（line 125-134）后增加 EMA 冻结控制成员：
 // P0: EMA 冻结帧数上限
 int         freeze_max_frames_{3};       // 连续冻结超过此帧数则改用慢速 EMA
 double      ground_ema_alpha_slow_{0.25}; // 慢速 EMA 权重（台阶工况）
 // 栅格状态（line 136 后）
 std::vector<int> freeze_count_;          // 需在 initGrids() 中 resize 并清零

 ---
 5. src/rc26_terrain/src/terrain_semantic_node.cpp

 改动 5a：在 initGrids() 或等效初始化位置为 freeze_count_ 分配并清零（与 ground_z_filtered_ 同步初始化）。

 改动 5b：estimateCellHeights()（lines 541-556）替换 EMA 冻结逻辑：
 // Before (line 541-546):
 if (last_seen_sec_[idx] >= 0.0 &&
     std::abs(ground_z - ground_z_filtered_[idx]) > static_cast<float>(jump_thresh_m_)) {
     top_z_[idx] = top_z;
     last_seen_sec_[idx] = stamp_sec;
     continue;  // 跳过本帧，无上限
 }

 // After:
 if (last_seen_sec_[idx] >= 0.0 &&
     std::abs(ground_z - ground_z_filtered_[idx]) > static_cast<float>(jump_thresh_m_)) {
     top_z_[idx] = top_z;
     last_seen_sec_[idx] = stamp_sec;
     if (++freeze_count_[idx] < freeze_max_frames_) continue;  // 冻结帧数未超限，跳过
     // 超限：改用慢速 EMA 强制收敛，防止台阶场景永久冻结
     ground_z_filtered_[idx] = static_cast<float>(ground_ema_alpha_slow_) * ground_z +
                               static_cast<float>(1.0 - ground_ema_alpha_slow_) * ground_z_filtered_[idx];
     freeze_count_[idx] = 0;
     continue;
 }
 freeze_count_[idx] = 0;  // 正常帧，重置计数
 // （以下 EMA 正常路径不变）

 ---
 6. src/rc26_terrain/config/terrain_semantic.yaml

 修改两行（P0 最低要求）：
 # line 116:
 jump_thresh_m: 0.23    # 原 0.15，台阶 200mm 场景阈值上调

 # 新增于 P0.3 latency 块之后：
 freeze_max_frames: 3
 ground_ema_alpha_slow: 0.25

 ---
 P1 改动（强烈建议）

 7. src/rc26_nav_mode_manager/include/rc26_nav_mode_manager/terrain_mode_adapter.hpp

 扩展 TerrainCfg 结构体（line 16-19）：
 struct TerrainCfg {
     std::string unknown_policy;
     double      drop_forward_sector_deg;
     int         min_obstacle_area_cells{2};      // 新增
     std::string obstacle_neighbor_mode{"edge4"}; // 新增
     double      jump_thresh_m{0.23};             // 新增
 };

 ---
 8. src/rc26_nav_mode_manager/src/terrain_mode_adapter.cpp

 改动 8a：更新 profile_map_（lines 12-18），补充 3 个新字段：
 profile_map_ = {
     {"normal",      {"aggressive",   180.0, 3, "edge4", 0.23}},
     {"safe",        {"conservative", 360.0, 1, "edge8", 0.23}},
     {"mf_traverse", {"conservative", 360.0, 1, "edge8", 0.23}},
     {"mf_exit",     {"conservative", 270.0, 1, "edge8", 0.23}},
     {"mf_approach", {"aggressive",   180.0, 1, "edge8", 0.23}},
 };

 改动 8b：applyConfig() 重写（lines 49-71），增加 5 参数下发、重试、回读：
 void TerrainModeAdapter::applyConfig(const std::string& profile, const TerrainCfg& cfg) {
     if (!service_ready_) {
         if (!param_client_->wait_for_service(std::chrono::milliseconds(120))) {
             RCLCPP_WARN(this->get_logger(), "terrain param service not available");
             return;
         }
         service_ready_ = true;
     }

     const std::vector<rclcpp::Parameter> params = {
         rclcpp::Parameter("unknown_policy",           cfg.unknown_policy),
         rclcpp::Parameter("drop_forward_sector_deg",  cfg.drop_forward_sector_deg),
         rclcpp::Parameter("min_obstacle_area_cells",  cfg.min_obstacle_area_cells),
         rclcpp::Parameter("obstacle_neighbor_mode",   cfg.obstacle_neighbor_mode),
         rclcpp::Parameter("jump_thresh_m",            cfg.jump_thresh_m),
     };

     for (int attempt = 0; attempt < 3; ++attempt) {
         auto future = param_client_->set_parameters(params);
         if (future.wait_for(std::chrono::milliseconds(120)) == std::future_status::ready) {
             // 回读校验 unknown_policy
             auto gf = param_client_->get_parameters({"unknown_policy"});
             if (gf.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
                 const auto vals = gf.get();
                 if (!vals.empty() && vals[0].as_string() == cfg.unknown_policy) {
                     last_applied_profile_ = profile;
                     RCLCPP_INFO(this->get_logger(), "applied profile: %s", profile.c_str());
                     return;
                 }
             }
             last_applied_profile_ = profile;  // set_parameters 成功，回读超时仍接受
             return;
         }
         if (attempt < 2) rclcpp::sleep_for(std::chrono::milliseconds(30));
     }
     RCLCPP_ERROR(this->get_logger(),
                  "set_parameters failed after 3 attempts for profile: %s", profile.c_str());
     // 注：P2 后续可在此触发 safe 模式
 }

 ---
 9. src/rc26_decision/src/decision_node.cpp

 两处修改（lines 322-324）：
 // Before:
 pub_kfs_state_ = this->create_publisher<rc26_interfaces::msg::MfKfsState>(
     kfs_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
 kfs_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), ...);

 // After:
 pub_kfs_state_ = this->create_publisher<rc26_interfaces::msg::MfKfsState>(
     kfs_topic, rclcpp::QoS(rclcpp::KeepLast(3)).reliable());   // depth 10→3
 kfs_timer_ = this->create_wall_timer(std::chrono::milliseconds(200), ...); // 2Hz→5Hz

 ---
 10. src/rc26_kfs_keepout/config/mf_grid_layout.yaml

 新增元信息字段供未来自检使用（grid_id 坐标标定后填写，此处仅补结构）：
 meta:
   team: ""           # 赛场换边后必须手动设置
   layout_version: 1
   validated: false   # 标定完成后设 true；false 则 kfs 应警告
   grid_spacing_m: 1.2  # 相邻格中心距（用于自检容差）

 ---
 编译验证

 cd /home/potato/RC_2026
 colcon build --parallel-workers 1 --packages-select \
     rc26_kfs_keepout rc26_terrain rc26_nav_mode_manager rc26_bringup rc26_decision

 ---
 功能验证（对应改进方案5 §6）

 场景: KFS 释放（BUG-1）
 验收指标: 单次命中后，无新命中 1.5s 内 costmap 禁区解除
 方法: ros2 topic echo /kfs_filter_mask 观察
 ────────────────────────────────────────
 场景: 台阶 EMA（BUG-2）
 验收指标: 200mm 台阶跨越时 ground_z_filtered ≤300ms 收敛
 方法: terrain diagnostics + RViz
 ────────────────────────────────────────
 场景: 启动窗口（BUG-3）
 验收指标: ros2 node list 显示 kfs_block_fuser 在 nav2 前就绪
 方法: launch 日志时间戳
 ────────────────────────────────────────
 场景: TerrainModeAdapter 重试
 验收指标: 手动停止 terrain_semantic 再恢复，profile 切换仍可成功
 方法: 故障注入
 ────────────────────────────────────────
 场景: KFS 发布频率
 验收指标: ros2 topic hz /mf_kfs_state ≈ 5Hz
 方法: CLI
 ────────────────────────────────────────
 场景: 端到端禁区延迟
 验收指标: KFS 状态变化 → costmap 禁区生效 P95 ≤ 300ms
 方法: 时间戳脚本
╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌╌
