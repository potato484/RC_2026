# rc26_bringup

## 模块定位

`rc26_bringup` 是 R2 整车链路的统一装配入口，负责选择 launch、参数、命名空间、启动顺序和可选外设。它不拥有导航、里程计、视觉、机构或底盘 transport 的算法真源。

## 当前装配口径

[bringup.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/bringup.launch.py) 默认 `run_mode:=navigation`，当前导航模式只装配以下链路：

- `rc26_mcu_transport`：默认消费 `/cmd_vel` 并下发 `POSE_TARGET(0x0C)`，同时提供 `/mechanism/send_command` 与 `/mechanism/command_feedback`。
- [odometry.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/odometry.launch.py)：启动 Point-LIO、`rc26_odom_interface` 和必要 TF/odom 输出；导航模式传入 `start_sensor_scan:=false`，并显式传入 `odom_interface_publish_bootstrap_pose:=false`，避免后续 `OdomDriveX/Y` 把 bootstrap 零位姿 `/odom` 当成真实闭环输入。
- `rc26_decision`：加载当前红/蓝运行配置中 `r2_runtime.paths.behavior_tree_file` 指向的行为树。`bringup.launch.py` 现在尊重运行配置中的既有 `startup_wait_for_odom` 参数；标准红/蓝配置显式设为 `false`，因此 `start_r2_auto.sh` 启动后立即创建并 tick 行为树，不再等待全局 startup odom gate。未显式传入 `runtime_config_file` 时，`bringup.launch.py` 会按 `r2_active_side.yaml` 的 `preselection_mode` 覆盖默认树：`first` 使用 `mc_repeat_preselection_tree.xml`；`second` 默认使用 `second_preselection_combo_tree.xml`，仅当 `second_preselection_kfs_search_compat_enable: true` 时改用 `second_preselection_kfs_compat_combo_tree.xml`。
- RealSense D455：仅当 `use_realsense:=true` 时启动，用于视觉任务，不属于导航必需节点。

`preselection_mode: second` 的默认 managed 装配以 `second_preselection_combo_tree.xml` 为入口，两条人工限位路径统一切换到 `second_preselection_climb_place_tree.xml`。入口直接收到人工触发外部限位 2 上行 `0x10` 时，组合树完成 `0x11/同 seq 0x0D` 后立即切树，不执行斜坡；入口收到人工触发外部限位 1 上行 `0x06` 时，先完成同一握手并执行合并斜坡前进、停车等待，再由只接受 `0x10` 的斜坡后 gate 完成第二次握手并切到同一目标树。兼容开关开启时，入口与握手语义保持不变，但装配改用 `second_preselection_kfs_compat_combo_tree.xml`，两条路径切换到 `second_preselection_kfs_compat_climb_place_tree.xml`：树首执行 `0x14/同 seq 0x12` 放下机械臂、停车 0.5s、沿 `+X` 前进 1.5m，再执行 `0x12/同 seq 0x11` KFS 夹取，不发送默认目标树树首预装 `0x15`，也不启用视觉搜索、对齐、趋近或消失验证；收尾在 25s 后以 `wait_ack=false` 单次尝试 `0x15`，立即固定等待 10s，再下发最终 `0x13`。`second_preselection_tree.xml` 继续保留为可显式选择的独立完整视觉流程，不属于 managed 默认或兼容入口。直接显式运行任一目标树时，树不会等待外部限位，也不会自行发送 `0x11` 或等待 `0x0D`；managed 使用时这些握手由对应组合树负责。

MC 武馆区端头视觉使用外接 FHD Webcam，而不是 RealSense D455。当前红/蓝运行配置把 `mc_camera_device` 固定到 `/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._FHD_Webcam_SN0001-video-index0`，该 by-id 路径对应 Vidar/Sonix FHD Webcam 的 video-index0；`video-index1` 是 UVC metadata，不能作为 OpenCV 图像源。`mc_auto_scan_camera=false`，固定路径失效时直接报错，不再兜底扫描其它 `/dev/video*`。

导航模式不装配地图定位、外部地图规划链、代价图、路径规划/控制平滑链或 `rc26_sensor_scan`。`/cmd_vel` 的发布权威在决策侧，默认消费方在 `rc26_mcu_transport`；同一时刻不得再启动遥控、测试动作或其它速度发布者。标准自动链已关闭整棵行为树前的 `/odom` 启动 gate，但没有删除 odom 闭环动作自身的输入检查：`OdomDriveX/Y`、转向、台阶 heading 等动作仍必须捕获新鲜真实 `/odom` 后才会发布运动速度；没有 `/odom` 时它们会保持停车等待或按各自动作超时语义结束。

`run_mode:=mapping` 仍用于建图/定位相关联调，可按需要启动 `localization.launch.py`。这条链路不是默认导航闭包，不改变导航模式的最小装配边界。

## 关键入口

- [start_r2_auto.sh](/home/potato/RC_2026/start_r2_auto.sh)：根目录自动决策/比赛链路快捷入口，默认读取 `r2_active_side.yaml` 并以 `use_realsense:=false` 启动导航决策链；旧视觉树或相机调试需显式使用 `--use-realsense`，`--no-realsense` 继续作为兼容参数。标准红/蓝配置关闭全局 startup odom gate，脚本 `--dry-run` 摘要会打印实际配置值；脚本链路还会额外启动上行人工触发外部限位 3 `0x13` 红蓝切换监听器。
- [launch/bringup.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/bringup.launch.py)：整车导航/建图统一入口。
- [scripts/startup_ready_notify_node.py](/home/potato/RC_2026/src/rc26_bringup/scripts/startup_ready_notify_node.py)：导航决策模式下默认启用的启动就绪通知节点。它只等待 `rc26_decision` 发布“正在人工限位 branch gate 等待”的内部状态和 `/mechanism/send_command` service 就绪，然后以 `wait_ack=false` 下发一次 `STARTUP_READY_WAITING_LIMIT(0x20)` 空 payload；不再订阅或等待 RealSense。若人工限位 `0x06/0x10` 先到则本次启动跳过通知。
- [launch/odometry.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/odometry.launch.py)：Point-LIO、里程计接口、静态外参和可选 sensor scan 装配。
- [launch/grid_heading.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/grid_heading.launch.py)：独立 yaw heading 校准入口，只启动 odom、MCU transport 和 `grid_heading_tree.xml`。
- 独立 odom 单轴右转分段入口：保留为包内验证入口，只启动 odom、MCU transport 和独立右转验证树。
- [config/r2_active_side.yaml](/home/potato/RC_2026/src/rc26_bringup/config/r2_active_side.yaml)：默认红蓝方与 first/second 预选入口选择入口，`active_side` 指向 `r2_red.yaml` 或 `r2_blue.yaml`，`preselection_mode` 选择 managed 默认树，`second_preselection_kfs_search_compat_enable` 选择 second managed 是否启用 KFS 搜索兼容 profile，并显式维护 first MC 重复开关、重复次数和前进步进。
- [config/r2_red.yaml](/home/potato/RC_2026/src/rc26_bringup/config/r2_red.yaml) / [config/r2_blue.yaml](/home/potato/RC_2026/src/rc26_bringup/config/r2_blue.yaml)：红/蓝双方独立完整运行配置，维护点云路径、行为树路径、MCU transport 和决策参数；first MC 重复策略默认由 `r2_active_side.yaml` 维护，避免红/蓝配置内出现第二份默认值。
- [rviz/navigation_default.rviz](/home/potato/RC_2026/src/rc26_bringup/rviz/navigation_default.rviz)：只用于观察 odom/TF 的轻量预设。

## 红蓝配置口径

默认 `bringup.launch.py` 在未显式传入 `runtime_config_file` 时读取 `r2_active_side.yaml`，按 `active_side: red|blue` 选择 `r2_red.yaml` 或 `r2_blue.yaml`，并按 `preselection_mode: first|second` 覆盖默认行为树。`second_preselection_kfs_search_compat_enable` 默认 `false`，只在 managed `second` 下改变组合树 profile；`first` 模式忽略该值。first 模式还会从 `r2_active_side.yaml` 读取 `first_preselection_mc_repeat_enable`、`first_preselection_mc_repeat_max_count` 和 `first_preselection_mc_repeat_forward_x_step_m.red/blue`，作为 first MC 重复策略的默认真源；第 0 轮距离使用当前红/蓝运行配置的 `mc_nav_forward_x_m`，后续重复轮在此基础上按当前 `active_side` 对应的带符号 step 调整。现场切换比赛方、first/second 入口、second KFS 兼容 profile 或 first MC 重复策略优先改 `r2_active_side.yaml`。如需临时调试其它完整配置，仍可传入 `runtime_config_file:=/abs/path.yaml` 覆盖；显式配置不会被 `preselection_mode`、second 兼容开关或 first repeat selector 改写。

`start_r2_auto.sh` 专用上行人工触发外部限位 3 `0x13` 监听器只订阅 `/mechanism/command_feedback`，收到 `feedback_id=0x13` 后写回 `r2_active_side.yaml` 顶层 `active_side`：当前 `red` 切到 `blue`，当前 `blue` 切到 `red`。该监听器不由 `bringup.launch.py` 自动启动；直接 `ros2 launch rc26_bringup bringup.launch.py run_mode:=navigation` 不会启用自动红蓝切换，且该切换只影响下一次启动选择的红/蓝运行配置。现有第二预选赛下行 `SECOND_PRESELECTION_PLACE_KFS(0x13)` 命令保持不变，上下行 0x13 分属不同协议方向。

`startup_ready_notify_node.py` 是 `bringup.launch.py` 的普通装配节点，不直接打开串口。它只依赖 `/mechanism/send_command`、`/mechanism/command_feedback` 与 `/decision/preselection_gate_state`。默认 `startup_ready_notify_enable` 为空时在 navigation 且 `use_decision=true` 的装配中启用，与 `use_realsense` 无关；若 gate 或机构 service 在 `startup_ready_notify_timeout_s`（默认 60.0s）内未满足，节点只告警，不阻塞原人工限位 gate。发送条件还要求本次尚未发送且未先收到人工限位 `0x06/0x10`；限位先到时永久跳过本次 `0x20`。

红蓝切换写回采用可恢复的两阶段提交：候选配置先写入同目录临时文件并同步文件与父目录，再提升为隐藏 `.pending`；正式 `r2_active_side.yaml` 完成原子替换和父目录同步后才删除 `.pending`。若监听器在持久化提交后被强制终止或设备断电，`start_r2_auto.sh` 会在读取 `active_side` 之前执行 `--recover-only`，使用 `.pending` 完成替换；同时兼容恢复旧实现遗留且修改时间不早于正式配置的 `.tmp` 文件。若旧 `.tmp` 已写完整则采用其中的目标配置，若只创建或写入了一部分，则把该文件视为已收到红蓝切换请求的持久化意图，并基于当前正式配置重建一次切换。恢复后的比赛方直接用于本次重启。

该保证从事务文件已经在本机文件系统中可见开始成立；如果进程在收到 MCU 消息后、尚未来得及创建任何事务文件之前即被强杀或整机断电，本机没有可恢复证据，无法凭空确认该次请求。MCU 侧仍应在未获得后续业务确认时保留重试能力。

每个红/蓝运行配置的 `r2_runtime.paths` 当前只维护：

- `prior_pcd_file`
- `behavior_tree_file`

`r2_runtime.mcu_transport` 维护目标 MCU 串口、底盘 `/cmd_vel` consumer 和发送限幅参数。任何真实运动或机构动作链都必须确保该 provider 已启动，除非现场明确由其它同等 provider 接管。

`r2_runtime.decision.ros__parameters` 中与导航直接相关的参数包括：

- `startup_wait_for_odom` / `startup_odom_*`：可选的整树启动前 `/odom` 新鲜度和低速稳定 gate。标准红/蓝自动配置将开关设为 `false`；其余阈值保留给显式重新启用 gate 的自定义配置和独立验证入口。
- `odom_relative_nav_*`：`OdomDriveX`、`OdomDriveY`、`OdomTurnToYaw` 共享 topic、速度、增益、容差、稳定 tick 和超时。
- `team`：红蓝方场地镜像选择；`r2_red.yaml` 固定 `red`，`r2_blue.yaml` 固定 `blue`。路线数值仍按红方基准维护，`team:=blue` 时由 `rc26_decision` 启动加载阶段派生蓝方 Y/yaw 镜像值。bringup 只负责选择配置和传参，不在 launch 中承载红蓝方路线逻辑。
- `mc_nav_forward_x_m`、`mc_nav_right_turn_delta_rad`、`mc_nav_reverse_x_m`、`mc_after_rotate_retreat_y_m`、`mc_nav_timeout_sec`：MC 去程复合动作和夹取后退让参数，继续保留红/蓝运行配置中的现场标定值。first 默认重复流程以 `mc_nav_forward_x_m` 作为第 0 轮距离，通过内部黑板值 `mc_preselection_effective_forward_x_m` 临时传入每轮 MC；`mc_nav_right_turn_delta_rad` 和旋转退让方向仍按 `team` 镜像，`mc_nav_reverse_x_m` 不随 repeat 自动步进；夹取后旋转完成的 `OdomDriveY` 由 `mc_after_rotate_retreat_y_m` 显式配置，当前蓝方为正向 `0.2m`，不再反向退让。
- `mf_preselect_entry2_nav_segment1_x_m`、`mf_preselect_entry2_nav_segment1_y_m`、`mf_preselect_entry2_nav_timeout_sec`：MF 预选 2 号入口红方基准单轴段，默认 `+X 2.0m -> -Y 1.8m`，不在入口树前额外转向；蓝方自动镜像 Y。
- `mf_preselect_kfs_align_target_line_offset_px`：MF KFS 视觉横移对齐时，识别框中线要对齐的目标线相对图像中心线的像素偏置；默认 `0`，负值表示目标线向图像左侧移动。
- `mf_preselect_kfs_depth_roi_size`、`mf_preselect_kfs_depth_min_valid_count`、`mf_preselect_kfs_depth_bbox_sample_ratios`、`mf_preselect_kfs_depth_bbox_min_success_count`：MF R2 KFS 有效深度点配置，分别控制单点 ROI 边长、单 ROI 最少有效点、bbox 多点采样比例和最少成功采样点数；默认值写在 `r2_red.yaml` / `r2_blue.yaml`，bringup 只负责传参。
- `preselection_entry_continue_delay_msec`：first 每轮人工触发外部限位 1 上行 `0x06 -> 0x10/0x0C` 启动握手完成后进入 MC 前的延时。
- `first_preselection_mc_repeat_enable`、`first_preselection_mc_repeat_max_count`、`first_preselection_mc_repeat_forward_x_step_m.red/blue`：first MC-only 重复流程参数，默认开启；`max_count` 表示初始 MC 后最多重复次数。第 0 轮使用当前红/蓝运行配置中的 `mc_nav_forward_x_m`，后续重复轮直接叠加当前 `active_side` 对应的带符号 step；当前 red 为 `+0.2m`，三轮 MC X 为 `0.05m -> 0.25m -> 0.45m`，当前 blue 为 `-0.2m`，三轮 MC X 为 `1.05m -> 0.85m -> 0.65m`。`r2_active_side.yaml` 必须同时提供 red/blue 两个 step，不再支持单个 scalar step。
- `preselection_ramp_forward_x_m`、`preselection_ramp_max_speed_mps`、`preselection_ramp_min_speed_mps`、`preselection_ramp_forward_timeout_s`：second managed 中人工触发外部限位 1 上行 `0x06` 分支的合并斜坡前进参数；该段只沿 `+X` 发布，超调不反向拉回，超时停车后继续后续 gate。
- `second_preselect_after_ramp_turn_delta_rad`、`second_preselect_after_ramp_turn_timeout_s`：历史 second managed 斜坡后转向参数，当前默认 `second_preselection_combo_tree.xml` 不再使用。`0x06` 分支合并斜坡后停车等待人工触发外部限位 2 上行 `0x10`，再次完成 `0x11/0x0D` 握手后切到 `second_preselection_climb_place_tree.xml`；入口 `0x10` 分支完成首次握手后也直接切到同一目标树。
- `second_preselect_pre_approach_lower_command_id`、`second_preselect_pre_approach_lower_done_feedback_id`、`second_preselect_pre_approach_lower_settle_s`：第二预选赛视觉对齐后、odom 前向趋近前的机械臂彻底放下握手参数。当前默认下发 `0x14`，等待同 `seq` 的 `0x12`，再停车等待 `0.5s` 后才允许前进。
- `second_preselect_pickup_command_id`、`second_preselect_pickup_done_feedback_id`、`second_preselect_search_*`、`second_preselect_r2_target_*`、`second_preselect_r1_*`、`second_preselect_kfs_*`、`second_preselect_grab_verify_*`、`second_preselect_grab_settle_s`：第二预选赛搜索夹取链参数。当前树内前向趋近后的 `0x12` 用作 KFS 夹取触发，ACK 后先等待同 `seq` 的 MCU 上行 `0x11` 夹取完成反馈，再由视觉消失验证确认夹取。
- `second_preselect_post_pickup_forward_x_m`、`second_preselect_nav_y1_m`、`second_preselect_total_x_target_m`、`second_preselect_total_x_tolerance_m`：第二预选赛夹取后的总 X 闭环参数。夹取确认后先沿 `+X 1.5m`，再按红方基准 `+Y 0.75m`（blue 自动镜像为 `-Y`），随后以最初搜索前记录的 odom 位姿和搜索起始 yaw 为原点，将搜索、夹取趋近和固定 `+X 1.5m` 的真实净投影补齐到 `4.2m`；若进入节点时已经超出目标则停车成功，不倒退补偿。
- `second_preselect_place_fixed_forward_x_m`、`second_preselect_place_fixed_forward_timeout_s`、`second_preselect_place_observe_timeout_s`、`second_preselect_place_occupied_*`：第二预选赛放置准备参数。当前放置准备使用中/下层视觉分层：中层 KFS 只表示占据，下层 KFS 只作为横向中心对齐目标；连续稳定帧数由 `second_preselect_place_occupied_stable_frames` 控制。每个观察 checkpoint 若超过 `second_preselect_place_observe_timeout_s` 仍没有稳定判断，会进入固定前进放置，终止原因写为 `PLACE_OBSERVE_TIMEOUT_FIXED_FORWARD`，不直接失败也不跳过到 `0x13`；下层 KFS 横向对齐仍由 `second_preselect_kfs_align_timeout_s` 控制。最终固定沿 `+X second_preselect_place_fixed_forward_x_m` 前进，运动超时使用 `second_preselect_place_fixed_forward_timeout_s`，超时停车后继续唯一一次首次 `0x13`。
- `second_preselect_climb_place_*`：默认 `second_preselection_climb_place_tree.xml` 与 KFS 兼容目标树共享路线、人工前激光、推杆、odom 后轮段和最终放置参数。默认目标树进入后立即且只下发一次预装 KFS 夹取 `0x15`，只等待通用 ACK；ACK 成功或机构 service/请求/ACK 异常容错后立即开始路线，不等待同 seq `0x14`。兼容目标树不执行这个树首命令，而是复用 `second_preselect_pre_approach_lower_*` 以 `0x14/同 seq 0x12` 放下机械臂，按 `second_preselect_pre_approach_lower_settle_s=0.5` 停车稳定后执行现有 `forward_x=1.5m`，再复用 `second_preselect_pickup_*` 以 `0x12/同 seq 0x11` 完成 KFS 夹取；两步机构失败沿用通用 WARN 后继续语义，整个兼容树不消费视觉。随后执行红方基准 `lateral_y=+0.3m`（blue 自动镜像为 `-0.3m`）、人工前激光上阶和 `rear_forward_x=0.6m` 后轮 odom 段。默认收尾为 `0x0B -> 从节点进入起累计 25s -> 0x13`；兼容收尾为 `0x0B -> 同样累计 25s -> 单次 0x15(wait_ack=false) -> 固定 second_preselect_climb_place_compat_pickup_to_final_delay_s=10s -> 0x13`，不等待 0x15 的 service response、ACK 或 `0x14`。两种收尾的机构失败均只告警续跑或正常结束；树内均不含 managed gate，也不自行发送 `0x11` 或等待 `0x0D`。
- `second_preselect_dynamic_roi_ui_enable`、`second_preselect_dynamic_roi_ui_window_name`：第二预选赛本地 OpenCV 视觉调试窗口参数。现场开启后会贯穿 KFS 搜索、视觉对齐、夹取完成反馈等待、夹取消失验证、占位判断和任意 KFS 放置对齐，显示识别框、锁定目标、目标线和阶段状态，不改变决策结果。
这些参数描述相对分段和 odom yaw 目标生成，不是地图位姿。现场标定时应按启动姿态重新调整每段 `distance_m` 和相对/绝对 yaw；蓝方若只做标准镜像，保持 `r2_blue.yaml` 的 `team: blue` 即可复用同一组红方基准值。

## 独立入口

`grid_heading.launch.py` 会加载 `grid_heading_tree.xml`，执行 `GridTurn -> GridHeadingAlign`。它用于 MF 格间 heading 能力或单独 yaw 校准，不是通用分段导航转向动作。该入口启动 odometry 时显式关闭 sensor scan。

`grid_heading.launch.py` 与完整导航模式一样，启动 odometry 时显式关闭 `odom_interface` bootstrap `/odom`。该入口会直接发布 `/cmd_vel` 做 yaw 对齐，不能在真实里程计未接管时用占位零位姿放行动作。

独立右转验证入口会加载包内右转验证树，执行 `OdomDriveX(+0.4m) -> RelativeYawTarget(-90deg) -> OdomTurnToYaw -> OdomDriveX(-0.7m)`。右转独立入口专用参数族已退役，不再由红/蓝运行配置维护或由 `decision_node` 加载；该入口的启动 odom gate 复用通用 startup / odom 相对导航参数或固定默认值。该入口默认关闭 bootstrap odom，先等真实 odom 稳定后再 tick 行为树；启动 odometry 时同样显式关闭 sensor scan。

## RViz 与测试资产

`navigation_default.rviz` 只保留 Grid、TF 和 odom 相关观察能力，不提供外部目标发布工具，不作为状态真源，也不得接管 `/cmd_vel`。

旧地图规划配置、地图资产、外部 action 测试 launch、外部 controller bag 评估脚本和绝对位姿采点脚本已从安装闭包移除。相对分段导航由当前红/蓝运行配置中的显式段参数人工标定。

## 边界

- `rc26_bringup` 只负责装配和参数选择，不承载导航控制算法。
- 导航模式下不启动定位、sensor scan、地图服务、外部规划/控制/平滑链。
- `rc26_odom_interface` 是 `/odom` 与动态基座 TF 的当前运行时来源；目标 MCU 底盘执行由 `rc26_mcu_transport` 提供。
- `/cmd_vel` 默认由 `rc26_decision` 的导航/动作节点串行发布，由 `rc26_mcu_transport` 消费；运行遥控或其它测试入口时必须显式保证命令权威唯一。

## 本轮同步

2026-07-14 同步：second KFS 兼容目标树删除视觉搜索链，改为切树后立即执行机械臂放下 `0x14/同 seq 0x12`、停车 0.5s、现有 `OdomDriveX(+1.5m)`、KFS 夹取 `0x12/同 seq 0x11`，随后继续原 Y/20s/上阶/兼容收尾。兼容树不再创建 `SecondPreselectionKfsPickup`，因此不依赖 RealSense 图像、视觉对齐、深度趋近或目标消失验证；默认 managed second 与显式完整视觉树保持不变。

2026-07-14 同步：`start_r2_auto.sh` 默认改为 `use_realsense=false`，新增 `--use-realsense` 显式启用入口并保留 `--no-realsense`。启动就绪 `0x20` 与 RealSense 解耦：navigation+decision 默认创建通知节点，只等待 managed gate `waiting=true` 与机构 service 就绪；`0x06/0x10` 先到则跳过，超时只告警。dry-run 同时显示 RealSense 与 `0x20` 的实际开关。

2026-07-14 同步：标准 `r2_red.yaml` / `r2_blue.yaml` 新增既有参数 `startup_wait_for_odom=false`，`bringup.launch.py` 改为尊重运行配置而不再在 navigation 模式强制覆盖为 `true`。因此 `start_r2_auto.sh` 启动后会立即创建并 tick 行为树，人工限位 branch gate 不再被全局 odom 稳定条件挡住；脚本 `--dry-run` 会显示实际开关值。该调整没有删除 `OdomDriveX/Y`、yaw、台阶等动作自身的新鲜 odom 检查，也没有重新启用 bootstrap `/odom`；独立右转验证 launch 仍显式开启启动 gate。

2026-07-14 同步：`r2_active_side.yaml` 新增默认关闭的 `second_preselection_kfs_search_compat_enable`。它只在未显式传入 `runtime_config_file` 且 `preselection_mode: second` 时生效：默认继续使用现有组合树和目标树；开启后改用独立兼容组合树，两条人工限位路径都切到独立 KFS 兼容目标树。`start_r2_auto.sh --dry-run` 会打印该值和作用域，红蓝切换事务只改 `active_side` 并保留兼容开关。红蓝运行配置新增 `second_preselect_climb_place_compat_pickup_to_final_delay_s=10.0`，用于兼容收尾的固定等待。

2026-07-14 同步：运行参数名、红蓝数值和 ROS 接口保持不变，但机构失败结果统一改为 WARN 后继续既定下一阶段；service 不可用、请求异常、拒绝、ACK/done 超时、串口暂不可用和非 BUSY `0xFE` 不再终止 decision 行为树。普通 odom、视觉、人工限位/激光和非法运行上下文仍保持原语义。目标上阶夹取放置树的树首 `0x15` 现在只等通用 ACK，不等待 `0x14`；兼容参数 `second_preselect_climb_place_preload_pickup_done_feedback_id` 继续保留在红蓝配置中但不被该树首节点消费。

2026-07-13 同步：`second_preselection_climb_place_tree.xml` 的预装 KFS 夹取顺序前移。目标树现在先下发唯一一次 `0x15`；2026-07-14 起只等通用 ACK 后即执行 Odom 路线和上阶，不再等待同 seq `0x14`。后轮上阶完成后只下发 `0x0B`，从该完成时点起等待 25s 后发送 `0x13`，后段不再重复 `0x15`。红蓝参数值与参数名不变，只同步了真实消费语义和注释。

2026-07-13 同步：second managed 统一目标树的前推杆前 yaw 对齐改为 best-effort。对齐失败或超时不再使行为树返回 `FAILURE`，而是停车、记录 WARN 后继续发送 `0x08`；前推杆后仍保持零速等待 `0x15`，后轮 `0.6m` 超时继续。2026-07-14 起其它机构失败也统一告警续跑，人工激光和普通 Odom 等非机构失败不变。

2026-07-13 同步：second managed 统一目标树的前推杆后阶段改为持续零速等待人工前激光 `0x15`；`0x08` ACK 和稳定延时结束后不再发布前轮行驶速度，只有新 `0x15` 才进入前收/后伸。其后的 `+X 0.6m` 后轮 odom 段按当前红蓝配置 `5.0s` 超时，捕获有效目标后的动作超时会停车成功并继续最终夹取/放置；其它 OdomDrive 默认失败语义不变。本轮没有修改 ROS topic/service、MCU ID 或运行配置参数集合。

2026-07-13 同步：`preselection_mode: second` 的两条真实装配路径统一切到 `second_preselection_climb_place_tree.xml`：入口直接上行 `0x10` 在 `0x11/同 seq 0x0D` 后跳过斜坡切树；入口上行 `0x06` 在首次握手后执行合并斜坡并停车，斜坡后只接受 `0x10`，再次握手后切到同一树。组合树不再自动进入 `SecondPreselectionTree` 搜索流程；`second_preselection_tree.xml` 继续作为显式独立完整树。目标树本身不等待 gate，也不发送 `0x11/0x0D`，managed 使用时握手由组合树完成。

2026-07-13 同步：红/蓝运行配置新增 `second_preselect_climb_place_*` 参数组，供显式选择和 second managed 目标树共同使用。该树不扩展 `r2_active_side.yaml` 的 `preselection_mode`，仍由 `second_preselection_combo_tree.xml` 作为 second managed 入口；显式独立选择时应关闭依赖 managed gate 的启动就绪通知。

2026-07-13 同步：second managed 入口斜坡由两段 `OdomDriveX` 合并为 `SecondPreselectionRampForward` 单段前进。红/蓝运行配置改用 `preselection_ramp_forward_x_m=5.00` 和 `preselection_ramp_forward_timeout_s=5.0`；该动作只发布非负 `linear.x`，到达、超调或超时都会停车并继续后续人工限位 2 gate，不再用负速度拉回目标点。

2026-07-13 同步：`startup_ready_notify_node.py` 作为 ROS2 launch libexec 节点安装，源码必须保持可执行权限；`--symlink-install` 下 libexec 入口指向源码，测试已覆盖 shebang 与可执行位，避免完整自动链路启动时被 ROS2 判定为 executable not found。该节点的 `command_id` 参数允许 launch 传入 `0x20` 字符串或十进制整数，节点内部统一解析为 `uint8`，避免 ROS2 参数类型推断导致启动就绪通知节点提前退出；`start_r2_auto.sh` 专用 active-side 监听器现在在 Ctrl+C / 外部关闭时静默退出，不再打印正常停止路径的 traceback。

2026-07-12 同步：完整自动链路引入启动就绪 `0x20` no-ack 通知；当前实现已在 2026-07-14 与 RealSense 解耦。`bringup.launch.py` 在 navigation、`use_decision=true` 且 `startup_ready_notify_enable` 未关闭时启动节点；节点等待 `/decision/preselection_gate_state` 的 `waiting=true` 与机构 service 就绪后，以 `wait_ack=false` 发送一次空 payload。可通过 `--extra-launch-arg startup_ready_notify_enable:=false` 关闭。

2026-07-10 同步：上行人工触发外部限位 3 `0x13` 红蓝切换写回改为可恢复两阶段提交。监听器持久化 `.pending`，保留恢复副本直到正式配置替换和父目录 `fsync` 完成；`start_r2_auto.sh` 在解析红蓝方前执行 `--recover-only`，可恢复强杀、进程崩溃或断电遗留的 `.pending`，并兼容旧实现留下且不早于正式配置的完整或不完整 `.tmp` 文件。新增回归测试覆盖正常提交、pending 恢复、旧临时文件恢复和过期临时文件忽略。

2026-07-09 同步：MC 夹取端头后 `RotateInPlace` 之后的 `OdomDriveY` 从 XML 固定 `-0.4m` 改为读取 `mc_after_rotate_retreat_y_m`。`r2_red.yaml` 保持旧 `-0.4m` 退让；`r2_blue.yaml` 显式配置为正向 `0.2m`，因此 `r2_active_side.yaml` 选中 `active_side: blue` 时不再执行反向 Y 退让。

2026-07-12 同步：first MC repeat 的 `first_preselection_mc_repeat_forward_x_step_m` 改为 red/blue 必选带符号增量映射，不再按第 0 轮 X 符号强制取绝对值，也不再支持单个 scalar step。当前 red first 使用 `mc_nav_forward_x_m=0.05m` 和 step `+0.2m`，三轮 MC X 为 `0.05m -> 0.25m -> 0.45m`；当前 blue first 使用 `mc_nav_forward_x_m=1.05m` 和 step `-0.2m`，三轮 MC X 为 `1.05m -> 0.85m -> 0.65m`。


2026-07-09 同步：删除决策节点旧全局 0x10 监听链路。人工触发外部限位 2 的上行 `0x10` 现在只由 `WaitPreselectionBranchGate` 在当前行为树位置消费，并按 XML 配置的 `mc` 或 `second` profile 完成握手与切树；bringup 不再注入旧全局监听关闭参数，红/蓝运行配置也不再保留旧全局触发参数。

2026-07-08 同步：`start_r2_auto.sh` 新增专用上行人工触发外部限位 3 `0x13` 红蓝切换监听器。脚本非 dry-run 启动时会在后台订阅 `/mechanism/command_feedback`，收到上行 `feedback_id=0x13` 后只修改 `r2_active_side.yaml` 顶层 `active_side`，用于下一次启动选择红/蓝运行配置；dry-run 仅打印监听器命令。该能力不进入 `bringup.launch.py`，避免直接 bringup 启动时隐式写配置。下行 `SECOND_PRESELECTION_PLACE_KFS(0x13)` 保持第二预选赛放置命令语义，与上行人工触发外部限位 3 分属不同方向。

2026-07-08 同步：`r2_blue.yaml` 重新对齐红方基准配置，除 `team: blue` 和蓝方现场标定保留的 `mc_nav_forward_x_m` 外，第二预选赛、台阶和其它决策参数值与 `r2_red.yaml` 保持一致；同时补齐 `second_preselect_kfs_lost_servo_speed_scale` 与 `second_preselect_kfs_align_offset_filter_alpha`，避免蓝方配置缺少红方已有的短暂丢框伺服和 offset 滤波入口。

2026-07-05 历史同步：当时的第二预选赛放置链曾使用 `+Y 0.7m -> +X 4.5m -> R1_KFS 对齐 -> +X 0.8m`；该路线已被 2026-07-10 的总 X 与任意 KFS 放置流程替代。

2026-07-10 同步：红/蓝运行配置维护搜索起点总 X `4.2m` 闭环、夹取后固定 `+X 1.5m`、`±Y 0.75m`、双框占位两级避让和任意 KFS 真实深度放置趋近；新增 `second_preselect_place_no_depth_forward_x_m=1.5`，用于场地清空后连续无真实深度时的累计降级前进。`second_preselect_kfs_approach_timeout_s=8.0` 现在共用于初次夹取趋近、无深度等待和最终放置趋近，旧 `second_preselect_place_approach_timeout_s` 已删除；降级前进运动超时继续使用 `second_preselect_nav_timeout_s`。红蓝 YAML 只维护红方基准距离，Y 段和占位避让方向继续由 `team` 镜像。

2026-07-05 同步：红/蓝运行配置新增第二预选赛视觉对齐后的机械臂放下握手参数：`second_preselect_pre_approach_lower_command_id=0x14`、`second_preselect_pre_approach_lower_done_feedback_id=0x12` 和 `second_preselect_pre_approach_lower_settle_s=0.5`。`rc26_decision` 收到同 `seq` 放下完成反馈并停车等待后，才开始原有 KFS odom 前向趋近。

2026-07-04 同步：红/蓝运行配置跟随第二预选赛新流程补齐搜索夹取链参数。`0x12` 现在在第二预选赛内作为 KFS 夹取触发命令使用，ACK 后先等待同 `seq` 的 `second_preselect_pickup_done_feedback_id=0x11`，再由 `rc26_decision` 做视觉消失验证；旧 `second_preselect_arm_high_raise_*`、`second_preselect_nav_x1_m` 与放置前 KFS 必见 gate 已从运行配置中删除。

2026-07-04 同步：红/蓝运行配置新增第二预选赛 OpenCV 视觉调试窗口参数。窗口现在贯穿 KFS 搜索/夹取验证和放置前视觉对齐；若窗口创建失败，决策侧会告警并继续无 UI 运行。

2026-07-04 同步：红/蓝运行配置完成统一，除 `r2_blue.yaml` 保留现场标定值 `mc_nav_forward_x_m` 和 `team: blue` 外，其余参数值、顺序与注释按红方基准维护；历史单文件 `r2_runtime.yaml` 已删除，默认和调试入口都应显式使用 `r2_red.yaml` / `r2_blue.yaml` 或其它完整自定义配置。

2026-07-03 同步：红/蓝运行配置不再显式写入 `second_preselect_grid_label_prefixes: []` 和 `second_preselect_grid_label_exact_names: []`。这两个第二预选赛标签过滤参数在默认不过滤时应省略，由 `rc26_decision` 的空 vector 默认值表达“所有非空 class_name 有效”；这样避免 ROS2 launch 通过 Python dict 传空数组时无法推断数组元素类型，并在延时启动 `decision_node` 前抛出空 tuple 参数异常。

2026-07-03 同步：红/蓝运行配置新增 `mf_preselect_kfs_depth_roi_size`、`mf_preselect_kfs_depth_min_valid_count`、`mf_preselect_kfs_depth_bbox_sample_ratios`、`mf_preselect_kfs_depth_bbox_min_success_count`。这些参数只配置 `rc26_decision` 的 R2 KFS 深度有效点判定，不把视觉算法逻辑放入 bringup。

2026-07-03 同步：`r2_active_side.yaml` 新增 `preselection_mode: first|second`。未显式传入 `runtime_config_file` 时，bringup 按该模式覆盖默认树；当前 first 为 `mc_repeat_preselection_tree.xml`，second 为 `second_preselection_combo_tree.xml`；managed 模式由 `WaitPreselectionBranchGate` 统一处理人工触发外部限位 1/2 的上行 0x06/0x10 分支。second 决策族中两条分支都使用 0x11/0x0D 握手。红/蓝运行配置新增 first gate 延时、second 合并斜坡前进和历史斜坡后转向参数；正式 MC 末尾流程不再依赖视觉配准 gate。

2026-07-02 同步：新增根目录 `start_r2_auto.sh` 作为自动决策/比赛链路快捷入口。脚本只封装 `ros2 launch rc26_bringup bringup.launch.py run_mode:=navigation`，默认读取 `r2_active_side.yaml` 并打印当前红/蓝方和选中的运行配置；当前自 2026-07-14 起默认传入 `use_realsense:=false`，需要相机时使用 `--use-realsense`。红蓝方路线、行为树、MCU transport 与决策参数仍由 `rc26_bringup` 和对应运行配置负责。

2026-07-02 同步：MCU 上行 0x10 收敛为人工触发外部限位 2 事件。当前 managed first/second 入口下，该事件由 branch gate 消费；旧 first 组合树曾使用 0x10/0x0C 后切梅林树，当前 first 默认重复树仅在 MC 末尾把 0x10 作为舵机放下握手，second 使用 0x11/0x0D 后切对抗区树。

2026-07-02 同步：默认运行配置拆分为 `r2_red.yaml` / `r2_blue.yaml`，由 `r2_active_side.yaml` 选择当前比赛方。显式传入 `runtime_config_file` 仍可覆盖，供临时调试自定义完整配置使用。

2026-07-01 同步：右转导航独立入口专用参数族已从运行配置中删除，并且 `rc26_decision` 不再声明、读取或写入这些参数。独立右转验证入口若继续保留，启动 odom gate 改用通用 startup / odom 相对导航参数或固定默认值。

2026-07-01 同步：`team` 参数现在由 bringup 传入 `rc26_decision` 后作为红蓝方场地镜像选择使用。运行配置中 MC/MF 路线值继续按红方基准维护；`team:=blue` 时，决策节点在启动加载参数阶段派生蓝方入口 Y、侧向 yaw、入口 1/3 号横移和假 KFS 侧列绕行等镜像行为。bringup 未新增第二套 XML，现场切蓝方优先切换 `r2_active_side.yaml`。

2026-07-01 同步：运行配置新增 `mf_preselect_kfs_align_target_line_offset_px`，用于现场标定 MF KFS 夹取时识别框中线的目标线。默认 `0` 保持图像中心线口径；实车若夹爪肉眼已对齐但日志 offset 仍为负，可按该负值附近配置偏置，让决策的 offset 以新的目标线为 0。

2026-06-30 同步：完整 `bringup.launch.py run_mode:=navigation` 启动 odometry 时显式传入 `odom_interface_publish_bootstrap_pose:=false`，`grid_heading.launch.py` 作为独立运动入口同样关闭 bootstrap `/odom`。决策启动 gate 现在不会再被 `rc26_odom_interface` 启动占位零位姿放行；若真实 Point-LIO `/odom` 未接管，导航会继续等待并按启动 gate 超时失败，而不是进入 `OdomDriveX` 后持续下发前进速度。

2026-06-30 同步：默认导航装配切为 odom-only 决策闭环链路。`bringup.launch.py run_mode:=navigation` 只启动 MCU transport、odometry、decision 和按需 RealSense；导航模式关闭 sensor scan，并移除旧地图规划链路参数、配置、资产和测试入口。运行配置维护 odom 单轴分段导航参数和 startup odom gate；MC 默认路线为 `+X 0.2m -> 右转 90° -> -X 0.6m`，MF 预选入口默认路线为 `+X 2.0m -> -Y 1.8m`。
