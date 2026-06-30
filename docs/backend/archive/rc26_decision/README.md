# rc26_decision

## 模块定位

`rc26_decision` 是 R2 的主决策包，采用 BehaviorTree.CPP 组织比赛流程。当前它也是完整导航链中的运动命令发布权威：导航、视觉伺服、台阶和 MF 格间动作都在行为树内串行执行，并通过 `/cmd_vel` 输出给 `rc26_mcu_transport`。

## 当前实现

- 构建产物：`rc26_decision_nodes`、`decision_node`
- 运行入口：不提供独立 launch；完整运行由 [bringup.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/bringup.launch.py) 装配
- 行为树目录：[behavior_trees](/home/potato/RC_2026/src/rc26_decision/behavior_trees)
- 当前导航实现：[bt_odom_relative_nav.cpp](/home/potato/RC_2026/src/rc26_decision/src/navigation/bt_odom_relative_nav.cpp)
- 运行参数真源：[r2_runtime.yaml](/home/potato/RC_2026/src/rc26_bringup/config/r2_runtime.yaml)

`decision_node` 启动时会加载 `r2_runtime.decision.ros__parameters`，把共享参数写入 blackboard，并注册 MC、MF、MF 预选赛、台阶和 odom 导航节点。完整导航链中默认启用启动 odom gate：只有 `/odom` 连续新鲜且低速稳定后才创建并 tick 行为树，避免开机里程计未稳定时直接运动。

## Odom 单轴分段导航

当前通用导航原语只有三类动作：

- `OdomDriveX`：进入动作时捕获当前 `/odom` 位姿和 yaw，把 `distance_m` 解释为启动车体系 X 轴相对距离；后续只闭环该轴向进度，只发布 `cmd_vel.linear.x`，并用 `angular.z` 保持启动 yaw。
- `OdomDriveY`：进入动作时捕获当前 `/odom` 位姿和 yaw，把 `distance_m` 解释为启动车体系 Y 轴相对距离；后续只闭环该轴向进度，只发布 `cmd_vel.linear.y`，并用 `angular.z` 保持启动 yaw。
- `OdomTurnToYaw`：把 `target_yaw_rad` 解释为绝对 odom yaw，只发布 `cmd_vel.angular.z`，到达 yaw 容差并稳定后成功。

`RelativeYawTarget` 仍保留为小工具节点：它订阅 `/odom`，把当前 yaw 加 `yaw_delta_rad` 后写入输出端口，供右转等相对转向测试树生成绝对 yaw 目标。它不发布 `/cmd_vel`。

`OdomDriveX` / `OdomDriveY` 端口：

- `distance_m`
- `cmd_vel_topic`
- `odom_topic`
- `max_speed_mps`
- `min_speed_mps`
- `xy_kp`
- `heading_kp`
- `heading_max_speed_radps`
- `xy_tolerance_m`
- `yaw_tolerance_deg`
- `stable_ticks`
- `odom_timeout_s`
- `timeout_s`

`OdomTurnToYaw` 端口：

- `target_yaw_rad`
- `cmd_vel_topic`
- `odom_topic`
- `kp`
- `max_speed_radps`
- `yaw_tolerance_deg`
- `stable_ticks`
- `odom_timeout_s`
- `timeout_s`

共享 blackboard 观测键：

- `relative_nav_last_exec_state`: `IDLE | RUNNING | WAITING_FOR_ODOM | SUCCEEDED | FAILED | HALTED`
- `relative_nav_last_failure_reason`
- `relative_nav_last_distance_remaining`

失败和停机语义：

- odom 未就绪或过期时发布零速等待。
- 超过动作 `timeout_s` 后发布零速并返回 `FAILURE`。
- 成功、失败和 halt 都会发布一次零速。
- 这些节点只处理 odom 闭环和 `/cmd_vel` 发布，不处理串口协议、不调用机构 service、不维护外部导航 action 兼容层。

## 行为树导航流程

- `mc_tree.xml`：`OdomDriveX(mc_nav_forward_x_m=+0.2m) -> RelativeYawTarget(mc_nav_right_turn_delta_rad=-pi/2) -> OdomTurnToYaw -> OdomDriveX(mc_nav_reverse_x_m=-0.6m) -> VisualServoGrab -> Delay -> RotateInPlace -> WaitForever`。右转后的绝对 odom yaw 会传给 `VisualServoGrab target_yaw_rad`，作为视觉阶段 heading hold 目标。
- `mf_preselection_tree.xml`：可选入口导航为 `OdomDriveX(mf_preselect_entry2_nav_segment1_x_m=+2.0m) -> OdomDriveY(mf_preselect_entry2_nav_segment1_y_m=-1.8m)`，随后进入 `MfPreselectionFlow`；该入口不在树前额外转向。
- `odom_right_turn_nav_tree.xml`：`OdomDriveX(odom_right_turn_nav_forward_x_m) -> RelativeYawTarget(odom_right_turn_nav_right_turn_delta_rad) -> OdomTurnToYaw -> OdomDriveX(odom_right_turn_nav_reverse_x_m)`。
- `relative_segment_nav_tree.xml`：独立验收用单轴分段树，依次示例调用 `OdomDriveX`、`OdomDriveY`、`OdomTurnToYaw`。

旧外部 action 位姿导航节点、TF 采点节点、旧双点位姿测试树和相关 action helper 已删除。当前 MC/MF 入口坐标不再是绝对地图位姿；`mc_nav_forward_x_m` / `mc_nav_reverse_x_m` 与 `mf_preselect_entry2_nav_segment1_*` 是按启动姿态和分段动作顺序标定的相对单轴段。

## MC 链路

`VisualServoGrabAction` 内嵌相机采集和端头推理，在工作线程中执行目标锁定、横移 P 控制、odom yaw 姿态保持、限位前探和 `GRAB_TIP(0x01)` service 请求。它只通过 `/cmd_vel`、`/mechanism/send_command` 和 `/mechanism/command_feedback` 与下层交互，不直接打开串口。

`VisualServoGrabAction` 可通过 `target_yaw_rad` 输入端口接收当前 MC 导航右转后的绝对 odom yaw；没有传入时才使用 `mc_align_target_yaw_rad` 静态兜底。`RotateInPlaceAction` 订阅 `mc_odom_topic`，发布 `cmd_vel.angular.z`。未显式传入 `target_yaw_rad` 时按相对角度旋转；传入时按绝对 odom yaw 对齐。

## MF 与台阶链路

MF 格间动作仍由 `PlanGridTransition -> GridTurn -> GridHeadingAlign -> GridTransition -> GridCenterAlign` 串行完成。`GridTurn` / `GridHeadingAlign` 服务 MF 格间 yaw 对齐和独立 heading 校准入口；它们不是通用分段导航转向节点，通用分段转向使用 `OdomTurnToYaw`。

`GridTransition` 负责选择上/下台阶动作、发布台阶直行速度、等待激光事件并提交 `current_grid`。`GridCenterAlign` 在台阶完成后依据 `mf_center_grid_step_m` 执行二维格中心归位。`MfPreselectionFlow` 内部继续负责入口探测、KFS 视觉锁定、横移视觉闭环、新视觉帧复核、机构命令、夹取视觉消失验证、入口/格间台阶和最终离场归位。KFS 横移现在复用 `rc26_vision::tip_alignment` 的目标选择、目标锁定、横移速度和 yaw gate 口径：未锁定时选择识别框中心离图像中心线最近的有效 `T_*` KFS，锁定后跟踪同一物理目标，像素 offset 以图像中心线为 0；yaw 超出 `mf_preselect_kfs_align_heading_gate_deg` 时只修正朝向，odom 不新鲜时停车等待。入口横移探测中若 KFS 还在入口中断窗口外侧，不立即停车夹取，而是继续扫线等待目标进入可夹取窗口；当前入口中断窗口以 `mf_preselect_entry_interrupt_max_offset_px` 为基础，并可按 MCU 半余弦减速模型、入口横移速度、锁定深度、相机 `fx` 回填值和延迟估计动态放大，补偿高速扫线切零速时的停车尾巴。入口横移中断后会按同一 MCU 减速模型持续发布零速等待一小段时间，再开始 KFS 视觉横移对齐；这套补偿只作用于入口横移中断，不改变梅林内部 KFS 对齐。像素误差和 yaw 误差同时进入容差后按新视觉帧累计 `mf_preselect_kfs_align_stable_frames`，稳定后由锁定深度减 `mf_preselect_kfs_grab_distance_m` 规划车体系 X 轴距离，并捕获当前 `/odom` 起点和 yaw 后闭环执行规划距离。KFS 横移不再维护目标线偏置、释放滞回或 no-progress 快速失败；横移对齐总超时时，如果最后有效锁定目标仍在 `mf_preselect_kfs_align_timeout_pickup_tolerance_px` 内且深度有效，则继续进入前向趋近和夹取，否则按原失败路线继续。前向趋近超时会使 `MfPreselectionFlow` 失败停车；成功、失败和 halt 都发布零速。假 KFS 避障不再硬编码为上阶：从中列绕到 1 号侧或 3 号侧旁列时，会先通过 `MerlinMapManager` 静态高度表计算高度差，再选择上/下台阶和机构预调。完成该横向格间动作后不再进入 `direct_exit` 直行兜底，而是沿避障后的旁列继续前向推进：1 号侧旁列按 `grid1 -> grid4 -> grid7 -> grid10`，3 号侧旁列按 `grid3 -> grid6 -> grid9 -> grid12`。每个旁列格间转换前复用现有 `TransitionObserve` 正前方观察和机构预调逻辑，只看前方 R2 KFS；看到则按现有 KFS 夹取链处理，未看到则继续对应上/下阶梯，抵达出口行后复用最终下阶离场。

`MfPreselectionFlow` 的入口、行前方、周身和 `TransitionObserve` 检测窗口在未发现 R2 KFS 时必须先等满对应 `mf_preselect_*_detect_timeout_s`，再按 `mf_preselect_detect_lost_stable_frames` 确认未命中并切到下一阶段；连续丢失帧只用于抗抖，不再提前截短检测窗口。发现 R2 KFS 或假 KFS 仍可在窗口内立即触发对应夹取/避障分支。R2 KFS 候选若被过滤，会把最近一次拒绝原因代码写入 `mf_preselect_r2_lock_reject_reason/detail/sequence` 黑板键，并在同一检测窗口内按原因去重打印中文 INFO 日志；检测 miss 日志会附带中文摘要，便于区分夹取数已满、视觉帧无效、标签不匹配、已忽略目标、深度采样失败或目标选择失败。

台阶动作既可通过 `stair_climb_tree.xml` / `stair_descend_tree.xml` 独立加载测试，也可由 MF 状态机复用。它们通过 `/mechanism/send_command` 请求推杆动作，通过 `/mechanism/command_feedback` 等待对应反馈，通过 `/cmd_vel` 发布受限直行速度；失败或 halt 时只发布零速，不做额外推杆补偿。

梅林预选赛入口高侧 KFS 夹取使用 `rc26_serial` 真源中的 `ENTRY_GRAB_KFS_UP(0x0F)`，并在 service ACK 后等待同 `seq` 的 `ENTRY_GRAB_KFS_UP_DONE(0x0B)`；ACK 只代表 transport 通用确认，真正计数仍延迟到后续视觉消失验证成功。

## 参数口径

导航相关参数集中在 `r2_runtime.yaml`：

- `odom_relative_nav_*`：单轴平移与通用 yaw 容差、增益、速度、topic 和超时。
- `startup_odom_*`：完整导航链启动前 odom 新鲜度和低速稳定 gate。
- `mc_nav_forward_x_m`、`mc_nav_right_turn_delta_rad`、`mc_nav_reverse_x_m`、`mc_nav_timeout_sec`：MC 去程原始动作顺序，默认 `+X 0.2m -> 右转 90° -> -X 0.6m`。
- `mf_preselect_entry2_nav_segment1_x_m`、`mf_preselect_entry2_nav_segment1_y_m`、`mf_preselect_entry2_nav_timeout_sec`：MF 预选入口单轴段，默认 `+X 2.0m -> -Y 1.8m`。
- `odom_right_turn_nav_*`：独立右转入口的前进、相对 yaw 捕获、绝对 yaw 对齐和后退参数。

参数在节点构造时声明并写入 blackboard；当前没有运行期参数变更回调，`ros2 param set` 不会自动回写已经进入树的参数。

## 边界

- `rc26_decision` 拥有比赛流程、行为树编排、导航段调用顺序和 `/cmd_vel` 发布时序。
- `rc26_decision` 不拥有串口协议解析、相机驱动、点云里程计、定位算法或 MCU transport。
- 完整导航链同一时刻只能有一个 `/cmd_vel` 发布权威；运行遥控、视觉动作测试、台阶独立测试或分段导航测试前必须停用其它运动发布者。
- `rc26_interfaces` 当前不提供自定义导航 action；导航对外契约只保留 `/cmd_vel` 速度输出。

## 本轮同步

2026-07-01 同步：`MfPreselectionFlow::findR2LockObservation()` 新增 R2 KFS 候选拒绝诊断。行前方、周身和 `TransitionObserve` 检测中若画面未形成可夹取目标，会记录 `pickup_limit_reached`、`vision_not_running`、`snapshot_invalid`、`no_label_match`、`ignored_target`、`depth_invalid` 或 `selection_failed`，同步写入 `mf_preselect_r2_lock_reject_reason/detail/sequence` 黑板键，并把最近拒绝中文摘要追加到“未发现”日志；黑板中的 reason 保持英文代码，现场 INFO 日志使用中文原因和中文详情。该改动只增强现场排查信息，不改变标签、深度、目标选择、夹取次数、台阶或 `/cmd_vel` 行为。

2026-07-01 同步：`MfPreselectionFlow` 新增入口横移 MCU 动态补偿。入口横移中断窗口现在可按 MCU 端 `CHASSIS_MAX_VY_ACC` 对应的半余弦减速模型计算停车距离，并叠加视觉/BT/下发延迟估计后换算为像素补偿；补偿参数为 `mf_preselect_entry_interrupt_dynamic_comp_enable`、`mf_preselect_entry_interrupt_latency_s`、`mf_preselect_entry_interrupt_fx_px`、`mf_preselect_entry_interrupt_extra_px_min/max`。`mf_preselect_entry_interrupt_fx_px` 已按当前 R2 D455 在 `640x480x30` 彩色流 `/camera/color/camera_info` 的 `CameraInfo.K[0]` 回填为 `385.83319091796875`；`aligned_depth_to_color/camera_info` 与彩色流内参一致，适用于当前深度对齐到彩色图的入口横移补偿。入口横移一旦中断进入 KFS 夹取，还会按 `mf_preselect_entry_mcu_vy_acc_mps2`、`mf_preselect_entry_mcu_stop_margin_s` 和 `mf_preselect_entry_mcu_stop_max_wait_s` 持续发布零速等待 MCU 收完横移速度规划，再开始视觉横移对齐；该等待不计入 KFS 对齐总超时。该改动只作用于入口横移中断，不改变 `/cmd_vel` 接口、MCU 串口协议、梅林内部 KFS 对齐或前向 odom 趋近口径。

2026-07-01 同步：决策侧机构指令日志统一补充 `/mechanism/send_command` response 返回的真实 `seq`。单条机构命令、台阶前后推杆并发命令、`MfPreselectionFlow` 内部机构命令和 MC `GRAB_TIP` 的 ACK / rejected 日志都会打印 `seq`；`/cmd_vel` 本身没有 `seq` 字段，本次不为速度指令伪造序号。`decision_node` 新增 `decision_last_failure_source/reason/detail` 黑板失败汇总，行为树最终 `FAILURE` 日志会用中文打印失败来源和详细原因；导航、MC、MF、台阶和 MF 预选赛动作失败时会尽量写入阶段、当前格、命令、`seq`、完成反馈、odom/topic、超时等上下文，方便现场从最终失败日志直接定位卡点。

2026-06-30 同步：`MfPreselectionFlow` 的入口横移 KFS 处理改为“看到后尽量夹取，但不被贴边框过早打断”。入口横移中有效 R2 KFS 若像素偏差超过 `mf_preselect_entry_interrupt_max_offset_px`，流程继续横移扫线，等目标进入窗口后再停车进入 KFS 对齐；KFS 对齐超时若最后有效目标仍在 `mf_preselect_kfs_align_timeout_pickup_tolerance_px` 内且有深度，则不放弃目标，直接进入前向 odom 趋近和夹取链。入口来源的对齐失败不再把该 KFS 加入 ignored 列表，后续重新进入窗口时仍可再次尝试。

2026-06-30 同步：修正 `MfPreselectionFlow::tickDetection()` 的未命中窗口语义。入口检测、行前方检测、周身扫描和旁列 `TransitionObserve` 在没有 R2 KFS 时会先停满 `mf_preselect_entry_detect_timeout_s` 或 `mf_preselect_scan_detect_timeout_s`，再用 `mf_preselect_detect_lost_stable_frames` 做稳定丢失确认；不会再因为 RealSense 连续几帧无目标而把 2 秒观察窗口提前缩短到约半秒。看到 R2 或假 KFS 的命中分支仍保持即时响应。

2026-06-30 同步：`MfPreselectionFlow` 的 KFS 识别横移对齐已收口到 `rc26_vision::tip_alignment` 口径。KFS 不再按最高置信度或夹爪偏置线选框，而是在经过 `T_*` 标签、ignored target 和深度窗口过滤后，按识别框中心距离图像中心线最近获取锁定目标；锁定窗口内继续跟踪同一物理 KFS。旧的目标线偏置、释放滞回和 no-progress 快速失败参数已删除，横移失败不再由 no-progress 触发，只保留目标丢失等待和 `mf_preselect_kfs_align_timeout_s` 总超时兜底。

2026-06-30 同步：KFS 横移速度直接复用 `computeTipAlignmentVy()`，`mf_preselect_kfs_align_kp`、`mf_preselect_kfs_align_min_speed_mps`、`mf_preselect_kfs_align_max_speed_mps` 和 `mf_preselect_kfs_invert_lateral_direction` 是横移调参入口；当前配置按 `mf_preselect_kfs_invert_lateral_direction=false` 维护。横移阶段捕获当前 odom yaw 作为 `tip_alignment` 的 heading target，`mf_preselect_kfs_odom_yaw_tolerance_deg` 作为 yaw 进入容差，新增 `mf_preselect_kfs_align_heading_gate_deg` 控制 yaw gate；odom 不新鲜时停车等待，yaw 超 gate 时暂停横移只修正朝向。

2026-06-30 同步：KFS 前向趋近仍使用锁定深度规划 X 轴距离，并由 `mf_preselect_kfs_odom_xy_kp`、`mf_preselect_kfs_approach_odom_tolerance_m`、`mf_preselect_kfs_odom_yaw_tolerance_deg`、`mf_preselect_kfs_odom_stable_ticks`、`mf_preselect_kfs_approach_speed_mps` 和 `mf_preselect_kfs_approach_min_speed_mps` 控制 odom 闭环执行；趋近闭环只执行锁定时规划出的距离，不读取实时深度连续停车。

2026-06-30 同步：`MfPreselectionFlow` 的入口区 R2 KFS 发现仍要求有效深度，但入口深度窗口已经从通用窗口拆出独立配置。入口 2 号、入口 1/3 号定点检测、入口横移中断检测，以及这些入口目标后续横移复核使用 `mf_preselect_entry_depth_min_m/max_m`；梅林内部行检测、旁列 `TransitionObserve` 和直出补夹继续使用 `mf_preselect_depth_min_m/max_m`。两套窗口只影响 R2 KFS 锁定和 odom 闭环趋近前的新帧复核，趋近阶段仍使用锁定深度规划目标距离，不按实时深度连续停车。

2026-06-30 同步：`MfPreselectionFlow` 的假 KFS 避障分支改为高度表驱动的旁列前向观察推进。初始避障从中列绕到旁列时先通过 `prepareTransitionTo()` 读取静态高度差，不再硬编码 `StairMode::Climb`；后续 `FakeAvoidAlignExit` 不再设置 `direct_exit_mode_` 或进入 `DirectExitDrive`，而是进入旁列模式，按当前旁列固定向前格复用 `TransitionTurn -> TransitionArmAdjust -> TransitionObserve -> TransitionStair -> GridCenterAlign`；出口行 `grid10/grid12` 复用最终下阶离场。从 `RowFront` 发现假 KFS 切入避障时会显式结束当前检测窗口，避免旁列 `TransitionObserve` 继承旧 `RowFront` 状态后误入周身扫描。旁列前向推进的观察朝向与台阶执行朝向分开：先面向目标格正前方观察/夹取 R2 KFS；若该边按高度表是下阶，再在 `TransitionStair` 前转到后轮先下的台阶 yaw。该改动只调整预选赛策略分支，不改变 `/cmd_vel`、机构 service、视觉标签或台阶原语接口。

2026-06-30 同步：`MfPreselectionFlow` 的默认机构协议 ID 改为引用 `rc26_serial::CommandID/FeedbackID`，其中入口高侧 KFS 夹取对应 `ENTRY_GRAB_KFS_UP(0x0F)` / `ENTRY_GRAB_KFS_UP_DONE(0x0B)`。决策层继续只消费 `/mechanism/send_command` 与 `/mechanism/command_feedback`，不解析串口帧。

2026-06-30 同步：移除旧外部 action 位姿导航链，导航权威收敛到 `rc26_decision` 内部 odom 单轴分段闭环。新增并注册 `OdomDriveX`、`OdomDriveY`、`OdomTurnToYaw`，保留 `RelativeYawTarget`；MC 改回原始 `+X 0.2m -> 右转 90° -> -X 0.6m`，MF 预选入口改回 `+X 2.0m -> -Y 1.8m`。`relative_nav_last_*` 是当前导航观测黑板键，旧 action result/recovery 语义不再维护。
