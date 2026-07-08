# rc26_decision

## 模块定位

`rc26_decision` 是 R2 的主决策包，采用 BehaviorTree.CPP 组织比赛流程。当前它也是完整导航链中的运动命令发布权威：导航、视觉伺服、台阶和 MF 格间动作都在行为树内串行执行，并通过 `/cmd_vel` 输出给 `rc26_mcu_transport`。

## 当前实现

- 构建产物：`rc26_decision_nodes`、`decision_node`
- 运行入口：不提供独立 launch；完整运行由 [bringup.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/bringup.launch.py) 装配
- 行为树目录：[behavior_trees](/home/potato/RC_2026/src/rc26_decision/behavior_trees)
- 当前导航实现：[bt_odom_relative_nav.cpp](/home/potato/RC_2026/src/rc26_decision/src/navigation/bt_odom_relative_nav.cpp)
- 运行参数真源：由 [r2_active_side.yaml](/home/potato/RC_2026/src/rc26_bringup/config/r2_active_side.yaml) 选择 [r2_red.yaml](/home/potato/RC_2026/src/rc26_bringup/config/r2_red.yaml) 或 [r2_blue.yaml](/home/potato/RC_2026/src/rc26_bringup/config/r2_blue.yaml)

`decision_node` 启动时会加载 `r2_runtime.decision.ros__parameters`，把共享参数写入 blackboard，并注册 MC、MF、MF 预选赛、台阶和 odom 导航节点。`team` 是当前红蓝方场地镜像参数：`red` 使用 YAML 中维护的红方基准路线，`blue` 在启动加载参数时自动派生侧向 Y 和 yaw 的镜像值；非法值会告警并按 `red` 运行。完整导航链中默认启用启动 odom gate：只有 `/odom` 连续新鲜且低速稳定后才创建并 tick 行为树，避免开机里程计未稳定时直接运动。

## Odom 相对闭环导航

当前通用导航原语有四类动作：

- `OdomDriveX`：进入动作时捕获当前 `/odom` 位姿和 yaw，把 `distance_m` 解释为启动车体系 X 轴相对距离；后续只闭环该轴向进度，只发布 `cmd_vel.linear.x`，并用 `angular.z` 保持启动 yaw。
- `OdomDriveY`：进入动作时捕获当前 `/odom` 位姿和 yaw，把 `distance_m` 解释为启动车体系 Y 轴相对距离；后续只闭环该轴向进度，只发布 `cmd_vel.linear.y`，并用 `angular.z` 保持启动 yaw。
- `OdomTurnToYaw`：把 `target_yaw_rad` 解释为绝对 odom yaw，只发布 `cmd_vel.angular.z`，到达 yaw 容差并稳定后成功。
- `OdomDriveXTurnX`：进入动作时捕获当前 `/odom` 位姿和 yaw，把 `first_x_m -> yaw_delta_rad -> second_x_m` 解释为旧串行 `X -> yaw -> X` 路线的最终位姿；运行期间跟踪进入动作起点到该终点的 odom 最短线段，并按线段进度平滑插值目标 yaw，合成发布 `cmd_vel.linear.x/y` 与 `cmd_vel.angular.z`，用于麦克纳姆底盘边旋转边移动的复合导航段。

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

`OdomDriveXTurnX` 端口：

- `first_x_m`
- `yaw_delta_rad`
- `second_x_m`
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
- `target_yaw_rad`

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

- `mc_repeat_preselection_tree.xml`：first managed 默认入口，只执行 MC-only 可重复流程，不进入 `MFPreselectionAfterMCTree`。每轮先由入口 gate 只接受人工触发外部限位 1 `FRONT_LIMIT_SWITCH_TRIGGERED(0x06)`，下发 `COMPETITION_START(0x10)` 并等待同 `seq` 的 `COMPETITION_START_DONE(0x0C)`，随后按 `preselection_entry_continue_delay_msec` 延时进入 `MCAreaTree`；`MCAreaTree` 末尾 gate 只接受人工触发外部限位 2 `MF_PRESELECTION_TRIGGER(0x10)`，再执行同一组 `0x10/0x0C` 握手作为舵机重新放下动作，握手完成后不切树。`MCPreselectionRepeatControl` 默认开启，初始 MC 后最多重复 1 次，默认共 2 轮，并把 `mc_preselection_effective_forward_x_m` 写为红方 `+0.2/+0.4m` 或蓝方 `-0.2/-0.4m`。
- `mc_mf_preselection_tree.xml`：历史 MC+MF 组合入口保留为兼容/调试树，不再是 `preselection_mode: first` 默认树。旧树仍按人工触发外部限位 1 `0x06 -> MC -> MFPreselectionAfterMCTree` 或人工触发外部限位 2 `0x10 -> mf_preselection_tree.xml` 的分支语义运行。
- `mc_tree.xml`：`OdomDriveXTurnX(mc_preselection_effective_forward_x_m, mc_nav_right_turn_delta_rad=team 派生侧向 yaw, mc_nav_reverse_x_m) -> VisualServoGrab -> RotateRetreat(retreat_x_m=-0.4, retreat_y_m=-0.4) -> WaitPreselectionBranchGate`。MC 入口复合动作仍按旧 `X -> yaw -> X` 串行路线计算最终位姿，但运行时跟踪进入动作起点到终点的 odom 最短线段，并同步发布 `linear.x/y` 和 `angular.z`，减少前进、转向、后退之间的停车顿挫。`VisualServoGrab` 在进入视觉阶段后会保持复合动作输出的目标 odom yaw；无端头时按 team 派生低速横移搜寻，red 为车体系 `+Y`、blue 为车体系 `-Y`，直到端头出现；同屏多个端头时初次获锁优先选择画面左侧 bbox，随后在锁定窗口内持续跟踪同一物理端头。端头框短暂不稳定时，MC 视觉伺服沿用最近 offset 并按 `mc_align_lost_servo_speed_scale` 低速继续伺服，重新识别后用 `mc_align_offset_filter_alpha` 平滑 offset，再参与稳定帧和夹取触发。MC 末尾正式流程不再执行 `WaitForRegistrationConfirm`、5s 视觉配准等待或旋转前后 0.5s 延时；`RotateRetreat` 使用 MC 旋转角度/方向参数计算目标 yaw，并跟踪动作起点到按目标 yaw 车体系 `-X/-Y` 各 0.4m 退让终点的 odom 最短线段，再进入可由 XML/黑板配置的 branch gate。默认 first 重复树把该 gate 配置为只收 0x10 且不切树；旧组合树仍保持 0x06/0x10 分支。复合动作输出的目标 odom yaw 会传给 `VisualServoGrab target_yaw_rad`，作为视觉阶段 heading hold 目标；`team=blue` 时侧向 yaw、无端头横移搜寻方向和后续旋转退让方向相对红方基准取反。
- `mf_preselection_tree.xml`：保持原独立调试入口不变，可选入口导航为 `OdomDriveX(mf_preselect_entry2_nav_segment1_x_m=+2.0m) -> OdomDriveY(mf_preselect_entry2_nav_segment1_y_m=team 派生横移 Y)`，随后进入 `MfPreselectionFlow`；该入口不在树前额外转向。
- `mf_preselection_after_mc_tree.xml`：MC 后置 MF 预选专用入口，可选入口导航为 `OdomDriveX(mc_to_mf_preselect_nav_segment1_x_m=-2.4m) -> RelativeYawTarget(mc_to_mf_preselect_nav_turn_delta_rad=team 派生右转 yaw) -> OdomTurnToYaw -> OdomDriveX(mc_to_mf_preselect_nav_segment2_x_m=+1.6m)`，随后进入 `MfPreselectionFlow`。`MfPreselectionFlow` 内部的入口 1/3 号横移、假 KFS 侧列绕行、出口 yaw、周身扫描 yaw 和第四行收尾 yaw 同样按 `team` 从红方基准派生。
- `preselection_ramp_forward_tree.xml`：second managed 入口的斜坡子树，按 `preselection_ramp_approach_x_m` 与 `preselection_ramp_climb_x_m` 连续执行两段 `OdomDriveX`，速度和超时由 `preselection_ramp_*` 参数控制，动作完成即停车。
- `second_preselection_combo_tree.xml`：second managed 组合入口，入口 `WaitPreselectionBranchGate` 同时等待人工触发外部限位 1/2 上行 `0x06/0x10`，两条分支都下发 `SECOND_PRESELECTION_START(0x11)` 并等待同 `seq` 的 `SECOND_PRESELECTION_START_DONE(0x0D)`。`0x06` 分支握手完成后继续执行 `PreselectionRampForwardTree` 两段斜坡前进，再进入内嵌 `SecondPreselectionTree`；`0x10` 分支握手完成后直接切到 `second_preselection_tree.xml`。两条路径都会通过黑板键跳过 `SecondPreselectionTree` 内重复 0x11；当前组合树不再执行斜坡后 90° 转向。
- `second_preselection_tree.xml`：第二个预选赛独立树。若不是从 branch gate 直达，会先执行 `SECOND_PRESELECTION_START(0x11)` 并等待同 `seq` 的 `SECOND_PRESELECTION_START_DONE(0x0D)`；随后进入 `SecondPreselectionKfsPickup`，启动视觉、odom、`/cmd_vel` 和 `/mechanism/send_command`/`/mechanism/command_feedback`。该节点按红方基准沿车体系 `+X` 低速搜索前方 KFS：任意识别标签属于 KFS 的最近有效目标都会停车进入视觉横移对齐和夹取，不再区分 R2/R1 或等待特定 KFS 类型。视觉对齐复用 `rc26_vision::tip_alignment` 的目标线偏置、目标锁定、像素容差、稳定帧、heading hold/yaw gate 和丢帧停车口径，并可在已有真实锁定深度约束下用 KFS 尺寸估距兜底。对齐完成后先发送 `second_preselect_pre_approach_lower_command_id=0x14`，service ACK 后等待同 `seq` 的 `second_preselect_pre_approach_lower_done_feedback_id=0x12`，再按 `second_preselect_pre_approach_lower_settle_s` 停车等待机械臂彻底放下；随后按 `max(0, locked_depth - second_preselect_kfs_grab_distance_m) * second_preselect_kfs_approach_x_sign` 规划车体系 X 轴 odom 前向趋近，保持进入趋近时 yaw。趋近完成后发送 `second_preselect_pickup_command_id=0x12`，service ACK 后必须等待同 `seq` 的 `second_preselect_pickup_done_feedback_id=0x11`，再用原目标 label 与 bbox IoU 做视觉消失验证，连续新帧消失达到阈值才算夹取成功。夹取确认后执行红方基准 `OdomDriveY(second_preselect_nav_y1_m=+0.7m) -> OdomDriveX(second_preselect_nav_x2_m=+4.5m)`，blue 启动时只镜像 Y 段。随后 `SecondPreselectionR1KfsPlaceAlign` 观察前方 `R1_KFS`，过滤低于 `second_preselect_r1_kfs_min_score` 的 `R1_KFS`；若同帧存在多个有效 R1KFS，优先选择 bbox 中心距离相机图像中线最近的目标，平局时再按深度更近、置信度更高排序；选中后复用同一套 KFS 视觉横移对齐、目标锁定、heading hold/yaw gate 和失败停车口径；对齐完成后执行 `OdomDriveX(second_preselect_place_forward_x_m=+0.8m)`，再发送 ACK-only 下行 `SECOND_PRESELECTION_PLACE_KFS(0x13)`。九宫格放置观察、动态 ROI 投影、中层选空位、`selected_lateral` 横移和放置后后退不再属于当前正式流程。可通过 `second_preselect_dynamic_roi_ui_enable` 打开本地 OpenCV 调试窗口；该窗口贯穿 KFS 搜索、视觉对齐、`0x14/0x12` 放下握手、odom 趋近、`0x12/0x11` 夹取握手、视觉消失验证和 R1KFS 放置对齐，持续叠加 KFS 检测框、当前阶段、锁定目标和目标线。窗口创建或渲染失败只告警并自动关闭 UI，不改变搜索、夹取、对齐或放置结果。搜索、对齐、放下握手、趋近、夹取完成反馈、夹取验证、R1KFS 放置对齐、odom、命令任一阶段失败或 halt 都会停车并写 `DecisionFailure`。`start_r2_auto.sh` 专用的上行 `feedback_id=0x13` 红蓝切换监听器属于 bringup 快捷入口能力，不改变这里的下行放置命令语义。
- 独立右转验证树：右转专用参数族已退役，树内固定验收路线为 `OdomDriveX(+0.4m) -> RelativeYawTarget(-90deg) -> OdomTurnToYaw -> OdomDriveX(-0.7m)`，速度、topic、容差和超时复用通用 odom 相对导航参数。
- `relative_segment_nav_tree.xml`：独立验收用单轴分段树，依次示例调用 `OdomDriveX`、`OdomDriveY`、`OdomTurnToYaw`。

旧外部 action 位姿导航节点、TF 采点节点、旧双点位姿测试树和相关 action helper 已删除。当前 MC/MF 入口坐标不再是绝对地图位姿；`mc_nav_forward_x_m` / `mc_nav_reverse_x_m` 是 MC 复合 `X-turn-X` 动作的两段 X 距离，`mf_preselect_entry2_nav_segment1_*` 与 `mc_to_mf_preselect_nav_*` 仍是按启动姿态和分段动作顺序标定的相对单轴段。YAML 中 yaw/Y 路线值按红方基准维护并由 `team` 镜像；first MC 重复流程保留红/蓝配置中已有的 `mc_nav_forward_x_m` 现场标定值，重复距离由独立的 `first_preselection_mc_repeat_base_forward_x_m` 和 `first_preselection_mc_repeat_forward_x_step_m` 经 `team` 派生后写入 `mc_preselection_effective_forward_x_m`。

## MC 链路

`VisualServoGrabAction` 内嵌相机采集和端头推理，在工作线程中执行目标锁定、无端头横移搜寻、横移 P 控制、短暂丢框预测伺服、odom yaw 姿态保持、限位前探和 `GRAB_TIP(0x01)` service 请求。它只通过 `/cmd_vel`、`/mechanism/send_command` 和 `/mechanism/command_feedback` 与下层交互，不直接打开串口。

`VisualServoGrabAction` 可通过 `target_yaw_rad` 输入端口接收当前 MC 导航右转后的绝对 odom yaw；没有传入时才使用 `mc_align_target_yaw_rad` 静态兜底。`RotateInPlaceAction` 订阅 `mc_odom_topic`，发布 `cmd_vel.angular.z`。未显式传入 `target_yaw_rad` 时按相对角度旋转；传入时按绝对 odom yaw 对齐。`RotateRetreatAction` 是 MC 末尾专用复合动作：进入时捕获当前 `/odom`，按 `mc_rotate_angle_deg` 与 team 派生后的 `mc_rotate_direction` 得到目标 yaw，并把 `retreat_x_m/retreat_y_m` 解释为目标 yaw 车体系位移；运行期间跟踪动作起点到退让终点的 odom 最短线段，并按线段进度同步插值 yaw，同一控制环内合成 `linear.x/y` 与 `angular.z`。

`WaitPreselectionBranchGate` 是 first/second managed 入口和 MC 末尾的分支 gate。节点只消费 `/mechanism/command_feedback` 上的人工触发外部限位 1 `FRONT_LIMIT_SWITCH_TRIGGERED(0x06)` 与人工触发外部限位 2 `MF_PRESELECTION_TRIGGER(0x10)`，不采集 MC 相机基准帧，也不依赖 MC 末尾视觉配准。XML 通过 `accepted_branch` 选择 `both`、`continue_only` 或 `switch_only`；不被当前 gate 接受的 0x06/0x10 会记录节流告警并忽略。`continue_start_profile` 和 `switch_start_profile` 决定分支握手使用 `mc` 还是 `second` profile：`mc` profile 下发 `COMPETITION_START(0x10)` 并等待同 `seq` 的 `COMPETITION_START_DONE(0x0C)`，`second` profile 下发 `SECOND_PRESELECTION_START(0x11)` 并等待同 `seq` 的 `SECOND_PRESELECTION_START_DONE(0x0D)`。gate 可通过 `continue_pre_command_delay_msec` / `switch_pre_command_delay_msec` 在收到对应人工触发限位后、下发启动命令前等待；0x06 分支握手成功后返回 `SUCCESS` 继续当前树；0x10 分支握手成功后若 `switch_tree_file` 为空则返回 `SUCCESS`，否则写入黑板切树请求。旧全局 0x10 监听链路已删除，0x10 只能在 branch gate 内按当前 XML gate 语义消费。

## MF 与台阶链路

MF 格间动作仍由 `PlanGridTransition -> GridTurn -> GridHeadingAlign -> GridTransition -> GridCenterAlign` 串行完成。`GridTurn` / `GridHeadingAlign` 服务 MF 格间 yaw 对齐和独立 heading 校准入口；它们不是通用分段导航转向节点，通用分段转向使用 `OdomTurnToYaw`。

`GridTransition` 负责选择上/下台阶动作、发布台阶直行速度、等待激光事件并提交 `current_grid`。`GridCenterAlign` 在台阶完成后依据 `mf_center_grid_step_m` 执行二维格中心归位。`MfPreselectionFlow` 内部继续负责入口探测、KFS 视觉锁定、横移视觉闭环、新视觉帧复核、机构命令、夹取视觉消失验证、入口/格间台阶和最终离场归位。梅林内部 R2 KFS 夹取成功后会先复用当前格 `GridCenterAlign` 归回梅林格中心，再继续原本的转向、格间台阶或直出动作；入口侧夹取仍走入口回 2 号入口或准备入场的既有逻辑。`MfPreselectionFlow` 现在会在入口回 2 号入口的相对横移段上扣除 MCU 半余弦停车尾巴和下发延迟对应的距离补偿，既覆盖“入口中途夹取成功后回 2 号入口”，也覆盖 1/3 号入口未被 KFS 打断时的固定回中线，避免回中线段在减速滑移后越过 2 号入口中心。KFS 识别入口会先在同一帧 R2 KFS 与 R1 阻挡 KFS 中按 bbox 多点 ROI 深度选出唯一最近候选；排序先比 `distance_m`，相同再比目标线 offset 绝对值，仍相同则比 score。R2 夹取链只允许这个最近候选进入 `rc26_vision::tip_alignment`，最近候选为 R1 或无有效深度时不会从其它 R2 框重新选择；R1 停车/等待也只在最近候选为 R1 时触发，后方或侧边但非最近的 R1 框不会单独停车。fake KFS 的 `F_` 避障逻辑不纳入这轮 R1/R2 最近 KFS 竞争。KFS 横移继续复用 `rc26_vision::tip_alignment` 的目标锁定、横移速度和 yaw gate 口径：目标线默认为图像中心线，并可通过 `mf_preselect_kfs_align_target_line_offset_px` 平移到 `图像中心线 + 偏置`；锁定后跟踪同一物理目标，像素 offset 以该目标线为 0；yaw 超出 `mf_preselect_kfs_align_heading_gate_deg` 时只修正朝向，odom 不新鲜时停车等待。入口横移探测中若最近 R2 KFS 还在入口中断窗口外侧，不立即停车夹取，而是继续扫线等待目标进入可夹取窗口；当前入口中断窗口以 `mf_preselect_entry_interrupt_max_offset_px` 为基础，并可按 MCU 半余弦减速模型、入口横移速度、锁定深度、相机 `fx` 回填值和延迟估计动态放大，补偿高速扫线切零速时的停车尾巴。入口横移中断后会按同一 MCU 减速模型持续发布零速等待一小段时间，再开始 KFS 视觉横移对齐；这套补偿只作用于入口横移中断，不改变梅林内部 KFS 对齐。像素误差和 yaw 误差同时进入容差后按新视觉帧累计 `mf_preselect_kfs_align_stable_frames`，稳定且当前横移链路有可用深度后由该深度减 `mf_preselect_kfs_grab_distance_m` 规划车体系 X 轴距离，并捕获当前 `/odom` 起点和 yaw 后闭环执行规划距离；横移跟踪可在单帧中心深度洞时继续使用 RGB bbox offset，真实深度优先从 bbox 内 `3x3` 多点 ROI 更新，仍失败时可在已有真实锁定深度约束下用 `350mm x 350mm` KFS 尺寸估距兜底。KFS 横移不再维护释放滞回或 no-progress 快速失败；横移对齐总超时时，如果最后有效锁定目标仍在 `mf_preselect_kfs_align_timeout_pickup_tolerance_px` 内且当前链路深度有效，则继续进入前向趋近和夹取，否则按原失败路线继续。前向趋近超时会使 `MfPreselectionFlow` 失败停车；成功、失败和 halt 都发布零速。假 KFS 避障不再硬编码为上阶：从中列绕到 1 号侧或 3 号侧旁列时，会先通过 `MerlinMapManager` 静态高度表计算高度差，再选择上/下台阶和机构预调。完成该横向格间动作并归位到 `grid1` 或 `grid3` 后，不再进入 `direct_exit` 直行兜底，而是立即按旁列下一格建立前向观察目标并复用 `TransitionObserve` 看正前方 R2 KFS；随后沿避障后的旁列继续前向推进：1 号侧旁列按 `grid1 -> grid4 -> grid7 -> grid10`，3 号侧旁列按 `grid3 -> grid6 -> grid9 -> grid12`。每个旁列格间转换前复用现有 `TransitionObserve` 正前方观察和机构预调逻辑，只看前方 R2 KFS；看到则按现有 KFS 夹取链处理，未看到则继续对应上/下阶梯，抵达出口行后复用最终下阶离场。

台阶动作进入跨阶直行前会先用 `stair_heading_*` 参数完成 yaw 对齐；直行期间只叠加小幅 heading hold，若 yaw 偏差再次超过 `stair_heading_gate_deg`，会暂停线速度并原地纠偏，且不推进当前激光事件或定时直行窗口。

`MfPreselectionFlow` 的入口、行前方、周身和 `TransitionObserve` 检测窗口在未发现最近 R2 KFS 时必须先等满对应 `mf_preselect_*_detect_timeout_s`，再按 `mf_preselect_detect_lost_stable_frames` 确认未命中并切到下一阶段；连续丢失帧只用于抗抖，不再提前截短检测窗口。发现最近 R2 KFS 或假 KFS 仍可在窗口内立即触发对应夹取/避障分支。KFS 横移对齐按 second 预选赛同口径处理识别框闪烁：短暂丢框且未达到 `mf_preselect_kfs_lost_stop_frames` 时沿用最近锁定 offset，并按 `mf_preselect_kfs_lost_servo_speed_scale` 缩放后低速继续伺服；重新锁定后用 `mf_preselect_kfs_align_offset_filter_alpha` 对 offset 做低通滤波，再参与稳定帧、速度计算和前向趋近规划。R2 KFS 夹取命令完成后若视觉验证超时且原目标仍可见，不再把该目标加入 ignored 列表：入口侧会按本次前向趋近规划距离反向后退到可重新识别位置，再重新进入 KFS 视觉对齐和夹取；梅林内部路径前方或台阶前观察夹取失败时，会先复用当前格 `GridCenterAlign` 归中，再回到原前方观察阶段继续夹取。除 2 号入口必须无限重试的夹取确认失败外，没有新视觉帧、目标已消失但未达到稳定消失帧、机构命令失败、odom 运动失败或归中失败仍沿原失败/硬失败语义处理。R1 阻挡标签新增 `R1_KFS` 兼容，但会用 `mf_preselect_r1_kfs_min_score` 做低置信度过滤，默认 0.50，低于阈值的 `R1_KFS` 不参与最近 R1/R2 KFS 判断。R2 KFS 候选若被过滤或最近 KFS 不是 R2，会把最近一次拒绝原因代码写入 `mf_preselect_r2_lock_reject_reason/detail/sequence` 黑板键，并在同一检测窗口内按原因去重打印中文 INFO 日志；检测 miss 日志会附带中文摘要，便于区分夹取数已满、视觉帧无效、标签不匹配、已忽略目标、深度采样失败、最近 KFS 不是 R2 或目标选择失败。最近 KFS 门控日志会打印类型、标签、深度、offset、depth source 和被屏蔽候选数。

台阶动作既可通过 `stair_climb_tree.xml` / `stair_descend_tree.xml` 独立加载测试，也可由 MF 状态机复用。它们通过 `/mechanism/send_command` 请求推杆动作，通过 `/mechanism/command_feedback` 等待对应反馈，通过 `/cmd_vel` 发布受限直行速度；跨阶前先完成 yaw 预对齐，跨阶直行时若 yaw 超出 gate 会停止线速度并只修正朝向。失败或 halt 时只发布零速，不做额外推杆补偿。

2026-07-06 同步：`MfPreselectionFlow` 新增 MF 内部下阶梯直冲模式。`mf_preselect_descend_direct_rush_enable=true` 时，所有 `StairMode::Descend` 仍先执行 yaw 预对齐：中列格间和假 KFS 旁列按目标边自然前进方向对齐，最终离场按 `entry_heading_yaw_` 对齐；yaw 对齐完成后进入 `DescendDirectRushTimedDrive`，按 `mf_preselect_descend_direct_rush_speed_mps` 的 x 正向速度定时冲下 `mf_preselect_descend_direct_rush_duration_s`。直冲阶段不走推杆下阶序列，也不等待 0x05/0x07 激光突变事件推进；台阶前视觉检测、KFS/假 KFS 分支、下阶后居中归位和最终 `FinalExitVirtual` 外推归位不被跳过。`false` 时完整回退当前 `edge_yaw + pi` 后轮先下、推杆 + 激光事件下阶实现；独立 `stair_descend_tree.xml` 和共享 `GridTransitionAction` 不受该 MF 专用开关影响。红/蓝运行配置当前默认启用，速度 `0.45m/s`、时长 `2.0s`。

2026-07-06 同步：红方运行配置 `r2_red.yaml` 已按新推杆设备首版保守口径缩短台阶推杆 accepted 后零速等待：前置推杆从 60mm/s 升级到 100mm/s，后置推杆从 100mm/s 升级到 180mm/s 后，`stair_climb_front_extend_delay_s=3.2`、`stair_climb_retract_rear_extend_delay_s=2.7`、`stair_climb_rear_retract_delay_s=3.1`、`stair_descend_rear_extend_delay_s=3.1`、`stair_descend_retract_front_extend_delay_s=2.7`、`stair_descend_front_retract_delay_s=1.8`。本次只调整运行配置中的推杆等待裕量，不改变台阶状态机、激光事件推进条件、底盘跨阶速度、机构 command/feedback 协议或 `/cmd_vel` 权威。

梅林预选赛到达 2 号入口后会先发送普通 `ARM_RAISE(0x04)` 并等待 `ARM_RAISE_DONE(0x02)`，确认机械臂进入普通高侧姿态后才启动入口 2 号视觉识别窗口。2 号入口正前方 R2 KFS 夹取使用 `rc26_serial` 真源中的普通高侧 `GRAB_KFS_UP(0x03)`；后续视觉横移对齐、锁定深度、odom 前向趋近和普通上夹取都建立在该普通高侧姿态上。夹取完成仍以视觉消失验证为准；若 2 号入口未完成视觉夹取验证，会持续停车、重新识别、重新对齐、重新趋近并重新夹取，直到原目标连续新帧消失确认成功，不会忽略该目标或继续上阶。入口 2 号夹取确认成功后会直接复用已完成的普通高侧姿态进入首阶台阶动作，不再重复下发 `ARM_RAISE(0x04)` 或再次等待 `ARM_RAISE_DONE(0x02)`。入口专用 `ENTRY_GRAB_KFS_UP(0x0F)` / `ENTRY_GRAB_KFS_UP_DONE(0x0B)` 仅保留给仍显式启用 `entry_high_protocol` 的入口高侧场景；ACK 只代表 transport 通用确认，真正计数仍延迟到后续视觉消失验证成功。

## 参数口径

导航相关参数由 `r2_active_side.yaml` 选择的 `r2_red.yaml` / `r2_blue.yaml` 提供：

- `odom_relative_nav_*`：单轴平移、复合 X-turn-X 和通用 yaw 容差、增益、速度、topic 和超时。
- `startup_odom_*`：完整导航链启动前 odom 新鲜度和低速稳定 gate。
- `team`：红蓝方场地镜像选择；`red` 使用红方基准，`blue` 自动镜像 MC/MF/第二预选赛侧向 Y 和 yaw，非法值按 `red`。
- `mc_nav_forward_x_m`、`mc_nav_right_turn_delta_rad`、`mc_nav_reverse_x_m`、`mc_nav_timeout_sec`：MC 去程复合 `OdomDriveXTurnX` 路线参数，继续保留红/蓝运行配置中的现场标定值。first 默认重复树不把 `mc_nav_forward_x_m` 当作重复距离 base；运行时实际传给 MC 的是 `mc_preselection_effective_forward_x_m`。`mc_nav_right_turn_delta_rad`、无端头搜寻方向和旋转退让方向继续按 `team` 镜像，`mc_nav_timeout_sec` 是复合动作总超时。
- `preselection_entry_continue_delay_msec`：first 每轮入口 gate 完成人工触发外部限位 1 的 `0x06 -> 0x10/0x0C` 握手后进入 MC 前的 `Delay`。参数侧按有符号整数读取并归零裁剪，写入 blackboard 时按 BehaviorTree.CPP `unsigned int` 端口类型保存。
- `first_preselection_mc_repeat_enable`、`first_preselection_mc_repeat_max_count`、`first_preselection_mc_repeat_base_forward_x_m`、`first_preselection_mc_repeat_forward_x_step_m`：first MC-only 重复控制参数。默认开启、初始 MC 后最多重复 1 次，默认共 2 轮；基准距离和步进都是绝对值，运行时按 `team` 派生方向，因此红方默认两轮为 `+0.2/+0.4m`，蓝方默认两轮为 `-0.2/-0.4m`。
- `preselection_ramp_approach_x_m`、`preselection_ramp_climb_x_m`、`preselection_ramp_max_speed_mps`、`preselection_ramp_min_speed_mps`、`preselection_ramp_timeout_s`：second managed 斜坡前进两段 `OdomDriveX` 参数。
- `second_preselect_after_ramp_turn_delta_rad`、`second_preselect_after_ramp_turn_timeout_s`：历史 second managed 斜坡后 90° 相对转向参数；当前默认 `second_preselection_combo_tree.xml` 不再使用，`0x06` 分支斜坡后直接进入 `SecondPreselectionTree` 搜寻，`0x10` 分支完成 `0x11/0x0D` 握手后直接切到 `second_preselection_tree.xml`。
- `mf_preselect_r1_kfs_min_score`：`R1_KFS` 低置信度过滤阈值，默认 `0.50`；只影响 R1 阻挡判断，不影响 R2 KFS 夹取目标和假 KFS 避障目标。
- `mf_preselect_kfs_depth_roi_size`、`mf_preselect_kfs_depth_min_valid_count`、`mf_preselect_kfs_depth_bbox_sample_ratios`、`mf_preselect_kfs_depth_bbox_min_success_count`：R2 KFS 深度有效点判定参数。单点 ROI 边长、ROI 内最少有效深度点、bbox 内横纵采样比例和 bbox 多点采样最少成功点数都由红/蓝运行配置提供；默认保持原 `7x7` ROI、每个 ROI 至少 `10` 个有效点、bbox 内 `0.25/0.50/0.75` 组成 `3x3` 采样点且至少 `1` 个点成功。
- `mf_preselect_entry2_nav_segment1_x_m`、`mf_preselect_entry2_nav_segment1_y_m`、`mf_preselect_entry2_nav_timeout_sec`：MF 预选独立入口红方基准单轴段，默认 `+X 2.0m -> -Y 1.8m`；蓝方只镜像 Y，X 距离不变。
- `mc_to_mf_preselect_nav_segment1_x_m`、`mc_to_mf_preselect_nav_turn_delta_rad`、`mc_to_mf_preselect_nav_segment2_x_m`、`mc_to_mf_preselect_nav_timeout_sec`：MC 后置 MF 预选组合树专用入口段，默认 `-X 2.4m -> 右转 90° -> +X 1.6m`；蓝方只镜像 yaw，X 距离不变。
- `second_preselect_*`：第二个预选赛独立树参数。命令参数默认 `0x11/0x0D` 开始握手、视觉对齐后 `0x14/0x12` 机械臂放下握手、`0x12/0x11` KFS 夹取触发与动作完成握手、下行 `0x13` ACK-only 放置。`0x14` service ACK 只表示 MCU 收到放下命令，`SecondPreselectionKfsPickup` 必须等同 `seq` 的 `SECOND_PRESELECTION_ARM_LOWER_DONE(0x12)` 并按 `second_preselect_pre_approach_lower_settle_s` 停车等待后，才允许 odom 前向趋近。后续 `0x12` 夹取触发 service ACK 只表示 MCU 收到命令，节点必须等同 `seq` 的 `SECOND_PRESELECTION_PICKUP_KFS_DONE(0x11)` 后，才进入原目标视觉消失验证。搜索/夹取参数包括 `second_preselect_cmd_vel_topic`、`second_preselect_search_*`、R2/R1 标签集合、深度窗口、`second_preselect_kfs_align_*`、`second_preselect_pre_approach_lower_*`、`second_preselect_kfs_approach_*`、`second_preselect_kfs_grab_distance_m`、bbox 深度采样、尺寸估距、`second_preselect_grab_verify_*` 和 `second_preselect_grab_settle_s`；这些参数按 MF 当前 KFS 夹取口径维护，但作用范围只在第二预选赛。导航参数按红方基准维护，新流程夹取成功后默认 `+Y 0.7m -> +X 4.5m` 到 R1KFS 放置对齐观察位，blue 启动时自动翻转 Y 段；随后 `SecondPreselectionR1KfsPlaceAlign` 观察并锁定前方 `R1_KFS`，使用 `second_preselect_r1_kfs_min_score` 过滤低置信度 `R1_KFS`，多目标时选择 bbox 中心距相机图像中线最近者，视觉横移对齐后由 `second_preselect_place_forward_x_m=0.8m -> 下行 0x13` 完成放置。第二预选赛不再读取九宫格动态 ROI、格子标签过滤、占据 mask、中层空位或放置后后退参数。`second_preselect_dynamic_roi_ui_enable` 控制第二预选赛本地 OpenCV 视觉调试窗口；开启后在 KFS 搜索/对齐/放下握手/趋近/夹取验证和 R1KFS 放置对齐阶段绘制识别框、锁定目标、目标线和阶段状态，`second_preselect_dynamic_roi_ui_window_name` 控制窗口名。上行 `feedback_id=0x13` 只由 `start_r2_auto.sh` 启动的监听器消费，用于写回 `r2_active_side.yaml`，不属于 decision 行为树内部切换。
参数在节点构造时声明并写入 blackboard；当前没有运行期参数变更回调，`ros2 param set` 不会自动回写已经进入树的参数。

## 边界

- `rc26_decision` 拥有比赛流程、行为树编排、导航段调用顺序和 `/cmd_vel` 发布时序。
- `rc26_decision` 不拥有串口协议解析、相机驱动、点云里程计、定位算法或 MCU transport。
- 完整导航链同一时刻只能有一个 `/cmd_vel` 发布权威；运行遥控、视觉动作测试、台阶独立测试或分段导航测试前必须停用其它运动发布者。
- `rc26_interfaces` 当前不提供自定义导航 action；导航对外契约只保留 `/cmd_vel` 速度输出。

## 本轮同步

2026-07-08 同步：first 默认入口改为 `mc_repeat_preselection_tree.xml`。新树只执行 MC 可重复流程，不再进入 `MFPreselectionAfterMCTree`；入口 gate 只接受人工触发外部限位 1 的 `0x06` 并执行 `0x10/0x0C` 后启动 MC，MC 末尾 gate 只接受人工触发外部限位 2 的 `0x10` 并执行 `0x10/0x0C` 作为舵机放下握手且不切树。新增 `MCPreselectionRepeatControl` 维护 `mc_preselection_effective_forward_x_m`，默认初始 MC 后最多重复 1 次，默认共 2 轮；红方为 `+0.2/+0.4m`，蓝方为 `-0.2/-0.4m`。该 repeat 距离使用 `r2_active_side.yaml` 中的独立参数计算，不改写红/蓝配置中已有的 `mc_nav_forward_x_m` 现场标定值，也不在红/蓝运行配置中重复声明默认 repeat 参数。旧 `mc_mf_preselection_tree.xml` 和 `mf_preselection_after_mc_tree.xml` 保留为兼容/调试入口。

2026-07-05 同步：第二预选赛放置链删除九宫格动态 ROI 观察、中层选空位、`selected_lateral` 横移和放置后后退。`second_preselection_tree.xml` 现在在夹取验证成功后执行 `+Y 0.7m`（blue 镜像）和 `+X 4.5m`，然后 `SecondPreselectionR1KfsPlaceAlign` 观察前方 `R1_KFS` 并视觉横移对齐；若同帧存在多个有效 R1KFS，则选择 bbox 中心距离相机图像中线最近者，平局再比较深度和置信度。对齐后 `+X 0.8m` 前进，最后发送 ACK-only `SECOND_PRESELECTION_PLACE_KFS(0x13)`。

2026-07-05 同步：第二预选赛 KFS 搜索夹取链在视觉横移对齐稳定后新增机械臂彻底放下握手。`SecondPreselectionKfsPickup` 会先发送 `SECOND_PRESELECTION_ARM_LOWER(0x14)`，等待同 `seq` 的 `SECOND_PRESELECTION_ARM_LOWER_DONE(0x12)`，再按 `second_preselect_pre_approach_lower_settle_s` 停车等待，之后才启动原有 KFS odom 前向趋近。原趋近后的 `SECOND_PRESELECTION_ARM_HIGH_RAISE/KFS_PICKUP(0x12)` 与 `SECOND_PRESELECTION_PICKUP_KFS_DONE(0x11)` 夹取完成握手、视觉消失验证和后续放置路线保持不变。

2026-07-09 同步：second 默认组合树对齐 `mc_mf_preselection_tree.xml` 的入口分支模型。`second_preselection_combo_tree.xml` 先由入口 `WaitPreselectionBranchGate` 同时等待上行 `0x06/0x10`，两条分支都执行 `SECOND_PRESELECTION_START(0x11)` / `SECOND_PRESELECTION_START_DONE(0x0D)` 握手；`0x06` 分支继续斜坡并进入内嵌 `SecondPreselectionTree` 搜寻，`0x10` 分支直接请求切到 `second_preselection_tree.xml`。组合树删除斜坡后 `RelativeYawTarget + OdomTurnToYaw`；gate 对 second profile 的 continue/switch 分支都会写入 second-start-done 黑板键，避免进入独立树时重复发送 `0x11`。

2026-07-09 同步：删除决策节点旧全局 0x10 监听链路、对应参数和 helper。人工触发外部限位 2 的上行 `0x10` 不再能绕过行为树直接触发独立树；它只在当前 XML 中显式放置的 `WaitPreselectionBranchGate` 内生效，并按 gate profile 完成握手、继续或切树。

2026-07-04 同步：第二预选赛 `SecondPreselectionTree` 内部流程改为搜索夹取链。树内新增 `SecondPreselectionKfsPickup`，在 `0x11` start-once 后沿 `+X` 搜索最近有效 KFS，任意识别标签属于 KFS 的目标都会按 MF 口径做视觉横移对齐、odom 前向趋近、发送 `0x12` 夹取触发，等待同 `seq` 的 MCU 上行 `0x11` 夹取完成反馈后，再用原目标视觉消失验证夹取成功，不再因 R1/R2 类型差异继续搜索。旧固定第一段、观察前高抬节点、对应兼容参数及放置前 KFS 必见 gate 已删除；夹取后的放置链以 2026-07-05 当前口径为准。

2026-07-04 同步：第二预选赛本地 OpenCV UI overlay 扩展为全流程视觉调试窗口。`second_preselect_dynamic_roi_ui_enable` 开启后，`SecondPreselectionKfsPickup` 会在 KFS 搜索、视觉对齐、odom 趋近、等待 `0x11` 夹取完成反馈和视觉消失验证期间持续显示彩色帧、识别框、锁定目标、目标线和阶段状态；放置前 UI 以 2026-07-05 R1KFS 放置对齐口径为准。该 UI 只用于现场调试观察，不发布 topic、不改变行为树返回值、不新增 `/cmd_vel` 权威；窗口创建或渲染失败时仅告警并自动关闭 UI。

2026-07-08 同步：`OdomDriveXTurnX` 与 `RotateRetreat` 从直接闭环最终平面目标改为直线轨迹跟踪。两者仍在进入动作时捕获当前 odom 并按既有参数计算最终位姿，但运行期间先把当前 odom 投影到起终点线段上，按单调进度选择线段前视参考点，平移控制跟踪该参考点，yaw 按线段进度平滑插值到目标 yaw；成功条件、topic、端口、红蓝镜像参数和 `/cmd_vel` 权威不变。

2026-07-08 同步：`mc_tree.xml` 将 MC 末尾 `RotateInPlace -> Delay -> OdomDriveX(-0.4) -> OdomDriveY(-0.4)` 合并为 `RotateRetreat`。该动作不保留旋转前后 0.5s 延时，进入时捕获当前 odom，按 MC 旋转参数计算目标 yaw，并把按目标 yaw 车体系 `-X/-Y` 各 0.4m 得到的退让终点作为直线轨迹终点；仍只通过 `/cmd_vel` 输出，不新增运动命令权威、ROS topic/service/action 或机构协议。

2026-07-04 同步：`MfPreselectionFlow` 修正入口 2 号 R2 KFS 夹取成功后的上首阶准备逻辑。入口启动时已经完成普通 `ARM_RAISE(0x04)` / `ARM_RAISE_DONE(0x02)` 并记录机械臂处于高侧姿态；夹取视觉消失验证成功后，`EntryPrepareClimb` 会直接进入 `EntryClimb`，不再重复下发普通 `ARM_RAISE`，避免机构对重复抬臂不回完成反馈时卡在首阶前。

2026-07-03 同步：`MfPreselectionFlow` 的 2 号入口 R2 KFS 夹取改用普通高侧 `GRAB_KFS_UP(0x03)`，不再走入口专用 `ENTRY_GRAB_KFS_UP(0x0F)`；到达 2 号入口后必须先完成普通 `ARM_RAISE(0x04)` / `ARM_RAISE_DONE(0x02)`，再启动 Entry2 视觉识别、视觉横移对齐、odom 前向趋近和夹取。2 号入口夹取完成仍以视觉消失验证为准，未完成时会无限重试夹取链，不忽略该目标且不推进到上阶或 1/3 号入口探测。入口专用 `ENTRY_GRAB_KFS_UP(0x0F)` / `ENTRY_GRAB_KFS_UP_DONE(0x0B)` 仅保留给显式 `entry_high_protocol` 场景。

2026-07-03 同步：台阶动作改为先 yaw 对齐再跨阶直行。独立 `StairClimb` / `StairDescend` 复用既有 `stair_heading_*` 参数，在开始推杆和直行前先完成 yaw 预对齐；`MfPreselectionFlow` 内嵌入口、格间和最终离场台阶也使用同一口径。跨阶直行期间只保留小幅 heading hold，若 yaw 偏差超过 `stair_heading_gate_deg`，会暂停线速度并原地纠偏，且不推进当前激光事件等待或定时直行窗口。本轮不改变 `/cmd_vel`、机构 service、MCU 反馈协议、`OdomDriveX/Y`、`GridCenterAlign` 或 KFS odom 前向趋近控制策略。

2026-07-03 同步：修复 first managed 行为树启动时 `Delay.delay_msec` 端口类型冲突。`preselection_entry_continue_delay_msec` 与兼容旧组合树的 `preselection_after_mc_continue_delay_msec` 仍从 ROS 参数按 `int` 读取并裁剪为非负值，但写入 blackboard 时统一转换为 `unsigned int`，与 BehaviorTree.CPP 内置 `Delay` 节点端口类型一致，避免创建 `mc_mf_preselection_tree.xml` 时因 `int` / `unsigned int` 混用导致 `BT::RuntimeError`。

2026-07-03 同步：新增 `WaitPreselectionBranchGate` 并以 first/second managed 入口替代历史启动 XML。当时旧 first 组合树语义为武馆+梅林完整人工触发外部限位 1 分支 `0x06 -> 下行 0x10 -> 上行 0x0C -> MC -> MC 后置 MF`，以及人工触发外部限位 2 分支 `0x10 -> 下行 0x10 -> 上行 0x0C -> mf_preselection_tree.xml`；当前 first 默认入口已改为 2026-07-08 的 MC-only 重复树，second 默认组合树已改为 2026-07-09 的 `0x06 -> 0x11/0x0D -> 斜坡 -> SecondPreselectionTree` 与 `0x10 -> 0x11/0x0D -> second_preselection_tree.xml`。`mc_tree.xml` 末尾删除 `WaitForRegistrationConfirm` 和 5s 视觉配准等待，旋转后改为同一 branch gate。新增 `preselection_ramp_forward_tree.xml` 与 `second_preselection_combo_tree.xml`，历史 `mf_preselection_start_tree.xml` 已删除。

2026-07-04 同步：`WaitPreselectionBranchGate` 新增 `continue_pre_command_delay_msec` / `switch_pre_command_delay_msec` 端口，用于在收到人工触发外部限位 1/2 的 0x06/0x10 分支反馈后、下发对应启动命令前等待。旧 `mc_mf_preselection_tree.xml` 的 `mc_tree.xml` 末尾 0x06 分支将 `continue_pre_command_delay_msec` 绑定到 `preselection_after_mc_continue_delay_msec`，所以旧组合树可在人工触发外部限位 1 触发后先等 5s，再下发 `COMPETITION_START(0x10)`；当前 first 默认重复树把 MC 末尾 gate 配置为只收 0x10 且不切树。

2026-07-03 同步：第二预选赛标签过滤参数的默认空列表继续由 `SecondPreselectionParams` 和 `declare_parameter<std::vector<std::string>>` 提供；红/蓝运行配置省略 `second_preselect_grid_label_prefixes` 与 `second_preselect_grid_label_exact_names`，不再显式写 `[]`。当前行为不变，仍表示所有非空 `class_name` 有效，同时避免完整 bringup 延时创建 `decision_node` 时被 ROS2 launch 的空数组参数类型推断拦截。

2026-07-03 同步：`MfPreselectionFlow` 的 R2 KFS 深度有效点判定改为可配置。`mf_preselect_kfs_depth_roi_size` 和 `mf_preselect_kfs_depth_min_valid_count` 控制单个深度 ROI 的有效点口径，`mf_preselect_kfs_depth_bbox_sample_ratios` 和 `mf_preselect_kfs_depth_bbox_min_success_count` 控制 bbox 内多点采样位置和最少成功点数；红/蓝运行配置和历史兼容配置已写入默认值，保持现有行为但允许现场在 `rc26_bringup/config` 中调松或调严。

2026-07-08 同步：统一 MCU 上行外部限位语义：0x06、0x10、0x13 都来自人工触发的外部限位开关。`rc26_decision` 当前仍只在 managed branch gate 内消费人工触发外部限位 1/2 的上行 0x06/0x10；脚本专用人工触发外部限位 3 的上行 0x13 保持由 `start_r2_auto.sh` 监听并只影响下一次启动的红蓝配置，decision 不新增 0x13 分支。

2026-07-02 同步：MCU 上行 `MF_PRESELECTION_TRIGGER(0x10)` 定义为人工触发外部限位 2 事件；当前 managed first/second 入口下，0x10 由 `WaitPreselectionBranchGate` 消费，并按当前 XML profile 完成 0x10/0x0C 或 0x11/0x0D 握手后再继续或切树。

2026-07-02 同步：新增第二个预选赛独立树 `second_preselection_tree.xml`。该树通过 `/mechanism/send_command` 下发 `0x11` 并等待同 `seq` 的 `0x0D` 后启动路线，按车体系 `+X 1.8m -> +Y 1.2m -> +X 2.5m` 到达观察位，再下发 `0x12` 并等待同 `seq` 的 `0x0F` 后用深度相机视觉快照判断九宫格中层是否被占据；被占据时依次尝试 `+Y 0.2m` 与 `-Y 0.6m` 两个观察点，任一为空则 `+X 0.7m`、发送 ACK-only `0x13` 放置 KFS 并 `-X 0.7m` 后退。本轮只新增独立树和 `second_preselect_*` 参数，不修改红/蓝默认行为树入口，也不新增 `/cmd_vel` 权威或高层 action。

2026-07-03 同步：第二预选赛从固定单 ROI/三次横移搜索升级为动态 3x3 ROI 观察与中层选位。`SecondPreselectionObserve` 现在读取 `FrameSnapshot::detections` 与观察期 odom delta，按 Color 640x480 默认内参和九宫格实地模型投影 9 个格子的 ROI；命中规则是检测框中心点落入动态 ROI 且 `class_name` 非空，标签前缀/精确名过滤为空时不过滤。九宫格列宽按红方图纸实际尺寸维护：左列 `0.54m`、中列 `0.58m`、右列 `0.50m`，行距 `0.54m`；动态 ROI 投影和 `OdomDriveY` 横移量都使用这些列中心位置，不再假设三列等距。每帧会形成 `second_preselection_grid_occupied_mask`，本轮只在中层三格中按中列、场地可移动空间侧、另一侧选空位；场地可移动空间侧在 red 为车体 `+Y`，在 blue 为车体 `-Y`。输出的 `second_preselection_selected_lateral_m` 已按 `team_mirror_sign` 派生，默认 red 场地可移动空间侧约 `+0.56m`、blue 约 `-0.56m`。红/蓝 HSV 与占据面积参数已从红蓝运行配置和加载逻辑中删除，格子占据不再按颜色区分，但第二预选赛导航和几何符号仍按红方基准由 `team` 镜像。

2026-07-02 同步：MC 末尾视觉 gate 曾从红色 HSV 检测改为背景配准确认；当前正式 first managed XML 已删除该视觉配准 gate，源码中的旧动作仅保留为历史实现，不再被正式 XML、运行配置或文档入口依赖。

2026-07-02 同步：组合树启动 gate 增加 `COMPETITION_START_DONE(0x0C)` 等待，`COMPETITION_START(0x10)` 的通用 ACK 只表示 MCU 收到命令，只有同 `seq` 上行 `0x0C` 到达后才延时 500ms 进入 MC。默认运行配置拆分为 `r2_red.yaml` / `r2_blue.yaml`，由 `r2_active_side.yaml` 选择当前比赛方；显式传入 `runtime_config_file` 仍可覆盖。

2026-07-01 同步：新增 `mc_mf_preselection_tree.xml` 作为默认 MC + MF 预选组合入口，并新增 `mf_preselection_after_mc_tree.xml` 承载 MC 后置 MF 预选入口导航；当前组合树启动与 MC 末尾切换都已收敛到 `WaitPreselectionBranchGate`。后置入口导航使用 `mc_to_mf_preselect_nav_*` 参数，路线为 `OdomDriveX(-2.4m) -> RelativeYawTarget(team 派生 -90deg) -> OdomTurnToYaw -> OdomDriveX(+1.6m)`；该改动只调整行为树编排、启动通知和 odom 单轴段参数，不新增 `/cmd_vel` 权威或 ROS topic/service/action。

2026-07-01 同步：右转导航独立入口专用参数族已退役，`decision_node` 启动时不再声明、读取或写入对应 blackboard 键。通用 `OdomDriveX`、`OdomDriveY`、`OdomTurnToYaw` 和 `RelativeYawTarget` 动作继续保留，供 MC/MF 和其它行为树复用；独立右转验证树若加载运行，则使用树内固定路线和通用 odom 相对导航参数。

2026-07-01 同步：`team` 参数扩展为决策层红蓝方场地镜像契约。红/蓝运行配置继续维护红方基准路线；`decision_node` 启动时规范化 `team`，写入 `team_mirror_sign`，并用同一镜像符号派生 MC 侧向 yaw、MC 原地旋转方向、MF 预选入口 Y、MF 入口 1/3 号横移、假 KFS 侧列绕行、出口 yaw、周身扫描 yaw 和第四行收尾 yaw。`team=blue` 只改变场地几何方向，不新增 XML、topic、service、action、MCU 协议或视觉参数；非法 `team` 会告警并按 `red` 运行。

2026-07-01 同步：`MfPreselectionFlow` 修复 KFS 横移阶段中心深度洞导致目标丢失的问题。R2 初始检测仍必须有真实深度，但深度采样从单个中心 `7x7` ROI 扩展为 bbox 内 `3x3` 多点 ROI；横移对齐阶段即使当前帧没有可用深度，只要 RGB bbox 仍能被 `tip_alignment` 锁定，就继续按 offset 横移，只有在像素和 yaw 已对齐且当前链路有可用深度时才累计稳定并进入前向 odom 趋近。新增 `mf_preselect_kfs_mono_distance_fallback_enable`、`mf_preselect_kfs_mono_target_width_m/height_m`、`mf_preselect_kfs_mono_fx_px/fy_px`、`mf_preselect_kfs_mono_min_bbox_px` 和 `mf_preselect_kfs_mono_max_delta_from_locked_m`，用于在真实深度洞持续存在时按 `350mm x 350mm` KFS bbox 尺寸做保守估距；尺寸估距只在已有真实锁定深度后启用，并且必须落在当前深度窗口内、且与最近真实深度差值不超过配置阈值。横移、稳定计数和前向趋近日志会打印中文 `depth_source=中心ROI/bbox多点ROI/尺寸估距/无` 及估距详情；不新增 ROS topic/service/action，不改变机构协议、ignored 目标、台阶流程或 `/cmd_vel` 权威。

2026-07-01 同步：`MfPreselectionFlow` 新增 `mf_preselect_kfs_align_target_line_offset_px`，用于标定 KFS 识别框中线要对齐的目标线。目标线默认为图像中心线，偏置为负值时向图像左侧移动；offset、稳定计数、速度计算和超时补夹窗口都以 `图像中心线 + 偏置` 为 0 点。该改动只影响 MF KFS 横移对齐目标线，不改变标签、深度窗口、ignored 目标、台阶状态机或 `/cmd_vel` 权威。

2026-07-01 同步：`MfPreselectionFlow` 的梅林内部 R2 KFS 夹取成功后新增夹取后格中心归位。物理夹取通过视觉消失验证并完成 `grab_settle` 后，若本次夹取来源不是入口侧且未使用入口高侧协议，会先按当前 `current_grid` 和现有 `mf_center_reference_*` 执行 `post_grab_center` 归位，再恢复原来的 `AfterEntry`、`TransitionStair` 或 `DirectExitDrive` 等成功阶段；直出途中补夹会重新捕获直出相对移动起点。该改动只调整夹取成功后的运动顺序，不新增视觉接口、机构命令、运行参数或 `/cmd_vel` 发布权威。

2026-07-01 同步：`MfPreselectionFlow` 的假 KFS 横向避障落到 1 号侧或 3 号侧旁列后，会立即通过 `startFakeAvoidForwardObservation()` 选择旁列下一格并复用现有 `TransitionArmAdjust -> TransitionObserve` 正前方观察链路；观察到 R2 KFS 时沿现有夹取链处理，未观察到时继续原旁列格间上/下阶梯推进。该改动只调整避障后旁列观察调度和日志，不新增视觉接口、机构命令或 `/cmd_vel` 发布权威。

2026-07-01 同步：`MfPreselectionFlow::findR2LockObservation()` 新增 R2 KFS 候选拒绝诊断。行前方、周身和 `TransitionObserve` 检测中若画面未形成可夹取目标，会记录 `pickup_limit_reached`、`vision_not_running`、`snapshot_invalid`、`no_label_match`、`ignored_target`、`depth_invalid` 或 `selection_failed`，同步写入 `mf_preselect_r2_lock_reject_reason/detail/sequence` 黑板键，并把最近拒绝中文摘要追加到入口、行前方、周身和台阶观察的“未发现”日志；黑板中的 reason 保持英文代码，现场 INFO 日志使用中文原因和中文详情。`depth_invalid` 详情会按 bbox 内 `3x3` 多点 ROI 与单点 `7x7` 最小有效点口径输出采样点、深度类型、深度尺寸、窗口内有效点、原始深度中位数和 `ROI无有效原始深度 / 有效深度低于窗口 / 有效深度高于窗口 / 窗口内有效点不足` 等主因，帮助区分深度空洞、采到背景和窗口配置问题。KFS 横移对齐丢帧和超时日志也会附带最近 R2 候选拒绝摘要。该改动只增强现场排查信息，不改变标签、深度窗口、目标选择、夹取次数、台阶或 `/cmd_vel` 行为。

2026-07-01 同步：针对实车出现的 `snapshot_invalid` 且详情为“有显示图=是、有彩色图=否、有深度图=是”的情况，根因修正在 `rc26_vision::VisionInferenceManager::getLatestFrameSnapshot()`：推理线程消费 `latest_color_` 后，快照 API 会用与 detections 同源的 display 帧回填 `color_bgr`。`MfPreselectionFlow` 仍要求 color/depth/display/sequence 完整，不放宽夹取入口校验；该修复只保证视觉快照完整性，不改变 R2 KFS 策略、深度窗口、目标选择、台阶流程或 `/cmd_vel` 行为。

2026-07-03 同步：`MfPreselectionFlow` 扩展入口相对横移的回中线距离补偿。除“入口中途夹取成功后回 2 号入口”外，`EntryReturnFromStair1` 和 `EntryReturnFromStair3` 未被 KFS 打断时的固定回 2 号入口横移也会按 MCU 半余弦减速模型和 `mf_preselect_entry_interrupt_latency_s` 估计停车尾巴，并从 `mf_preselect_entry_probe_return_distance_m` 中扣除相应补偿；补偿后若距离已落入 `mf_preselect_move_tolerance_m`，会直接停车进入上阶准备。该补偿复用 `mf_preselect_entry_mcu_vy_acc_mps2`、`mf_preselect_entry_interrupt_latency_s` 和现有回中线距离参数，不新增接口或参数，不改变入口外扩探测、1 号到 3 号扫线、`/cmd_vel` 接口、MCU 串口协议、梅林内部 KFS 对齐或前向 odom 趋近口径。

2026-07-01 同步：`MfPreselectionFlow` 新增入口相对横移的距离补偿。除入口横移中断窗口外，入口“中途夹取成功后回 2 号入口”也会按 MCU 半余弦减速模型和 `mf_preselect_entry_interrupt_latency_s` 估计停车尾巴，并从回中线目标距离中扣除相应补偿，避免减速滑移越过入口中心。该补偿复用 `mf_preselect_entry_mcu_vy_acc_mps2`、`mf_preselect_entry_interrupt_latency_s`，不新增接口或参数；补偿后若距离已落入 `mf_preselect_move_tolerance_m`，会直接停车进入上阶准备。入口横移中断继续沿用既有动态像素补偿和 MCU 停稳等待口径；这次改动只影响入口侧回 2 号入口的相对横移距离标定，不改变 `/cmd_vel` 接口、MCU 串口协议、梅林内部 KFS 对齐或前向 odom 趋近口径。

2026-07-01 同步：决策侧机构指令日志统一补充 `/mechanism/send_command` response 返回的真实 `seq`。单条机构命令、台阶前后推杆并发命令、`MfPreselectionFlow` 内部机构命令和 MC `GRAB_TIP` 的 ACK / rejected 日志都会打印 `seq`；`/cmd_vel` 本身没有 `seq` 字段，本次不为速度指令伪造序号。`decision_node` 新增 `decision_last_failure_source/reason/detail` 黑板失败汇总，行为树最终 `FAILURE` 日志会用中文打印失败来源和详细原因；导航、MC、MF、台阶和 MF 预选赛动作失败时会尽量写入阶段、当前格、命令、`seq`、完成反馈、odom/topic、超时等上下文，方便现场从最终失败日志直接定位卡点。

2026-07-04 同步：决策侧等待机构业务完成反馈的节点开始解析 `/mechanism/command_feedback` 上的两字节 `0xFE` 机械臂诊断 payload。`payload[0]` 会被记录为 `failed_cmd`，`payload[1]` 会转换为 `PLANAR_ARM_FAIL_*` 名称、中文含义和建议处理；`BUSY(0x01)` 只记录“处理中”并继续等待同 `seq` 最终反馈，不触发行为树失败。`INVALID_PAYLOAD/NOT_INIT/HAL_ERROR/INVALID_STATE` 会写入 `decision_last_failure_source/reason/detail`，当前机构等待返回 `FAILURE`，最终日志可直接看到原始命令、错误码和处置建议。决策层仍只消费 `/mechanism/send_command` 与 `/mechanism/command_feedback`，不直接解析串口帧。

2026-06-30 同步：`MfPreselectionFlow` 的入口横移 KFS 处理改为“看到后尽量夹取，但不被贴边框过早打断”。入口横移中有效 R2 KFS 若像素偏差超过 `mf_preselect_entry_interrupt_max_offset_px`，流程继续横移扫线，等目标进入窗口后再停车进入 KFS 对齐；KFS 对齐超时若最后有效目标仍在 `mf_preselect_kfs_align_timeout_pickup_tolerance_px` 内且有深度，则不放弃目标，直接进入前向 odom 趋近和夹取链。入口来源的对齐失败不再把该 KFS 加入 ignored 列表，后续重新进入窗口时仍可再次尝试。

2026-06-30 同步：修正 `MfPreselectionFlow::tickDetection()` 的未命中窗口语义。入口检测、行前方检测、周身扫描和旁列 `TransitionObserve` 在没有 R2 KFS 时会先停满 `mf_preselect_entry_detect_timeout_s` 或 `mf_preselect_scan_detect_timeout_s`，再用 `mf_preselect_detect_lost_stable_frames` 做稳定丢失确认；不会再因为 RealSense 连续几帧无目标而把 2 秒观察窗口提前缩短到约半秒。看到 R2 或假 KFS 的命中分支仍保持即时响应。

2026-06-30 同步：`MfPreselectionFlow` 的 KFS 识别横移对齐已收口到 `rc26_vision::tip_alignment` 口径。KFS 不再按最高置信度选框，而是在经过 `T_*` 标签、ignored target 和深度窗口过滤后，按识别框中心距离当前目标线最近获取锁定目标；锁定窗口内继续跟踪同一物理 KFS。旧的释放滞回和 no-progress 快速失败参数已删除，横移失败不再由 no-progress 触发，只保留目标丢失等待和 `mf_preselect_kfs_align_timeout_s` 总超时兜底。

2026-06-30 同步：KFS 横移速度直接复用 `computeTipAlignmentVy()`，`mf_preselect_kfs_align_kp`、`mf_preselect_kfs_align_min_speed_mps`、`mf_preselect_kfs_align_max_speed_mps` 和 `mf_preselect_kfs_invert_lateral_direction` 是横移调参入口；当前配置按 `mf_preselect_kfs_invert_lateral_direction=false` 维护。横移阶段捕获当前 odom yaw 作为 `tip_alignment` 的 heading target，`mf_preselect_kfs_odom_yaw_tolerance_deg` 作为 yaw 进入容差，新增 `mf_preselect_kfs_align_heading_gate_deg` 控制 yaw gate；odom 不新鲜时停车等待，yaw 超 gate 时暂停横移只修正朝向。

2026-06-30 同步：KFS 前向趋近仍使用锁定深度规划 X 轴距离，并由 `mf_preselect_kfs_odom_xy_kp`、`mf_preselect_kfs_approach_odom_tolerance_m`、`mf_preselect_kfs_odom_yaw_tolerance_deg`、`mf_preselect_kfs_odom_stable_ticks`、`mf_preselect_kfs_approach_speed_mps` 和 `mf_preselect_kfs_approach_min_speed_mps` 控制 odom 闭环执行；趋近闭环只执行锁定时规划出的距离，不读取实时深度连续停车。

2026-06-30 同步：`MfPreselectionFlow` 的入口区 R2 KFS 发现仍要求有效深度，但入口深度窗口已经从通用窗口拆出独立配置。入口 2 号、入口 1/3 号定点检测、入口横移中断检测，以及这些入口目标后续横移复核使用 `mf_preselect_entry_depth_min_m/max_m`；梅林内部行检测、旁列 `TransitionObserve` 和直出补夹继续使用 `mf_preselect_depth_min_m/max_m`。两套窗口只影响 R2 KFS 锁定和 odom 闭环趋近前的新帧复核，趋近阶段仍使用锁定深度规划目标距离，不按实时深度连续停车。

2026-06-30 同步：`MfPreselectionFlow` 的假 KFS 避障分支改为高度表驱动的旁列前向观察推进。初始避障从中列绕到旁列时先通过 `prepareTransitionTo()` 读取静态高度差，不再硬编码 `StairMode::Climb`；后续 `FakeAvoidAlignExit` 不再设置 `direct_exit_mode_` 或进入 `DirectExitDrive`，而是进入旁列模式，按当前旁列固定向前格复用 `TransitionTurn -> TransitionArmAdjust -> TransitionObserve -> TransitionStair -> GridCenterAlign`；出口行 `grid10/grid12` 复用最终下阶离场。从 `RowFront` 发现假 KFS 切入避障时会显式结束当前检测窗口，避免旁列 `TransitionObserve` 继承旧 `RowFront` 状态后误入周身扫描。旁列前向推进的观察朝向与台阶执行朝向分开：先面向目标格正前方观察/夹取 R2 KFS；若该边按高度表是下阶，再在 `TransitionStair` 前转到后轮先下的台阶 yaw。该改动只调整预选赛策略分支，不改变 `/cmd_vel`、机构 service、视觉标签或台阶原语接口。

2026-06-30 同步：`MfPreselectionFlow` 的默认机构协议 ID 改为引用 `rc26_serial::CommandID/FeedbackID`。决策层继续只消费 `/mechanism/send_command` 与 `/mechanism/command_feedback`，不解析串口帧。

2026-06-30 同步：移除旧外部 action 位姿导航链，导航权威收敛到 `rc26_decision` 内部 odom 单轴分段闭环。新增并注册 `OdomDriveX`、`OdomDriveY`、`OdomTurnToYaw`，保留 `RelativeYawTarget`；MC 改回原始 `+X 0.2m -> 右转 90° -> -X 0.6m`，MF 预选入口改回 `+X 2.0m -> -Y 1.8m`。`relative_nav_last_*` 是当前导航观测黑板键，旧 action result/recovery 语义不再维护。
