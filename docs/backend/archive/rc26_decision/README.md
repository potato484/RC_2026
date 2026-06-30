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

`GridTransition` 负责选择上/下台阶动作、发布台阶直行速度、等待激光事件并提交 `current_grid`。`GridCenterAlign` 在台阶完成后依据 `mf_center_grid_step_m` 执行二维格中心归位。`MfPreselectionFlow` 内部继续负责入口探测、KFS 视觉锁定、横移开环段、新视觉帧复核、机构命令、夹取视觉消失验证、入口/格间台阶和最终离场归位。假 KFS 避障不再硬编码为上阶：从中列绕到 1 号侧或 3 号侧旁列时，会先按 `MerlinMapManager` 静态高度表计算高度差，再选择上/下台阶和机构预调。完成该横向格间动作后不再进入 `direct_exit` 直行兜底，而是沿避障后的旁列继续前向推进：1 号侧旁列按 `grid1 -> grid4 -> grid7 -> grid10`，3 号侧旁列按 `grid3 -> grid6 -> grid9 -> grid12`。每个旁列格间转换前复用现有 `TransitionObserve` 正前方观察和机构预调逻辑，只看前方 R2 KFS；看到则按现有 KFS 夹取链处理，未看到则继续对应上/下阶梯，抵达出口行后复用最终下阶离场。

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

2026-06-30 同步：`MfPreselectionFlow` 的入口区 R2 KFS 发现仍要求有效深度，但入口深度窗口已经从通用窗口拆出独立配置。入口 2 号、入口 1/3 号定点检测、入口横移中断检测，以及这些入口目标后续横移复核使用 `mf_preselect_entry_depth_min_m/max_m`；梅林内部行检测、旁列 `TransitionObserve` 和直出补夹继续使用 `mf_preselect_depth_min_m/max_m`。两套窗口只影响 R2 KFS 锁定和开环趋近前的新帧复核，开环趋近阶段仍使用锁定深度规划固定前进，不按实时深度闭环停车。

2026-06-30 同步：`MfPreselectionFlow` 的假 KFS 避障分支改为高度表驱动的旁列前向观察推进。初始避障从中列绕到旁列时先通过 `prepareTransitionTo()` 读取静态高度差，不再硬编码 `StairMode::Climb`；后续 `FakeAvoidAlignExit` 不再设置 `direct_exit_mode_` 或进入 `DirectExitDrive`，而是进入旁列模式，按当前旁列固定向前格复用 `TransitionTurn -> TransitionArmAdjust -> TransitionObserve -> TransitionStair -> GridCenterAlign`；出口行 `grid10/grid12` 复用最终下阶离场。从 `RowFront` 发现假 KFS 切入避障时会显式结束当前检测窗口，避免旁列 `TransitionObserve` 继承旧 `RowFront` 状态后误入周身扫描。旁列前向推进的观察朝向与台阶执行朝向分开：先面向目标格正前方观察/夹取 R2 KFS；若该边按高度表是下阶，再在 `TransitionStair` 前转到后轮先下的台阶 yaw。该改动只调整预选赛策略分支，不改变 `/cmd_vel`、机构 service、视觉标签或台阶原语接口。

2026-06-30 同步：`MfPreselectionFlow` 的默认机构协议 ID 改为引用 `rc26_serial::CommandID/FeedbackID`，其中入口高侧 KFS 夹取对应 `ENTRY_GRAB_KFS_UP(0x0F)` / `ENTRY_GRAB_KFS_UP_DONE(0x0B)`。决策层继续只消费 `/mechanism/send_command` 与 `/mechanism/command_feedback`，不解析串口帧。

2026-06-30 同步：移除旧外部 action 位姿导航链，导航权威收敛到 `rc26_decision` 内部 odom 单轴分段闭环。新增并注册 `OdomDriveX`、`OdomDriveY`、`OdomTurnToYaw`，保留 `RelativeYawTarget`；MC 改回原始 `+X 0.2m -> 右转 90° -> -X 0.6m`，MF 预选入口改回 `+X 2.0m -> -Y 1.8m`。`relative_nav_last_*` 是当前导航观测黑板键，旧 action result/recovery 语义不再维护。
