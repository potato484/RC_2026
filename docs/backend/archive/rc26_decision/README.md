# rc26_decision

## 模块定位

`rc26_decision` 是 R2 的主决策包，采用 BehaviorTree.CPP 组织比赛流程。

## 当前实现

- 构建产物:
  - `rc26_decision_nodes`
  - `decision_node`
- 运行入口:
  - `rc26_decision` 不再提供独立 launch 入口；决策运行和测试统一由 `rc26_bringup/launch/bringup.launch.py` 装配。涉及真实运动或机构动作时必须启动 `rc26_mcu_transport`，完整 bringup 默认会按 `r2_runtime.mcu_transport` 启动
- 关键行为树:
  - `behavior_trees/main_tree.xml`
  - `behavior_trees/mf_tree.xml`
  - `behavior_trees/mc_tree.xml`
  - `behavior_trees/combat_tree.xml`
  - `behavior_trees/grid_heading_tree.xml`（正式 Grid heading 转向/对齐入口，只执行原地转向和 yaw 精对齐）
  - `behavior_trees/two_pose_nav_tree.xml`（独立双点 Nav2 导航入口，默认主流程不引用）
  - `behavior_trees/stair_climb_tree.xml`（独立上台阶测试入口，默认主流程不引用）
  - `behavior_trees/stair_descend_tree.xml`（独立下台阶测试入口，默认主流程不引用）
  - `behavior_trees/mf_red_middle_column_tree.xml`（红方中间列连续台阶独立入口，默认主流程不引用）
  - `behavior_trees/mf_preselection_tree.xml`（梅林区预选赛专属正式入口，默认完整 bringup 可指向本树）
- 关键源码:
  - `src/decision_node.cpp`
  - `src/navigation/bt_nav2_pose.cpp`
  - `src/mf/mf_area.cpp`（MF 节点注册入口）
  - `src/mf/merlin_map.cpp`（梅林静态深度表与相邻关系）
  - `src/mf/select_next_grid.cpp`（下一格选择）
  - `src/mf/grid_transition_plan.cpp`（格间动作合法性校验与目标 yaw 规划）
  - `src/mf/grid_heading.cpp`（正式 GridTurn / GridHeadingAlign 转向与对齐动作）
  - `src/mf/grid_center.cpp`（MF 入口 grid2 中心参考建立与格间二维中心归位）
  - `src/mf/grid_transition.cpp`（离散格间上/下台阶动作）
  - `src/mf/conditions.cpp`（MF 退出条件）
  - `src/mf_preselection/mf_preselection_flow.cpp`（梅林预选赛专属入口探测、KFS 夹取、R1/假 KFS 干扰、四行推进和离场收尾）
  - `src/mc/mc_area.cpp`（注册 + `loadMCParams`）
  - `src/mc/visual_servo_grab.cpp`、`src/mc/rotate_in_place.cpp`、`src/mc/wait_forever.cpp`
  - `src/stair/stair_climb.cpp`、`src/stair/stair_descend.cpp`、`src/stair/stair_action_base.cpp`

## 当前 MF 格间与导航调用口径

- 梅林区格间运动不再用 `NavToPose` 表达；`mf_tree.xml` 中不再维护 12 个格中心 pose 分支
- `SelectNextGrid` 仍负责写入 `target_grid`；`PlanGridTransition` 校验 `current_grid -> target_grid` 是否为相邻且高度差一档的可执行边，并计算本次格间目标 yaw
- MF 离散格坐标到地图/odom yaw 的约定为：行号增加对应 `+X`（例如 `grid2 -> grid5` 为 `0°`），列号减少对应 `+Y`（例如 `grid2 -> grid1` 为 `+90°`）
- `PlanGridTransition` 按高度差选择 `CLIMB` 或 `DESCEND` 的朝向语义：上台阶使用边方向作为目标 yaw，下台阶使用边方向反向 yaw，让后轮在前下台阶
- `GridTurn` 与 `GridHeadingAlign` 负责格间动作前的正式原地转向和 yaw 精对齐；它们只订阅 `grid_heading_odom_topic` 并发布 `grid_heading_cmd_vel_topic`，不触发推杆或机构反馈。两段角速度上限分别由 `grid_heading_turn_max_speed_radps` 与 `grid_heading_align_max_speed_radps` 控制
- `GridTransition` 只负责台阶机构、直行、激光事件等待和状态提交；直行阶段继续用同一个目标 yaw 在 `cmd_vel.angular.z` 叠加 heading hold，动作成功后才提交 `current_grid=target_grid`
- `GridCenterAlign` 在 `GridTransition` 成功后按 MF 离散格中心做二维归位：以已记录的参考格中心和 `mf_center_grid_step_m` 推算目标格中心，订阅 `mf_center_odom_topic` 并向 `mf_center_cmd_vel_topic` 发布 `linear.x/y + angular.z`，同时收敛 x/y 和 yaw 后才让树继续执行
- `MFEntryCenterAdvance` 只用于红方中间列独立树首段入口：`StairClimb` 上到 `grid2` 后，沿目标 yaw 前进 `mf_center_entry_forward_offset_m`（默认 `0.25m`）到 `grid2` 中心，并把该点记录为后续二维格中心参考
- 平地同高格间移动当前不支持，`GridTransition` 会返回 `FAILURE` 并写入 `mf_transition_error=flat_transition_unsupported`
- `MFAreaTree` 不再被旧 keepout runtime 包裹；相关 keepout runtime service、MF KFS 状态和 terrain grid 公开接口已经删除
- MF 格位选择只使用 `MerlinMapManager` 的包内静态深度表、BT 黑板状态和离散格号

`PlanGridTransition` / `GridTransition` 会维护以下黑板键：

- `last_grid`
- `current_grid`
- `last_transition_kind`: `CLIMB | DESCEND`
- `last_height_delta`
- `last_transition_target_yaw`
- `grid_transition_target_yaw`
- `grid_transition_planned_from_grid`
- `grid_transition_planned_target_grid`
- `grid_transition_planned_height_delta`
- `mf_transition_error`

MF 格中心归位会维护以下黑板键：

- `mf_center_reference_grid`
- `mf_center_reference_x`
- `mf_center_reference_y`
- `mf_center_reference_yaw`
- `mf_center_target_grid`
- `mf_center_target_x`
- `mf_center_target_y`
- `mf_center_error_x`
- `mf_center_error_y`
- `mf_center_error_distance`
- `mf_center_error`

## Grid heading 正式入口

`behavior_trees/grid_heading_tree.xml` 是正式的独立转向/对齐入口，默认主比赛流程不引用它。运行它需要通过 `rc26_bringup/launch/grid_heading.launch.py` 显式拉起：

```bash
ros2 launch rc26_bringup grid_heading.launch.py
```

本入口只串行执行 `GridTurn -> GridHeadingAlign`，不读取 `current_grid` / `target_grid`，不调用 `/mechanism/send_command`，也不等待 0x04/0x05/0x07 激光事件。它会直接发布 `grid_heading_cmd_vel_topic`（默认 `cmd_vel`），运行前必须停用 Nav2 controller、遥控或其它运动命令权威。

方向和目标 yaw 由 `rc26_bringup/config/r2_runtime.yaml` 的 `grid_heading_*` 参数维护：`grid_heading_direction` 可取 `forward | left | right | backward`，分别映射到 `grid_heading_forward_yaw_rad`、`grid_heading_left_yaw_rad`、`grid_heading_right_yaw_rad`、`grid_heading_backward_yaw_rad`。`GridTurn` 的粗转向角速度上限使用 `grid_heading_turn_max_speed_radps`，`GridHeadingAlign` 的精对齐角速度上限使用 `grid_heading_align_max_speed_radps`；不再声明或兼容合并角速度参数。`decision_node` 启动时会把选中的目标 yaw 写入黑板 `grid_heading_target_yaw_rad`；方向非法时节点启动失败，避免未知方向误动作。

`NavToPose` 仍作为非 MF 格间移动的 BT 节点保留，调用 Nav2 `/navigate_to_pose`，action 类型为 `nav2_msgs/action/NavigateToPose`。当前 `mc_tree.xml`、`two_pose_nav_tree.xml` 等连续位姿导航入口仍可使用它。

`NavToPose` 会维护以下黑板观测键：

- `nav_last_exec_state`: `PENDING | RUNNING | SUCCEEDED | FAILED`
- `nav_last_failure_code`
- `nav_last_failure_reason`
- `nav_last_distance_remaining`
- `nav_last_recovery_count`

`NavToPose` 与其它复用 `BtActionNode` 的 action 节点在启动时会在 `timeout_sec` 内持续等待下游 action server 可用；超过 `timeout_sec` 仍未发现 action server 时才返回 `FAILURE`。`NavToPose` 额外通过 `/bt_navigator/get_state` 等待 Nav2 lifecycle 进入 `active`，并要求目标坐标系到 `base_footprint` 的最新 TF 足够新鲜后才发送目标，避免完整 bringup 中 `decision_node` 早于 Nav2 active 或定位 TF 稳定前开始 tick 时，把目标过早发给尚未可用的导航链。Nav2 返回 `SUCCEEDED` 后，`NavToPose` 还会用最新 TF 校验 `base_footprint` 是否真的接近 XML 中的目标位姿；默认校验容差为 `success_xy_tolerance=0.20m`、`success_yaw_tolerance=0.25rad`，可在单个 BT 节点端口上覆盖。

Nav2 action result 映射规则：

- `SUCCEEDED` 且决策层位姿后验通过 -> BT `SUCCESS`
- `SUCCEEDED` 但实际位姿未进入后验容差 -> BT `FAILURE`，`error_code=130`
- `ABORTED` -> BT `FAILURE`，`error_code=120`
- `CANCELED` -> BT `FAILURE`，`error_code=121`
- action server missing、invalid goal、timeout 继续沿用 `BtActionNode` 的通用错误码

## 独立双点导航树

`behavior_trees/two_pose_nav_tree.xml` 是一棵独立可加载的 Nav2 双点导航树，默认 `main_tree.xml`、`mf_tree.xml` 与 `mc_tree.xml` 都不引用它。运行它需要通过 `decision_node` 的 `tree_file` 显式指向该 XML，或者临时把完整 bringup 的 `r2_runtime.paths.behavior_tree_file` 改成该 XML 的绝对路径。

本树只用一个 `Sequence` 依次执行两个 `NavToPose`：先到 `map` 下 `(x=0.9675, y=0.0082, yaw=-0.0035)`，成功后再到 `(x=0.9778, y=0.7744, yaw=-1.4857)`；两段导航当前各自显式设置 `timeout_sec=180.0`，避免低速实车在 60 秒默认超时内被决策层提前取消。任一导航目标返回 `FAILURE` 时，整棵树立即失败并停止后续动作。它不做视觉夹取、台阶动作、黑板状态写入或默认入口切换。

## 梅林区预选赛专属链路

`behavior_trees/mf_preselection_tree.xml` 是梅林区预选赛专属正式入口，完整 bringup 可通过 `r2_runtime.paths.behavior_tree_file` 指向它。该树先按 `mf_preselect_entry2_nav_enable` 决定是否执行 2 号入口 `NavToPose`：关闭时跳过入口导航并假设机器人已经位于 2 号入口预备姿态；开启时若 `NavToPose` 返回 `FAILURE`，整棵预选赛树会停止在入口阶段，不会继续进入 `MfPreselectionFlow` 下发机构或运动命令。入口 `NavToPose` 默认通过 `mf_preselect_entry2_nav_behavior_tree_file` 指向 `rc26_bringup/config/nav2_bt_mf_preselect_entry_x_positive_y_negative.xml`，让 Nav2 只使用 `MFPreselectEntryXPositiveYNegative` controller，速度采样限制为车体系 `linear.x >= 0`、`linear.y <= 0`、`|angular.z| <= 0.10rad/s`，并在 `FollowPath` 中显式指定 `general_goal_checker`，避免 Humble Nav2 把空 goal checker 名称传给 controller server；该约束只覆盖入口导航，不影响后续 `MfPreselectionFlow` 内部直接发布 `/cmd_vel` 的相对移动、台阶、转向和离场状态机。当前 `rc26_bringup/config/nav2_params.yaml` 的共享 Nav2 costmap 加载 obstacle layer 与 inflation layer，因此该入口导航会消费动态障碍，并在合成 costmap 中对静态和动态障碍生成膨胀代价。入口导航成功后会立即执行 `GridTurn -> GridHeadingAlign`，目标 yaw 来自 `grid_heading_target_yaw_rad`，也就是 `grid_heading_direction` 和 `grid_heading_*_yaw_rad` 参数选出的方向；任一阶段失败都会阻止进入 `MfPreselectionFlow`。随后执行单个 `MfPreselectionFlow` 状态机，流程成功驶出梅林后进入 `WaitForever`，保持永久静止。

`MfPreselectionFlow` 只消费 `rc26_vision` 的 KFS 模型快照、`/odom`、`/cmd_vel` 和 `rc26_mcu_transport` 提供的 `/mechanism/send_command` / `/mechanism/command_feedback`。视觉标签按当前模型语义处理：`T_*` 是 R2 可夹取 KFS，`R_R1/B_R1` 是 R1 阻挡目标，`F_*` 是假 KFS；这些标签列表和前缀都由 `mf_preselect_*` 参数配置。R2 KFS 单局最多夹取 `mf_preselect_max_pickup_count` 个，默认 2 个，达到上限后不再触发 KFS 夹取。检测阶段稳定看到 `T_*` 后，正式流程不会直接发送 `GRAB_KFS_UP/DOWN`，而是先进入 `kfs_visual_align`：用目标框中心相对画面中心竖线的偏差发布 `cmd_vel.linear.y` 横移；横移连续稳定后只锁定一次 `mf_preselect_depth_min_m ~ mf_preselect_depth_max_m` 内的有效深度。若当前是向下夹取，则在开环前进前先发送 `ARM_SECOND_LOWER(0x0E)`，等待同 `seq` 的 `ARM_SECOND_LOWER_DONE(0x0A)`，确认第二节机械臂彻底放下后才启动开环计时；若锁定距离已经为 0，也仍先等待 `0x0A` 后直接夹取。随后按 `max(0, locked_depth - mf_preselect_kfs_grab_distance_m) / mf_preselect_kfs_approach_speed_mps` 做 x 方向纯开环趋近。开环阶段不再依据持续识别框或实时深度闭环停车，框消失、框跳变或深度波动不会触发重搜；计划时长到达后停车并发送方向对应的 `GRAB_KFS_UP/DOWN`。夹取计数不在 ACK 后立即提交：状态机会保存进入开环前锁定的 `label + bbox + sequence`，ACK 成功后进入最多 `mf_preselect_grab_verify_timeout_s` 的视觉验证窗口；只有同标签且 bbox IoU 达到 `mf_preselect_grab_verify_iou_threshold` 的原目标连续 `mf_preselect_grab_verify_lost_stable_frames` 个新推理帧不可见，才判定物理夹取成功并更新计数。同目标匹配 helper 与 `rc26_vision` 的独立 KFS action test 共用 `shared/target/visual_target_match`，避免两条链路的 bbox/label 语义分叉。若对齐阶段目标连续丢失或验证超时、原目标仍可见、没有新视觉帧，当前目标会被本轮忽略、不计数，并按“该检测点未夹到目标”的路线继续；service rejected、ACK 超时、第二节机械臂放下完成反馈超时、视觉启动失败和开环计划超过安全超时仍是流程失败。

为便于实车观察行为树卡点，`MfPreselectionFlow` 会输出中文阶段日志，覆盖检测阶段、R2/假 KFS/R1 分支、KFS 视觉横移对齐、开环趋近锁定信息、向下夹取前第二节机械臂放下、机构命令发送与 ACK/完成反馈、夹取视觉验证、相对移动、转向、零速等待、台阶激光事件、夹取计数和最终停车；日志只在阶段切换或关键事件发生时输出，避免按 tick 高频刷屏。

`src/mf_preselection/mf_preselection_flow.cpp` 已补充面向维护者的中文结构注释，覆盖文件级流程边界、入口探测、中间列推进、视觉稳定帧、R1 等待、机构异步命令、相对移动、转向、台阶激光事件、夹取计数提交和参数加载分组；这些注释只解释当前真实状态机，不新增运行时接口、黑板键或运动命令权威。

入口阶段先检测 2 号入口；若无 R2 KFS，则发送 `ARM_HIGH_RAISE(0x0D)` 并等待同 `seq` 的 `ARM_HIGH_RAISE_DONE(0x09)`，再按配置横移探测 1 号、3 号阶梯；这个高抬升只服务预选赛入口侧探测，不替代梅林内部普通机械臂升降。任一检测点探测到 R2 KFS 后都会先执行夹取前视觉横移对齐和一次锁深度；如果当前高低侧语义为向下夹取，则先发送 `ARM_SECOND_LOWER(0x0E)` 并等待 `ARM_SECOND_LOWER_DONE(0x0A)`，再执行开环趋近或直接夹取，最后保持当前机械臂高低侧语义发送 `GRAB_KFS_UP(0x03)` 或 `GRAB_KFS_DOWN(0x02)`。返回 2 号入口后，上阶梯前若已经处于入口高抬升保持态，不重复发送普通 `ARM_RAISE(0x04)`；否则先发送普通抬升并等待 `ARM_RAISE_DONE(0x02)`。进入梅林内部后，格间高低台阶切换观察前、第 2/3 行前方守卫检测前以及第 2/3 行左侧/背向周身扫描前，都会按静态格高差先执行普通机械臂预调：低到高发送 `ARM_RAISE(0x04)` 并等待 `ARM_RAISE_DONE(0x02)`，高到低发送 `ARM_LOWER(0x05)` 并等待 `ARM_LOWER_DONE(0x03)`；随后才进入对应检测窗口，检测到 R2 KFS 时也按该高低侧语义选择 `GRAB_KFS_UP(0x03)` 或 `GRAB_KFS_DOWN(0x02)`。

内部路线按中间列格位推进：`grid2 -> grid5 -> grid8 -> grid11`，常规出口再经 `grid12` 下阶梯离开梅林。入口上到 `grid2` 后，`MfPreselectionFlow` 内部会保持 `entry_heading_yaw` 并按 `mf_center_entry_forward_offset_m` 前进，建立 `grid2` 中心参考；之后每次中间列格间台阶完成都会用 `mf_center_grid_step_m` 推算目标格中心并执行二维归位，成功后才进入下一段检测、扫描、转向或离场准备。入场前已夹取 R2 KFS 时切入直出模式；这里的直出模式仍沿中间列台阶路线推进，只跳过第 2/3 行周身搜索，路径前方的 R1、R2 和假 KFS 守卫仍生效。入场前未夹取时，第 1 行只做前方检测，第 2/3 行先做前方守卫检测，未发现目标或假 KFS 后再执行左转和背向扫描，第 4 行强制收尾。路径前方遇 R1 目标时零速等待，默认无总超时，直到连续丢失稳定帧满足 `mf_preselect_detect_lost_stable_frames` 后才放行；周身扫描阶段看到 R1 只忽略。第 1/2/3 行前方遇假 KFS 时，按入场拾取来源选择 1 号或 3 号方向避障后上到侧向格，记录新的 `current_grid` 并执行格中心归位，再重新朝入口 heading 所定义的出口方向直行；第 4 行到达 `grid11` 后先执行强制出口转向，转向后若正前方仍是假 KFS，则 180° 转向并下阶梯离场。常规离场到达并归位 `grid12` 后，会先在台阶上对齐 `entry_heading_yaw + pi`，让车头与离开梅林正方向相反，再发送 `ARM_LOWER(0x05)` 并执行下阶梯；最终下阶离开梅林后，状态机会从最后一个格中心沿入口 heading 正向外推 `mf_preselect_final_exit_center_offset_m` 生成虚拟外侧目标，并保持同一个反向 yaw 归位。该虚拟目标在黑板中以 `mf_center_target_grid=0` 表示，不属于 `grid1..grid12`。

本链路维护以下黑板键：

- `mf_preselect_state`
- `mf_preselect_pickup_count`
- `mf_preselect_pickup_source`
- `mf_preselect_current_grid`
- `mf_preselect_done`
- `mf_preselect_error`

其中 `mf_preselect_state` 现在会出现 `kfs_visual_align`、`kfs_second_arm_lower`、`kfs_open_loop_approach`、`entry_center_align`、`grid_center_align` 与 `final_exit_center_align`，分别表示正式 MF 内部正在进行 KFS 夹取前横移对齐、向下夹取前等待第二节机械臂放下、定时开环趋近、入口 grid2 中心参考建立、格间台阶后的目标格中心归位和最终离场虚拟目标归位。`mf_preselect_kfs_lost_stop_frames` 只影响横移对齐阶段的目标丢失保护，不影响第二节机械臂等待或开环趋近；`mf_preselect_kfs_grab_distance_m` 是机械臂可触达距离/开环停止距离，不是前进途中闭环深度阈值。

旧 `KfsStairPickup` 阶梯等待 BT 节点和 `kfs_stair_*_test_tree.xml` 独立测试树已删除；MF 预选赛的 R2 KFS 夹取由 `MfPreselectionFlow` 内部状态机完成“稳定检测 -> 横移对齐 -> 锁定一次深度 -> 向下夹取时等待 `ARM_SECOND_LOWER_DONE(0x0A)` -> 定时开环趋近 -> GRAB_KFS_UP/DOWN -> 原目标视觉消失验证”。独立 `T_*` 对齐、趋近、夹取实机验收仍可由 `rc26_vision/test_kfs_vision.launch.py action_enable:=true` 承担，但该测试入口不替代正式 MF 决策权威。

## 武馆区 (MC) 行为树

武馆区已重构为一条专属行为树 `behavior_trees/mc_tree.xml`（`MCAreaTree`），运行时通过完整 bringup 装配后执行，流程：

1. `NavToPose` —— 发布红方武馆区 xy 导航目标并确认到达（复用 Nav2 节点，目标点经黑板键 `mc_nav_x/y/yaw/frame_id/timeout_sec` 由 XML 端口重映射注入；MC 专用 Nav2 BT 在本阶段忽略 yaw 到点且不允许角速度输出）
2. `RotateInPlace target_yaw_rad={mc_nav_yaw}` —— `NavToPose` 到达指定 x/y 后，才发布 `cmd_vel.angular.z` 做绝对 yaw 对齐
3. `VisualServoGrab` —— 视觉伺服夹取
4. `RotateInPlace` —— 原地旋转 180°
5. `WaitForever` —— 无限期等待（恒 `RUNNING`，树停留持续 tick）

`decision_node` 通过 `tree_file` 参数加载行为树；该参数现在支持绝对路径。完整 bringup 默认从 `rc26_bringup/config/r2_runtime.yaml` 的 `r2_runtime.paths.behavior_tree_file` 读取行为树 XML 绝对路径。决策包自身不再安装独立 launch 文件，避免只拉起单节点时误判完整链路状态。

`mc_tree.xml` 的武馆区去程 Nav2 目标不再写死在 XML 中，而是读取黑板中的 `mc_nav_x`、`mc_nav_y`、`mc_nav_yaw`、`mc_nav_frame_id` 与 `mc_nav_timeout_sec`；这些值由 `rc26_bringup/config/r2_runtime.yaml` 的 `mc_nav_*` 参数提供。当前默认运行口径切到红方武馆区配置，并通过 `mc_nav_behavior_tree_file` 给去程 `NavToPose` 指定 `rc26_bringup/config/nav2_bt_mc_red_positive_xy.xml`。完成视觉夹取和原地旋转后，MC 树直接进入 `WaitForever`，不再执行固定返程 `NavToPose`，也不再声明或写入 `mc_return_nav_*` 黑板键。决策层仍只向 `/navigate_to_pose` 发送去程目标和可选 Nav2 BT 路径，不直接发布导航阶段速度；红方 MC 去程的 `cmd_vel.linear.x/y` 正向约束由 `MCPositiveXYRed` 承担。当前共享 Nav2 costmap 加载 obstacle layer 与 inflation layer，因此 MC 去程导航会消费动态障碍，并在合成 costmap 中对静态和动态障碍生成膨胀代价。视觉伺服 heading hold 默认同用 `mc_nav_yaw` 作为期望车身朝向，避免导航目标 yaw 和取端头对线 yaw 分裂。

### 节点职责

- `VisualServoGrabAction`（`src/mc/visual_servo_grab.cpp`）：内嵌直连相机 + `rc26_vision::InferenceEngine`，在独立工作线程中执行"取帧→推理→锁定同一物理端头→雷达 odom yaw 姿态保持 + 横移 P 控制对齐→对齐稳定后以 `cmd_vel.linear.x` 负方向前探→等待 `/mechanism/command_feedback` 上行 `FRONT_LIMIT_SWITCH_TRIGGERED(0x06)`→立即停车→经 `/mechanism/send_command` 下发 `GRAB_TIP(0x01)`"。多框同时出现时，目标选择复用 `rc26_vision` 的 tip alignment helper：初次按离画面中心最近获锁，短暂丢失不立即切到另一侧框。单端头场景下，若 `mc_odom_topic` yaw 偏离目标 yaw 超过 `mc_align_heading_gate_deg`，动作只发布 `cmd_vel.angular.z` 修正车身朝向，暂停 `linear.y` 横移和前探；只有像素偏差进入 `mc_align_tolerance_px` 且 yaw 偏差进入 `mc_align_heading_tolerance_deg` 后才累计稳定帧并进入前探。每次动作生命周期最多发送一次实际进入 `async_send_request` 的 `GRAB_TIP`；service 未就绪时不消耗这次发送机会并在停车状态下继续等待，前探等待 0x06 超过 `mc_grab_approach_timeout_s` 则停车失败。`/mechanism/send_command` 返回 `accepted=false` 只表示通用 ACK 未被可靠确认或 transport 拒绝，MC 夹取动作不会因此立即让行为树失败。**完成判定**：夹取已下发后端头持续消失达 `mc_grab_done_lost_time_s` → `SUCCESS`；端头未消失则超 `mc_servo_timeout_s` → `FAILURE`。该宽容 ACK 语义只适用于 MC 视觉夹取，不改变台阶动作对 accepted 的严格判定。
- `RotateInPlaceAction`（`src/mc/rotate_in_place.cpp`）：发布 `cmd_vel.angular.z`，订阅 `mc_odom_topic`（默认 `odom`）。未提供 BT 端口 `target_yaw_rad` 时保持旧语义：使用 `rc26_odom_interface` 发布的雷达标准 `/odom` yaw 增量作为 180° 闭环反馈，按 `mc_rotate_direction` 的符号判断累计角度和剩余角度。提供 `target_yaw_rad` 时切换为绝对 yaw 对齐模式，按当前 odom yaw 到目标 yaw 的最短角误差发布角速度，`剩余角度 ≤ mc_rotate_yaw_tolerance_deg` → 停车 `SUCCESS`。角速度由 `mc_rotate_speed_radps` 配置为最大值，末段按 `mc_rotate_slowdown_angle_deg` 线性降到 `mc_rotate_min_speed_radps` 以降低过冲；`mc_rotate_odom_timeout_s` 内无新 odom 时停车等待，超 `mc_rotate_timeout_s` → `FAILURE`（yaw 直接由四元数解算，不依赖 tf2）。
- `CaptureCurrentPoseAction`（`src/navigation/bt_nav2_pose.cpp`）：订阅 TF 并等待 `mc_nav_frame_id -> base_footprint` 新鲜，在超时内捕获当前 `x/y/yaw` 写入对应黑板输出键。它作为通用 BT 节点保留，当前 MC 树不再使用它来生成返程目标位姿。
- `WaitForeverAction`（`src/mc/wait_forever.cpp`）：恒 `RUNNING`。

### 参数

全部武馆区运行参数以 `mc_*` 前缀集中于 `rc26_bringup/config/r2_runtime.yaml` 的 `r2_runtime.decision.ros__parameters`，由 `loadMCParams()` 在 `decision_node` 构造时声明/读取为 `McParams`（`src/mc/mc_params.hpp`）并写入黑板 `mc_params`，同时把去程导航目标和去程 Nav2 BT 路径写入 `mc_nav_*` 黑板键。参数支持启动时通过 YAML/launch 覆盖；当前没有运行期参数变更回调，`ros2 param set` 不会自动回写已经进入黑板和动作节点的运行参数。参数涵盖：相机/推理（`mc_camera_*`、`mc_model_id`、`mc_target_labels`）、对齐（`mc_align_*`，其中 `mc_align_target_lock_*` 控制端头锁定，`mc_align_invert_direction` 当前默认按后置相机反转横移方向，`mc_align_heading_*` 控制 odom yaw 姿态保持）、夹取与限位前探（`mc_grab_command_id`、`mc_grab_service_name`、`mc_grab_limit_switch_feedback_*`、`mc_grab_approach_*`、`mc_grab_done_lost_time_s`、`mc_servo_timeout_s`）、旋转（`mc_rotate_angle_deg`、`mc_rotate_speed_radps`、`mc_rotate_min_speed_radps`、`mc_rotate_slowdown_angle_deg`、`mc_rotate_direction`、`mc_rotate_yaw_tolerance_deg`、`mc_rotate_cmd_vel_topic`、`mc_odom_topic`、`mc_rotate_odom_timeout_s`、`mc_rotate_timeout_s`）、去程导航目标与 Nav2 goal 执行树（`mc_nav_*`、`mc_nav_behavior_tree_file`）。固定返程点及 `mc_return_nav_*` 参数已从当前 MC 树契约中移除。
注意：ROS2 不支持 YAML 空数组参数，`mc_grab_payload` 留空时须省略该项（用 C++ 默认空向量），不可写 `[]`。

### 与测试节点的差异

- 夹取服务调用改为 `async_send_request`（非阻塞 + 响应回调记录 accepted/seq），不再用测试节点的嵌套 `spin_until_future_complete`——因 `decision_node` 已运行于 `rclcpp::spin`，嵌套 executor 会冲突；一次 accepted=false 不会直接让 MC 行为树失败，完成判定本就以端头消失为准。
- 剔除测试节点的 OpenCV 窗口/叠加绘制/距离估计等与决策无关代码。

## 当前 BT 边界

- 行为树继续作为 `rc26_decision` 包内编排实现存在
- 对外不再提供第一方 BT 运行时 topic、service 或配套调试消息
- `decision_node` 当前只保留 `tick_rate_ms` 自动执行模式，不再保留手动单步、播放/暂停、外部重置或运行时发布面
- `nav_last_*` 等字段当前只作为黑板内部状态存在，不再代表公开观测契约

## 台阶动作与 MF 复用

台阶动作既可通过 `stair_climb_tree.xml` / `stair_descend_tree.xml` 独立加载测试，也可由 MF 的 `GridTransition` 串行复用。独立树仍只执行单个台阶动作；MF 主树则通过 `target_grid` 和静态深度表决定本次格间边应当上台阶还是下台阶。

- `StairClimb`：先通过 `/mechanism/send_command` 下发 `FRONT_PUSHROD_EXTEND`，accepted 后按 `stair_climb_front_extend_delay_s` 零速等待；随后以 `stair_climb_front_drive_*` profile 对应的 `x` 正方向速度规划直行等待 `/mechanism/command_feedback` 中的 `FRONT_LASER_HEIGHT_JUMP(0x04)`。收到前轮突变后立即停车，并在同一 BT tick 内连续发出 `FRONT_PUSHROD_RETRACT` 与 `REAR_PUSHROD_EXTEND` 两条异步 service 请求；两条都 accepted 后按 `stair_climb_retract_rear_extend_delay_s` 零速等待，再以 `stair_climb_rear_drive_*` profile 对应的 `x` 正方向速度规划等待 `REAR_LASER_HEIGHT_JUMP(0x05)`；最后停车并下发 `REAR_PUSHROD_RETRACT`，accepted 后按 `stair_climb_rear_retract_delay_s` 零速等待，再返回成功。当前上台阶只消费前轮 `0x04` 与后轮 `0x05` 两个激光事件，前轮和后轮速度 profile 同步作用于独立 `StairClimb`、MF `GridTransition` 和 `MfPreselectionFlow` 内嵌台阶。
- `StairDescend`：先以 `stair_descend_rear_drive_speed_mps` 对应的 `x` 负方向固定速度直行等待 `REAR_LASER_HEIGHT_JUMP(0x05)`，停车并下发 `REAR_PUSHROD_EXTEND`；accepted 后按 `stair_descend_rear_extend_delay_s` 零速等待。随后以 `stair_descend_front_second_drive_*` profile 对应的 `x` 负方向速度规划等待前轮第二个激光测距模块 `FRONT_SECOND_LASER_HEIGHT_JUMP(0x07)`，停车并在同一 BT tick 内连续发出 `REAR_PUSHROD_RETRACT` 与 `FRONT_PUSHROD_EXTEND` 两条异步 service 请求；两条都 accepted 后按 `stair_descend_retract_front_extend_delay_s` 零速等待，再以 `stair_descend_front_retract_timed_drive_speed_mps` 对应的 `x` 负方向固定速度连续发送 `stair_descend_front_retract_drive_duration_s`。定时行驶结束后停车，下发 `FRONT_PUSHROD_RETRACT`，accepted 后按 `stair_descend_front_retract_delay_s` 零速等待，再返回成功。当前下台阶只消费后轮 `0x05` 与前轮第二激光 `0x07` 两个激光事件；全下阶梯链路不需要 `FRONT_LASER_HEIGHT_JUMP(0x04)` 作为阶段推进条件。
- `GridTurn` / `GridHeadingAlign` 直接发布 `grid_heading_cmd_vel_topic`（默认 `cmd_vel`）；台阶动作和 `GridTransition` 直接发布 `stair_cmd_vel_topic`（默认 `cmd_vel`）；`MFEntryCenterAdvance` / `GridCenterAlign` 直接发布 `mf_center_cmd_vel_topic`（默认 `cmd_vel`）做格中心前进和二维归位。运行时必须停用 Nav2 controller、遥控或其它运动命令权威；任何命令拒绝、服务等待超时、激光事件等待超时、heading odom 超时或格中心归位超时都会发布零速并返回 `FAILURE`，`onHalted()` 只发布零速，不额外补偿推杆状态。

台阶参数以 `stair_*` 前缀集中在 `rc26_bringup/config/r2_runtime.yaml`，启动时由 `loadStairParams()` 写入黑板 `stair_params`；当前没有运行期参数变更回调。旧泛化速度参数 `stair_climb_drive_speed_mps`、`stair_descend_drive_speed_mps` 与 `stair_descend_front_retract_drive_speed_mps` 已删除，启动时不再声明、读取或兼容。上台阶前轮 `0x04` 段由 `stair_climb_front_drive_fast_speed_mps`、`stair_climb_front_drive_slow_speed_mps` 与 `stair_climb_front_drive_slowdown_duration_s` 控制；上台阶后轮 `0x05` 段由 `stair_climb_rear_drive_fast_speed_mps`、`stair_climb_rear_drive_slow_speed_mps` 与 `stair_climb_rear_drive_slowdown_duration_s` 控制。下台阶后轮 `0x05` 段使用固定 `stair_descend_rear_drive_speed_mps`；下台阶前轮第二激光 `0x07` 段使用 `stair_descend_front_second_drive_fast_speed_mps`、`stair_descend_front_second_drive_slow_speed_mps` 与 `stair_descend_front_second_drive_slowdown_duration_s`，当前运行配置为 `0.10m/s -> 0.05m/s`、`1.0s`；前推杆收回前的定时负向行驶使用固定 `stair_descend_front_retract_timed_drive_speed_mps` 和 `stair_descend_front_retract_drive_duration_s`。所有 profile 速度取绝对值，`slow` 会夹到不超过 `fast`，降速时长小于 0 时按 0 处理；固定速度同样按绝对值读取，方向由状态机显式加正负号。上台阶零速等待参数为 `stair_climb_front_extend_delay_s`、`stair_climb_retract_rear_extend_delay_s` 与 `stair_climb_rear_retract_delay_s`；下台阶零速等待参数为 `stair_descend_rear_extend_delay_s`、`stair_descend_retract_front_extend_delay_s` 与 `stair_descend_front_retract_delay_s`。姿态微调参数包括 `stair_odom_topic`、`stair_heading_hold_enable`、`stair_heading_kp`、`stair_heading_max_speed_radps`、`stair_heading_tolerance_deg`、`stair_heading_gate_deg`、`stair_heading_stable_ticks`、`stair_heading_odom_timeout_s` 与 `stair_heading_align_timeout_s`；独立台阶动作默认捕获启动后的当前 yaw 作为保持方向，MF `GridTransition` 会按格间边显式写目标 yaw。

MF 格中心归位参数以 `mf_center_*` 前缀集中在同一个 `r2_runtime.yaml` 决策参数区，启动时由 `loadGridCenterParams()` 写入黑板 `mf_center_params`。其中 `mf_center_entry_forward_offset_m` 是红方中间列独立树首段上到 `grid2` 后继续前进到格中心的可调距离，默认 `0.25m`；`mf_center_grid_step_m` 是后续格中心推算使用的离散格距，默认 `1.2m`。`MfPreselectionFlow` 也复用这组参数执行入口、格间、假 KFS 避障和最终离场后的中心归位；最终离场额外用 `mf_preselect_final_exit_center_offset_m` 从最后一个格中心外推虚拟目标，并以 `mf_center_target_grid=0` 写黑板。`mf_center_xy_tolerance_m`、`mf_center_yaw_tolerance_deg`、`mf_center_stable_ticks` 和 `mf_center_align_timeout_s` 决定二维归位成功/失败语义。

## 红方中间列连续台阶树

`behavior_trees/mf_red_middle_column_tree.xml` 是一棵独立可加载的红方 MF 中间列离散格间测试树，默认 `main_tree.xml`、`mf_tree.xml` 与 `mc_tree.xml` 都不引用它。运行它需要通过 `decision_node` 的 `tree_file` 显式指向该 XML，或者临时把完整 bringup 的 `r2_runtime.paths.behavior_tree_file` 改成该 XML 的绝对路径并设置 `team=red`，保证 `merlin_map` 用红方高度表初始化。

- 路线固定为红方 `梅林外 -> grid2 -> grid5 -> grid8 -> grid11 -> grid12 -> 梅林外`；首段由独立 `StairClimb` 上台阶进入 `grid2`，随后 `MFEntryCenterAdvance` 继续前进可调距离并记录 `grid2` 二维中心参考，成功后才把黑板 `current_grid` 写为 `2`。中间四段由 `GridTransition` 执行，分别为 `CLIMB`、`CLIMB`、`DESCEND`、`DESCEND`，每段成功后都通过 `GridCenterAlign` 回到目标格中心，最后再由独立 `StairDescend` 从已归位的 `grid12` 继续下阶梯离开梅林。
- 本树不调用 `NavToPose`；入口段复用独立上台阶状态机并用 `mf_center_entry_forward_offset_m` 建立 `grid2` 中心参考，随后验证 `PlanGridTransition -> GridTurn -> GridHeadingAlign -> GridTransition -> GridCenterAlign` 的同一套 MF 离散格间动作、yaw 对齐、台阶执行、状态提交与二维格中心归位，并在到达 `grid12` 后复用独立下台阶状态机完成梅林外离场。最后一段离场不再提交新的 `current_grid`，黑板中的离散格仍停在 `12`。
- 运行时依赖由 `/odom`、`grid_heading_*` 参数、`rc26_mcu_transport` 提供的 `/mechanism/send_command`、`/mechanism/command_feedback`、`stair_odom_topic` 与台阶动作参数 `stair_*` 共同提供。执行该树时必须确保 Nav2 controller、遥控或其它测试节点不会同时发布运动命令。

## 当前边界

- 负责流程编排、目标选择和策略切换
- MF `GridTransition` 负责选择格间边的台阶动作类型和 yaw 对齐策略，`GridCenterAlign` 负责台阶完成后的二维格中心归位；两者都不处理串口协议帧或机构底层状态机
- 不拥有 Nav2 planner/controller 的内部配置
- 不依赖旧 base-ground、terrain 或 keepout 包；不订阅旧 terrain/base-ground/keepout 输出，也不调用旧 keepout runtime service

## 本轮收口

- `mf_tree.xml` 改为离散格间动作版本，删除 MF 内全部 `NavToPose` 格中心分支
- 新增正式 `GridTurn` / `GridHeadingAlign` BT 节点和 `grid_heading_tree.xml` 入口：方向由 `grid_heading_direction` 与四个方向 yaw 参数选择，只负责原地转向和 yaw 精对齐，不触发推杆；粗转向和精对齐最大角速度分别由 `grid_heading_turn_max_speed_radps`、`grid_heading_align_max_speed_radps` 配置，不再兼容合并角速度参数。
- MF 格间动作改为 `PlanGridTransition -> GridTurn -> GridHeadingAlign -> GridTransition -> GridCenterAlign`：前者按 `current_grid -> target_grid` 的相邻边和静态高度差选择 `CLIMB` / `DESCEND` 并计算目标 yaw，`GridTransition` 负责台阶执行和成功后提交 `current_grid`，`GridCenterAlign` 负责后验二维格中心归位
- `StairActionBase` 增加 `stair_heading_*` 参数和 odom yaw heading hold；独立 `StairClimb` / `StairDescend` 默认保持启动时 yaw，MF `GridTransition` 显式设置格间目标 yaw
- 台阶速度参数改为阶段专属：上台阶前轮 `0x04` 与后轮 `0x05` 段均使用 fast->slow 线性 profile；下台阶后轮 `0x05` 与前推杆收回前定时后退保持固定速度，下台阶前轮第二激光 `0x07` 段使用 `0.10m/s -> 0.05m/s`、`1.0s` profile。旧 `stair_climb_drive_speed_mps`、`stair_descend_drive_speed_mps` 与 `stair_descend_front_retract_drive_speed_mps` 不再兼容。
- `SelectNextGrid` 不再把平地同高格当作可执行移动目标；同高格间移动留待后续定义
- `mf_red_middle_column_tree.xml` 改为固定 `StairClimb -> MFEntryCenterAdvance -> GridTransition(5) + GridCenterAlign(5) -> GridTransition(8) + GridCenterAlign(8) -> GridTransition(11) + GridCenterAlign(11) -> GridTransition(12) + GridCenterAlign(12) -> StairDescend` 测试树，不再依赖 Nav2 pose；首个独立上台阶动作从梅林外进入 `grid2` 后先前进可调距离建立中心参考，再提交 `current_grid=2`，最后一个独立下台阶动作从已归位的 `grid12` 继续离开梅林，离场后不再更新离散格号
- `CheckExitCondition` 现在只判断 `current_grid` 是否属于出口格
- MF 区域实现已按职责拆分：`mf_area.cpp` 只保留注册，地图、选边、格间台阶动作和退出条件分别维护；KFS/扫描链路已删除，BT 节点名、黑板键和现有台阶行为语义保持不变。

## 2026-06-12 更新

- `mc_target_labels` 修正为 `["JK"]`：`tip_default` 模型 profile（`tip.onnx`）的标签表中目前只有 `JK` 这一个类别，原先的 `["D_0", "D_1"]` 无法匹配任何检测结果，导致视觉伺服永远找不到目标。修改后 `resolveTargetClassIds()` 能正确映射到模型输出的类别 ID。
- `src/mc/visual_servo_grab.cpp`、`src/mc/rotate_in_place.cpp`、`src/mc/wait_forever.cpp` 全部补加了 `onStart`/`onRunning`/`onHalted` 的中文注释，说明每个阶段的具体职责：资源初始化、每 tick 轮询逻辑、以及外部中断时的安全停机清理。
- `src/stair/*.cpp` 与独立 `stair_climb_tree.xml` / `stair_descend_tree.xml` 已补加中文逐段注释，说明推杆命令、激光突变事件、速度发布、超时失败和 halt 零速的每一步决策逻辑；本次只增加注释，不改变台阶动作运行语义。
- MC 参数清理为当前真实语义：视觉目标选择只保留 `mc_target_labels`，夹取只保留命令、服务和完成/超时判定参数；旋转速度参数统一为 `mc_rotate_speed_radps`，按 `rad/s` 直接发布到 `cmd_vel.angular.z`。
- 端头模型 profile ID 已从历史测试命名 `tip_test` 改为 `tip_default`；MC 决策默认 `mc_model_id` 同步使用 `tip_default`，模型文件仍由 `rc26_vision/config/vision_models.yaml` 指向 `models/tip.onnx`。
- 2026-06-13 同步：删除包内 `config/decision_params.yaml` 与独立 `decision.launch.py` 入口；决策参数与行为树入口统一由完整 bringup 从 `rc26_bringup/config/r2_runtime.yaml` 读取，测试口径改为验证所有相关节点拉起后的链路效果。
