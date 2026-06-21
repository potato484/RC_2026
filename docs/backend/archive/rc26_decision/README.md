# rc26_decision

## 模块定位

`rc26_decision` 是 R2 的主决策包，采用 BehaviorTree.CPP 组织比赛流程。

## 当前实现

- 构建产物:
  - `rc26_decision_nodes`
  - `decision_node`
- 运行入口:
  - `rc26_decision` 不再提供独立 launch 入口；决策运行和测试统一由 `rc26_bringup/launch/bringup.launch.py` 装配，以保证定位、Nav2、`merge_odom` 与决策节点处在同一完整链路中验证
- 关键行为树:
  - `behavior_trees/main_tree.xml`
  - `behavior_trees/mf_tree.xml`
  - `behavior_trees/mc_tree.xml`
  - `behavior_trees/combat_tree.xml`
  - `behavior_trees/stair_climb_tree.xml`（独立上台阶测试入口，默认主流程不引用）
  - `behavior_trees/stair_descend_tree.xml`（独立下台阶测试入口，默认主流程不引用）
  - `behavior_trees/mf_red_middle_column_tree.xml`（红方中间列连续台阶独立入口，默认主流程不引用）
- 关键源码:
  - `src/decision_node.cpp`
  - `src/navigation/bt_nav2_pose.cpp`
  - `src/mf/mf_area.cpp`
  - `src/mc/mc_area.cpp`（注册 + `loadMCParams`）
  - `src/mc/visual_servo_grab.cpp`、`src/mc/rotate_in_place.cpp`、`src/mc/wait_forever.cpp`
  - `src/stair/stair_climb.cpp`、`src/stair/stair_descend.cpp`、`src/stair/stair_action_base.cpp`

## 当前导航调用口径

- 梅林区导航统一使用 `NavToPose` BT 节点
- `NavToPose` 调用 Nav2 `/navigate_to_pose`，action 类型为 `nav2_msgs/action/NavigateToPose`
- BT XML 中显式写入 `frame_id / x / y / yaw / behavior_tree / timeout_sec`，不再通过动态字符串拼接目标
- `SelectNextGrid` 仍负责写入 `target_grid`；动态格位导航通过显式分支选择对应 `NavToPose`
- `current_grid:=target_grid` 等脚本在对应 pose 成功后继续保持原有语义
- `MFAreaTree` 不再被 `WithKeepoutRuntime` 包裹；决策不再调用 `/kfs_keepout/set_runtime`，也不再发布 `/mf_kfs_state`
- MF 格位选择只使用 `MerlinMapManager` 的包内静态深度表、BT 黑板状态和视觉结果，不再订阅 `/terrain_grid_map`

`NavToPose` 会维护以下黑板观测键：

- `nav_last_exec_state`: `PENDING | RUNNING | SUCCEEDED | FAILED`
- `nav_last_failure_code`
- `nav_last_failure_reason`
- `nav_last_distance_remaining`
- `nav_last_recovery_count`

Nav2 action result 映射规则：

- `SUCCEEDED` -> BT `SUCCESS`
- `ABORTED` -> BT `FAILURE`，`error_code=120`
- `CANCELED` -> BT `FAILURE`，`error_code=121`
- action server missing、invalid goal、timeout 继续沿用 `BtActionNode` 的通用错误码

## 武馆区 (MC) 行为树

武馆区已重构为一条专属行为树 `behavior_trees/mc_tree.xml`（`MCAreaTree`），运行时通过完整 bringup 装配后执行，流程：

1. `NavToPose` —— 发布导航目标点并确认到达（复用 Nav2 节点，目标点经黑板键 `mc_nav_x/y/yaw/frame_id/timeout_sec` 由 XML 端口重映射注入）
2. `VisualServoGrab` —— 视觉伺服夹取
3. `RotateInPlace` —— 原地旋转 180°
4. `WaitForever` —— 无限期等待（恒 `RUNNING`，树停留持续 tick）

`decision_node` 通过 `tree_file` 参数加载行为树；该参数现在支持绝对路径。完整 bringup 默认从 `rc26_bringup/config/r2_runtime.yaml` 的 `r2_runtime.paths.behavior_tree_file` 读取行为树 XML 绝对路径。决策包自身不再安装独立 launch 文件，避免只拉起单节点时误判完整链路状态。

### 节点职责

- `VisualServoGrabAction`（`src/mc/visual_servo_grab.cpp`）：内嵌直连相机 + `rc26_vision::InferenceEngine`，在独立工作线程中复刻 `rc26_vision/test/tip_vision_test_node.cpp` 的"取帧→推理→锁定同一物理端头→横移 P 控制对齐(`cmd_vel.linear.y`)→对齐稳定后以 `cmd_vel.linear.x` 负方向前探→等待 `/mechanism/command_feedback` 上行 `FRONT_LIMIT_SWITCH_TRIGGERED(0x19)`→立即停车→经 `/mechanism/send_command` 下发 `GRAB_TIP(0x01)`"。多框同时出现时，目标选择复用 `rc26_vision` 的 tip alignment helper：初次按离画面中心最近获锁，短暂丢失不立即切到另一侧框。每次动作生命周期最多发送一次实际进入 `async_send_request` 的 `GRAB_TIP`；service 未就绪时不消耗这次发送机会并在停车状态下继续等待，前探等待 0x19 超过 `mc_grab_approach_timeout_s` 则停车失败。**完成判定**：夹取已下发后端头持续消失达 `mc_grab_done_lost_time_s` → `SUCCESS`；超 `mc_servo_timeout_s` → `FAILURE`。
- `RotateInPlaceAction`（`src/mc/rotate_in_place.cpp`）：发布 `cmd_vel.angular.z`，订阅 `mc_odom_topic`（默认 `merge_odom`），默认依赖 `rc26_merge_odom` 提供的稳定 `/merge_odom` 底盘局部反馈契约；角速度由 `mc_rotate_speed_radps` 配置，单位 `rad/s`；由相邻 yaw 增量积分实现闭环转角，`|累计| ≥ 角度-容差` → 停车 `SUCCESS`；超 `mc_rotate_timeout_s` → `FAILURE`（yaw 直接由四元数解算，不依赖 tf2）。如果完整武馆/混合决策入口没有启动 `merge_odom` 底盘执行链或没有真实 `/merge_odom` publisher，旋转按超时失败，这是启动配置错误而不是正常降级。
- `WaitForeverAction`（`src/mc/wait_forever.cpp`）：恒 `RUNNING`。

### 参数

全部武馆区运行参数以 `mc_*` 前缀集中于 `rc26_bringup/config/r2_runtime.yaml` 的 `r2_runtime.decision.ros__parameters`，由 `loadMCParams()` 在 `decision_node` 构造时声明/读取为 `McParams`（`src/mc/mc_params.hpp`）并写入黑板 `mc_params`。这些参数支持启动时通过 YAML/launch 覆盖；当前没有运行期参数变更回调，`ros2 param set` 不会自动回写已经进入黑板和动作节点的运行参数。参数涵盖：相机/推理（`mc_camera_*`、`mc_model_id`、`mc_target_labels`）、对齐（`mc_align_*`，其中 `mc_align_target_lock_*` 控制端头锁定，`mc_align_invert_direction` 当前默认按后置相机反转横移方向）、夹取与限位前探（`mc_grab_command_id`、`mc_grab_service_name`、`mc_grab_limit_switch_feedback_*`、`mc_grab_approach_*`、`mc_grab_done_lost_time_s`、`mc_servo_timeout_s`）、旋转（`mc_rotate_angle_deg`、`mc_rotate_speed_radps`、`mc_rotate_direction`、`mc_rotate_yaw_tolerance_deg`、`mc_rotate_cmd_vel_topic`、`mc_odom_topic`、`mc_rotate_timeout_s`）、导航目标（`mc_nav_*`）。
注意：ROS2 不支持 YAML 空数组参数，`mc_grab_payload` 留空时须省略该项（用 C++ 默认空向量），不可写 `[]`。

### 与测试节点的差异

- 夹取服务调用改为 `async_send_request`（非阻塞 + 响应回调记录 accepted/seq），不再用测试节点的嵌套 `spin_until_future_complete`——因 `decision_node` 已运行于 `rclcpp::spin`，嵌套 executor 会冲突；完成判定本就以端头消失为准。
- 剔除测试节点的 OpenCV 窗口/叠加绘制/距离估计等与决策无关代码。

## 当前 BT 边界

- 行为树继续作为 `rc26_decision` 包内编排实现存在
- 对外不再提供第一方 BT 运行时 topic、service 或配套调试消息
- `decision_node` 当前只保留 `tick_rate_ms` 自动执行模式，不再保留手动单步、播放/暂停、外部重置或运行时发布面
- `nav_last_*` 等字段当前只作为黑板内部状态存在，不再代表公开观测契约

## 独立台阶行为树

台阶动作当前只作为独立 BT XML 能力注册到 `decision_node`，默认 `main_tree.xml`、`mf_tree.xml` 与 `mc_tree.xml` 都不引用它们；`MF_Exit` 当前只保留 Nav2 pose 退出点，不再隐式执行下台阶。

- `StairClimb`：先通过 `/mechanism/send_command` 下发 `FRONT_PUSHROD_EXTEND`，accepted 后按 `stair_climb_front_extend_delay_s` 零速等待；随后以 `x` 正方向直行等待 `/mechanism/command_feedback` 中的 `FRONT_LASER_HEIGHT_JUMP(0x17)`。收到前轮突变后立即停车，并在同一 BT tick 内连续发出 `FRONT_PUSHROD_RETRACT` 与 `REAR_PUSHROD_EXTEND` 两条异步 service 请求；两条都 accepted 后按 `stair_climb_retract_rear_extend_delay_s` 零速等待，再恢复 `x` 正方向直行等待 `REAR_LASER_HEIGHT_JUMP(0x18)`，最后停车并下发 `REAR_PUSHROD_RETRACT`，accepted 后返回成功。当前上台阶只消费前轮 `0x17` 与后轮 `0x18` 两个激光事件。
- `StairDescend`：先以 `x` 负方向直行等待 `REAR_LASER_HEIGHT_JUMP(0x18)`，停车并下发 `REAR_PUSHROD_EXTEND`；accepted 后按 `stair_descend_rear_extend_delay_s` 零速等待。随后继续负向直行等待前轮第二个激光测距模块 `FRONT_SECOND_LASER_HEIGHT_JUMP(0x1A)`，停车并在同一 BT tick 内连续发出 `REAR_PUSHROD_RETRACT` 与 `FRONT_PUSHROD_EXTEND` 两条异步 service 请求；两条都 accepted 后按 `stair_descend_retract_front_extend_delay_s` 零速等待，再以 `x` 负方向按 `stair_descend_front_retract_drive_speed_mps` 默认 `0.025m/s` 连续发送 `stair_descend_front_retract_drive_duration_s` 默认 `4.0s`。定时行驶结束后停车，下发 `FRONT_PUSHROD_RETRACT`，accepted 后按 `stair_descend_front_retract_delay_s` 零速等待，再返回成功。当前下台阶只消费后轮 `0x18` 与前轮第二激光 `0x1A` 两个激光事件；全下阶梯链路不需要 `FRONT_LASER_HEIGHT_JUMP(0x17)` 作为阶段推进条件。
- 两个动作都直接发布 `stair_cmd_vel_topic`（默认 `cmd_vel`），只应在单独加载 `stair_climb_tree.xml` / `stair_descend_tree.xml` 且停用其它运动命令权威时运行；任何命令拒绝、服务等待超时或激光事件等待超时都会发布零速并返回 `FAILURE`，`onHalted()` 只发布零速，不额外补偿推杆状态。

台阶参数以 `stair_*` 前缀集中在 `rc26_bringup/config/r2_runtime.yaml`，启动时由 `loadStairParams()` 写入黑板 `stair_params`；当前没有运行期参数变更回调。上台阶零速等待参数为 `stair_climb_front_extend_delay_s`（默认 `2.0s`）与 `stair_climb_retract_rear_extend_delay_s`（默认 `2.5s`）；下台阶零速等待参数为 `stair_descend_rear_extend_delay_s`（默认 `2.5s`）、`stair_descend_retract_front_extend_delay_s`（默认 `2.5s`）与 `stair_descend_front_retract_delay_s`（默认 `2.5s`）。下台阶前推杆收回前的定时负向行驶由 `stair_descend_front_retract_drive_speed_mps`（默认 `0.025m/s`，按绝对值读取）和 `stair_descend_front_retract_drive_duration_s`（默认 `4.0s`）控制。这些延时和定时行驶时长小于 0 时都会按 0 处理，等待期间持续发布零速，定时行驶期间持续发布带符号 `x` 负向速度。

## 红方中间列连续台阶树

`behavior_trees/mf_red_middle_column_tree.xml` 是一棵独立可加载的红方 MF 中间列连续运动树，默认 `main_tree.xml`、`mf_tree.xml` 与 `mc_tree.xml` 都不引用它。运行它需要通过 `decision_node` 的 `tree_file` 显式指向该 XML，或者临时把完整 bringup 的 `r2_runtime.paths.behavior_tree_file` 改成该 XML 的绝对路径并设置 `team=red`。

- 路线固定为红方中间列 `grid2 -> grid5 -> grid8 -> grid11`：入口前定位点到第一排第二列 200mm 台阶，随后两段上台阶到 400mm、600mm，再以后轮在前的姿态下到第四排第二列 400mm，最后导航到现有 MF 中列出口点。
- 本树只复用 `NavToPose`、`StairClimb` 与 `StairDescend`，用 XML `Script` 记录 `current_grid` 等黑板状态；不调用 `ScanSurroundings`、`SelectNextGrid`、`GrabKFS`，也不执行视觉夹取。
- 运行时同时依赖 Nav2 `/navigate_to_pose`、共享机构 `/mechanism/send_command`、`/mechanism/command_feedback` 与台阶动作参数 `stair_*`。由于 `StairClimb` / `StairDescend` 会直接发布 `cmd_vel`，执行该树时必须确保 Nav2 controller、遥控或其它测试节点不会同时发布运动命令。

## 当前边界

- 负责流程编排、目标选择和策略切换
- 不直接做底层控制求解
- 不拥有 Nav2 planner/controller 的内部配置
- 不订阅 `base_ground/*`、terrain GridMap、keepout heartbeat，也不调用 KFS keepout runtime service

## 本轮收口

- 删除旧导航 BT 节点源码，新增 `bt_nav2_pose.cpp/.hpp`
- `rc26_decision` 增加 `nav2_msgs` 依赖
- `main_tree.xml` 改为 include `mf_tree.xml`
- `mf_tree.xml` 中梅林区目标点固化为 Nav2 pose，并为 `target_grid` 建立显式分支
- 删除全部第一方 BT 运行时 topic/service、手动调试控制面和中文本地化链，仅保留内部行为树执行
- 本轮移除 `keepout_runtime` 与 `merlin_rule_world_model` 源码/构建目标，删除 base-ground 订阅和 `/mf_kfs_state` 发布，使决策包不再消费或生产已归档三包的数据

## 2026-06-12 更新

- `mc_target_labels` 修正为 `["JK"]`：`tip_default` 模型 profile（`tip.onnx`）的标签表中目前只有 `JK` 这一个类别，原先的 `["D_0", "D_1"]` 无法匹配任何检测结果，导致视觉伺服永远找不到目标。修改后 `resolveTargetClassIds()` 能正确映射到模型输出的类别 ID。
- `src/mc/visual_servo_grab.cpp`、`src/mc/rotate_in_place.cpp`、`src/mc/wait_forever.cpp` 全部补加了 `onStart`/`onRunning`/`onHalted` 的中文注释，说明每个阶段的具体职责：资源初始化、每 tick 轮询逻辑、以及外部中断时的安全停机清理。
- `src/stair/*.cpp` 与独立 `stair_climb_tree.xml` / `stair_descend_tree.xml` 已补加中文逐段注释，说明推杆命令、激光突变事件、速度发布、超时失败和 halt 零速的每一步决策逻辑；本次只增加注释，不改变台阶动作运行语义。
- MC 参数清理为当前真实语义：视觉目标选择只保留 `mc_target_labels`，夹取只保留命令、服务和完成/超时判定参数；旋转速度参数统一为 `mc_rotate_speed_radps`，按 `rad/s` 直接发布到 `cmd_vel.angular.z`。
- 端头模型 profile ID 已从历史测试命名 `tip_test` 改为 `tip_default`；MC 决策默认 `mc_model_id` 同步使用 `tip_default`，模型文件仍由 `rc26_vision/config/vision_models.yaml` 指向 `models/tip.onnx`。
- 2026-06-13 同步：删除包内 `config/decision_params.yaml` 与独立 `decision.launch.py` 入口；决策参数与行为树入口统一由完整 bringup 从 `rc26_bringup/config/r2_runtime.yaml` 读取，测试口径改为验证所有相关节点拉起后的链路效果。
