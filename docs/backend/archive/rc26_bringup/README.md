# rc26_bringup

## 模块定位

`rc26_bringup` 是 R2 整车链路的统一装配入口，负责选择 launch、参数、命名空间、启动顺序和可选外设。它不拥有导航、里程计、视觉、机构或底盘 transport 的算法真源。

## 当前装配口径

[bringup.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/bringup.launch.py) 默认 `run_mode:=navigation`，当前导航模式只装配以下链路：

- `rc26_mcu_transport`：默认消费 `/cmd_vel` 并下发 `POSE_TARGET(0x0C)`，同时提供 `/mechanism/send_command` 与 `/mechanism/command_feedback`。
- [odometry.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/odometry.launch.py)：启动 Point-LIO、`rc26_odom_interface` 和必要 TF/odom 输出；导航模式传入 `start_sensor_scan:=false`，并显式传入 `odom_interface_publish_bootstrap_pose:=false`，避免决策启动 gate 把 bootstrap 零位姿 `/odom` 当成真实里程计后直接运动。
- `rc26_decision`：加载 `r2_runtime.paths.behavior_tree_file` 指向的行为树，并在导航模式强制启用 startup odom gate。
- RealSense D455：仅当 `use_realsense:=true` 时启动，用于视觉任务，不属于导航必需节点。

导航模式不装配地图定位、外部地图规划链、代价图、路径规划/控制平滑链或 `rc26_sensor_scan`。`/cmd_vel` 的发布权威在决策侧，默认消费方在 `rc26_mcu_transport`；同一时刻不得再启动遥控、测试动作或其它速度发布者。导航模式的 `/odom` 启动 gate 只应由真实 Point-LIO 经 `rc26_odom_interface` 接管后的输出放行；若真实 `/odom` 未接管，决策应等待或超时失败，不应靠 bootstrap `/odom` 开始闭环运动。

`run_mode:=mapping` 仍用于建图/定位相关联调，可按需要启动 `localization.launch.py`。这条链路不是默认导航闭包，不改变导航模式的最小装配边界。

## 关键入口

- [launch/bringup.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/bringup.launch.py)：整车导航/建图统一入口。
- [launch/odometry.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/odometry.launch.py)：Point-LIO、里程计接口、静态外参和可选 sensor scan 装配。
- [launch/grid_heading.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/grid_heading.launch.py)：独立 yaw heading 校准入口，只启动 odom、MCU transport 和 `grid_heading_tree.xml`。
- [launch/odom_right_turn_nav.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/odom_right_turn_nav.launch.py)：独立 odom 单轴右转分段入口，只启动 odom、MCU transport 和 `odom_right_turn_nav_tree.xml`。
- [config/r2_runtime.yaml](/home/potato/RC_2026/src/rc26_bringup/config/r2_runtime.yaml)：点云路径、行为树路径、MCU transport 和决策参数真源。
- [rviz/navigation_default.rviz](/home/potato/RC_2026/src/rc26_bringup/rviz/navigation_default.rviz)：只用于观察 odom/TF 的轻量预设。

## r2_runtime.yaml 口径

`r2_runtime.paths` 当前只维护：

- `prior_pcd_file`
- `behavior_tree_file`

`r2_runtime.mcu_transport` 维护目标 MCU 串口、底盘 `/cmd_vel` consumer 和发送限幅参数。任何真实运动或机构动作链都必须确保该 provider 已启动，除非现场明确由其它同等 provider 接管。

`r2_runtime.decision.ros__parameters` 中与导航直接相关的参数包括：

- `startup_odom_*`：完整导航链创建行为树前等待 `/odom` 新鲜且低速稳定。
- `odom_relative_nav_*`：`OdomDriveX`、`OdomDriveY`、`OdomTurnToYaw` 共享 topic、速度、增益、容差、稳定 tick 和超时。
- `mc_nav_forward_x_m`、`mc_nav_right_turn_delta_rad`、`mc_nav_reverse_x_m`、`mc_nav_timeout_sec`：MC 去程原始动作顺序，默认 `+X 0.2m -> 右转 90° -> -X 0.6m`。
- `mf_preselect_entry2_nav_segment1_x_m`、`mf_preselect_entry2_nav_segment1_y_m`、`mf_preselect_entry2_nav_timeout_sec`：MF 预选 2 号入口单轴段，默认 `+X 2.0m -> -Y 1.8m`，不在入口树前额外转向。
- `odom_right_turn_nav_*`：右转独立入口前进段、相对 yaw 捕获、绝对 yaw 对齐和后退段。

这些参数描述相对分段和 odom yaw 目标生成，不是地图位姿。现场标定时应按启动姿态重新调整每段 `distance_m` 和相对/绝对 yaw。

## 独立入口

`grid_heading.launch.py` 会加载 `grid_heading_tree.xml`，执行 `GridTurn -> GridHeadingAlign`。它用于 MF 格间 heading 能力或单独 yaw 校准，不是通用分段导航转向动作。该入口启动 odometry 时显式关闭 sensor scan。

`grid_heading.launch.py` 与完整导航模式一样，启动 odometry 时显式关闭 `odom_interface` bootstrap `/odom`。该入口会直接发布 `/cmd_vel` 做 yaw 对齐，不能在真实里程计未接管时用占位零位姿放行动作。

`odom_right_turn_nav.launch.py` 会加载 `odom_right_turn_nav_tree.xml`，执行 `OdomDriveX(+forward) -> RelativeYawTarget(delta) -> OdomTurnToYaw -> OdomDriveX(reverse)`。该入口默认关闭 bootstrap odom，先等真实 odom 稳定后再 tick 行为树；启动 odometry 时同样显式关闭 sensor scan。

## RViz 与测试资产

`navigation_default.rviz` 只保留 Grid、TF 和 odom 相关观察能力，不提供外部目标发布工具，不作为状态真源，也不得接管 `/cmd_vel`。

旧地图规划配置、地图资产、外部 action 测试 launch、外部 controller bag 评估脚本和绝对位姿采点脚本已从安装闭包移除。相对分段导航由 `r2_runtime.yaml` 中的显式段参数人工标定。

## 边界

- `rc26_bringup` 只负责装配和参数选择，不承载导航控制算法。
- 导航模式下不启动定位、sensor scan、地图服务、外部规划/控制/平滑链。
- `rc26_odom_interface` 仍是 `/odom` 与动态基座 TF 的当前运行时来源；`rc26_merge_odom` 不属于默认装配。
- `/cmd_vel` 默认由 `rc26_decision` 的导航/动作节点串行发布，由 `rc26_mcu_transport` 消费；运行遥控或其它测试入口时必须显式保证命令权威唯一。

## 本轮同步

2026-06-30 同步：完整 `bringup.launch.py run_mode:=navigation` 启动 odometry 时显式传入 `odom_interface_publish_bootstrap_pose:=false`，`grid_heading.launch.py` 作为独立运动入口同样关闭 bootstrap `/odom`。决策启动 gate 现在不会再被 `rc26_odom_interface` 启动占位零位姿放行；若真实 Point-LIO `/odom` 未接管，导航会继续等待并按启动 gate 超时失败，而不是进入 `OdomDriveX` 后持续下发前进速度。

2026-06-30 同步：默认导航装配切为 odom-only 决策闭环链路。`bringup.launch.py run_mode:=navigation` 只启动 MCU transport、odometry、decision 和按需 RealSense；导航模式关闭 sensor scan，并移除旧地图规划链路参数、配置、资产和测试入口。`r2_runtime.yaml` 维护 odom 单轴分段导航参数和 startup odom gate；MC 默认路线为 `+X 0.2m -> 右转 90° -> -X 0.6m`，MF 预选入口默认路线为 `+X 2.0m -> -Y 1.8m`。
