# rc26_bringup

## 模块定位

`rc26_bringup` 是 R2 整车链路的统一装配入口，负责选择 launch、参数、命名空间、启动顺序和可选外设。它不拥有导航、里程计、视觉、机构或底盘 transport 的算法真源。

## 当前装配口径

[bringup.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/bringup.launch.py) 默认 `run_mode:=navigation`，当前导航模式只装配以下链路：

- `rc26_mcu_transport`：默认消费 `/cmd_vel` 并下发 `POSE_TARGET(0x0C)`，同时提供 `/mechanism/send_command` 与 `/mechanism/command_feedback`。
- [odometry.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/odometry.launch.py)：启动 Point-LIO、`rc26_odom_interface` 和必要 TF/odom 输出；导航模式传入 `start_sensor_scan:=false`，并显式传入 `odom_interface_publish_bootstrap_pose:=false`，避免决策启动 gate 把 bootstrap 零位姿 `/odom` 当成真实里程计后直接运动。
- `rc26_decision`：加载当前红/蓝运行配置中 `r2_runtime.paths.behavior_tree_file` 指向的行为树，并在导航模式强制启用 startup odom gate。未显式传入 `runtime_config_file` 时，`bringup.launch.py` 会按 `r2_active_side.yaml` 的 `preselection_mode` 覆盖默认树：`first` 使用 `mc_mf_preselection_tree.xml`，`second` 使用 `second_preselection_combo_tree.xml`。
- RealSense D455：仅当 `use_realsense:=true` 时启动，用于视觉任务，不属于导航必需节点。

导航模式不装配地图定位、外部地图规划链、代价图、路径规划/控制平滑链或 `rc26_sensor_scan`。`/cmd_vel` 的发布权威在决策侧，默认消费方在 `rc26_mcu_transport`；同一时刻不得再启动遥控、测试动作或其它速度发布者。导航模式的 `/odom` 启动 gate 只应由真实 Point-LIO 经 `rc26_odom_interface` 接管后的输出放行；若真实 `/odom` 未接管，决策应等待或超时失败，不应靠 bootstrap `/odom` 开始闭环运动。

`run_mode:=mapping` 仍用于建图/定位相关联调，可按需要启动 `localization.launch.py`。这条链路不是默认导航闭包，不改变导航模式的最小装配边界。

## 关键入口

- [start_r2_auto.sh](/home/potato/RC_2026/start_r2_auto.sh)：根目录自动决策/比赛链路快捷入口，默认读取 `r2_active_side.yaml` 并以 `use_realsense:=true` 启动完整导航决策链。
- [launch/bringup.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/bringup.launch.py)：整车导航/建图统一入口。
- [launch/odometry.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/odometry.launch.py)：Point-LIO、里程计接口、静态外参和可选 sensor scan 装配。
- [launch/grid_heading.launch.py](/home/potato/RC_2026/src/rc26_bringup/launch/grid_heading.launch.py)：独立 yaw heading 校准入口，只启动 odom、MCU transport 和 `grid_heading_tree.xml`。
- 独立 odom 单轴右转分段入口：保留为包内验证入口，只启动 odom、MCU transport 和独立右转验证树。
- [config/r2_active_side.yaml](/home/potato/RC_2026/src/rc26_bringup/config/r2_active_side.yaml)：默认红蓝方与 first/second 预选入口选择入口，`active_side` 指向 `r2_red.yaml` 或 `r2_blue.yaml`，`preselection_mode` 选择 managed 默认树。
- [config/r2_red.yaml](/home/potato/RC_2026/src/rc26_bringup/config/r2_red.yaml) / [config/r2_blue.yaml](/home/potato/RC_2026/src/rc26_bringup/config/r2_blue.yaml)：红/蓝双方独立完整运行配置，维护点云路径、行为树路径、MCU transport 和决策参数。
- [rviz/navigation_default.rviz](/home/potato/RC_2026/src/rc26_bringup/rviz/navigation_default.rviz)：只用于观察 odom/TF 的轻量预设。

## 红蓝配置口径

默认 `bringup.launch.py` 在未显式传入 `runtime_config_file` 时读取 `r2_active_side.yaml`，按 `active_side: red|blue` 选择 `r2_red.yaml` 或 `r2_blue.yaml`，并按 `preselection_mode: first|second` 覆盖默认行为树。现场切换比赛方或 first/second 入口优先改 `r2_active_side.yaml`；如需临时调试其它完整配置，仍可传入 `runtime_config_file:=/abs/path.yaml` 覆盖，显式配置不会再被 `preselection_mode` 改写。

每个红/蓝运行配置的 `r2_runtime.paths` 当前只维护：

- `prior_pcd_file`
- `behavior_tree_file`

`r2_runtime.mcu_transport` 维护目标 MCU 串口、底盘 `/cmd_vel` consumer 和发送限幅参数。任何真实运动或机构动作链都必须确保该 provider 已启动，除非现场明确由其它同等 provider 接管。

`r2_runtime.decision.ros__parameters` 中与导航直接相关的参数包括：

- `startup_odom_*`：完整导航链创建行为树前等待 `/odom` 新鲜且低速稳定。
- `odom_relative_nav_*`：`OdomDriveX`、`OdomDriveY`、`OdomTurnToYaw` 共享 topic、速度、增益、容差、稳定 tick 和超时。
- `team`：红蓝方场地镜像选择；`r2_red.yaml` 固定 `red`，`r2_blue.yaml` 固定 `blue`。路线数值仍按红方基准维护，`team:=blue` 时由 `rc26_decision` 启动加载阶段派生蓝方 Y/yaw 镜像值。bringup 只负责选择配置和传参，不在 launch 中承载红蓝方路线逻辑。
- `mc_nav_forward_x_m`、`mc_nav_right_turn_delta_rad`、`mc_nav_reverse_x_m`、`mc_nav_timeout_sec`：MC 去程红方基准动作顺序，默认 `+X 0.2m -> 右转 90° -> -X 0.6m`；蓝方自动镜像侧向 yaw 和原地旋转方向，X 距离不变。
- `mf_preselect_entry2_nav_segment1_x_m`、`mf_preselect_entry2_nav_segment1_y_m`、`mf_preselect_entry2_nav_timeout_sec`：MF 预选 2 号入口红方基准单轴段，默认 `+X 2.0m -> -Y 1.8m`，不在入口树前额外转向；蓝方自动镜像 Y。
- `mf_preselect_kfs_align_target_line_offset_px`：MF KFS 视觉横移对齐时，识别框中线要对齐的目标线相对图像中心线的像素偏置；默认 `0`，负值表示目标线向图像左侧移动。
- `mf_preselect_kfs_depth_roi_size`、`mf_preselect_kfs_depth_min_valid_count`、`mf_preselect_kfs_depth_bbox_sample_ratios`、`mf_preselect_kfs_depth_bbox_min_success_count`：MF R2 KFS 有效深度点配置，分别控制单点 ROI 边长、单 ROI 最少有效点、bbox 多点采样比例和最少成功采样点数；默认值写在 `r2_red.yaml` / `r2_blue.yaml`，bringup 只负责传参。
- `preselection_entry_continue_delay_msec`、`preselection_after_mc_continue_delay_msec`：first managed gate 的 0x06 分支延时参数。入口 gate 仍在握手完成后延时进入 MC；MC 末尾 gate 会在收到 0x06 后先按 `preselection_after_mc_continue_delay_msec` 等待，再下发 `COMPETITION_START(0x10)`。
- `preselection_ramp_approach_x_m`、`preselection_ramp_climb_x_m`、`preselection_ramp_max_speed_mps`、`preselection_ramp_min_speed_mps`、`preselection_ramp_timeout_s`：second managed 斜坡两段 odom 前进参数。
- `second_preselect_after_ramp_turn_delta_rad`、`second_preselect_after_ramp_turn_timeout_s`：second managed 斜坡后 90° 转向参数；红方默认 `-1.5708`，`team=blue` 时由 `rc26_decision` 参数加载阶段镜像为 `+1.5708`。
- `second_preselect_pickup_command_id`、`second_preselect_search_*`、`second_preselect_r2_target_*`、`second_preselect_r1_*`、`second_preselect_kfs_*`、`second_preselect_grab_verify_*`、`second_preselect_grab_settle_s`：第二预选赛搜索夹取链参数。当前树内 `0x12` 用作 KFS 夹取触发，ACK 后由视觉消失验证确认夹取。
- `second_preselect_nav_y1_m`、`second_preselect_nav_x2_m`、`second_preselect_place_forward_x_m`、`second_preselect_retreat_x_m`：第二预选赛夹取成功后的放置导航段。夹取确认后默认按红方基准 `+Y 0.7m -> +X 2.5m` 到九宫格观察位，blue 运行时由 `rc26_decision` 自动镜像 Y 段。
- `mf_preselection_external_trigger_*`：历史全局 MCU 上行 `MF_PRESELECTION_TRIGGER(0x10)` 触发参数。managed first/second 模式下，bringup 会强制 `mf_preselection_external_trigger_enable=false`，避免旧监听绕过 `WaitPreselectionBranchGate`；0x10 在 managed 模式中只表示第二限位开关事件，具体握手由当前树的 gate profile 决定。
- `second_preselect_grid_label_prefixes` / `second_preselect_grid_label_exact_names`：第二预选赛动态 ROI 的可选标签过滤列表。红/蓝运行配置默认省略这两个键，由 `rc26_decision` 使用空过滤列表，表示所有非空 `class_name` 有效；不要在 launch 运行配置中写 `[]`，空数组经 Python dict 传给 ROS2 launch 时没有元素类型，会在创建 `decision_node` 前触发参数类型异常。
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
- `rc26_odom_interface` 仍是 `/odom` 与动态基座 TF 的当前运行时来源；`rc26_merge_odom` 不属于默认装配。
- `/cmd_vel` 默认由 `rc26_decision` 的导航/动作节点串行发布，由 `rc26_mcu_transport` 消费；运行遥控或其它测试入口时必须显式保证命令权威唯一。

## 本轮同步

2026-07-04 同步：红/蓝运行配置跟随第二预选赛新流程补齐搜索夹取链参数。`0x12` 现在在第二预选赛内作为 KFS 夹取触发命令使用，ACK 后由 `rc26_decision` 做视觉消失验证；旧 `second_preselect_arm_high_raise_*`、`second_preselect_nav_x1_m` 与放置前 KFS 必见 gate 已从运行配置中删除。夹取成功后的导航参数改为红方基准 `second_preselect_nav_y1_m: 0.7` 与 `second_preselect_nav_x2_m: 2.5`，未观察到前方 KFS 时继续按空位逻辑放置。

2026-07-04 同步：红/蓝运行配置完成统一，除 `r2_blue.yaml` 保留现场标定值 `mc_nav_forward_x_m: 0.98` 和 `team: blue` 外，其余参数值、顺序与注释均同步红方基准；历史单文件 `r2_runtime.yaml` 已删除，默认和调试入口都应显式使用 `r2_red.yaml` / `r2_blue.yaml` 或其它完整自定义配置。

2026-07-03 同步：红/蓝运行配置不再显式写入 `second_preselect_grid_label_prefixes: []` 和 `second_preselect_grid_label_exact_names: []`。这两个第二预选赛标签过滤参数在默认不过滤时应省略，由 `rc26_decision` 的空 vector 默认值表达“所有非空 class_name 有效”；这样避免 ROS2 launch 通过 Python dict 传空数组时无法推断数组元素类型，并在延时启动 `decision_node` 前抛出空 tuple 参数异常。

2026-07-03 同步：红/蓝运行配置新增 `mf_preselect_kfs_depth_roi_size`、`mf_preselect_kfs_depth_min_valid_count`、`mf_preselect_kfs_depth_bbox_sample_ratios`、`mf_preselect_kfs_depth_bbox_min_success_count`。这些参数只配置 `rc26_decision` 的 R2 KFS 深度有效点判定，不把视觉算法逻辑放入 bringup。

2026-07-03 同步：`r2_active_side.yaml` 新增 `preselection_mode: first|second`。未显式传入 `runtime_config_file` 时，bringup 按该模式覆盖默认树：first 为 `mc_mf_preselection_tree.xml`，second 为 `second_preselection_combo_tree.xml`；managed 模式同时强制 `mf_preselection_external_trigger_enable=false`，由 `WaitPreselectionBranchGate` 统一处理 0x06/0x10 分支。first 决策族中两条分支都使用 0x10/0x0C 握手；second 决策族中两条分支都使用 0x11/0x0D 握手。红/蓝运行配置新增 first gate 延时、second 斜坡前进和斜坡后转向参数；正式 MC 末尾流程不再依赖视觉配准 gate。

2026-07-02 同步：新增根目录 `start_r2_auto.sh` 作为自动决策/比赛链路快捷入口。脚本只封装 `ros2 launch rc26_bringup bringup.launch.py run_mode:=navigation`，默认读取 `r2_active_side.yaml`、打印当前红/蓝方和选中的运行配置，并默认传入 `use_realsense:=true`；红蓝方路线、行为树、MCU transport 与决策参数仍由 `rc26_bringup` 和对应运行配置负责。

2026-07-02 同步：红/蓝运行配置新增 `mf_preselection_external_trigger_*` 参数。当前 managed first/second 入口默认禁用该旧全局监听，0x10 由 branch gate 作为第二限位开关事件消费；first 使用 0x10/0x0C 后切梅林树，second 使用 0x11/0x0D 后切对抗区树。

2026-07-02 同步：默认运行配置拆分为 `r2_red.yaml` / `r2_blue.yaml`，由 `r2_active_side.yaml` 选择当前比赛方。显式传入 `runtime_config_file` 仍可覆盖，供临时调试自定义完整配置使用。

2026-07-01 同步：右转导航独立入口专用参数族已从运行配置中删除，并且 `rc26_decision` 不再声明、读取或写入这些参数。独立右转验证入口若继续保留，启动 odom gate 改用通用 startup / odom 相对导航参数或固定默认值。

2026-07-01 同步：`team` 参数现在由 bringup 传入 `rc26_decision` 后作为红蓝方场地镜像选择使用。运行配置中 MC/MF 路线值继续按红方基准维护；`team:=blue` 时，决策节点在启动加载参数阶段派生蓝方入口 Y、侧向 yaw、入口 1/3 号横移和假 KFS 侧列绕行等镜像行为。bringup 未新增第二套 XML，现场切蓝方优先切换 `r2_active_side.yaml`。

2026-07-01 同步：运行配置新增 `mf_preselect_kfs_align_target_line_offset_px`，用于现场标定 MF KFS 夹取时识别框中线的目标线。默认 `0` 保持图像中心线口径；实车若夹爪肉眼已对齐但日志 offset 仍为负，可按该负值附近配置偏置，让决策的 offset 以新的目标线为 0。

2026-06-30 同步：完整 `bringup.launch.py run_mode:=navigation` 启动 odometry 时显式传入 `odom_interface_publish_bootstrap_pose:=false`，`grid_heading.launch.py` 作为独立运动入口同样关闭 bootstrap `/odom`。决策启动 gate 现在不会再被 `rc26_odom_interface` 启动占位零位姿放行；若真实 Point-LIO `/odom` 未接管，导航会继续等待并按启动 gate 超时失败，而不是进入 `OdomDriveX` 后持续下发前进速度。

2026-06-30 同步：默认导航装配切为 odom-only 决策闭环链路。`bringup.launch.py run_mode:=navigation` 只启动 MCU transport、odometry、decision 和按需 RealSense；导航模式关闭 sensor scan，并移除旧地图规划链路参数、配置、资产和测试入口。运行配置维护 odom 单轴分段导航参数和 startup odom gate；MC 默认路线为 `+X 0.2m -> 右转 90° -> -X 0.6m`，MF 预选入口默认路线为 `+X 2.0m -> -Y 1.8m`。
