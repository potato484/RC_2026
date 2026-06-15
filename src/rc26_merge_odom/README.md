# rc26_merge_odom 姿态融合与位姿下发模块

本模块是 RC 2026 R2 全自动机器人的核心里程计与位姿下发组件。当前实机合理启动口径以 WheelOdom 为底盘局部反馈源，默认通过目标 MCU 单口接收 `ODOM_DATA` 并下发控制；CAN 里程计代码仍保留为非默认调试能力，但不属于当前主链。模块同时负责将上层规划的控制指令进行安全限幅与平滑处理后下发给底盘执行机构。

## 核心功能

### 1. WheelOdom 局部反馈
当前实机合理启动口径以目标 MCU 单口 `ODOM_DATA` 为 WheelOdom 输入，并向下游提供稳定 `/merge_odom` 局部反馈。CAN odom 代码仍保留为 `merge_odom_node` 内的非默认单源调试选择，但当前项目不再维护 CAN/Wheel 双路软融合节点，也不再提供双源融合输出链路。

### 2. 环境适应性协方差膨胀
针对实战场地中常见的打滑现象，模块内嵌了自适应滑移检测。通过比对轮速积分的角速度与 IMU 实际测量值，动态计算滑移分数。在检测到打滑时，临时膨胀对应传感器的协方差矩阵，降低其对全局状态估计的影响；在恢复抓地力后，协方差将按指数规律平滑恢复，确保估计结果的连续性。

### 3. 多级 IMU 异常保护
实战环境中强烈的物理冲撞极易导致 IMU 数据产生尖峰毛刺，从而引发底盘失控。模块在输入端设计了基于统计学的中值滤波，有效剔除极短时间内的偶发毛刺而不损伤高动态响应能力。在控制端，进一步引入马氏距离检验与轮速一致性判别：当 IMU 异常突变且与轮速加速度严重不符时，判定为受击干扰，此时系统将对输出速度进行软衰减以维持稳定，避免触发无端的剧烈急刹。

### 4. 预测控制与安全包络
控制指令下发端集成了轻量级的预测控制逻辑，彻底取代了粗暴的单轴限幅。系统不仅在独立坐标轴上施加加速度约束，更引入了严格的二维模长约束机制。当指令可能超出底盘物理极值时，模块会自动进行等比例缩放，使得机器人在任意方向上的合成速度与加速度均严格受控于安全包络面内。这保证了即使在极端摇杆输入或上层规划器突变的情况下，底盘依然能展现出柔和且符合运动学物理极限的响应。

## 系统架构与节点职责

- **融合下发节点**：作为系统主循环，负责掌控整个模块的节拍，维持与底层执行机构的串口通信，并实时派发经过处理后的底盘指令。
- **CAN 里程计解析节点**：保留为非默认调试能力，当前主链不以 CAN odom 作为合理启动口径。
- **IMU 预处理节点**：专职订阅高精度达妙陀螺仪数据流，负责初步的去噪清洗与物理单位换算。

## 调试与运维

为了便于现场快速排障，本模块提供了全面的 ROS2 参数化配置能力，并配套了详尽的诊断话题。根目录集中式调试文档已删除；实机指令级排查、参数效果验证和异常故障复现说明后续应维护在本 README、launch 参数说明或包内脚本中。

## 当前运行时补充

- R2 主运行时已经固定为 `mecanum_4wheel`，不再保留履带/差速 fallback；`ODOM_DATA` 只接受四轮 `16B / 4 float`：`<v_fl, v_rl, v_rr, v_fr>`。
- `WheelOdom` 与 `PoseSender` 都会保留真实 `vy`；`/pose_sender/target_protected` 与 `POSE_TARGET/POSE_FEEDBACK` 不再把 `linear.y` 清零。
- `wheel_odom_node` 的独立几何参数已经与主链统一为 `wheel_base=0.62326`、`track_width=0.7`，避免 standalone 调试与总装链口径不一致。
- `merge_odom.launch.py` 已移除 `chassis_model`、`wheel_feedback_format`、`left_motor_can_id/right_motor_can_id`、`track_speed_max_mps`、`track_accel_max_mps2` 等履带专用参数。
- 旧 CAN/Wheel 双路软融合链路已删除；当前项目不再保留双源融合入口，也不再提供双源融合输出话题。
- `merge_odom.launch.py` 当前支持稳定 `/merge_odom` 输出契约：
  - `merge_odom_output_topic` 默认 `merge_odom`
  - `require_merge_odom_output` 默认 `true`
  - `start_ekf=true` 时，raw WheelOdom 作为 EKF 输入，EKF 输出 remap 到 `merge_odom_output_topic`
  - EKF 参数显式 `publish_tf=false`，只输出 `/merge_odom` Odometry，不发布 `odom -> base_link` 动态 TF
  - `start_ekf=false` 时，选中的 raw WheelOdom 源直接发布到 `merge_odom_output_topic`
  - 没有真实 odom 源时启动失败，不发布零 odom 或伪造数据
- `merge_odom.launch.py` 当前支持 `start_imu` 与 `stats_log_enable`：
  - `start_imu=false` 时不会启动 `dm_imu_node`
  - `imu_topic` 会被置空，`WheelOdom`、`PoseSender` 都不会再创建 IMU 订阅
  - `slip_enable`、`imu_gate_enable`、`latency_comp_enable` 会一起关闭
- 当前默认 MCU 口径已经临时收口为单口：
  - `target_serial_port` 默认是 `/dev/ttyUSB0`
  - `feedback_serial_port` 默认是 `__disabled__`
  - `/mechanism/send_command`、`/mechanism/command_feedback`、`POSE_TARGET` 与 `ODOM_DATA` 都默认走这条单口链
  - `merge_odom_node` 会把 `ODOM_DATA` 分发给 WheelOdom，把其它业务机构反馈继续交给 mechanism transport
- `feedback_serial_port` 仍允许显式传真实设备，作为保留的独立反馈口参数，但不属于当前默认链路；当前启动说明不应把独立反馈口写成实机必选步骤
- `pose_sender_node` 现在也会在最小 MCU 链里挂出共享 mechanism 命令接口，因此 `./start_r2_teleop.sh --stack minimal-mcu` 仍可承接 ACK 型机构命令
- 当前双推杆协议已经收口为 `FRONT_PUSHROD_EXTEND/RETRACT(0x0E/0x0F)` 与 `REAR_PUSHROD_EXTEND/RETRACT(0x10/0x11)`，统一复用共享 transport 走 ACK 路径；若 MCU 额外上送 `0x13~0x16` 业务 ACK，桥接层会继续发布到 `/mechanism/command_feedback`
- 台阶激光测距事件 `FRONT_LASER_HEIGHT_JUMP(0x17)` 与 `REAR_LASER_HEIGHT_JUMP(0x18)` 也属于普通业务 feedback，桥接层不会过滤，会原样发布到 `/mechanism/command_feedback` 供独立台阶 BT 动作消费
- EKF 启动前会先对参数里的科学计数法字符串做归一化，避免 `robot_localization` 因数组里混入字符串而在 launch 阶段报错
- 仓库根目录的 `start_r2_teleop.sh` 已是当前唯一正式遥控入口；脚本当前默认 `--stack minimal-mcu`，并固定按 `target_serial_port=/dev/ttyUSB0`、`feedback_serial_port=__disabled__` 启动单口 MCU 链
- 在这套单口默认口径下，`start_r2_teleop.sh --stack full` 仍可保留 `merge_odom` 装配，并显式传 `require_merge_odom_output:=false` 表示遥控调试降级入口；可通过 `--start-ekf` 或 `--pose-mode imu|no-imu|wheel-only` 临时打开/关闭 EKF 与 IMU 链路
- 2026-06-12 同步：完整决策/导航链的合理底盘反馈口径为 `/dev/ttyUSB0` 单口 WheelOdom；CAN odom 和独立 feedback 串口均不作为当前主链启动建议。
