# rc26_merge_odom 姿态融合与位姿下发模块

本模块是 RC 2026 R2 全自动机器人的核心里程计与位姿下发组件。它负责处理底盘轮速计、外部 CAN 里程计以及高精度 IMU 数据，通过多源传感器的软融合，为上层导航系统提供平滑、高频率、高可靠性的局部里程计估计，同时负责将上层规划的控制指令进行安全限幅与平滑处理后下发给底盘执行机构。

## 核心功能

### 1. 多源里程计软融合
摒弃了传统的非此即彼的硬切换逻辑，采用健康度感知的软融合架构。模块能实时订阅轮式里程计与 CAN 独立里程计数据，并结合 IMU 数据进行一致性校验。当任意一路里程计出现超时、丢帧或与实际机体运动不符（如车轮打滑）时，系统会自动评估其健康度并降低权重。即便某路传感器发生断联故障，系统也能无缝过渡到另一路有效数据，保证下游滤波器输入不中断。

### 2. 环境适应性协方差膨胀
针对实战场地中常见的打滑现象，模块内嵌了自适应滑移检测。通过比对轮速积分的角速度与 IMU 实际测量值，动态计算滑移分数。在检测到打滑时，临时膨胀对应传感器的协方差矩阵，降低其对全局状态估计的影响；在恢复抓地力后，协方差将按指数规律平滑恢复，确保估计结果的连续性。

### 3. 多级 IMU 异常保护
实战环境中强烈的物理冲撞极易导致 IMU 数据产生尖峰毛刺，从而引发底盘失控。模块在输入端设计了基于统计学的中值滤波，有效剔除极短时间内的偶发毛刺而不损伤高动态响应能力。在控制端，进一步引入马氏距离检验与轮速一致性判别：当 IMU 异常突变且与轮速加速度严重不符时，判定为受击干扰，此时系统将对输出速度进行软衰减以维持稳定，避免触发无端的剧烈急刹。

### 4. 预测控制与安全包络
控制指令下发端集成了轻量级的预测控制逻辑，彻底取代了粗暴的单轴限幅。系统不仅在独立坐标轴上施加加速度约束，更引入了严格的二维模长约束机制。当指令可能超出底盘物理极值时，模块会自动进行等比例缩放，使得机器人在任意方向上的合成速度与加速度均严格受控于安全包络面内。这保证了即使在极端摇杆输入或上层规划器突变的情况下，底盘依然能展现出柔和且符合运动学物理极限的响应。

## 系统架构与节点职责

- **融合下发节点**：作为系统主循环，负责掌控整个模块的节拍，维持与底层执行机构的串口通信，并实时派发经过处理后的底盘指令。
- **CAN 里程计解析节点**：专注于解析独立于主驱动之外的副板 CAN 里程计数据流，进行独立的误差模型评估。
- **IMU 预处理节点**：专职订阅高精度达妙陀螺仪数据流，负责初步的去噪清洗与物理单位换算。
- **软融合仲裁节点**：居中协调，持续发布链路的诊断状态报告，输出加权融合后的最优基础里程计供给后端的高级卡尔曼滤波器。

## 调试与运维

为了便于现场快速排障，本模块提供了全面的 ROS2 参数化配置能力，并配套了详尽的诊断话题。根目录集中式调试文档已删除；实机指令级排查、参数效果验证和异常故障复现说明后续应维护在本 README、launch 参数说明或包内脚本中。

## 当前运行时补充

- R2 主运行时已经固定为 `mecanum_4wheel`，不再保留履带/差速 fallback；`ODOM_DATA` 只接受四轮 `16B / 4 float`：`<v_fl, v_rl, v_rr, v_fr>`。
- `WheelOdom`、`CanOdom`、`WheelOdomFuser` 与 `PoseSender` 都会保留真实 `vy`；`/pose_sender/target_protected` 与 `POSE_TARGET/POSE_FEEDBACK` 不再把 `linear.y` 清零。
- `wheel_odom_node` 的独立几何参数已经与主链统一为 `wheel_base=0.62326`、`track_width=0.7`，避免 standalone 调试与总装链口径不一致。
- `merge_odom.launch.py` 与 `merge_odom_fused.launch.py` 已移除 `chassis_model`、`wheel_feedback_format`、`left_motor_can_id/right_motor_can_id`、`track_speed_max_mps`、`track_accel_max_mps2` 等履带专用参数。
- `merge_odom.launch.py` 当前支持 `start_imu` 与 `stats_log_enable`：
  - `start_imu=false` 时不会启动 `dm_imu_node`
  - `imu_topic` 会被置空，`WheelOdom`、`CanOdom`、`PoseSender` 都不会再创建 IMU 订阅
  - `slip_enable`、`imu_gate_enable`、`latency_comp_enable` 会一起关闭
- 当前默认 MCU 口径已经临时收口为单口：
  - `target_serial_port` 默认是 `/dev/ttyUSB0`
  - `feedback_serial_port` 默认是 `__disabled__`
  - `/mechanism/send_command`、`/mechanism/command_feedback` 与 `POSE_TARGET` 都默认走这条 `ttyUSB0` 链
- `feedback_serial_port` 仍允许显式传真实设备，作为保留中的旧反馈链入口：
  - 若反馈串口不可用或保持默认禁用，`merge_odom_node` 会跳过 `WheelOdom`
  - 只有显式重新接回反馈串口时，`ODOM_DATA` / `POSE_FEEDBACK` 才会回到默认运行时闭环
- `pose_sender_node` 现在也会在最小 MCU 链里挂出共享 mechanism 命令接口，因此 `./start_r2_teleop.sh --stack minimal-mcu` 仍可承接 ACK 型机构命令
- 当前双推杆协议已经收口为 `FRONT_PUSHROD_EXTEND/RETRACT(0x0E/0x0F)` 与 `REAR_PUSHROD_EXTEND/RETRACT(0x10/0x11)`，统一复用共享 transport 走 ACK 路径；若 MCU 额外上送 `0x13~0x16` 业务 ACK，桥接层会继续发布到 `/mechanism/command_feedback`
- EKF 启动前会先对参数里的科学计数法字符串做归一化，避免 `robot_localization` 因数组里混入字符串而在 launch 阶段报错
- 仓库根目录的 `start_r2_teleop.sh` 已是当前唯一正式遥控入口；脚本当前默认 `--stack minimal-mcu`，并固定按 `target_serial_port=/dev/ttyUSB0`、`feedback_serial_port=__disabled__` 启动单口 MCU 链
- 在这套单口默认口径下，`start_r2_teleop.sh --stack full` 仍可保留 `merge_odom` 装配，但会直接拒绝 `--pose-mode` 与 `--start-ekf`
