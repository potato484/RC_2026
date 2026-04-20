# rc26_merge_odom

## 模块定位

`rc26_merge_odom` 是 R2 当前的多源里程计融合、位姿下发和目标 MCU 串口桥接包，负责把轮速、CAN 里程计、达妙 IMU、底盘控制下发与机构共享串口整合到同一条运行链路里。

## 当前实现

这个包不是单节点实现，而是一个多库、多可执行拼装的子系统：

- `can_odom` + `can_odom_node`
- `wheel_odom` + `wheel_odom_node`
- `dm_imu_driver` + `dm_imu_node`
- `wheel_odom_fuser` + `wheel_odom_fuser_node`
- `pose_sender` + `pose_sender_node`
- `merge_odom_node`
- 调试节点：`single_wheel_test_node`

源码目录已经按功能拆开：

- `src/can/`：CAN 里程计解析
- `src/wheel/`：轮式里程计
- `src/imu/`：达妙 IMU 驱动与预处理
- `src/fuser/`：多源里程计软融合
- `src/pose/`：位姿/速度下发
- `src/merge_odom_node.cpp`：总装与统一启动入口

关键配置与启动文件：

- `config/merge_odom_params.yaml`
- `config/ekf_params.yaml`
- `config/pose_sender_cmd_vel_teleop.yaml`
- `launch/merge_odom.launch.py`
- `launch/merge_odom_fused.launch.py`
- `launch/can_odom_only.launch.py`
- `launch/wheel_odom_only.launch.py`
- `launch/dm_imu_only.launch.py`

除了底盘相关职责外，`merge_odom_node` 当前还维护一套固定的双串口职责口径；现场排查时不要只按 `ttyUSB0/1` 编号硬记，优先按“这条线负责什么”来记：

- `feedback_serial_port`，当前默认 `/dev/ttyUSB0`
  - `WheelOdom` 从这条链路接收 `ODOM_DATA`
  - `PoseSender` 同时在这条链路上按 `50Hz` 下发 `POSE_FEEDBACK(0x1E)`
  - 这条链路不承载 `rc26_mechanism` 或 teleop 的 transport 命令
  - 当现场只有目标 MCU 下发串口时，这个参数允许显式置为 `__disabled__`，节点会跳过 WheelOdom / POSE_FEEDBACK 并进入目标串口单链路降级模式
- `target_serial_port`，当前默认 `/dev/ttyUSB1`
  - `PoseSender` 复用这条链路下发 `POSE_TARGET(0x1F)`
  - `MechanismTransportBridge` 继续作为共享桥接接口，供 `rc26_mechanism` 与 teleop 前置履带 / 推杆 sidecar 复用同一串口
  - service：`/mechanism/transport/send_command`
  - topic：`/mechanism/transport/feedback`
  - `pose_sender_node` 现在也会在最小 MCU 栈里挂出这组共享 transport 接口，因此 `minimal-mcu` 不再只有速度下发
  - 真机上只有 `merge_odom_node` 会真正打开这条物理串口；其它上层只能通过 transport 复用，不能再次直连同一设备

其中桥接层会过滤 `ACK / HEARTBEAT_ACK / ODOM_DATA` 这类高频非业务反馈，只把业务侧真正关心的反馈继续发布出去；`FRONT_TRACK_UP/DOWN` 在桥接层会直接走 no-ACK 单发，`PUSHROD_EXTEND/RETRACT` 与其它普通机制命令仍保留 ACK 路径。

`launch/merge_odom.launch.py` 当前除了 `use_can_odom` / `start_ekf` 外，还支持 `use_imu_for_ekf`、`start_imu` 与 `stats_log_enable`：

- `use_imu_for_ekf=true`：EKF 按默认口径继续融合 `DM_IMU`
- `use_imu_for_ekf=false`：只把 `imu0` 和 `imu0_*` 从 EKF 参数里移除，最终 `merge_odom` 改为纯 wheel/CAN 输入的 EKF 输出
- `start_imu=true`：启动 `dm_imu_node`，并允许 `WheelOdom`、`CanOdom`、`PoseSender` 订阅 IMU
- `start_imu=false`：不启动 `dm_imu_node`，并把 `imu_topic` 置空，同时关闭 `slip_enable`、`imu_gate_enable` 与 `latency_comp_enable`
- `stats_log_enable=true`：打开 PoseSender 的 1 秒统计日志
- 因此 `use_imu_for_ekf=false` 只影响 EKF 的最终融合位姿；若需要“完全不读 IMU”，还必须把 `start_imu` 关掉

当前实现新增了统一的 `chassis_model` 参数，支持两种底盘口径：

- `mecanum_4wheel`：保留现有四轮全向解算与二维速度保护
- `tracked_diff`：按真实两电机差速底盘运行；串口 `wheel_odom` 接收 `v_left / v_right` 两路浮点速度，`can_odom` 接收左右两个驱动电机反馈，输出 `vx / wz`，运行时固定 `vy=0`

与履带模式直接相关的新配置口径有两项：

- `wheel_feedback_format`：串口 `ODOM_DATA` 的 payload 形态，支持 `legacy_4wheel_16b | tracked_lr_8b`
- `left_motor_can_id / right_motor_can_id`：履带模式下左右驱动电机的 CAN 反馈 ID

当前仓库默认底盘模式已经切到 `tracked_diff`；若要回切四轮，需显式把 `chassis_model` 改回 `mecanum_4wheel`，并把 `wheel_feedback_format` 一并改回 `legacy_4wheel_16b`。

## 源码入口与阅读顺序
- 先看 `launch/merge_odom_fused.launch.py` 和 `launch/merge_odom.launch.py`，理解这个子系统是如何把多个节点拼起来的。
- 再看 `src/merge_odom_node.cpp`，它是总装入口。
- 然后分模块看 `src/can/`、`src/wheel/`、`src/imu/`、`src/fuser/`、`src/pose/`。
- 最后看参数 YAML 和调试脚本，确认串口/CAN/速度保护的部署值。

## 目录解剖
- `src/merge_odom_node.cpp`：统一创建和装配 CAN、轮速、IMU、融合器、位姿下发器，以及机构共享串口桥。
- `src/can/`：CAN 里程计采集与解码。
- `src/wheel/`：轮里程计串口接入和速度解算。
- `src/imu/`：达妙 IMU 驱动与解析。
- `src/fuser/`：CAN/轮里程计软融合。
- `src/pose/`：结合反馈和保护器向下位机发速度。

## 关键文件体量
- `src/pose/pose_sender.cpp`：875 行，下发保护逻辑最重。
- `src/imu/dm_imu_driver.cpp`：454 行。
- `src/can/can_odom.cpp`：475 行。
- `src/fuser/wheel_odom_fuser.cpp`：369 行。
- `src/wheel/wheel_odom.cpp`：380 行。
- `src/merge_odom_node.cpp`：369 行，总装入口。

## 关键源码行段速览
- `src/rc26_merge_odom/src/merge_odom_node.cpp:1-221`：节点组合、参数分发和各子模块 wiring；`222-234`：`main()`。
- `src/rc26_merge_odom/src/can/can_odom.cpp:24-99`：构造和接口初始化；`100-225`：CAN 打开、接收线程和帧解析；`226-388`：里程计发布与状态访问。
- `src/rc26_merge_odom/src/wheel/wheel_odom.cpp:21-134`：串口轮速解包与机体系速度换算；`135-304`：里程计发布、状态获取与复位。
- `src/rc26_merge_odom/src/imu/dm_imu_driver.cpp:107-216`：串口打开与初始化；`247-374`：接收线程和缓冲解析；`375-454`：帧级解析。
- `src/rc26_merge_odom/src/fuser/wheel_odom_fuser.cpp:42-163`：输入缓存和源状态构造；`164-315`：定时融合主路径；`316-353`：健康度发布。
- `src/rc26_merge_odom/src/pose/pose_sender.cpp`：输入订阅和基础状态缓存、IMU spike 与 fallback/governor 保护、反馈和目标发送定时器。

## 模块边界

- 这个包输出的是局部融合里程计和下发保护，不是全局定位
- 它不替代 `rc26_localization` 的地图配准职责
- 它也不做上层路径规划，只为控制和定位提供更稳的底层状态与执行接口
- 它现在拥有目标 MCU 串口的运行时权威，但并不替代 `rc26_mechanism` 的动作语义和 Action 服务职责

## 近期实现说明

- `wheel_odom` 保留老四轮 `16B / 4 float` 串口解析，同时新增 `tracked_lr_8b` 接收格式；履带模式的真实输入现在是 `v_left / v_right` 两个 `float32`。
- `can_odom` 保留老四电机 CAN 解算；履带模式切换为左右两个电机 ID，可在 YAML 里分别配置。
- `wheel_odom_fuser` 在履带模式下继续保留双源融合，但把 `vy` 收敛为非完整约束量。
- `PoseSender` 在履带模式下仍按 `(vx, vy, wz)` 协议下发，但会在保护器里强制 `vy=0`，并改为左右履带空间限速/限加速度。
- `PoseSender` 现在把 `POSE_FEEDBACK(0x1E)` 与 `POSE_TARGET(0x1F)` 都固定为 `50Hz` 连续下发；自动导航链仍可保持 `30Hz /cmd_vel`，由 PoseSender 在串口侧重发最近一次目标速度。
- `merge_odom_node` 现在额外挂出 `/mechanism/transport/send_command` 与 `/mechanism/transport/feedback`，把机构命令和前置履带遥控命令都复用到同一条目标串口上。
- `pose_sender_node` 现在也会挂出同一组 `/mechanism/transport/*`，因此 `minimal-mcu` 栈除了速度下发外，也能承接基于 ACK 的共享机构 transport。
- 新增 `PUSHROD_EXTEND(0x10)` / `PUSHROD_RETRACT(0x11)` 两条 ACK 协议；它们和其它普通机构命令一样走可靠 send + ACK，不要求额外 `DONE` 反馈。
- 真实部署下，`rc26_mechanism` 不应再单独打开 `/dev/ttyUSB1`；若 teleop 或 bringup 已经启动 `merge_odom`，则机制侧应使用 `hal_type:=shared_serial`。
- `terrain_speed_limit` 运行时链路已经从 `rc26_merge_odom` 中删除；`PoseSender` 不再消费来自 `rc26_terrain` 的外部限速话题。
- 遥控链现在可以通过仓库根目录的 `start_r2_teleop.sh --pose-mode imu|no-imu|wheel-only` 切换融合口径：
  - `imu`：EKF 融合 IMU
  - `no-imu`：EKF 不融合 IMU，但仍继续读取 IMU 供执行保护链使用
  - `wheel-only`：不启动也不读取 IMU，EKF 只基于 `wheel_odom` 输出最终 `merge_odom`
- `launch/merge_odom.launch.py` 现在会在运行时先规范化 EKF 参数里的科学计数法数值，避免 `wheel-only` / `no-imu` 这类会启用 EKF 的 teleop 路径因 `initial_estimate_covariance` 数组里混入字符串而在 launch 阶段直接报错。
- `merge_odom_node` 现在接受 `feedback_serial_port:=__disabled__` 的降级输入；如果没有独立反馈串口，会跳过 WheelOdom，但继续保留 `POSE_TARGET` 和 `/mechanism/transport/*` 的目标串口链路。
- 仓库根目录的 `start_r2_teleop.sh` 现在还支持 `--stack full|minimal-mcu`：默认 `full` 走完整遥控链，`minimal-mcu` 会拉起 `pose_sender_node + joy_node + rc26_telecontrol + rc26_telecontrol_pushrod_dpad` 的最小串口链。
- `start_r2_teleop.sh` 在 `full` 和 `minimal-mcu` 两个栈下都会自动兼容“只有一个目标串口”的场景：当默认 `target_serial_port=/dev/ttyUSB1` 不存在且 `/dev/ttyUSB0` 存在时，会自动把目标串口切到 `/dev/ttyUSB0`，并把反馈串口降级为 `__disabled__`。
- 遥控链通过 `start_r2_teleop.sh` 启动时，不再需要额外传地形限速相关参数；teleop 模式天然不会受 terrain 限速影响，同时会自动把前置履带单发 transport 的按钮节点一起拉起，不再默认拉起 `rc26_mechanism`。
