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

除了底盘相关职责外，`merge_odom_node` 当前还维护一套“保留双口参数、默认单口运行”的串口职责口径；现场排查时不要只按 `ttyUSB0/1` 编号硬记，优先按“这条线负责什么”来记：

- `target_serial_port`，当前默认 `/dev/ttyUSB0`
  - `PoseSender` 默认从这条链路下发 `POSE_TARGET(0x1F)`
  - `MechanismTransportBridge` 继续作为共享桥接接口，供 `rc26_mechanism` 与 teleop 前/后推杆 sidecar 复用同一串口
  - service：`/mechanism/send_command`
  - topic：`/mechanism/command_feedback`
  - `pose_sender_node` 在最小 MCU 栈里也会挂出这组共享 mechanism 命令接口，因此 `minimal-mcu` 仍不只有速度下发
  - 真机上只有 `merge_odom_node` 会真正打开这条物理串口；其它上层只能通过 transport 复用，不能再次直连同一设备
- `feedback_serial_port`，当前默认 `__disabled__`
  - 这是一条保留中的旧反馈链入口；只有显式传入真实设备时，`WheelOdom` 才会重新从这条链路接收 `ODOM_DATA`
  - 同样只有显式启用反馈串口时，`PoseSender` 才会在这条链路上按 `50Hz` 下发 `POSE_FEEDBACK(0x1E)`
  - 当前默认运行时不会再依赖这条链路承载 teleop / mechanism transport

其中桥接层会过滤 `ACK / HEARTBEAT_ACK / ODOM_DATA` 这类高频非业务反馈，只把业务侧真正关心的反馈继续发布出去；当前双推杆 4 条命令都已经切到可靠 ACK 路径，因此若 MCU 不回通用 `ACK(0x00)`，会像其它可靠命令一样自动重传并打印超时日志；MCU 额外上送的 `0x13~0x16` 业务 ACK 会继续透传到 `/mechanism/command_feedback`。

新增机制业务命令时，`MechanismTransportBridge` 默认不需要按命令改代码：只要新反馈 ID 不复用 `ACK / HEARTBEAT_ACK / ODOM_DATA` 这些被过滤的系统反馈，桥接层就会继续按通用 transport 透传。

`launch/merge_odom.launch.py` 当前除了 `use_can_odom` / `start_ekf` 外，还支持 `use_imu_for_ekf`、`start_imu` 与 `stats_log_enable`：

- `use_imu_for_ekf=true`：EKF 按默认口径继续融合 `DM_IMU`
- `use_imu_for_ekf=false`：只把 `imu0` 和 `imu0_*` 从 EKF 参数里移除，最终 `merge_odom` 改为纯 wheel/CAN 输入的 EKF 输出
- `start_imu=true`：启动 `dm_imu_node`，并允许 `WheelOdom`、`CanOdom`、`PoseSender` 订阅 IMU
- `start_imu=false`：不启动 `dm_imu_node`，并把 `imu_topic` 置空，同时关闭 `slip_enable`、`imu_gate_enable` 与 `latency_comp_enable`
- `stats_log_enable=true`：打开 PoseSender 的 1 秒统计日志
- 因此 `use_imu_for_ekf=false` 只影响 EKF 的最终融合位姿；若需要“完全不读 IMU”，还必须把 `start_imu` 关掉

当前运行时已经收口到单一 `mecanum_4wheel` 口径，不再保留履带/差速分支：

- `wheel_odom` 只接收 `ODOM_DATA(0x20)` 的四轮 `16B / 4 float` payload：`<v_fl, v_rl, v_rr, v_fr>`
- `can_odom`、`wheel_odom`、`wheel_odom_fuser`、`pose_sender` 都按麦克纳姆运动学保留真实 `vx / vy / wz`
- `PoseSender` 的保护器继续对 `(vx, vy)` 做二维投影限幅，不再把 `linear.y` 强制清零
- `merge_odom.launch.py` 与 `merge_odom_fused.launch.py` 已移除 `chassis_model`、`wheel_feedback_format`、`left_motor_can_id/right_motor_can_id`、`track_speed_max_mps`、`track_accel_max_mps2` 等履带专用参数
- 独立 `wheel_odom_node` 的几何参数已与主运行时统一为 `wheel_base=0.62326`、`track_width=0.7`

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

- `wheel_odom` 现在只保留麦克纳姆四轮 payload 解析，新增的 `odom_payload.hpp` / `mecanum_kinematics.hpp` 负责统一四轮解包与运动学换算。
- `can_odom` 只保留四轮 CAN 解算路径，输出 `vx / vy / wz`。
- `wheel_odom_fuser` 会继续融合 CAN 与 wheel 两路输入，但不再把 `vy` 收敛为 0。
- `PoseSender` 继续按 `(vx, vy, wz)` 协议下发，并把 `vy` 视为麦克纳姆主链上的有效保护量。
- `PoseSender` 代码仍保留 `POSE_FEEDBACK(0x1E)` 与 `POSE_TARGET(0x1F)` 的 `50Hz` 连续下发能力；但当前默认运行时只启用 `target_serial_port=/dev/ttyUSB0` 这条单口 MCU 链，因此默认只会连续发送 `POSE_TARGET`。
- `merge_odom_node` 现在额外挂出 `/mechanism/send_command` 与 `/mechanism/command_feedback`，把机构命令和前/后推杆遥控命令都复用到同一条目标串口上。
- `pose_sender_node` 现在也会挂出同一组共享 mechanism 命令接口，因此 `minimal-mcu` 栈除了速度下发外，也能承接基于 ACK 的共享机构 transport。
- 双推杆协议已经收口为 `FRONT_PUSHROD_EXTEND/RETRACT(0x0E/0x0F)` 与 `REAR_PUSHROD_EXTEND/RETRACT(0x10/0x11)`；它们和其它普通机构命令一样走可靠 send + 通用 `ACK(0x00)`，MCU 额外发布的 `0x13~0x16` 业务 ACK 会继续透传给 transport feedback。
- 真实部署下，`rc26_mechanism` 不应再单独打开默认目标口 `/dev/ttyUSB0`；若 teleop 或 bringup 已经启动 `merge_odom`，则机制侧应使用 `hal_type:=shared_serial`。
- `terrain_speed_limit` 运行时链路已经从 `rc26_merge_odom` 中删除；`PoseSender` 不再消费来自 `rc26_terrain` 的外部限速话题。
- `launch/merge_odom.launch.py` 仍会在运行时先规范化 EKF 参数里的科学计数法数值；这部分能力保留给手工 launch / 本地调试，不影响当前单口默认口径。
- `merge_odom_node` 现在默认就以 `feedback_serial_port:=__disabled__` 启动；如果没有显式重新接回反馈串口，会跳过 WheelOdom，但继续保留 `POSE_TARGET`、`/mechanism/send_command` 与 `/mechanism/command_feedback` 这条 `ttyUSB0` 目标串口链路。
- 仓库根目录的 `start_r2_teleop.sh` 现在还支持 `--stack full|minimal-mcu`：脚本默认 `minimal-mcu` 走最小串口链，`full` 仍可保留 `merge_odom` 装配做本地调试。
- 在这套单口默认口径下，`start_r2_teleop.sh` 会直接拒绝 `--pose-mode` 与 `--start-ekf`，避免误把已停用的 feedback / 融合速度链当作默认运行时能力。
- 遥控链通过 `start_r2_teleop.sh` 启动时，不再需要额外传地形限速相关参数；teleop 模式天然不会受 terrain 限速影响，同时会自动把前/后推杆单发 transport 的按钮节点一起拉起，不再默认拉起 `rc26_mechanism`。

## 配置注释口径

- `config/merge_odom_params.yaml`、`config/ekf_params.yaml` 与 `config/pose_sender_cmd_vel_teleop.yaml` 已保留常用/高影响字段的中文注释，说明串口、麦克纳姆几何、EKF 输入、执行保护、速度限幅和遥控链参数；本次同步清理了履带专用注释与启动参数。
