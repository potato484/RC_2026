# rc26_merge_odom

## 模块定位

`rc26_merge_odom` 是 R2 当前的里程计、位姿下发和目标 MCU 串口桥接包，负责把 WheelOdom、达妙 IMU、底盘控制下发与机构共享串口整合到同一条运行链路里。CAN odom 相关代码仍保留为非默认调试能力，但不属于当前实机合理启动口径。

## 当前实现

这个包不是单节点实现，而是一个多库、多可执行拼装的子系统：

- `can_odom` + `can_odom_node`
- `wheel_odom` + `wheel_odom_node`
- `dm_imu_driver` + `dm_imu_node`
- `pose_sender` + `pose_sender_node`
- `merge_odom_node`
- 调试节点：`single_wheel_test_node`

源码目录已经按功能拆开：

- `src/can/`：CAN 里程计解析
- `src/wheel/`：轮式里程计
- `src/imu/`：达妙 IMU 驱动与预处理
- `src/pose/`：位姿/速度下发
- `src/merge_odom_node.cpp`：总装与统一启动入口

关键配置与启动文件：

- `config/merge_odom_params.yaml`
- `config/ekf_params.yaml`
- `config/pose_sender_cmd_vel_teleop.yaml`
- `launch/merge_odom.launch.py`
- `launch/can_odom_only.launch.py`
- `launch/wheel_odom_only.launch.py`
- `launch/dm_imu_only.launch.py`

除了底盘相关职责外，`merge_odom_node` 当前维护“目标串口单口运行、独立 feedback 口保留但默认不用”的串口职责口径；现场排查时按“这条线负责什么”来记：

- `target_serial_port`，当前默认 `/dev/ttyUSB0`
  - `PoseSender` 默认从这条链路下发 `POSE_TARGET(0x1F)`
  - WheelOdom 默认也从这条链路接收 `ODOM_DATA(0x20)`，并发布稳定 `/merge_odom`
  - `MechanismTransportBridge` 继续作为共享桥接接口，供 `rc26_mechanism` 与 teleop 前/后推杆 sidecar 复用同一串口
  - service：`/mechanism/send_command`
  - topic：`/mechanism/command_feedback`
  - `merge_odom_node` 在单口模式下会把同一串口收到的 `ODOM_DATA` 分发给 WheelOdom，同时继续过滤并发布业务机构反馈
  - `pose_sender_node` 在最小 MCU 栈里也会挂出这组共享 mechanism 命令接口，因此 `minimal-mcu` 仍不只有速度下发；但最小遥控链不要求稳定 `/merge_odom`
  - 真机上只有 `merge_odom_node` 会真正打开这条物理串口；其它上层只能通过 transport 复用，不能再次直连同一设备
- `feedback_serial_port`，当前默认 `__disabled__`
  - 这是保留的独立反馈口参数，不属于当前默认链路
  - 当前不要把它写成实机启动步骤；保持禁用时，WheelOdom 已经使用目标串口单链路接收 `ODOM_DATA`
  - 只有后续明确恢复独立反馈硬件时，`PoseSender` 才会在这条链路上按 `50Hz` 下发 `POSE_FEEDBACK(0x1E)`
  - 当前默认运行时不会再依赖这条链路承载 teleop / mechanism transport

其中桥接层会过滤 `ACK / HEARTBEAT_ACK / ODOM_DATA` 这类高频非业务反馈，只把业务侧真正关心的反馈继续发布出去；当前双推杆 4 条命令都已经切到可靠 ACK 路径，因此若 MCU 不回通用 `ACK(0x00)`，会像其它可靠命令一样自动重传并打印超时日志；MCU 额外上送的 `0x13~0x16` 业务 ACK，以及台阶激光测距高度突变事件 `0x17/0x18`，都会继续透传到 `/mechanism/command_feedback`。

新增机制业务命令时，`MechanismTransportBridge` 默认不需要按命令改代码：只要新反馈 ID 不复用 `ACK / HEARTBEAT_ACK / ODOM_DATA` 这些被过滤的系统反馈，桥接层就会继续按通用 transport 透传。

`launch/merge_odom.launch.py` 当前除了 `use_can_odom` / `start_ekf` 外，还支持 `merge_odom_output_topic`、`require_merge_odom_output`、`use_imu_for_ekf`、`start_imu` 与 `stats_log_enable`。其中 `use_can_odom=false` 是当前实机口径，`true` 只保留为非默认调试能力：

- `merge_odom_output_topic` 默认 `merge_odom`，是对外稳定的底盘局部反馈 Odometry 话题
- `require_merge_odom_output` 默认 `true`，表示选中的真实 odom 源不可用时启动失败，不发布零 odom 或伪造数据
- `start_ekf=true`：Wheel raw odom 发布到 `wheel_odom`，EKF 输出 remap 到 `merge_odom_output_topic`
- `config/ekf_params.yaml` 显式设置 `publish_tf=false`，因此 EKF 只产出 `/merge_odom` Odometry，不发布 `odom -> base_link` 动态 TF；自动导航链动态 TF 权威仍留给 `rc26_odom_interface`
- `start_ekf=false`：选中的 WheelOdom raw odom 源直接发布到 `merge_odom_output_topic`，因此关闭 EKF 不会让 `/merge_odom` 消失
- `PoseSender` 在两种模式下都订阅同一个 `merge_odom_output_topic`，避免 EKF 开关改变反馈 topic
- `use_imu_for_ekf=true`：EKF 按默认口径继续融合 `DM_IMU`
- `use_imu_for_ekf=false`：只把 `imu0` 和 `imu0_*` 从 EKF 参数里移除，最终 `merge_odom` 改为纯 wheel 输入的 EKF 输出
- `start_imu=true`：启动 `dm_imu_node`，并允许 `WheelOdom`、`PoseSender` 订阅 IMU
- `start_imu=false`：不启动 `dm_imu_node`，并把 `imu_topic` 置空，同时关闭 `slip_enable`、`imu_gate_enable` 与 `latency_comp_enable`
- `stats_log_enable=true`：打开 PoseSender 的 1 秒统计日志
- 因此 `use_imu_for_ekf=false` 只影响 EKF 的最终融合位姿；若需要“完全不读 IMU”，还必须把 `start_imu` 关掉

当前运行时已经收口到单一 `mecanum_4wheel` 口径，不再保留履带/差速分支：

- `wheel_odom` 只接收 `ODOM_DATA(0x20)` 的四轮 `16B / 4 float` payload：`<v_fl, v_rl, v_rr, v_fr>`
- `wheel_odom` 与 `pose_sender` 都按麦克纳姆运动学保留真实 `vx / vy / wz`
- `PoseSender` 的保护器继续对 `(vx, vy)` 做二维投影限幅，不再把 `linear.y` 强制清零
- `merge_odom.launch.py` 已移除 `chassis_model`、`wheel_feedback_format`、`left_motor_can_id/right_motor_can_id`、`track_speed_max_mps`、`track_accel_max_mps2` 等履带专用参数
- 独立 `wheel_odom_node` 的几何参数已与主运行时统一为 `wheel_base=0.62326`、`track_width=0.7`

## 源码入口与阅读顺序
- 先看 `launch/merge_odom.launch.py`，理解这个子系统是如何把多个节点拼起来的。
- 再看 `src/merge_odom_node.cpp`，它是总装入口。
- 然后分模块看 `src/can/`、`src/wheel/`、`src/imu/`、`src/pose/`。
- 最后看参数 YAML 和调试脚本，确认串口/CAN/速度保护的部署值。

## 目录解剖
- `src/merge_odom_node.cpp`：统一创建和装配 CAN、轮速、IMU、融合器、位姿下发器，以及机构共享串口桥。
- `src/can/`：CAN 里程计采集与解码。
- `src/wheel/`：轮里程计串口接入和速度解算。
- `src/imu/`：达妙 IMU 驱动与解析。
- `src/pose/`：结合反馈和保护器向下位机发速度。

## 关键文件体量
- `src/pose/pose_sender.cpp`：875 行，下发保护逻辑最重。
- `src/imu/dm_imu_driver.cpp`：454 行。
- `src/can/can_odom.cpp`：475 行。
- `src/wheel/wheel_odom.cpp`：380 行。
- `src/merge_odom_node.cpp`：369 行，总装入口。

## 关键源码行段速览
- `src/rc26_merge_odom/src/merge_odom_node.cpp:1-221`：节点组合、参数分发和各子模块 wiring；`222-234`：`main()`。
- `src/rc26_merge_odom/src/can/can_odom.cpp:24-99`：构造和接口初始化；`100-225`：CAN 打开、接收线程和帧解析；`226-388`：里程计发布与状态访问。
- `src/rc26_merge_odom/src/wheel/wheel_odom.cpp:21-134`：串口轮速解包与机体系速度换算；`135-304`：里程计发布、状态获取与复位。
- `src/rc26_merge_odom/src/imu/dm_imu_driver.cpp:107-216`：串口打开与初始化；`247-374`：接收线程和缓冲解析；`375-454`：帧级解析。
- `src/rc26_merge_odom/src/pose/pose_sender.cpp`：输入订阅和基础状态缓存、IMU spike 与 fallback/governor 保护、反馈和目标发送定时器。

## 模块边界

- 这个包输出的是局部融合里程计和下发保护，不是全局定位
- 它不替代 `rc26_localization` 的地图配准职责
- 它也不做上层路径规划，只为控制和定位提供更稳的底层状态与执行接口
- 它现在拥有目标 MCU 串口的运行时权威，但并不替代 `rc26_mechanism` 的动作语义和 Action 服务职责

## 近期实现说明

- `wheel_odom` 现在只保留麦克纳姆四轮 payload 解析，新增的 `odom_payload.hpp` / `mecanum_kinematics.hpp` 负责统一四轮解包与运动学换算。
- `can_odom` 只保留四轮 CAN 解算路径，输出 `vx / vy / wz`。
- 旧 CAN/Wheel 双路软融合链路已从当前项目删除；当前主链不再保留双源融合入口，`use_can_odom` 仅作为 `merge_odom_node` 内的非默认单源调试选择。
- `PoseSender` 继续按 `(vx, vy, wz)` 协议下发，并把 `vy` 视为麦克纳姆主链上的有效保护量。
- `PoseSender` 代码仍保留 `POSE_FEEDBACK(0x1E)` 与 `POSE_TARGET(0x1F)` 的 `50Hz` 连续下发能力；但当前默认运行时只启用 `target_serial_port=/dev/ttyUSB0` 这条单口 MCU 链，因此默认只会连续发送 `POSE_TARGET`。
- `merge_odom_node` 现在额外挂出 `/mechanism/send_command` 与 `/mechanism/command_feedback`，把机构命令和前/后推杆遥控命令都复用到同一条目标串口上。
- `rc26_vision` 的 tip test 自动对线能力也按这个共享边界接入：横移只发布标准 `/cmd_vel`，对齐后抓取只调用 `/mechanism/send_command` 下发 `GRAB_TIP(0x01)` 空 payload，不绕过目标 MCU 串口权威。
- `pose_sender_node` 现在也会挂出同一组共享 mechanism 命令接口，因此 `minimal-mcu` 栈除了速度下发外，也能承接基于 ACK 的共享机构 transport。
- 双推杆协议已经收口为 `FRONT_PUSHROD_EXTEND/RETRACT(0x0E/0x0F)` 与 `REAR_PUSHROD_EXTEND/RETRACT(0x10/0x11)`；它们和其它普通机构命令一样走可靠 send + 通用 `ACK(0x00)`，MCU 额外发布的 `0x13~0x16` 业务 ACK 会继续透传给 transport feedback。
- 台阶激光测距事件 `FRONT_LASER_HEIGHT_JUMP(0x17)` 与 `REAR_LASER_HEIGHT_JUMP(0x18)` 同样经 mechanism transport 透传，不需要 `MechanismTransportBridge` 增加专用分支。
- 真实部署下，`rc26_mechanism` 不应再单独打开默认目标口 `/dev/ttyUSB0`；若 teleop 或 bringup 已经启动 `merge_odom`，则机制侧应使用 `hal_type:=shared_serial`。
- `terrain_speed_limit` 运行时链路已经从 `rc26_merge_odom` 中删除；`PoseSender` 不再消费来自 `rc26_terrain` 的外部限速话题。
- `launch/merge_odom.launch.py` 仍会在运行时先规范化 EKF 参数里的科学计数法数值；这部分能力保留给手工 launch / 本地调试，不影响当前单口默认口径。
- `merge_odom_node` 的节点参数 `require_odom_source` 默认 `false`，但 `merge_odom.launch.py` 会通过 `require_merge_odom_output:=true` 默认把稳定 `/merge_odom` 契约置为强要求。当前默认 `feedback_serial_port=__disabled__` 时，WheelOdom 使用目标串口单链路；如果目标串口不可用或没有真实 `ODOM_DATA`，不会伪造 `/merge_odom`。
- 若只是做遥控或 target 串口 transport 调试，可以显式传 `require_merge_odom_output:=false`，此时允许只有目标下发和 mechanism transport，不代表完整决策链满足 `/merge_odom` 契约。
- 2026-06-12 同步：单口模式下 `merge_odom_node` 新增目标串口接收分发，`ODOM_DATA` 进入 WheelOdom，`ACK / HEARTBEAT_ACK / ODOM_DATA` 以外的业务反馈继续由 mechanism transport 发布；当前合理启动口径因此是 `/dev/ttyUSB0` 单口 WheelOdom，而不是独立 feedback 串口或 CAN odom。
- 仓库根目录的 `start_r2_teleop.sh` 现在还支持 `--stack full|minimal-mcu`：脚本默认 `minimal-mcu` 走最小串口链，`full` 仍可保留 `merge_odom` 装配做本地调试。
- `start_r2_teleop.sh --stack full` 会显式传 `require_merge_odom_output:=false`，它是遥控调试降级入口，不代表完整决策运行已经满足稳定 `/merge_odom` 契约；该入口可通过 `--start-ekf` 或 `--pose-mode imu|no-imu|wheel-only` 临时打开/关闭 EKF 与 IMU 链路。
- 遥控链通过 `start_r2_teleop.sh` 启动时，不再需要额外传地形限速相关参数；teleop 模式天然不会受 terrain 限速影响，同时会自动把前/后推杆单发 transport 的按钮节点一起拉起，不再默认拉起 `rc26_mechanism`。

## 配置注释口径

- `config/merge_odom_params.yaml`、`config/ekf_params.yaml` 与 `config/pose_sender_cmd_vel_teleop.yaml` 已保留常用/高影响字段的中文注释，说明串口、麦克纳姆几何、EKF 输入、执行保护、速度限幅和遥控链参数；本次同步清理了履带专用注释与启动参数。
