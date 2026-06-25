# rc26_serial

## 模块定位

`rc26_serial` 是整个 R2 仓库的串口通信基础库，给机构控制、底盘下发和其他串口链路复用。

## 当前实现

- 构建产物：共享库 `serial_driver`
- 核心源码：`src/rc26_serial/src/serial_driver.cpp`
- 已有测试：
  - `test/test_adaptive_timeout.cpp`
  - `test/test_ring_parser.cpp`

当前实现重点不在“是否能通信”，而在“高频通信时是否稳定可控”：

- 环形缓冲与流式解析
- `epoll` 事件驱动 I/O
- 自适应 ACK/RTO 超时计算
- 断链快速失败
- 滑动窗口健康度统计
- 长度与负载保护

当前真实协议口径里，`ODOM_DATA(0x20)` 已经重新收口为麦克纳姆四轮唯一格式：

- `ODOM_DATA = <v_fl, v_rl, v_rr, v_fr>`，共 `16B / 4 float`
- 如果外部运行时复用本协议，上位机可由相应 WheelOdom 实现解析，并继续换算 `vx / vy / wz`

上位机下发给 MCU 的下行命令已收口为仅保留实机链路中实际使用的条目：

- `POSE_TARGET = 0x1F`
- 按 `50Hz` 连续发送，走公开的 `sendCommandNoAck()` 路径，不等待 ACK
- 当前目标 MCU 串口 owner 由 `rc26_mcu_transport` 提供；`POSE_TARGET(0x1F)` 默认由它按 `/cmd_vel` 下发

当前双推杆协议已经直接收口为前/后推杆四命令；遥控链也不再通过 `rc26_mechanism` 的 Action 兼容路径消费，而是直接走共享 transport：

- 下行命令：
  - `FRONT_PUSHROD_EXTEND = 0x0E`
  - `FRONT_PUSHROD_RETRACT = 0x0F`
  - `REAR_PUSHROD_EXTEND = 0x10`
  - `REAR_PUSHROD_RETRACT = 0x11`
- 上行业务反馈：
  - `FRONT_LASER_HEIGHT_JUMP = 0x17`
  - `REAR_LASER_HEIGHT_JUMP = 0x18`
  - `FRONT_LIMIT_SWITCH_TRIGGERED = 0x19`
  - `FRONT_SECOND_LASER_HEIGHT_JUMP = 0x1A`

当前真实口径是：

- `rc26_telecontrol_front_pushrod_buttons` 会在 `Y/A` 按下沿单次调用 `/mechanism/send_command`
- `rc26_telecontrol_rear_pushrod_buttons` 会在 `Select/Back` / `Start` 按下沿单次调用 `/mechanism/send_command`
- 4 条双推杆命令都通过 `rc26_mcu_transport` 走可靠 `sendCommand()` ACK 路径；若 MCU 不回通用 `ACK(0x00)`，会像其它可靠命令一样自动重传并打印超时日志
- `0x17~0x1A` 业务反馈会继续发布到 `/mechanism/command_feedback`，但不参与 `sendCommand()` 的可靠 ACK 判定
- `0x17/0x18` 只由 MCU 上行，v1 payload 为空或忽略，分别表示前轮 / 后轮激光测距模块检测到车体高度突变；当前两激光台阶 BT 动作只按这两个事件推进阶段
- `0x1A` 的 `FRONT_SECOND_LASER_HEIGHT_JUMP` 协议枚举保留，桥接层仍可透传，但当前上/下台阶 BT 不再等待或消费它作为阶段推进条件
- `0x19` 只由 MCU 上行，v1 payload 为空或忽略，表示武馆前方限位开关触发；武馆视觉夹取链在对齐后 x 负向前探并等待该事件，收到后立即停车再下发 `GRAB_TIP(0x01)`
- 串口层当前只把 `ACK(0x00)`、`NACK(0x01)` 和心跳场景下的 `HEARTBEAT_ACK(0x10)` 视为 ACK 等待结果
- `Dpad 左/右` 已回归底盘横移控制
- 真机部署时，目标 MCU 串口由 `rc26_mcu_transport` 独占打开；其它上层只复用 transport，不再次直连同一设备

旧的 tip test 视觉状态下发命令已经从下行协议中删除：

- 原下行编号 `0x12` 当前不重新分配给新的下行命令，避免旧 MCU 或日志误判。
- `rc26_vision` 的 tip test 链不再直连串口发送视觉状态；自动对线后先通过 `/cmd_vel` x 负向前探等待 0x19 限位反馈，随后通过 `/mechanism/send_command` 共享 transport 下发 `GRAB_TIP(0x01)` 空 payload。

当前维护边界还要再记一条：

- `rc26_serial` 只定义原始协议 ID、封帧、ACK/RTO 和串口 I/O
- 机构业务上的“这个命令能不能走 `/mechanism/run_command`、什么反馈算完成、默认 timeout 是多少”不在这里维护
- 这些业务语义当前统一收口在 `rc26_mechanism/catalog/mechanism_command_catalog.*`

## 源码入口与阅读顺序
- 先看 `src/serial_driver.cpp`，这是整个仓库复用的串口底座。
- 再看两个测试文件，理解 ACK/RTO 和环形解析器的验收点。
- 最后回到上层调用者，比如 `rc26_mechanism` 或 `rc26_mcu_transport`，确认是哪一层在赋予业务语义，以及哪一层拥有真实串口所有权。

## 目录解剖
- `serial_driver.cpp`：同时暴露可靠 `sendCommand()` 和公开的 no-ACK `sendCommandNoAck()`。
- `test/test_adaptive_timeout.cpp`：自适应超时测试。
- `test/test_ring_parser.cpp`：流式解析测试。

## 关键文件体量
- `src/serial_driver.cpp`：1143 行，基础库实现很厚。
- `test/test_ring_parser.cpp`：149 行。
- `test/test_adaptive_timeout.cpp`：49 行。

## 关键源码行段速览
- `src/rc26_serial/src/serial_driver.cpp:108-479`：端口打开/关闭、重连回调、重连线程与底层 fd 生命周期。
- `src/rc26_serial/src/serial_driver.cpp:529-802`：帧构造、ACK 等待和通用命令发送。
- `src/rc26_serial/src/serial_driver.cpp:808-850`：姿态/停车/心跳等高频业务辅助发送。
- `src/rc26_serial/src/serial_driver.cpp:953-1143`：帧分发和接收线程主循环。

## 模块边界

- 这个包是基础通信库，不直接代表某个业务节点
- 它不做上层动作语义，具体业务封装在 `rc26_mechanism` 等包里；目标 MCU 串口 owner 由 `rc26_mcu_transport` 承担
- 当业务异常时，要区分是协议层问题还是上层状态机问题，不能把所有故障都归到这个库
- 真实整车部署下目标 MCU 串口的权威所有者由 `rc26_mcu_transport` 承担；视觉 tip test 只能复用 `/cmd_vel` 与 `/mechanism/send_command`，不能再次直连同一设备

## 本轮同步

2026-06-25 同步：清理 `protocol.hpp` 中未接入实机链路的枚举值——移除 9 个未使用 CommandID（旋转四命令、MECH_UP/DOWN_MERLIN、MECH_UP_DUEL、PLACE_KFS_GROUND、POSE_FEEDBACK）和 16 个未使用 FeedbackID（CLIMBING_SLOPE、SLOPE_DONE、旋转 Done、MERLIN/DUE Done、PLACE_KFS_GROUND_DONE、STAIR_CLIMB/DESCEND_DONE、推杆四条 ACK）。协议定义现在只保留实机链路中实际收发的条目。
