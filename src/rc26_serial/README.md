# rc26_serial 串口通信模块

## 1. 简介

`rc26_serial` 是 R2 机器人项目的核心底层通信模块，负责上位机（AidLux 融合环境/ROS 2）与底层执行机构（MCU）之间的高效、稳定、双向数据传输。基于 v3.0 通信协议设计，该模块针对高频闭环控制与高频状态反馈（如 100Hz 的里程计 ODOM）场景进行了深度优化。

## 2. 核心改进与特性

本模块在初始版本的基础上，经历了一系列 Tier 0 级别的核心优化，以满足 R2 机器人在实战环境下的严苛要求。

### 2.1 高性能架构

*   **零拷贝环形缓冲 (RingParser)**: 彻底移除了线性缓冲（`memmove`）在解析成功或失败时的 O(n) 数据搬运开销。采用流式解析，显著降低了高频 ODOM 上报时的 CPU 缓存抖动，提升解析效率。
*   **事件驱动 I/O (epoll)**: 在 AidLux (Linux 5.15) 平台上，使用 `epoll` 完全替代了低效的带超时 `select` 轮询。当内核没有数据时，接收线程处于真正的休眠状态，而在数据到达时能够以最低延迟被唤醒。

### 2.2 稳健性与错误恢复

*   **RFC6298 自适应超时计算 (AdaptiveTimeout)**: 摒弃了简单的固定乘数估算（EWMA×4），引入了基于 SRTT（平滑往返时间）和 RTTVAR（往返时间方差）的标准 RTO 计算方法。该算法对 USB 串口在复杂调度下的抖动具有更强的鲁棒性，有效避免了因突发延迟导致的误判重传。
*   **断链快速失败机制 (Fast Fail on Close)**: 引入 `link_epoch` 机制。当物理串口断开或主动发起重连（`close()`）时，能够立即唤醒所有正在等待 ACK 的发送线程，使 `sendCommand` 在 <10ms 内返回失败。彻底根除了断线期间因死等超时（RTO_MAX）引发的线程卡死和日志风暴。
*   **滑动窗口健康监测 (CommHealth)**: 将串口健康状态评估（如解析错误率、ACK 超时率）从“永久累计”改为了“基于滑动窗口”的实时评估。当干扰消除后，通信健康状态（Level）能够在几百帧内自动从 `DEGRADED` 或 `CRITICAL` 恢复到 `HEALTHY`。
*   **严格的负荷与长度校验 (LEN Upper Bound)**: 在解析阶段增加了针对最大有效载荷（MAX_PAYLOAD_SIZE）的严格校验，杜绝了由于串口电平噪声导致超长伪造帧挤占缓冲区的问题。

### 2.3 观测与诊断

*   **发送序号追踪 (PoseSender Stats)**: 针对高频下发的控制位姿（Pose），增加了序列号（seq）追踪功能。可实时统计 1 秒时间窗口内的发送成功数、失败数以及由链路拥堵导致的缺帧/跳变率，为双链路冗余和微秒级调优提供了直接数据支撑。

### 2.4 当前协议补充

当前与遥控共享 transport 直接相关的双推杆命令编号已经收口为：

*   `FRONT_PUSHROD_EXTEND = 0x08`
*   `FRONT_PUSHROD_RETRACT = 0x09`
*   `REAR_PUSHROD_EXTEND = 0x0A`
*   `REAR_PUSHROD_RETRACT = 0x0B`

当前 KFS 阶梯预调与预留夹取命令编号为：

*   `ARM_RAISE = 0x04`
*   `ARM_LOWER = 0x05`
*   `ARM_SECOND_LOWER = 0x0E`
*   `GRAB_KFS_UP = 0x03`
*   `GRAB_KFS_DOWN = 0x02`
*   `ENTRY_GRAB_KFS_UP = 0x0F`
*   `COMPETITION_START = 0x10`
*   `SECOND_PRESELECTION_START = 0x11`
*   `SECOND_PRESELECTION_ARM_HIGH_RAISE = 0x12`
*   `SECOND_PRESELECTION_PLACE_KFS = 0x13`
*   `SECOND_PRESELECTION_ARM_LOWER = 0x14`
*   `SECOND_PRESELECTION_PRELOAD_KFS_PICKUP = 0x15`
*   `STARTUP_READY_WAITING_LIMIT = 0x20`
*   `PLACE_KFS_GRID = 0x06`

当前 KFS 机械臂业务反馈编号为：

*   `ARM_RAISE_DONE = 0x02`
*   `ARM_LOWER_DONE = 0x03`
*   `ARM_SECOND_LOWER_DONE = 0x0A`
*   `ENTRY_GRAB_KFS_UP_DONE = 0x0B`
*   `COMPETITION_START_DONE = 0x0C`
*   `SECOND_PRESELECTION_START_DONE = 0x0D`
*   `SECOND_PRESELECTION_ARM_HIGH_RAISE_DONE = 0x0F`
*   `MF_PRESELECTION_TRIGGER = 0x10`
*   `SECOND_PRESELECTION_PICKUP_KFS_DONE = 0x11`
*   `SECOND_PRESELECTION_ARM_LOWER_DONE = 0x12`
*   `SECOND_PRESELECTION_PRELOAD_KFS_PICKUP_DONE = 0x14`
*   `SECOND_PRESELECTION_MANUAL_FRONT_LASER_TRIGGERED = 0x15`

当前 MCU 错误 / 机械臂业务诊断反馈编号为：

*   `MCU_ERROR = 0xFE`：ACK 等待窗口内作为 transport 级下位机负响应参与 retry；同 `seq` 持续收到后 `sendCommand()` 失败并在 `lastError()` / diagnostics 中说明“下位机原因”。若上行 `0xFE` payload 长度为 2，则视为机械臂业务状态/失败反馈，`payload[0]` 为 `failed_cmd`，`payload[1]` 为 `error_code`，可经 `/mechanism/command_feedback` 透传给决策层诊断。

机械臂 `0xFE` 两字节 payload 的 `error_code` 当前定义为：

*   `PLANAR_ARM_FAIL_BUSY = 0x01`：命令仍在处理中，不等于最终失败
*   `PLANAR_ARM_FAIL_INVALID_PAYLOAD = 0x02`：命令 payload 非法
*   `PLANAR_ARM_FAIL_NOT_INIT = 0x03`：机械臂模块尚未初始化完成
*   `PLANAR_ARM_FAIL_HAL_ERROR = 0x04`：HAL 层或运动控制执行错误
*   `PLANAR_ARM_FAIL_INVALID_STATE = 0x05`：当前机械臂状态不允许执行该命令

当前这几类业务事件只由 MCU 上行：

*   `FRONT_LASER_HEIGHT_JUMP = 0x04`
*   `REAR_LASER_HEIGHT_JUMP = 0x05`
*   `FRONT_LIMIT_SWITCH_TRIGGERED = 0x06`
*   `FRONT_SECOND_LASER_HEIGHT_JUMP = 0x07`
*   `MF_PRESELECTION_TRIGGER = 0x10`
*   `SECOND_PRESELECTION_MANUAL_FRONT_LASER_TRIGGERED = 0x15`
*   脚本专用上行人工触发外部限位 3 `0x13`

这些事件的 v1 payload 为空或忽略：`0x04/0x05/0x07` 表示前轮 / 后轮 / 前轮第二个激光测距模块检测到车体高度突变；第二预选赛放置后人工触发前轮附近首个激光模块成功使用独立上行 `0x15`，不复用 `FRONT_LASER_HEIGHT_JUMP(0x04)`；MCU 上行 `0x06/0x10/0x13` 均来自人工触发的外部限位开关，当前分别作为人工触发外部限位 1/2/3 映射到不同上位机动作。上位机通过 `/mechanism/command_feedback` 消费它们；下行 `COMPETITION_START(0x10)`、`SECOND_PRESELECTION_PLACE_KFS(0x13)` 和下行 `SECOND_PRESELECTION_PRELOAD_KFS_PICKUP(0x15)` 与同 ID 上行事件分属不同协议方向。

当前真实运行时口径是：

*   4 条双推杆命令都通过 `rc26_mcu_transport` 走可靠 `sendCommand()` ACK 路径；若 MCU 不回通用 `ACK(0x00)`，会像其它可靠命令一样重传并打印超时日志。
*   `rc26_telecontrol_front_pushrod_buttons` 会在 `Y/A` 按下沿单次调用 `/mechanism/send_command`，分别桥成前推杆伸展 / 收缩。
*   `rc26_telecontrol_rear_pushrod_buttons` 会在 `Select/Back` / `Start` 按下沿单次调用 `/mechanism/send_command`，分别桥成后推杆伸展 / 收缩；`Dpad 左/右` 已回归底盘横移控制。
*   transport service 在 `wait_ack=true` 时返回 `accepted=true` 的前提仍然是 MCU 先回通用 `ACK(0x00)`；`0x02~0x07`、`0x09`、`0x0A`、`0x0B`、`0x0C`、`0x0D`、`0x0F`、`0x10`、`0x11` 与两字节 payload 的 `0xFE` 业务反馈会继续发布到 `/mechanism/command_feedback`，但不参与 `sendCommand()` 的可靠 ACK 判定。`wait_ack=false` 时 provider 调用 `sendCommandNoAck()`，写入串口成功即返回，不等待 ACK、不 retry。
*   KFS 向下夹取在 `ARM_LOWER_DONE(0x03)` 后完成视觉横移对齐和一次锁深度，开环前进前还会发送 `ARM_SECOND_LOWER(0x0E)`，并等待同 `seq` 的 `ARM_SECOND_LOWER_DONE(0x0A)` 后才前进或直接夹取。
*   梅林预选赛到达 2 号入口后会先发送普通 `ARM_RAISE(0x04)` 并等待 `ARM_RAISE_DONE(0x02)`，再启动入口识别；2 号入口 R2 KFS 夹取使用普通高侧 `GRAB_KFS_UP(0x03)`。入口专用 `ENTRY_GRAB_KFS_UP(0x0F)` / `ENTRY_GRAB_KFS_UP_DONE(0x0B)` 仅用于决策层仍显式启用 `entry_high_protocol` 的入口高侧场景。service ACK 不等同于夹取完成，真正计数仍由决策层视觉消失验证决定。
*   MCU 上行 `FRONT_LIMIT_SWITCH_TRIGGERED(0x06)` 是人工触发外部限位 1 事件；上行 `MF_PRESELECTION_TRIGGER(0x10)` 是人工触发外部限位 2 事件。两者只选择入口分支，不直接决定下行命令或目标树。
*   managed first 入口下，人工触发外部限位 1 的上行 `0x06` 与人工触发外部限位 2 的上行 `0x10` 都会先下发 `COMPETITION_START(0x10)` 并等待同 `seq` 的 `COMPETITION_START_DONE(0x0C)`；`0x06` 继续武馆+梅林完整树，`0x10` 切到 `mf_preselection_tree.xml`。
*   managed second 组合入口下，入口先等待人工触发外部限位 1/2 的上行 `0x06/0x10`，两条分支都会下发 `SECOND_PRESELECTION_START(0x11)` 并等待同 `seq` 的 `SECOND_PRESELECTION_START_DONE(0x0D)`；`0x06` 分支继续执行斜坡并停车等待斜坡后上行 `0x10`，收到后再次执行 `0x11/0x0D` 握手再进入第二预选赛搜寻，入口 `0x10` 分支直接切到 `second_preselection_tree.xml` 搜寻。
*   脚本专用上行人工触发外部限位 3 `0x13` 仍只由 `start_r2_auto.sh` 的红蓝切换监听器消费，写回下一次启动使用的 `r2_active_side.yaml`，不改变当前运行中的 decision tree。
*   下行 `STARTUP_READY_WAITING_LIMIT(0x20)` 是启动就绪通知：`start_r2_auto.sh` 默认链路中的 bringup 通知节点确认 RealSense color、aligned depth、camera info 都已出帧，并且 decision 已进入人工限位 branch gate 等待后，以 `wait_ack=false` 下发一次空 payload；MCU 不需要回复 ACK 或业务反馈。
*   第二个预选赛独立树使用 `SECOND_PRESELECTION_START(0x11)` 并等待同 `seq` 的 `SECOND_PRESELECTION_START_DONE(0x0D)` 后开始底盘导航；搜索夹取链视觉对齐后先使用 `SECOND_PRESELECTION_ARM_LOWER(0x14)` 并等待同 `seq` 的 `SECOND_PRESELECTION_ARM_LOWER_DONE(0x12)`，确认机械臂放下后才前进；随后使用 `SECOND_PRESELECTION_ARM_HIGH_RAISE/KFS_PICKUP(0x12)` 触发 KFS 夹取，ACK 后等待同 `seq` 的 `SECOND_PRESELECTION_PICKUP_KFS_DONE(0x11)`，再做原目标视觉消失验证；放置 KFS 使用下行 `SECOND_PRESELECTION_PLACE_KFS(0x13)`，该命令当前只要求通用 ACK。放置后决策层会后退，前置推杆伸出并发下发 `SECOND_PRESELECTION_PRELOAD_KFS_PICKUP(0x15)`，等待同 `seq` 的 `SECOND_PRESELECTION_PRELOAD_KFS_PICKUP_DONE(0x14)` 和可配置前推杆延时，再等待无 `seq` 的人工前轮激光放行 `SECOND_PRESELECTION_MANUAL_FRONT_LASER_TRIGGERED(0x15)`，随后执行剩余上阶流程并在最终延时后再次下发 `0x13`。
*   串口层当前只把 `ACK(0x00)` 和心跳场景下的 `HEARTBEAT_ACK(0x01)` 视为成功 ACK 等待结果；ACK 等待窗口内的 `MCU_ERROR(0xFE)` 仍作为下位机原因的负响应参与 retry 和错误说明，但两字节 payload 的 `0xFE` 会作为机械臂业务状态/失败反馈交给上层发布语义。旧即时负确认和旧动作完成反馈已经从协议中移除。
*   真机部署时，目标 MCU 串口由 `rc26_mcu_transport` 独占打开；其它上层只复用 transport，不再次直连同一设备。
*   端头视觉对齐后的抓取改为先经 `/cmd_vel` x 负向前探等待人工触发外部限位 1 的上行 `0x06`，再经 `/mechanism/send_command` 下发 `GRAB_TIP(0x01)` 空 payload。
*   旧 tip test 视觉状态下发命令已从下行协议中删除；下行编号 `0x12` 现由第二个预选赛 `SECOND_PRESELECTION_ARM_HIGH_RAISE` 复用，旧 MCU 或日志排查时应按当前 `protocol.hpp` 真源确认语义。

当前底盘反馈协议也已经统一回麦克纳姆四轮口径：

*   `ODOM_DATA(0x08)` 固定为 `<v_fl, v_rl, v_rr, v_fr>` 的 `16B / 4 float`
*   `POSE_TARGET(0x0C)` 继续保持 `(vx, vy, wz)` 三浮点协议；`POSE_TARGET` 当前由 `rc26_mcu_transport` 默认按 `50Hz` no-ack 路径下发
*   机构指令与底盘 `POSE_TARGET` 的目标 MCU 串口 owner 均由 `rc26_mcu_transport` 提供；`ODOM_DATA` 代码仍保留但不回到当前默认反馈主链

## 3. 设计原则与平台限制

1.  **帧结构稳定性**: 当前双推杆协议语义保持前/后推杆四命令；KFS 阶梯测试链使用机械臂升降预调命令 `0x04/0x05` 与完成反馈 `0x02/0x03`，向下夹取开环前第二节机械臂放下使用 `0x0E/0x0A`，`0x03/0x02` 作为上下阶梯 KFS 夹取命令，梅林预选赛入口高侧夹取使用 `0x0F/0x0B`。managed first 使用下行 `0x10` 等上行 `0x0C` 做开始握手；managed second 使用下行 `0x11` 等上行 `0x0D` 做开始握手；第二个预选赛搜索夹取链视觉对齐后使用下行 `0x14` 与同 `seq` 上行 `0x12` 做机械臂放下握手，再使用下行 `0x12` 与同 `seq` 上行 `0x11` 做 KFS 夹取完成握手，下行 `0x13` 做 ACK-only 放置命令；放置后预装 KFS 夹取使用下行 `0x15` 与同 `seq` 上行 `0x14`，启动就绪通知使用下行 no-ack `0x20`，人工前轮激光放行使用无 `seq` 上行 `0x15`。台阶激光测距高度突变事件使用独立上行 `0x04/0x05/0x07`，人工触发外部限位 1/2/3 分别使用独立上行 `0x06/0x10/0x13`，其中上行 `0x13` 当前只用于脚本专用红蓝配置切换；MCU 端 `0xFE` 既可作为 ACK 窗口 transport 负响应，也可在两字节 payload 时表示机械臂业务状态/失败诊断。除此之外，v3.0 的帧头、CRC32 MPEG-2 校验和最大长度限制保持不变。
2.  **兼容性第一**: 考虑到 AidLux 混合环境的特殊性，优先采用成熟稳定的 POSIX 接口（如 `epoll`）而非激进的异步 I/O（如 `io_uring` on TTY）。
3.  **单体非阻塞**: 核心驱动类不抛出未捕获异常，任何发送/接收回调均被妥善隔离，防止上层业务逻辑的故障波及底层通信主循环。

## 4. 相关文档

*   当前根目录集中式调试文档已删除；串口模块的验证入口以本包测试、上车验收手册和实际联调记录为准。
*   **上车验收手册**: 位于 `MVP技术方案/串口/执行方案/上车验收操作手册-执行方案2.md`，用于实车联调的最终确认。
