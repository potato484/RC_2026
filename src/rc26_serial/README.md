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
*   `PLACE_KFS_GRID = 0x06`

当前 KFS 机械臂业务反馈编号为：

*   `ARM_RAISE_DONE = 0x02`
*   `ARM_LOWER_DONE = 0x03`
*   `ARM_SECOND_LOWER_DONE = 0x0A`
*   `ENTRY_GRAB_KFS_UP_DONE = 0x0B`

当前 transport 级 MCU 错误反馈编号为：

*   `MCU_ERROR = 0xFE`：表示本轮可靠发送失败来自下位机原因；同 `seq` 收到后串口层继续按 retry `0x00~0x09` 重发，后续若收到 `ACK(0x00)` 则成功，持续 `0xFE` 则失败并在 `lastError()` / diagnostics 中说明“下位机原因”

当前这几类业务事件只由 MCU 上行：

*   `FRONT_LASER_HEIGHT_JUMP = 0x04`
*   `REAR_LASER_HEIGHT_JUMP = 0x05`
*   `FRONT_LIMIT_SWITCH_TRIGGERED = 0x06`
*   `FRONT_SECOND_LASER_HEIGHT_JUMP = 0x07`

这些事件的 v1 payload 为空或忽略：`0x04/0x05/0x07` 表示前轮 / 后轮 / 前轮第二个激光测距模块检测到车体高度突变，`0x06` 表示武馆前方限位开关触发；上位机通过 `/mechanism/command_feedback` 消费它们，不新增对应下行命令。

当前真实运行时口径是：

*   4 条双推杆命令都通过 `rc26_mcu_transport` 走可靠 `sendCommand()` ACK 路径；若 MCU 不回通用 `ACK(0x00)`，会像其它可靠命令一样重传并打印超时日志。
*   `rc26_telecontrol_front_pushrod_buttons` 会在 `Y/A` 按下沿单次调用 `/mechanism/send_command`，分别桥成前推杆伸展 / 收缩。
*   `rc26_telecontrol_rear_pushrod_buttons` 会在 `Select/Back` / `Start` 按下沿单次调用 `/mechanism/send_command`，分别桥成后推杆伸展 / 收缩；`Dpad 左/右` 已回归底盘横移控制。
*   transport service 返回 `accepted=true` 的前提仍然是 MCU 先回通用 `ACK(0x00)`；`0x02~0x07`、`0x09`、`0x0A` 与 `0x0B` 业务反馈会继续发布到 `/mechanism/command_feedback`，但不参与 `sendCommand()` 的可靠 ACK 判定。
*   KFS 向下夹取在 `ARM_LOWER_DONE(0x03)` 后完成视觉横移对齐和一次锁深度，开环前进前还会发送 `ARM_SECOND_LOWER(0x0E)`，并等待同 `seq` 的 `ARM_SECOND_LOWER_DONE(0x0A)` 后才前进或直接夹取。
*   梅林预选赛入口高侧 KFS 夹取使用 `ENTRY_GRAB_KFS_UP(0x0F)`，并等待同 `seq` 的 `ENTRY_GRAB_KFS_UP_DONE(0x0B)` 后再进入视觉消失验证；service ACK 不等同于夹取完成。
*   组合树启动 gate 收到人工 `FRONT_LIMIT_SWITCH_TRIGGERED(0x06)` 后，会通过 `/mechanism/send_command` 下发 `COMPETITION_START(0x10)` 空 payload，service ACK 只表示下位机已确认比赛开始通知。
*   串口层当前只把 `ACK(0x00)` 和心跳场景下的 `HEARTBEAT_ACK(0x01)` 视为成功 ACK 等待结果；`MCU_ERROR(0xFE)` 只作为下位机原因的负响应参与 retry 和错误说明，不作为业务反馈发布语义。旧即时负确认和旧动作完成反馈已经从协议中移除。
*   真机部署时，目标 MCU 串口由 `rc26_mcu_transport` 独占打开；其它上层只复用 transport，不再次直连同一设备。
*   端头视觉对齐后的抓取改为先经 `/cmd_vel` x 负向前探等待 0x06 限位反馈，再经 `/mechanism/send_command` 下发 `GRAB_TIP(0x01)` 空 payload。
*   旧 tip test 视觉状态下发命令已从下行协议中删除，原下行编号 `0x12` 当前不重新分配，避免旧 MCU 或日志误判。

当前底盘反馈协议也已经统一回麦克纳姆四轮口径：

*   `ODOM_DATA(0x08)` 固定为 `<v_fl, v_rl, v_rr, v_fr>` 的 `16B / 4 float`
*   `POSE_TARGET(0x0C)` 继续保持 `(vx, vy, wz)` 三浮点协议；`POSE_TARGET` 当前由 `rc26_mcu_transport` 默认按 `50Hz` no-ack 路径下发
*   机构指令与底盘 `POSE_TARGET` 的目标 MCU 串口 owner 均由 `rc26_mcu_transport` 提供；`ODOM_DATA` 代码仍保留但不回到当前默认反馈主链

## 3. 设计原则与平台限制

1.  **帧结构稳定性**: 当前双推杆协议语义保持前/后推杆四命令；KFS 阶梯测试链使用机械臂升降预调命令 `0x04/0x05` 与完成反馈 `0x02/0x03`，向下夹取开环前第二节机械臂放下使用 `0x0E/0x0A`，`0x03/0x02` 作为上下阶梯 KFS 夹取命令，梅林预选赛入口高侧夹取使用 `0x0F/0x0B`，组合树比赛开始通知使用 `0x10` 空 payload。台阶激光测距高度突变事件使用独立上行 `0x04/0x05/0x07`，武馆前方限位开关触发使用独立上行 `0x06`，MCU 端错误使用上行 `0xFE` 并只表示下位机原因。除此之外，v3.0 的帧头、CRC32 MPEG-2 校验和最大长度限制保持不变。
2.  **兼容性第一**: 考虑到 AidLux 混合环境的特殊性，优先采用成熟稳定的 POSIX 接口（如 `epoll`）而非激进的异步 I/O（如 `io_uring` on TTY）。
3.  **单体非阻塞**: 核心驱动类不抛出未捕获异常，任何发送/接收回调均被妥善隔离，防止上层业务逻辑的故障波及底层通信主循环。

## 4. 相关文档

*   当前根目录集中式调试文档已删除；串口模块的验证入口以本包测试、上车验收手册和实际联调记录为准。
*   **上车验收手册**: 位于 `MVP技术方案/串口/执行方案/上车验收操作手册-执行方案2.md`，用于实车联调的最终确认。
