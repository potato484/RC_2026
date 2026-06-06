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

*   `FRONT_PUSHROD_EXTEND = 0x0E`
*   `FRONT_PUSHROD_RETRACT = 0x0F`
*   `REAR_PUSHROD_EXTEND = 0x10`
*   `REAR_PUSHROD_RETRACT = 0x11`

当前双推杆业务反馈编号已经收口为：

*   `FRONT_PUSHROD_EXTEND_ACK = 0x13`
*   `FRONT_PUSHROD_RETRACT_ACK = 0x14`
*   `REAR_PUSHROD_EXTEND_ACK = 0x15`
*   `REAR_PUSHROD_RETRACT_ACK = 0x16`

当前真实运行时口径是：

*   4 条双推杆命令都通过 `rc26_merge_odom` 的共享 transport 走可靠 `sendCommand()` ACK 路径；若 MCU 不回通用 `ACK(0x00)`，会像其它可靠命令一样重传并打印超时日志。
*   `rc26_telecontrol_front_pushrod_buttons` 会在 `Y/A` 按下沿单次调用 `/mechanism/send_command`，分别桥成前推杆伸展 / 收缩。
*   `rc26_telecontrol_rear_pushrod_buttons` 会在 `Select/Back` / `Start` 按下沿单次调用 `/mechanism/send_command`，分别桥成后推杆伸展 / 收缩；`Dpad 左/右` 已回归底盘横移控制。
*   transport service 返回 `accepted=true` 的前提仍然是 MCU 先回通用 `ACK(0x00)`；`0x13~0x16` 业务 ACK 会继续发布到 `/mechanism/command_feedback`，但不参与 `sendCommand()` 的可靠 ACK 判定。
*   串口层当前只把 `ACK(0x00)`、`NACK(0x01)` 和心跳场景下的 `HEARTBEAT_ACK(0x10)` 视为 ACK 等待结果；MCU 已不再返回 `ACTION_FAIL/ERROR` 这类快捷失败反馈。
*   真机部署时，目标 MCU 串口仍由 `rc26_merge_odom` 运行时独占打开；当前默认主口是 `target_serial_port=/dev/ttyUSB0`，其它上层只复用 transport，不再次直连同一设备。

当前底盘反馈协议也已经统一回麦克纳姆四轮口径：

*   `ODOM_DATA(0x20)` 固定为 `<v_fl, v_rl, v_rr, v_fr>` 的 `16B / 4 float`
*   `POSE_FEEDBACK(0x1E)` / `POSE_TARGET(0x1F)` 继续保持 `(vx, vy, wz)` 三浮点协议和 `50Hz` 连续发送
*   但当前默认运行时只启用 `target_serial_port=/dev/ttyUSB0` 这条单口 MCU 链；`feedback_serial_port` 默认 `__disabled__`，因此 `ODOM_DATA` / `POSE_FEEDBACK` 代码仍保留，但不属于默认部署路径

## 3. 设计原则与平台限制

1.  **帧结构稳定性**: 当前双推杆协议语义已经更新为前/后推杆四命令，并要求 MCU 同步支持 `0x13~0x16` 业务 ACK；除此之外，v3.0 的帧头、CRC32 MPEG-2 校验和最大长度限制保持不变。
2.  **兼容性第一**: 考虑到 AidLux 混合环境的特殊性，优先采用成熟稳定的 POSIX 接口（如 `epoll`）而非激进的异步 I/O（如 `io_uring` on TTY）。
3.  **单体非阻塞**: 核心驱动类不抛出未捕获异常，任何发送/接收回调均被妥善隔离，防止上层业务逻辑的故障波及底层通信主循环。

## 4. 相关文档

*   当前根目录集中式调试文档已删除；串口模块的验证入口以本包测试、上车验收手册和实际联调记录为准。
*   **上车验收手册**: 位于 `MVP技术方案/串口/执行方案/上车验收操作手册-执行方案2.md`，用于实车联调的最终确认。
