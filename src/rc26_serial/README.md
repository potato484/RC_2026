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

当前与遥控共享 transport 直接相关的命令编号已经扩展到：

*   `FRONT_TRACK_UP = 0x0E`
*   `FRONT_TRACK_DOWN = 0x0F`
*   `PUSHROD_EXTEND = 0x10`
*   `PUSHROD_RETRACT = 0x11`

当前真实运行时口径是：

*   `FRONT_TRACK_UP/DOWN` 通过 `rc26_merge_odom` 的共享 transport 走 no-ACK 单发，适合按住期间连续发送。
*   `PUSHROD_EXTEND/RETRACT` 通过共享 transport 走可靠 `sendCommand()` ACK 路径；成功标准是 transport service 返回 `accepted=true`，不要求 MCU 再上送独立 `DONE` 反馈。
*   `rc26_telecontrol_front_track_test` 会在 `Y/A` 按住期间按 `50Hz` 连续调用 `/mechanism/transport/send_command`。
*   `rc26_telecontrol_pushrod_dpad` 会把 `Dpad 左/右` 直接桥成 `0x10 / 0x11` 的单次 ACK 命令。
*   真机部署时，目标 MCU 串口仍由 `rc26_merge_odom` 运行时独占打开；其它上层只复用 transport，不再次直连同一设备。

## 3. 设计原则与平台限制

1.  **协议不可变性**: 所有优化均在完全兼容现有 v3.0 通信协议（含帧头、CRC32 MPEG-2 校验、最大长度限制）的前提下进行，无需修改下位机 MCU 固件。
2.  **兼容性第一**: 考虑到 AidLux 混合环境的特殊性，优先采用成熟稳定的 POSIX 接口（如 `epoll`）而非激进的异步 I/O（如 `io_uring` on TTY）。
3.  **单体非阻塞**: 核心驱动类不抛出未捕获异常，任何发送/接收回调均被妥善隔离，防止上层业务逻辑的故障波及底层通信主循环。

## 4. 相关文档

*   [调试指南 (debug_guide.md)](./docs/debug_guide.md): 提供了如何编译、测试、模拟故障注入以及验证模块性能的具体步骤。
*   **上车验收手册**: 位于 `MVP技术方案/串口/执行方案/上车验收操作手册-执行方案2.md`，用于实车联调的最终确认。
