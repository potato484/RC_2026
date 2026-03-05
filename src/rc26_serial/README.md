# rc26_serial 串口通信模块

## 模块定位

`rc26_serial` 负责上位机与 MCU 的 UART 协议收发，提供：

- 指令发送（含 ACK/NACK 重试）
- 心跳发送与重连触发
- 接收帧解析与回调分发
- 通信健康度指标（CommHealth）

协议保持 `v3.0` 不变（`MAX_PAYLOAD_SIZE=32`、CRC32 MPEG-2、帧头尾 `AA55/55AA`）。

---

## Tier 0 改进（执行方案2已落地）

### 1) Ack 等待三态 + 断链快速失败

- 引入 `AckWaitResult { kReceived, kTimeout, kLinkDown }`
- 在 `close()` 中递增 `link_epoch_` 并 `ack_cv_.notify_all()`，打断等待中的 ACK
- `kLinkDown` 分支不计入 `ack_timeouts`，避免断链期间误降级

关键代码：`include/rc26_serial/serial_driver.hpp`、`src/serial_driver.cpp`

### 2) AdaptiveTimeout（RFC6298 风格）

- 使用 `srtt/rttvar` 估计 RTO
- `onTimeout()` 采用指数退避（上限 `RTO_MAX=500ms`）
- `get()` 输出当前退避后的 RTO

关键代码：`include/rc26_serial/adaptive_timeout.hpp`

### 3) CommHealth 滑动窗口

- 新增 `SlidingCounter<N>` 统计窗口错误率
- `parse_window(1000)`：解析错误率窗口
- `ack_window(200)`：ACK 超时率窗口
- 重连判定改为“60 秒窗口内重连次数”

关键代码：`include/rc26_serial/sliding_counter.hpp`、`include/rc26_serial/serial_driver.hpp`

### 4) RingParser 流式解析替代线性缓冲

- 新增 `RingParser`（4096B 环形缓冲）
- 校验顺序：帧头 → LEN 范围 → 帧尾 → CRC32
- 任一错误采用 `drop-1`，提升噪声恢复能力（覆盖 `AA AA 55` 模式）
- 统计 `len_invalid / tail_bad / crc_bad / head_drop / overflow_drop`

关键代码：`include/rc26_serial/ring_parser.hpp`、`src/serial_driver.cpp`

### 5) epoll 替代 select

- 接收线程改为 `epoll_wait(..., 50ms)`
- 统一处理 `EPOLLIN / EPOLLERR / EPOLLHUP`

关键代码：`src/serial_driver.cpp`

### 6) `sendPose` 序号输出（供上层统计）

- 新增重载：

```cpp
bool sendPose(CommandID cmd, float vx, float vy, float wz, uint8_t& out_seq);
```

- 上层 `rc26_merge_odom::PoseSender` 已接入序号统计与 1s 窗口日志。

---

## 构建与测试

```bash
colcon build --parallel-workers 1 --packages-select rc26_serial
colcon test --packages-select rc26_serial
colcon test-result --verbose
```

已新增单测：

- `test_ring_parser`
- `test_adaptive_timeout`

---

## 实机验收建议

1. 正常链路下确认 ODOM 收帧稳定（目标 100Hz）。
2. 热拔插串口，确认 `sendCommand` 在断链后快速返回 `false`，且不出现长时间超时重试。
3. 人为制造错误帧，观察 `CommHealth` 可在恢复后回落至 `HEALTHY`。
4. 目标机执行 `strace -e epoll_wait`，确认接收线程走 `epoll`（无 `select`）。
