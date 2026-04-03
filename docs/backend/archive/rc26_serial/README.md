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

当前真实协议口径里，`ODOM_DATA(0x20)` 已经不再固定只有四轮 `4 float`：

- 四轮模式：`<v_fl, v_rl, v_rr, v_fr>`
- 履带模式：`<v_left, v_right>`

上位机下发给 MCU 的 `POSE_FEEDBACK/POSE_TARGET` 仍保持 `(vx, vy, wz)` 三浮点协议不变。

## 源码入口与阅读顺序
- 先看 `src/serial_driver.cpp`，这是整个仓库复用的串口底座。
- 再看两个测试文件，理解 ACK/RTO 和环形解析器的验收点。
- 最后回到上层调用者，比如 `rc26_mechanism` 或 `rc26_merge_odom`，确认是哪一层在赋予业务语义。

## 目录解剖
- `serial_driver.cpp`：打开/关闭串口、重连、帧封装、ACK 等待、接收线程和回调分发。
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
- 它不做上层动作语义，具体业务封装在 `rc26_mechanism`、`rc26_merge_odom` 等包里
- 当业务异常时，要区分是协议层问题还是上层状态机问题，不能把所有故障都归到这个库
