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

上位机下发给 MCU 的 `POSE_FEEDBACK/POSE_TARGET` 仍保持 `(vx, vy, wz)` 三浮点协议不变，但当前编号与发送口径已经更新为：

- `POSE_FEEDBACK = 0x1E`
- `POSE_TARGET = 0x1F`
- 两者都按 `50Hz` 连续发送
- 两者都走公开的 `sendCommandNoAck()` 路径，不等待 ACK

当前协议已经定义前置履带动作相关编号，但遥控链不再通过 `rc26_mechanism` 的 Action 兼容路径消费，而是直接走共享 transport：

- 下行命令：
  - `FRONT_TRACK_UP = 0x0E`
  - `FRONT_TRACK_DOWN = 0x0F`
  - `PUSHROD_EXTEND = 0x10`
  - `PUSHROD_RETRACT = 0x11`
- 上行完成反馈：
  - `FRONT_TRACK_UP_DONE = 0x13`
  - `FRONT_TRACK_DOWN_DONE = 0x14`

当前真实口径是：

- `rc26_telecontrol_front_track_test` 会在按钮按住期间按 `50Hz` 连续调用 `/mechanism/transport/send_command`
- `FRONT_TRACK_UP/DOWN` 通过 `merge_odom` 桥接走 no-ACK 单发，不做重传
- `PUSHROD_EXTEND/RETRACT` 通过共享 transport 走可靠 `sendCommand()` ACK 路径；只要求 MCU 回 `ACK`，不要求再上送独立的完成反馈
- `0x13 / 0x14` 仍作为这两个动作的完成反馈发布到 `/mechanism/transport/feedback`
- 真机部署时，目标 MCU 串口仍由 `rc26_merge_odom` 独占打开；其它上层只复用 transport，不再次直连同一设备

## 源码入口与阅读顺序
- 先看 `src/serial_driver.cpp`，这是整个仓库复用的串口底座。
- 再看两个测试文件，理解 ACK/RTO 和环形解析器的验收点。
- 最后回到上层调用者，比如 `rc26_mechanism` 或 `rc26_merge_odom`，确认是哪一层在赋予业务语义，以及哪一层拥有真实串口所有权。

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
- 它不做上层动作语义，具体业务封装在 `rc26_mechanism`、`rc26_merge_odom` 等包里
- 当业务异常时，要区分是协议层问题还是上层状态机问题，不能把所有故障都归到这个库
