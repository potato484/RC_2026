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
- 上位机默认由 `rc26_merge_odom::WheelOdom` 解析，并继续换算 `vx / vy / wz`

上位机下发给 MCU 的 `POSE_FEEDBACK/POSE_TARGET` 仍保持 `(vx, vy, wz)` 三浮点协议不变，但当前编号与发送口径已经更新为：

- `POSE_FEEDBACK = 0x1E`
- `POSE_TARGET = 0x1F`
- 两者都按 `50Hz` 连续发送
- 两者都走公开的 `sendCommandNoAck()` 路径，不等待 ACK

当前双推杆协议已经直接收口为前/后推杆四命令；遥控链也不再通过 `rc26_mechanism` 的 Action 兼容路径消费，而是直接走共享 transport：

- 下行命令：
  - `FRONT_PUSHROD_EXTEND = 0x0E`
  - `FRONT_PUSHROD_RETRACT = 0x0F`
  - `REAR_PUSHROD_EXTEND = 0x10`
  - `REAR_PUSHROD_RETRACT = 0x11`
- 上行业务反馈：
  - `FRONT_PUSHROD_EXTEND_ACK = 0x13`
  - `FRONT_PUSHROD_RETRACT_ACK = 0x14`
  - `REAR_PUSHROD_EXTEND_ACK = 0x15`
  - `REAR_PUSHROD_RETRACT_ACK = 0x16`

当前真实口径是：

- `rc26_telecontrol_front_pushrod_buttons` 会在 `Y/A` 按下沿单次调用 `/mechanism/transport/send_command`
- `rc26_telecontrol_rear_pushrod_buttons` 会在 `Select/Back` / `Start` 按下沿单次调用 `/mechanism/transport/send_command`
- 4 条双推杆命令都通过 `merge_odom` 桥接走可靠 `sendCommand()` ACK 路径；若 MCU 不回通用 `ACK(0x00)`，会像其它可靠命令一样自动重传并打印超时日志
- `0x13~0x16` 业务 ACK 会继续发布到 `/mechanism/transport/feedback`，但不参与 `sendCommand()` 的可靠 ACK 判定
- `Dpad 左/右` 已回归底盘横移控制
- 真机部署时，目标 MCU 串口仍由 `rc26_merge_odom` 独占打开；其它上层只复用 transport，不再次直连同一设备

当前还保留一条仅供 `rc26_vision` tip 端头 test 节点复用的轻量 no-ACK 下行命令：

- `TIP_VISION = 0x12`
- payload 固定为 `5B`：`[grab_ready, dir_code, amp_code, ts16_lo, ts16_hi]`
- 当前由 `tip_vision_test_node` 通过 `sendCommandNoAck()` 发送，复用统一 RC26 封帧、CRC 和重连逻辑，但它仍属于端头视觉独立 test 链，不是共享 transport 的正式运行时权威口径

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
- 即使 `tip` test 节点现在复用了这个库，真实整车部署下目标 MCU 串口的权威所有者仍然是 `rc26_merge_odom`
