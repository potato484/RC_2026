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

## 模块边界

- 这个包是基础通信库，不直接代表某个业务节点
- 它不做上层动作语义，具体业务封装在 `rc26_mechanism`、`rc26_merge_odom` 等包里
- 当业务异常时，要区分是协议层问题还是上层状态机问题，不能把所有故障都归到这个库
