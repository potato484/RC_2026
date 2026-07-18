# rc26_serial

`rc26_serial` 是 R2 目标 MCU UART 帧协议和 I/O 基础库。协议真源是 `src/rc26_serial/include/rc26_serial/protocol.hpp`，运行时唯一串口 owner 是 `rc26_mcu_transport`。

## 运行职责

- 提供 CRC32 MPEG-2 帧编码、流式解析、payload 上限和断链快速失败。
- 可靠命令等待通用 `ACK(0x00)`，使用 `retry=0x00~0x09`；纯 ACK 超时只令本次发送失败。
- no-ack 路径用于 `POSE_TARGET(0x0C)` 和启动就绪通知等明确连续或单次通知。
- 只有真实读写、EOF、`epoll` error/hup 等 I/O 故障触发重连。
- ACK 等待窗口内先到达的同 seq 业务反馈会短暂延迟投递，保证 service response 先于 done topic。

## 正式协议

下行保留 `0x01~0x05`、`0x08~0x15` 和 `0x20` 中在 `CommandID` 明确定义的命令；其中 `0x0C` 是底盘 `POSE_TARGET`，`0x12` 是 `SECOND_PRESELECTION_PICKUP_KFS`。

上行保留通用 `ACK(0x00)`、业务反馈 `0x02~0x07`、`0x09~0x0D`、`0x10~0x15` 和 `MCU_ERROR(0xFE)`。人工限位 1/2/3 分别是 `MANUAL_LIMIT_SWITCH_1_TRIGGERED(0x06)`、`MANUAL_LIMIT_SWITCH_2_TRIGGERED(0x10)`、`MANUAL_LIMIT_SWITCH_3_TRIGGERED(0x13)`。

上下行 ID 空间独立，保留值不因清理而重新编号。完整名称和数值表见包内 `README.md` 与协议头文件。

## 边界

`/mechanism/send_command` 仍是 raw `uint8 command_id` transport，可透传未列入正式枚举的数值；这不代表该数值属于受维护协议。`rc26_mcu_transport` 只把正式上行业务 allowlist 发布到 `/mechanism/command_feedback`，未知或已退役反馈只计入 diagnostics 后丢弃。

## 本轮同步

2026-07-18：删除无调用的具名停止、旧九宫格放置和心跳协议，删除仅服务历史里程计包的轮速反馈及旧 second 高抬完成反馈；同时清理未安装测试节点和协议头中的决策遗留声明。保留协议 ID 数值不变，不提供旧 C++ 枚举兼容别名。
