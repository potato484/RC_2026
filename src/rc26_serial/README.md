# rc26_serial

`rc26_serial` 是 R2 目标 MCU 的 UART 帧协议与 I/O 基础库。它负责帧头帧尾、CRC32 MPEG-2、流式解析、可靠 ACK 重试、自适应超时、no-ack 连续发送和真实 I/O 故障重连，不拥有比赛动作状态机。

当前串口唯一运行时 owner 是 `rc26_mcu_transport`。常规机构命令通过 `/mechanism/send_command` 下发，业务反馈由 `/mechanism/command_feedback` 发布，底盘速度由 `/cmd_vel -> POSE_TARGET(0x0C)` 执行。

## 当前下行协议

| ID | CommandID |
|---:|---|
| `0x01` | `GRAB_TIP` |
| `0x02` | `GRAB_KFS_DOWN` |
| `0x03` | `GRAB_KFS_UP` |
| `0x04` | `ARM_RAISE` |
| `0x05` | `ARM_LOWER` |
| `0x08` | `FRONT_PUSHROD_EXTEND` |
| `0x09` | `FRONT_PUSHROD_RETRACT` |
| `0x0A` | `REAR_PUSHROD_EXTEND` |
| `0x0B` | `REAR_PUSHROD_RETRACT` |
| `0x0C` | `POSE_TARGET` |
| `0x0D` | `ARM_HIGH_RAISE` |
| `0x0E` | `ARM_SECOND_LOWER` |
| `0x0F` | `ENTRY_GRAB_KFS_UP` |
| `0x10` | `COMPETITION_START` |
| `0x11` | `SECOND_PRESELECTION_START` |
| `0x12` | `SECOND_PRESELECTION_PICKUP_KFS` |
| `0x13` | `SECOND_PRESELECTION_PLACE_KFS` |
| `0x14` | `SECOND_PRESELECTION_ARM_LOWER` |
| `0x15` | `SECOND_PRESELECTION_PRELOAD_KFS_PICKUP` |
| `0x20` | `STARTUP_READY_WAITING_LIMIT` |

## 当前上行协议

| ID | FeedbackID |
|---:|---|
| `0x00` | `ACK`，只由可靠发送状态机消费 |
| `0x02` | `ARM_RAISE_DONE` |
| `0x03` | `ARM_LOWER_DONE` |
| `0x04` | `FRONT_LASER_HEIGHT_JUMP` |
| `0x05` | `REAR_LASER_HEIGHT_JUMP` |
| `0x06` | `MANUAL_LIMIT_SWITCH_1_TRIGGERED` |
| `0x07` | `FRONT_SECOND_LASER_HEIGHT_JUMP` |
| `0x09` | `ARM_HIGH_RAISE_DONE` |
| `0x0A` | `ARM_SECOND_LOWER_DONE` |
| `0x0B` | `ENTRY_GRAB_KFS_UP_DONE` |
| `0x0C` | `COMPETITION_START_DONE` |
| `0x0D` | `SECOND_PRESELECTION_START_DONE` |
| `0x10` | `MANUAL_LIMIT_SWITCH_2_TRIGGERED` |
| `0x11` | `SECOND_PRESELECTION_PICKUP_KFS_DONE` |
| `0x12` | `SECOND_PRESELECTION_ARM_LOWER_DONE` |
| `0x13` | `MANUAL_LIMIT_SWITCH_3_TRIGGERED` |
| `0x14` | `SECOND_PRESELECTION_PRELOAD_KFS_PICKUP_DONE` |
| `0x15` | `SECOND_PRESELECTION_MANUAL_FRONT_LASER_TRIGGERED` |
| `0xFE` | `MCU_ERROR` |

上下行 ID 空间独立，例如下行 `0x13` 是第二预选赛放置命令，上行 `0x13` 是切换下一次启动 `active_side` 的人工限位事件。保留 ID 不重新编号，已退役位置保持空洞。

可靠命令等待通用 `ACK(0x00)`，最多使用 `retry=0x00~0x09` 发送十次；纯 ACK 超时令本次调用失败，但不触发重连。只有真实读写失败、EOF 或 `epoll` error/hup 等物理 I/O 故障触发重连。两字节 `MCU_ERROR(0xFE)` 的 payload 为 `failed_cmd,error_code`，供上层解释 BUSY 或最终机构错误。

## 本轮同步

2026-07-18：删除无运行调用的停止、旧九宫格放置、心跳和轮速反馈协议，删除旧 second 高抬完成反馈及串口头文件中的决策遗留类型；保留 ID 数值不变，并将当前实机使用的三个人工限位反馈正式纳入协议枚举。
