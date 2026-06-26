# rc26_serial

## 模块定位

`rc26_serial` 是 R2 目标 MCU 串口协议与 I/O 基础库，负责帧结构、CRC、可靠发送、心跳、ACK/RTO、自适应超时、断链快速失败和流式解析。它不拥有任何业务动作语义，也不直接决定“哪个机构动作完成”。

当前真实目标 MCU 串口 owner 由 `rc26_mcu_transport` 承担；机构命令通过 `/mechanism/send_command` 发送，业务反馈通过 `/mechanism/command_feedback` 透传，底盘速度通过 `/cmd_vel -> POSE_TARGET` no-ack 路径下发。

## 当前实现

- 构建产物：共享库 `serial_driver`
- 核心源码：`src/rc26_serial/src/serial_driver.cpp`
- 协议真源：`src/rc26_serial/include/rc26_serial/protocol.hpp`
- 关键测试：
  - `test/test_adaptive_timeout.cpp`
  - `test/test_ring_parser.cpp`
  - `test/test_protocol_ids.cpp`

当前实现重点：

- 环形缓冲与流式解析
- `epoll` 事件驱动 I/O
- 自适应 ACK/RTO 超时计算
- 断链快速失败
- 滑动窗口健康度统计
- 长度与负载保护

## 当前协议 ID

2026-06-26 起，串口协议上下行 ID 按各自旧顺序重新连续编号。这是 MCU 线协议破坏性变更，固件、调试脚本、配置默认值和外部消费者必须同步切到同一张表。

下行 `CommandID`：

| Name | ID |
|---|---:|
| `STOP` | `0x00` |
| `GRAB_TIP` | `0x01` |
| `GRAB_KFS_DOWN` | `0x02` |
| `GRAB_KFS_UP` | `0x03` |
| `ARM_RAISE` | `0x04` |
| `ARM_LOWER` | `0x05` |
| `PLACE_KFS_GRID` | `0x06` |
| `HEARTBEAT` | `0x07` |
| `FRONT_PUSHROD_EXTEND` | `0x08` |
| `FRONT_PUSHROD_RETRACT` | `0x09` |
| `REAR_PUSHROD_EXTEND` | `0x0A` |
| `REAR_PUSHROD_RETRACT` | `0x0B` |
| `POSE_TARGET` | `0x0C` |

上行 `FeedbackID`：

| Name | ID |
|---|---:|
| `ACK` | `0x00` |
| `HEARTBEAT_ACK` | `0x01` |
| `ARM_RAISE_DONE` | `0x02` |
| `ARM_LOWER_DONE` | `0x03` |
| `FRONT_LASER_HEIGHT_JUMP` | `0x04` |
| `REAR_LASER_HEIGHT_JUMP` | `0x05` |
| `FRONT_LIMIT_SWITCH_TRIGGERED` | `0x06` |
| `FRONT_SECOND_LASER_HEIGHT_JUMP` | `0x07` |
| `ODOM_DATA` | `0x08` |

已经移除的旧协议项包括旧组装动作、通用 KFS 夹取动作、旧动作完成反馈和即时负确认语义。可靠命令只通过通用 `ACK(0x00)` 成功；未收到确认时通过超时或断链失败收敛。心跳只等待 `HEARTBEAT_ACK(0x01)`。

## 运行时口径

- `POSE_TARGET(0x0C)` payload 为 `(vx, vy, wz)` 三个 float，由 `rc26_mcu_transport` 默认按 `50Hz` no-ack 路径从 `/cmd_vel` 下发。
- `ODOM_DATA(0x08)` payload 固定为 `<v_fl, v_rl, v_rr, v_fr>`，共 `16B / 4 float`，保留为麦克纳姆四轮反馈格式。
- 4 条推杆命令通过 `/mechanism/send_command` 走可靠 `sendCommand()`，service `accepted=true` 只表示 MCU 已返回通用 `ACK(0x00)`。
- `ARM_RAISE_DONE(0x02)` / `ARM_LOWER_DONE(0x03)` 是 KFS 机械臂预调完成反馈，决策层按同 `seq + feedback_id` 匹配。
- `FRONT_LASER_HEIGHT_JUMP(0x04)`、`REAR_LASER_HEIGHT_JUMP(0x05)`、`FRONT_SECOND_LASER_HEIGHT_JUMP(0x07)` 是台阶激光高度突变事件，v1 payload 为空或忽略。
- `FRONT_LIMIT_SWITCH_TRIGGERED(0x06)` 是武馆前方限位开关触发事件，视觉夹取链在对齐后 x 负向前探等待该事件，再下发 `GRAB_TIP(0x01)`。
- `PLACE_KFS_GRID(0x06)` 若仍需发送，只能作为 raw transport 命令走 `/mechanism/send_command`，不再绑定高层“完成反馈即成功”的封装。

## 模块边界

- `rc26_serial` 只定义原始协议 ID、封帧、ACK/RTO 和串口 I/O。
- `rc26_serial` 不维护高层机构 action、命令 catalog 或动作完成语义。
- 真实整车部署下，同一目标 MCU 串口只能由 `rc26_mcu_transport` 打开；视觉、决策、遥控和 mechanism 占位节点只能复用 transport。
- 如果协议 ID 变更，必须同步更新 `rc26_serial` 测试、`rc26_mcu_transport` 过滤逻辑、`rc26_decision` 默认参数、`rc26_bringup` 配置、遥控/视觉文档以及 MCU 固件。

## 源码入口与阅读顺序

1. 先看 `include/rc26_serial/protocol.hpp`，确认当前上下行 ID。
2. 再看 `src/serial_driver.cpp`，理解可靠发送、心跳、no-ack 和回调分发。
3. 再看 `test/test_protocol_ids.cpp`、`test/test_ring_parser.cpp`、`test/test_adaptive_timeout.cpp`，确认协议常量和基础通信回归点。
4. 最后回到 `rc26_mcu_transport`、`rc26_decision`、`rc26_telecontrol`、`rc26_vision` 等消费者，确认哪一层在使用 raw transport。

## 本轮同步

2026-06-26 同步：串口协议 ID 连续化并移除旧机构完成语义。`STOP(0x00)` 与 `ACK(0x00)` 保留；`GRAB_KFS_DOWN/UP` 调整为 `0x02/0x03`，`POSE_TARGET` 调整为 `0x0C`，`ODOM_DATA` 调整为 `0x08`。旧高层 mechanism action 与旧动作完成反馈不再作为当前协议或业务契约。
