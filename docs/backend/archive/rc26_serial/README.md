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
- 可靠命令 ACK 窗口内同 `seq` 业务反馈短暂延迟投递
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
| `ARM_HIGH_RAISE` | `0x0D` |
| `ARM_SECOND_LOWER` | `0x0E` |
| `ENTRY_GRAB_KFS_UP` | `0x0F` |

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
| `ARM_HIGH_RAISE_DONE` | `0x09` |
| `ARM_SECOND_LOWER_DONE` | `0x0A` |
| `ENTRY_GRAB_KFS_UP_DONE` | `0x0B` |
| `MCU_ERROR` | `0xFE` |

已经移除的旧协议项包括旧组装动作、通用 KFS 夹取动作、旧动作完成反馈和旧即时负确认语义。可靠命令只通过通用 `ACK(0x00)` 成功；同 `seq` 收到 `MCU_ERROR(0xFE)` 表示下位机原因，串口层会按现有 retry `0x00~0x09` 继续重发，若后续收到 `ACK(0x00)` 则成功，若持续收到 `0xFE` 则失败并在 `lastError()` / diagnostics 中明确写出下位机原因。未收到确认时仍通过超时或断链失败收敛。心跳只通过 `HEARTBEAT_ACK(0x01)` 成功，心跳收到 `0xFE` 会记录为下位机原因但不按串口断链触发重连。

## 运行时口径

- `POSE_TARGET(0x0C)` payload 为 `(vx, vy, wz)` 三个 float，由 `rc26_mcu_transport` 默认按 `50Hz` no-ack 路径从 `/cmd_vel` 下发。
- `ODOM_DATA(0x08)` payload 固定为 `<v_fl, v_rl, v_rr, v_fr>`，共 `16B / 4 float`，保留为麦克纳姆四轮反馈格式。
- 4 条推杆命令通过 `/mechanism/send_command` 走可靠 `sendCommand()`，service `accepted=true` 只表示 MCU 已返回通用 `ACK(0x00)`。
- `sendCommand()` 等待 `ACK(0x00)` 期间，若接收线程先拿到同 `seq` 的非控制业务反馈，会在串口层短暂缓存并于 ACK 成功后延迟投递给上层 receive callback；ACK 成功后的极短窗口内继续到达的同 `seq` 业务反馈也按同一口径延迟投递。这样 `/mechanism/send_command` 有机会先写入 `accepted=true + seq`，随后 `/mechanism/command_feedback` 再发布对应 done。`ACK(0x00)`、`HEARTBEAT_ACK(0x01)`、`MCU_ERROR(0xFE)` 不进入该缓存，仍按可靠发送/心跳/错误语义即时处理；失败、超时、重试、关闭或重连会丢弃本轮 ACK 窗口缓存。
- `ARM_RAISE_DONE(0x02)` / `ARM_LOWER_DONE(0x03)` 是 KFS 机械臂预调完成反馈，决策层按同 `seq + feedback_id` 匹配。
- `GRAB_KFS_DOWN(0x02)` / `GRAB_KFS_UP(0x03)` 是当前 KFS 下台阶/下降方向与上台阶/抬升方向夹取命令；transport 仍只提供通用 ACK，物理夹取是否成功由上层视觉消失验证等业务逻辑判断。
- `ARM_HIGH_RAISE(0x0D)` / `ARM_HIGH_RAISE_DONE(0x09)` 只服务梅林区预选赛入口 1/3 阶梯全域探测前的机械臂底座高抬升；它不替代普通 `ARM_RAISE(0x04)`，决策层仍按同 `seq + feedback_id` 匹配完成。
- `ARM_SECOND_LOWER(0x0E)` / `ARM_SECOND_LOWER_DONE(0x0A)` 只服务 KFS 向下夹取：上层在 `ARM_LOWER_DONE(0x03)` 后完成视觉横移对齐与一次锁深度，进入开环前进前先发送 `0x0E`，并等待同 `seq` 的 `0x0A` 后才允许前进或直接夹取。
- `ENTRY_GRAB_KFS_UP(0x0F)` / `ENTRY_GRAB_KFS_UP_DONE(0x0B)` 只服务梅林预选赛入口高侧 KFS 夹取链；决策层在入口 1/3 阶梯高侧锁定目标、横移复核并完成开环趋近后发送 `0x0F`，service ACK 仍只代表通用 `ACK(0x00)`，随后必须按同 `seq` 等待 `0x0B` 进入视觉消失验证。
- `MCU_ERROR(0xFE)` 是 MCU 端错误码，不是机构业务完成反馈；它只用于说明本轮失败来自下位机原因。可靠发送会继续重试，最终失败不会触发串口重连，调用方通过 service `accepted=false`、节点日志和 diagnostics `last_error` 判断。
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
3. 再看 `test/test_protocol_ids.cpp`、`test/test_ring_parser.cpp`、`test/test_adaptive_timeout.cpp`、`test/test_serial_driver_mcu_error.cpp`，确认协议常量和基础通信回归点。
4. 最后回到 `rc26_mcu_transport`、`rc26_decision`、`rc26_telecontrol`、`rc26_vision` 等消费者，确认哪一层在使用 raw transport。

## 本轮同步

2026-07-01 同步：`sendCommand()` 新增可靠 ACK 窗口内同 `seq` 业务反馈延迟投递机制。若 MCU 将 `ACK(0x00)` 和 `ARM_RAISE_DONE(0x02)` 等业务 done 背靠背发回，串口层会先让 ACK 唤醒 service 调用，再把同 `seq` 非控制反馈通过短延迟队列交给 receive callback，避免 `/mechanism/command_feedback` 早于 `/mechanism/send_command` response。该逻辑只在 `rc26_serial` 内部调整时序，不改变协议 ID、payload、ROS service/topic wire shape，也不缓存 `MCU_ERROR(0xFE)`。

2026-06-30 同步：把梅林预选赛已经使用的入口高侧 KFS 夹取协议收回到 `rc26_serial` 真源：新增下行 `ENTRY_GRAB_KFS_UP(0x0F)` 与上行 `ENTRY_GRAB_KFS_UP_DONE(0x0B)`。该命令仍走 raw transport 空 payload，`/mechanism/send_command.accepted=true` 只表示通用 ACK，动作完成需要同 `seq` 的 `0x0B`，后续物理夹取成功仍由决策侧视觉消失验证提交。

2026-06-29 同步：新增 MCU 上行 `MCU_ERROR(0xFE)`，表示可靠发送失败原因来自下位机。串口层同 `seq` 收到 `0xFE` 会唤醒 ACK 等待并沿用 retry `0x00~0x09` 重发；如果后续收到 `ACK(0x00)` 则成功，如果持续 `0xFE` 则失败并把“下位机原因”写入 `lastError()` 与 diagnostics，不触发串口重连。该反馈不属于机构业务完成事件。

2026-06-27 同步：新增 `ARM_SECOND_LOWER(0x0E)` 与 `ARM_SECOND_LOWER_DONE(0x0A)`，用于 KFS 向下夹取在锁定开环前进距离后、真正前进前确认第二节机械臂已经彻底放下；该命令仍走 raw transport 空 payload，完成反馈必须按同 `seq` 匹配。

2026-06-26 同步：串口协议 ID 连续化并移除旧机构完成语义。`STOP(0x00)` 与 `ACK(0x00)` 保留；`GRAB_KFS_DOWN/UP` 调整为 `0x02/0x03`，`POSE_TARGET` 调整为 `0x0C`，`ODOM_DATA` 调整为 `0x08`。梅林预选赛新增 `ARM_HIGH_RAISE(0x0D)` 与 `ARM_HIGH_RAISE_DONE(0x09)`，固件和上位机必须使用同一张表。旧高层 mechanism action 与旧动作完成反馈不再作为当前协议或业务契约。
