# rc26_mcu_transport

`rc26_mcu_transport` 是 R2 目标 MCU 的共享串口 transport provider。

它负责持有目标 MCU 串口，并同时提供机构 transport 契约与默认底盘 `/cmd_vel` consumer。旧 `rc26_mechanism` 高层 action 已下线，Nav2 / teleop / decision 仍只发布 `/cmd_vel`，由本节点把速度转成 `POSE_TARGET(0x0C)` 下发给 MCU。

## 职责

- 独占打开目标 MCU 串口，默认 `/dev/ttyUSB0 @ 1000000`
- 提供 service `/mechanism/send_command`
- 发布 topic `/mechanism/command_feedback`
- 把 `rc26_serial::SerialDriver::sendCommand()` 的通用 ACK 结果映射为 service response
- 默认订阅 `/cmd_vel`，以 `50Hz` no-ack 路径下发 `POSE_TARGET(0x0C)`，payload 为 `(vx, vy, wz)` 三个 float
- 透传 MCU 上行业务反馈，过滤底层 `ACK(0x00)`、`HEARTBEAT_ACK(0x01)` 与 `ODOM_DATA(0x08)`；payload 长度为 2 的 `MCU_ERROR(0xFE)` 作为机械臂业务状态/失败反馈透传给决策层，其它长度的 `0xFE` 仍按 transport 异常记录并丢弃
- 当前透传的业务反馈包括 KFS 机械臂升降完成 `0x02/0x03`、台阶激光事件 `0x04/0x05/0x07`、第一个限位事件 `0x06`、第二节机械臂放下完成 `0x0A`、入口高侧 KFS 夹取完成 `0x0B`、比赛开始完成 `0x0C`、第二预选赛开始/高抬完成反馈 `0x0D/0x0F`、第二限位事件 `0x10`、第二预选赛 KFS 夹取完成 `0x11` 和两字节 `0xFE` 机械臂业务诊断；`/mechanism/send_command.accepted=true` 仍只表示通用 `ACK(0x00)` 已可靠返回
- 发布 `/mcu_transport/diagnostics`，其中 `last_error` 和 `mcu_error_responses` 会暴露 MCU `0xFE` 下位机原因

## 边界

- `rc26_mcu_transport` 是目标 MCU 串口 owner；同一物理口不要再被其它节点直接打开。
- `rc26_mechanism` 当前只是轻量生命周期占位；机构命令语义由直接调用 `/mechanism/send_command` 的上层负责。
- `rc26_telecontrol` 前/后推杆 sidecar、`rc26_vision` tip test、`rc26_decision` 台阶/武馆动作都只消费 transport 契约或发布 `/cmd_vel`，不直接打开串口。
- 默认底盘速度上限为 `chassis_v_max_mps=2.0` 与 `chassis_w_max_radps=2.0`；如需只验证 `/cmd_vel` 输出，可传 `enable_chassis_cmd_vel_consumer:=false`。

## 运行

```bash
ros2 launch rc26_mcu_transport mcu_transport.launch.py \
  target_serial_port:=/dev/ttyUSB0 \
  target_baudrate:=1000000
```

常用底盘参数：

- `enable_chassis_cmd_vel_consumer`：是否订阅 `/cmd_vel`，默认 `true`
- `chassis_cmd_vel_topic`：速度话题，默认 `cmd_vel`
- `chassis_target_send_rate_hz`：`POSE_TARGET` 发送频率，默认 `50`
- `chassis_cmd_vel_timeout_ms`：速度超时，默认 `200`
- `chassis_v_max_mps`：平面线速度上限，默认 `2.0`
- `chassis_w_max_radps`：角速度上限，默认 `2.0`
- `chassis_stop_repeat_n`：超时后补发零速帧数，默认 `10`

涉及机构指令或底盘真实运动的运行链必须先启动或同时启动本服务：

- 遥控前/后推杆 sidecar
- 直接调用 `/mechanism/send_command` 的机构动作链
- `rc26_decision` 中等待限位后下发 `GRAB_TIP` 或台阶推杆命令的动作
- `rc26_vision` tip test 中对齐后下发 `GRAB_TIP` 的链路
- `rc26_decision` 梅林预选赛入口高侧下发 `ENTRY_GRAB_KFS_UP(0x0F)` 并等待 `ENTRY_GRAB_KFS_UP_DONE(0x0B)` 的链路
- `rc26_decision` 在 managed branch gate 内消费 MCU 上行 `0x06/0x10`，并按 first 的 `0x10/0x0C` 或 second 的 `0x11/0x0D` profile 握手后继续或切换目标树的链路
- Nav2、遥控、台阶动作、视觉对齐等发布 `/cmd_vel` 的链路

如果串口暂时不存在，节点不会退出；`/mechanism/send_command` 会拒绝发送，`/cmd_vel` consumer 会节流报告发送失败，并持续重试初始打开，直到目标串口可用。

如果 MCU 对可靠命令返回 transport 级 `MCU_ERROR(0xFE)`，底层会按现有 retry `0x00~0x09` 重发；若后续收到 `ACK(0x00)` 则 service 正常成功，若持续 `0xFE` 则 service 返回 `accepted=false`。payload 长度为 2 的上行 `0xFE` 另按机械臂业务状态/失败反馈发布到 `/mechanism/command_feedback`：`payload[0]` 是 `failed_cmd`，`payload[1]` 是 `error_code`。其中 `error_code=0x01` 表示 BUSY/仍在处理中，消费方继续等待最终反馈；其它已知错误码由决策层写入失败诊断并停止当前动作等待。

`rc26_bringup`、`start_r2_teleop.sh` 和 `rc26_mechanism/launch/mechanism.launch.py` 都能启动本服务；如果同一系统里已经存在一个 `rc26_mcu_transport`，后续入口必须关闭对应的 `start_mcu_transport`，避免重复打开同一物理串口。
