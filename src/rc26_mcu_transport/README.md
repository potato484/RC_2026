# rc26_mcu_transport

`rc26_mcu_transport` 是 R2 目标 MCU 的共享串口 transport provider。

它只负责持有目标 MCU 串口并提供 ROS 2 transport 契约，不维护机构动作语义，不发布底盘速度，也不接管 Nav2 / teleop 的 `/cmd_vel`。

## 职责

- 独占打开目标 MCU 串口，默认 `/dev/ttyUSB0 @ 1000000`
- 提供 service `/mechanism/send_command`
- 发布 topic `/mechanism/command_feedback`
- 把 `rc26_serial::SerialDriver::sendCommand()` 的通用 ACK 结果映射为 service response
- 透传 MCU 上行业务反馈，过滤底层 `ACK(0x00)`、`HEARTBEAT_ACK(0x10)` 与 `ODOM_DATA(0x20)`
- 发布 `/mcu_transport/diagnostics`

## 边界

- `rc26_mcu_transport` 是目标 MCU 串口 owner；同一物理口不要再被其它节点直接打开。
- `rc26_mechanism` 仍然是机构动作语义边界；它通过 `shared_serial` HAL 调用 `/mechanism/send_command`。
- `rc26_telecontrol` 前/后推杆 sidecar、`rc26_vision` tip test、`rc26_decision` 台阶/武馆动作都只消费 transport 契约。
- `/cmd_vel` 的底盘硬件消费方仍不由本包提供。

## 运行

```bash
ros2 launch rc26_mcu_transport mcu_transport.launch.py \
  target_serial_port:=/dev/ttyUSB0 \
  target_baudrate:=1000000
```

涉及机构指令的运行链必须先启动或同时启动本服务：

- 遥控前/后推杆 sidecar
- `rc26_mechanism` 的 `/mechanism/grab_tip`、`/mechanism/assemble_weapon`、`/mechanism/run_command`
- `rc26_decision` 中等待限位后下发 `GRAB_TIP` 或台阶推杆命令的动作
- `rc26_vision` tip test 中对齐后下发 `GRAB_TIP` 的链路

如果串口暂时不存在，节点不会退出；`/mechanism/send_command` 会拒绝发送并持续重试初始打开，直到目标串口可用。

`rc26_bringup`、`start_r2_teleop.sh` 和 `rc26_mechanism/launch/mechanism.launch.py` 都能启动本服务；如果同一系统里已经存在一个 `rc26_mcu_transport`，后续入口必须关闭对应的 `start_mcu_transport`，避免重复打开同一物理串口。
