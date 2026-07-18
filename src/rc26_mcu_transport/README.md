# rc26_mcu_transport

`rc26_mcu_transport` 是 R2 目标 MCU 的唯一共享串口 owner，同时提供机构 raw transport 与默认底盘 `/cmd_vel` consumer。

## 接口

- `/mechanism/send_command`：`rc26_interfaces/srv/SendMechanismTransportCommand`
- `/mechanism/command_feedback`：`rc26_interfaces/msg/MechanismTransportFeedback`
- `/mcu_transport/diagnostics`：`diagnostic_msgs/msg/DiagnosticArray`
- `/cmd_vel`：默认按 50Hz 转成 `POSE_TARGET(0x0C)` no-ack 帧

`wait_ack=true` 时，`accepted=true` 只表示目标 MCU 返回通用 `ACK(0x00)`；`wait_ack=false` 表示串口帧写入成功，不等待 ACK 或业务完成。service 保持 raw `uint8 command_id` 透传，不在 provider 增加下行 allowlist。

## 上行发布边界

transport 只发布正式业务反馈 `0x02~0x07`、`0x09~0x0D`、`0x10~0x15`，其中退役位置 `0x0F` 不发布；两字节 `MCU_ERROR(0xFE)` 作为机构业务状态发布。通用 ACK、未知 ID、退役 ID 和非法长度 `0xFE` 不进入 `/mechanism/command_feedback`。

diagnostics 除串口、ACK、重连、机构发送和底盘统计外，还提供：

- `unsupported_feedback_drop_count`
- `last_unsupported_feedback_id`

## 启动

```bash
ros2 launch rc26_mcu_transport mcu_transport.launch.py \
  target_serial_port:=/dev/ttyUSB0 \
  target_baudrate:=1000000
```

`rc26_bringup` 和 `start_r2_teleop.sh` 都会按各自运行配置启动本节点。同一物理串口只能存在一个 owner。

## 本轮同步

2026-07-18：机构和底盘执行权威完全收口到本包；上行业务反馈改为显式 allowlist，并增加未知反馈丢弃 diagnostics。已删除的历史里程计桥和机构生命周期占位包不再提供替代启动入口。
