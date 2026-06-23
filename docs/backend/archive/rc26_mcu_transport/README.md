# rc26_mcu_transport

`rc26_mcu_transport` 是当前 R2 目标 MCU 的共享串口 owner，提供机构 transport 与默认底盘 `/cmd_vel` consumer。

## 模块定位

- 独占打开目标 MCU 串口，默认 `/dev/ttyUSB0 @ 1000000`
- 提供 service `/mechanism/send_command`
- 发布 topic `/mechanism/command_feedback`
- 默认订阅 `/cmd_vel`，以 `50Hz` no-ack 路径下发 `POSE_TARGET(0x1F)`，payload 为 `(vx, vy, wz)` 三个 float
- 发布 `/mcu_transport/diagnostics`
- 不维护机构动作语义；机构动作语义仍归 `rc26_mechanism`

## 运行边界

- 只要运行链涉及机构动作指令，就必须启动本服务。
- 只要运行链需要真实执行 `/cmd_vel`，也必须启动本服务并保持 `enable_chassis_cmd_vel_consumer=true`。
- `rc26_mechanism` 仍是机构动作语义边界；本包只是串口 transport provider 与底盘速度帧发送方。
- `rc26_telecontrol` 前/后推杆 sidecar、`rc26_decision` 台阶/武馆动作和 `rc26_vision` tip test 都消费 `/mechanism/send_command`、`/mechanism/command_feedback` 或发布 `/cmd_vel`，不直接打开目标 MCU 串口。
- 同一物理目标 MCU 串口只能由本包打开；其它上层包不得再次直连同一设备。
- `rc26_merge_odom` 中保留的历史 bridge 代码不再作为默认 provider。

## 启动入口

```bash
ros2 launch rc26_mcu_transport mcu_transport.launch.py \
  target_serial_port:=/dev/ttyUSB0 \
  target_baudrate:=1000000
```

`rc26_bringup` 会按 `r2_runtime.mcu_transport` 配置启动本服务；`test_navigation.launch.py` 显式关闭它，因为导航联调入口只验证 `/cmd_vel` 输出而不真实动底盘。根目录 `start_r2_teleop.sh` 会启动本服务，并默认打开底盘 consumer。`rc26_mechanism/launch/mechanism.launch.py` 也会默认启动本服务；如果同一系统里已经有 provider，需要传 `start_mcu_transport:=false`。

如果串口暂时不存在，节点不会退出；`/mechanism/send_command` 会拒绝发送，`/cmd_vel` consumer 会节流报告发送失败，并在 diagnostics 中暴露串口不可用状态，同时持续重试初始打开。

## 接口

- `/mechanism/send_command`
  - type: `rc26_interfaces/srv/SendMechanismTransportCommand`
  - `accepted=true` 表示目标 MCU 已返回通用 ACK，`seq` 为串口帧序号
- `/mechanism/command_feedback`
  - type: `rc26_interfaces/msg/MechanismTransportFeedback`
  - 透传机构业务反馈，过滤底层 `ACK(0x00)`、`HEARTBEAT_ACK(0x10)` 与 `ODOM_DATA(0x20)`
- `/mcu_transport/diagnostics`
  - type: `diagnostic_msgs/msg/DiagnosticArray`
  - 暴露串口打开状态、ACK 超时、解析错误、重连次数、机构发送统计、底盘 `POSE_TARGET` 发送统计和最近错误
- `/cmd_vel`
  - type: `geometry_msgs/msg/Twist`
  - 默认启用，按 `chassis_v_max_mps=2.0` 与 `chassis_w_max_radps=2.0` 限幅，超时 `200ms` 后补发 `10` 帧零速

## 维护说明

`rc26_mcu_transport` 不新增业务命令目录。新增机构命令仍按既有顺序维护：

1. 在 `rc26_serial/protocol.hpp` 增加原始 `CommandID` / `FeedbackID`
2. 在 `rc26_mechanism` 命令目录中增加动作语义
3. 上层通过 `/mechanism/run_command`、专用 action 或直接 transport service 消费

## 本轮同步

2026-06-23 同步：`rc26_mcu_transport` 从单纯机构 transport 扩展为目标 MCU 串口 owner 和默认底盘 `/cmd_vel` consumer。底盘速度通过已有 `POSE_TARGET(0x1F)` no-ack 帧下发，线速度/角速度默认上限均为 `2.0`；`rc26_merge_odom` 仍保持归档状态，不重新进入默认执行链。
