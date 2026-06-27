# rc26_mcu_transport

`rc26_mcu_transport` 是当前 R2 目标 MCU 的共享串口 owner，提供机构 raw transport 与默认底盘 `/cmd_vel` consumer。

## 模块定位

- 独占打开目标 MCU 串口，默认 `/dev/ttyUSB0 @ 1000000`
- 提供 service `/mechanism/send_command`
- 发布 topic `/mechanism/command_feedback`
- 默认订阅 `/cmd_vel`，以 `50Hz` no-ack 路径下发 `POSE_TARGET(0x0C)`，payload 为 `(vx, vy, wz)` 三个 float
- 发布 `/mcu_transport/diagnostics`
- 不维护旧高层机构 action 或动作完成语义

## 运行边界

- 只要运行链涉及真实机构指令，就必须启动本服务。
- 只要运行链需要真实执行 `/cmd_vel`，也必须启动本服务并保持 `enable_chassis_cmd_vel_consumer=true`。
- `rc26_mechanism` 当前只是轻量 lifecycle 占位；本包才是目标 MCU 串口 provider。
- `rc26_telecontrol` 前/后推杆 sidecar、`rc26_decision` 台阶/武馆动作、`rc26_vision` tip test 和默认关闭的 KFS action test 都消费 `/mechanism/send_command`、`/mechanism/command_feedback` 或发布 `/cmd_vel`，不直接打开目标 MCU 串口。
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
  - `accepted=true` 表示目标 MCU 已返回通用 `ACK(0x00)`，`seq` 为串口帧序号
- `/mechanism/command_feedback`
  - type: `rc26_interfaces/msg/MechanismTransportFeedback`
  - 透传机构业务反馈，过滤底层 `ACK(0x00)`、`HEARTBEAT_ACK(0x01)` 与 `ODOM_DATA(0x08)`
  - 当前会透传 KFS 机械臂升降完成 `0x02/0x03`、台阶激光事件 `0x04/0x05/0x07`、前方限位事件 `0x06` 与第二节机械臂放下完成 `0x0A`；service 的 `accepted=true` 仍只表示可靠命令已收到通用 `ACK(0x00)`
- `/mcu_transport/diagnostics`
  - type: `diagnostic_msgs/msg/DiagnosticArray`
  - 暴露串口打开状态、ACK 超时、解析错误、重连次数、机构发送统计、底盘 `POSE_TARGET` 发送统计和最近错误
- `/cmd_vel`
  - type: `geometry_msgs/msg/Twist`
  - 默认启用，按 `chassis_v_max_mps=2.0` 与 `chassis_w_max_radps=2.0` 限幅，超时 `200ms` 后补发 `10` 帧零速

## 协议口径

当前 raw transport 直接发送 `rc26_serial::CommandID`。常用下行 ID：

- `GRAB_TIP = 0x01`
- `GRAB_KFS_DOWN = 0x02`
- `GRAB_KFS_UP = 0x03`
- `ARM_RAISE = 0x04`
- `ARM_LOWER = 0x05`
- `PLACE_KFS_GRID = 0x06`
- `FRONT_PUSHROD_EXTEND = 0x08`
- `FRONT_PUSHROD_RETRACT = 0x09`
- `REAR_PUSHROD_EXTEND = 0x0A`
- `REAR_PUSHROD_RETRACT = 0x0B`
- `POSE_TARGET = 0x0C`
- `ARM_HIGH_RAISE = 0x0D`
- `ARM_SECOND_LOWER = 0x0E`

本包不新增业务命令目录。新增机构命令时，先在 `rc26_serial/protocol.hpp` 定义原始 ID，再由需要该能力的上层直接调用 `/mechanism/send_command`。只有重新设计高层动作语义时，才需要恢复 action、完成反馈和中间层契约。

KFS 阶梯等待测试链属于直接 transport service 消费场景：决策层发送 `ARM_RAISE(0x04)` / `ARM_LOWER(0x05)`，并等待同 `seq` 的 `0x02/0x03` 完成反馈。梅林预选赛入口 1/3 阶梯探测会发送 `ARM_HIGH_RAISE(0x0D)`，并等待同 `seq` 的 `ARM_HIGH_RAISE_DONE(0x09)`；本包只透传 raw command 与业务反馈，不赋予高抬升额外动作语义。KFS 向下夹取在视觉锁定开环距离后还会发送 `ARM_SECOND_LOWER(0x0E)`，并等待同 `seq` 的 `ARM_SECOND_LOWER_DONE(0x0A)` 后才允许前进。`rc26_vision` 独立 KFS action test 也只把本包当 raw transport provider，开启后先按方向发送 `ARM_RAISE(0x04)` / `ARM_LOWER(0x05)` 并订阅完成反馈；`direction=down` 时开环前再发送 `ARM_SECOND_LOWER(0x0E)` 等待 `0x0A`，随后发送空 payload 的 `GRAB_KFS_DOWN(0x02)`，`direction=up` 仍直接趋近并发送 `GRAB_KFS_UP(0x03)`；本包的 service `accepted=true` 仍只代表通用 ACK，KFS 物理夹取成功由视觉节点或决策节点通过原目标消失验证判断。

## 本轮同步

2026-06-27 同步：KFS 向下夹取新增 `ARM_SECOND_LOWER(0x0E)` / `ARM_SECOND_LOWER_DONE(0x0A)`，本包仅作为 raw transport provider 透传命令和反馈，service wire shape 不变。

2026-06-26 同步：串口协议 ID 连续化后，本包文档同步到 `POSE_TARGET(0x0C)`、`ODOM_DATA(0x08)`、`HEARTBEAT_ACK(0x01)` 和新的机构业务反馈 ID。梅林预选赛新增 `ARM_HIGH_RAISE(0x0D)` / `ARM_HIGH_RAISE_DONE(0x09)` 后，本包仍保持 raw transport provider 职责，service wire shape 不变。
