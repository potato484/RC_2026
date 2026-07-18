# rc26_interfaces

`rc26_interfaces` 是 R2 跨包 ROS2 接口真源，不包含业务逻辑。

## 当前接口

- `TipDetection.msg`、`TipDetectionArray.msg`：`/vision/tip_detections`
- `MechanismTransportFeedback.msg`：`/mechanism/command_feedback`
- `SendMechanismTransportCommand.srv`：`/mechanism/send_command`

导航只公开标准 `geometry_msgs/msg/Twist` `/cmd_vel`，定位只使用标准 pose、diagnostics 与 TF。本包不生成自定义导航、定位或机构高层 action。

机构 service/topic 由 `rc26_mcu_transport` 提供。service 的 `accepted=true` 在可靠模式下只表示通用 ACK，在 no-ack 模式下只表示串口写入成功；动作完成由上层按业务反馈解释。

## 维护边界

- 接口生成清单以 `src/rc26_interfaces/CMakeLists.txt` 为准。
- 跨模块语义同步维护在 `docs/middle/modules/*.yaml`。
- 删除接口后应清理本包 `build/`、`install/` 生成物再构建，避免旧 rosidl 文件造成假接口。

## 本轮同步

2026-07-18：删除无生产者和消费者的机构动作历史、动态预测消息及其文档契约，当前接口包只保留机构 raw transport 与端头检测实际契约。
