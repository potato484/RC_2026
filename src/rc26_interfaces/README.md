# rc26_interfaces

`rc26_interfaces` 是 R2 当前跨包 ROS2 接口真源，只定义被现有发布者和消费者使用的消息与服务。

## 当前生成清单

- `msg/TipDetection.msg`
- `msg/TipDetectionArray.msg`
- `msg/MechanismTransportFeedback.msg`
- `srv/SendMechanismTransportCommand.srv`

定位、里程计和导航继续使用标准 ROS 消息与 TF。机构通过 `/mechanism/send_command` 和 `/mechanism/command_feedback` 交互，provider 是 `rc26_mcu_transport`。

`SendMechanismTransportCommand` 的请求字段保持 `command_id`、`payload`、`wait_ack`，响应保持 `accepted`、`seq`。`MechanismTransportFeedback` 保持 `seq`、`feedback_id`、`payload`。本轮没有改变 wire shape。

## 本轮同步

2026-07-18：删除没有发布者或订阅者的机构动作历史和动态预测消息，并移除不再需要的直接 `builtin_interfaces` 依赖。接口存在性以 `CMakeLists.txt` 的 `rosidl_generate_interfaces()` 清单为准。
