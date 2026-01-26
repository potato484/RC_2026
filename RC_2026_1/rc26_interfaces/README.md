# rc26_interfaces

`rc26_interfaces` 用于统一定义本项目的自定义 ROS2 消息（msg）与服务（srv），保证各模块之间通信数据结构一致。

## 已定义接口

### msg/NavSafetyState

导航安全状态发布消息（用于上层了解当前导航 Profile/安全策略的执行状态）。

- `std_msgs/Header header`：时间戳与坐标系（如需）
- `string current_profile`：当前生效的导航 Profile 名称（与 `nav_profiles.yaml` 的 key 对应）
- `string reason`：切换/状态更新原因（便于日志与排障）
- `bool stop_required`：是否要求机器人停止（用于上层快速做安全处理）
- `bool timed_out`：是否因为超时进入降级/回退（用于上层判定异常流程）

### srv/SetNavMode

设置导航 Profile（请求-响应）。

请求：
- `string profile`：目标 Profile 名称（与 `nav_profiles.yaml` 的 key 对应）
- `float32 timeout`：可选超时（秒）；为 0 表示使用模块默认值
- `string reason`：切换原因（用于日志记录）

响应：
- `bool success`：是否成功
- `string message`：失败原因或说明

