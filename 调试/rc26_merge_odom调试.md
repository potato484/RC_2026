# rc26_merge_odom 调试

## 模块定位

`rc26_merge_odom` 负责多源里程计融合、位姿下发和目标 MCU 串口桥接，也是遥控链和机构共享 transport 的真实串口持有者。

## 适用场景

- 验证底盘执行链、目标串口和 `/cmd_vel` 下发
- 切换 `wheel odom / can odom / fused` 口径
- 排查共享 transport、最小 MCU 栈和 wheel-only 口径

## 前置条件

- 已 `source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"`
- 目标串口可用；如果需要反馈链，反馈串口也应可用
- 若已通过建图或里程计总装链发布 `odom -> base_link`，不要再叠加另一条 TF 权威链

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_merge_odom rc26_telecontrol rc26_mechanism
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐启动

基础底盘执行链：

```bash
ros2 launch rc26_merge_odom merge_odom.launch.py
```

切到 CAN 里程计：

```bash
ros2 launch rc26_merge_odom merge_odom.launch.py use_can_odom:=true
```

wheel-only：

```bash
ros2 launch rc26_merge_odom merge_odom.launch.py \
  start_ekf:=true use_imu_for_ekf:=false start_imu:=false
```

fused 拓扑：

```bash
ros2 launch rc26_merge_odom merge_odom_fused.launch.py
```

最小 MCU 栈：

```bash
./start_r2_teleop.sh --stack minimal-mcu
```

## 最小验收

```bash
ros2 param get /merge_odom_node cmd_vel_timeout_ms
ros2 topic echo /pose_sender/target_protected --once
ros2 topic echo /mechanism/transport/feedback --once
ros2 topic echo /wheel_odom_fuser/health --once
ros2 topic echo /can_odom/slip_score --once
```

横向速度链路验收：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.20, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
ros2 topic echo /pose_sender/target_protected --once
```

保护链测试：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 5.0, y: 5.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 3.0}}" -1
```

## 优先排查

- `minimal-mcu` 栈没有 transport：先确认 `pose_sender_node` 已持有 `target_serial_port`。
- `wheel-only` 仍像在读 IMU：先确认命令里真的包含 `start_imu:=false`。
- `/pose_sender/target_protected.linear.y` 长期为 0：先确认上游 `/cmd_vel` 已带 `linear.y`，再检查 `feedback_serial_port` / `target_serial_port` 是否起在当前修改后的麦克纳姆参数集上。
- TF 冲突：如果已经通过 `rc26_bringup` 跑建图/里程计链，不要再额外叠同类权威节点。

## 相关入口

- [遥控启动](./遥控启动.md)
- [建图启动](./建图启动.md)
- [rc26_telecontrol调试](./rc26_telecontrol调试.md)
- [rc26_mechanism调试](./rc26_mechanism调试.md)
