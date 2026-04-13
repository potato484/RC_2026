# rc26_mechanism 调试

## 模块定位

`rc26_mechanism` 是 R2 的机构执行与生命周期管理边界，负责把上层动作语义下发到底层 MCU，并通过 HAL 复用真实串口、共享 transport、仿真或回放链路。

## 适用场景

- 单独验证 lifecycle、Action 和 transport 服务
- 排查机构动作为什么被拒绝、超时或取消
- 与 `rc26_merge_odom` 或 `start_r2_teleop.sh --stack minimal-mcu` 联调共享 transport

## 前置条件

- 已 `source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"`
- 如果用 `shared_serial`，需要先有 `rc26_merge_odom` 或 `pose_sender_node` 持有目标串口
- 如果用 `serial`，需要确认目标串口不与别的节点冲突

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_interfaces rc26_serial rc26_mechanism rc26_merge_odom
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐启动

真实共享串口链：

```bash
ros2 launch rc26_merge_odom merge_odom.launch.py chassis_model:=tracked_diff
ros2 launch rc26_mechanism mechanism.launch.py hal_type:=shared_serial
```

最小 MCU 共享链：

```bash
./start_r2_teleop.sh --stack minimal-mcu
ros2 launch rc26_mechanism mechanism.launch.py hal_type:=shared_serial
```

单包隔离：

```bash
ros2 launch rc26_mechanism mechanism.launch.py hal_type:=serial
```

## 最小验收

```bash
ros2 lifecycle get /mechanism_server
ros2 lifecycle set /mechanism_server configure
ros2 lifecycle set /mechanism_server activate
ros2 topic echo /mechanism/state --once
ros2 action send_goal /mechanism/execute rc26_interfaces/action/ExecuteMechanism \
  "{command_id: 7, payload: [], timeout_sec: 5.0}" --feedback
```

共享 transport 验证：

```bash
ros2 service call /mechanism/transport/send_command \
  rc26_interfaces/srv/SendMechanismTransportCommand "{command_id: 14, payload: []}"
ros2 topic echo /mechanism/transport/feedback --once
```

## 常用动作验收

```bash
ros2 action send_goal /mechanism/grab_tip rc26_interfaces/action/GrabTip "{tip_index: 0}" --feedback
ros2 action send_goal /mechanism/place_kfs_grid rc26_interfaces/action/PlaceKFSGrid "{grid_position: 1, layer: 0}" --feedback
ros2 action send_goal /mechanism/assemble_weapon rc26_interfaces/action/AssembleWeapon "{}" --feedback
```

## 优先排查

- Action 被拒绝：先看 lifecycle 是否已经 `activate`。
- `shared_serial` 起不来：先确认 `/mechanism/transport/send_command` 与 `/mechanism/transport/feedback` 是否存在。
- 直连串口打不开：优先排查 `/dev/ttyUSB1` 是否被 `rc26_merge_odom` 占用。

## 相关入口

- [遥控启动](./遥控启动.md)
- [决策启动](./决策启动.md)
- [rc26_merge_odom调试](./rc26_merge_odom调试.md)
