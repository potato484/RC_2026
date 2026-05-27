# rc26_mechanism 调试

## 模块定位

`rc26_mechanism` 是 R2 的机构执行与生命周期管理边界，负责把上层动作语义下发到底层 MCU，并通过共享串口 HAL 复用 `rc26_merge_odom` 或 `pose_sender_node` 已持有的目标 MCU 链路。

## 适用场景

- 单独验证 lifecycle、Action 和 transport 服务
- 排查机构动作为什么被拒绝、超时或取消
- 与 `rc26_merge_odom` 或 `start_r2_teleop.sh --stack minimal-mcu` 联调共享 transport

## 代码结构速记

- `include/rc26_mechanism/nodes` + `src/nodes`：生命周期节点与 Action 入口
- `include/rc26_mechanism/catalog` + `src/catalog`：命令目录真源
- `include/rc26_mechanism/runtime`：运行时辅助类型
- `include/rc26_mechanism/hal/shared_serial` + `src/hal/shared_serial`：共享串口 HAL
- `test/catalog`、`test/transport`：目录与 transport 回归测试

## 前置条件

- 已 `source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"`
- 真实链路下，需要先有 `rc26_merge_odom` 或 `pose_sender_node` 持有目标串口并提供 `/mechanism/send_command` 与 `/mechanism/command_feedback`
- `hal_type=serial` 已不再支持，不再保留 mechanism 直连目标串口的调试入口

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
ros2 launch rc26_merge_odom merge_odom.launch.py
ros2 launch rc26_mechanism mechanism.launch.py hal_type:=shared_serial
```

最小 MCU 共享链：

```bash
./start_r2_teleop.sh --stack minimal-mcu
ros2 launch rc26_mechanism mechanism.launch.py hal_type:=shared_serial
```

## 最小验收

```bash
ros2 lifecycle get /mechanism_server
ros2 lifecycle set /mechanism_server configure
ros2 lifecycle set /mechanism_server activate
ros2 topic echo /mechanism/status --once
ros2 action send_goal /mechanism/run_command rc26_interfaces/action/ExecuteMechanism \
  "{command_id: 7, payload: [], timeout_sec: 5.0}" --feedback
```

共享 transport 验证：

```bash
ros2 service call /mechanism/send_command \
  rc26_interfaces/srv/SendMechanismTransportCommand "{command_id: 14, payload: []}"
ros2 topic echo /mechanism/command_feedback --once
```

## 常用动作验收

```bash
ros2 action send_goal /mechanism/grab_tip rc26_interfaces/action/GrabTip "{tip_index: 0}" --feedback
ros2 action send_goal /mechanism/assemble_weapon rc26_interfaces/action/AssembleWeapon "{}" --feedback
ros2 action send_goal /mechanism/run_command rc26_interfaces/action/ExecuteMechanism \
  "{command_id: 7, payload: [], timeout_sec: 5.0}" --feedback
```

## 新增命令维护步骤

```text
1. 先在 rc26_serial/protocol.hpp 增加 CommandID / FeedbackID
2. 再在 rc26_mechanism/catalog/mechanism_command_catalog.* 增加命令目录项
3. 如果只需要通用执行，直接调用 /mechanism/run_command
4. 只有需要更强业务语义时，才新增专用 Action 包装
```

## 优先排查

- Action 被拒绝：先看 lifecycle 是否已经 `activate`，再看命令是否已经加入 `mechanism_command_catalog` 且 `execute_supported=true`
- `shared_serial` 起不来：先确认 `/mechanism/send_command` 与 `/mechanism/command_feedback` 是否存在
- transport 已起但动作仍超时：优先核对 MCU 是否回了该命令目录项声明的 terminal success feedback

## 相关入口

- [遥控启动](./遥控启动.md)
- [决策启动](./决策启动.md)
- [rc26_merge_odom调试](./rc26_merge_odom调试.md)
