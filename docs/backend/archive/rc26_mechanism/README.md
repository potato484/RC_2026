# rc26_mechanism

## 模块定位

`rc26_mechanism` 是 R2 的机构执行与生命周期管理模块，负责把上层动作语义可靠地下发给下位机，并回传状态。

## 当前实现

- 构建方式：组件库 + 独立可执行
- 导出节点：`mechanism_server_node`
- 启动文件：`launch/mechanism.launch.py`

核心源码目前体现的是“一个生命周期服务端 + 多态 HAL”架构：

- `mechanism_lifecycle_server.cpp`
  - 生命周期管理、Action 服务、状态发布、异常处理
- `serial_mechanism_hal.cpp`
  - 直接对接真实串口硬件，适合单包隔离调试
- `shared_serial_mechanism_hal.cpp`
  - 通过 ROS 2 service/topic 复用 `rc26_merge_odom` 已打开的目标 MCU 串口
- `replay_mechanism_hal.cpp`
  - 基于回放数据的离线调试实现
- `sim_mechanism_hal.cpp`
  - 无实车时的模拟实现
- `fault_injecting_hal.cpp`
  - 健壮性和故障注入测试实现

当前实现强调：

- 生命周期门禁
- Action 异步执行
- 状态广播
- 共享串口链路健康诊断
- 取消、超时、急停等安全路径

当前真实部署口径已经不是“`rc26_mechanism` 自己独占 `/dev/ttyUSB1`”，而是：

- 真机上由 `rc26_merge_odom` 作为目标 MCU 串口的唯一 owner
- `rc26_mechanism` 通过 `hal_type:=shared_serial` 复用这条链路
- 下行发送经由 `/mechanism/transport/send_command`
- 上行反馈经由 `/mechanism/transport/feedback`

也就是说，`/mechanism/execute` 仍然是上层动作执行入口，但真机串口发送职责已经下沉到 `rc26_merge_odom`。

当前 `ExecuteMechanism` 通用入口已经覆盖一批按 `command_id` 直接下发的串口动作，其中包含前置履带：

- `FRONT_TRACK_UP = 0x11`
- `FRONT_TRACK_DOWN = 0x12`

这两个动作的成功条件不是“命令写串口成功”，而是等待 MCU 回传：

- `FRONT_TRACK_UP_DONE = 0x13`
- `FRONT_TRACK_DOWN_DONE = 0x14`

当前约定下，这两个 `DONE` 会在 MCU 内部确认遥控发送链已经结束后再上报，因此上位机不额外实现 teleop 停止编排，只消费最终终态反馈。

仓库根目录的 `start_r2_teleop.sh` 现在会显式：

- 启动 `rc26_mechanism mechanism.launch.py hal_type:=shared_serial`
- 自动执行 `/mechanism_server` 的 `configure` 和 `activate`
- 让遥控按钮测试节点直接走 `/mechanism/execute`

## 源码入口与阅读顺序
- 先看 `launch/mechanism.launch.py`，确认 HAL 选择和生命周期装配。
- 再看 `src/mechanism_lifecycle_server.cpp`，生命周期节点、Action server 和反馈收敛都在这里。
- 然后看 `src/shared_serial_mechanism_hal.cpp`，确认真实部署如何桥接 `rc26_merge_odom` 的共享串口。
- 最后回到其余 HAL 文件，区分直连串口、回放、仿真和故障注入实现。

## 目录解剖
- `mechanism_lifecycle_server.cpp`：唯一核心服务边界，生命周期转换、Action 接口、状态广播、串口反馈收敛都在这里。
- `serial_mechanism_hal.cpp`：直连真实串口 HAL。
- `shared_serial_mechanism_hal.cpp`：共享串口 HAL，经 `/mechanism/transport/*` 对接 `rc26_merge_odom`。
- `replay_mechanism_hal.cpp`：离线回放 HAL。
- `sim_mechanism_hal.cpp`：纯模拟 HAL。
- `fault_injecting_hal.cpp`：故障注入与韧性测试 HAL。

## 关键文件体量
- `src/mechanism_lifecycle_server.cpp`：1427 行，是整个包的绝对主文件。
- `src/shared_serial_mechanism_hal.cpp`：107 行。
- `src/replay_mechanism_hal.cpp`：234 行。
- `src/fault_injecting_hal.cpp`：188 行。
- `src/sim_mechanism_hal.cpp`：141 行。
- `src/serial_mechanism_hal.cpp`：70 行。

## 关键源码行段速览
- `src/rc26_mechanism/src/mechanism_lifecycle_server.cpp:79-334`：生命周期节点构造、`on_configure/on_activate/on_deactivate/on_cleanup/on_error`。
- `src/rc26_mechanism/src/mechanism_lifecycle_server.cpp:416-947`：四类 Action 的 goal/cancel/accept/execute 主路径。
- `src/rc26_mechanism/src/mechanism_lifecycle_server.cpp:1075-1193`：带上下文的命令执行、超时、取消和结果收敛。
- `src/rc26_mechanism/src/mechanism_lifecycle_server.cpp:1194-1388`：串口反馈解析、缓冲结果清理和 `MechanismState` 发布。
- `src/rc26_mechanism/src/mechanism_lifecycle_server.cpp:1389-1399`：结束等待辅助逻辑。

## 近期实现说明

- `ExecuteMechanism` 仍是机构动作的统一执行入口；前置履带抬升/放下通过 `0x11 / 0x12` 下发，并等待 `0x13 / 0x14` 完成反馈。
- 真机链路新增 `shared_serial` HAL，`rc26_mechanism` 不再和 `rc26_merge_odom` 竞争打开同一个目标串口。
- `/mechanism/transport/send_command` 与 `/mechanism/transport/feedback` 现在是 mechanism 与 merge_odom 之间的内部桥接契约。
- 遥控模式下停止发送数据后，MCU 才回传前置履带完成反馈；机制侧只认最终反馈，不在上位机额外拼 teleop 停止流程。
- 新增 `test_shared_serial_transport`，覆盖共享串口发送、反馈收敛、服务异常和超时场景。

## 模块边界

- 它不做比赛级决策，只执行机构动作
- 它不负责底盘导航控制
- 它不直接做视觉识别或地图处理，只消费上层语义并驱动硬件
