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
  - 对接真实串口硬件
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
- 串口链路健康诊断
- 取消、超时、急停等安全路径

## 源码入口与阅读顺序
- 先看 `launch/mechanism.launch.py`，确认 HAL 选择和生命周期装配。
- 再看 `src/mechanism_lifecycle_server.cpp`，生命周期节点、Action server 和串口反馈处理都在这里。
- 最后回到各个 HAL 文件，区分真机、回放、仿真和故障注入实现。

## 目录解剖
- `mechanism_lifecycle_server.cpp`：唯一核心服务边界，生命周期转换、Action 接口、状态广播、串口反馈收敛都在这里。
- `serial_mechanism_hal.cpp`：真实串口 HAL。
- `replay_mechanism_hal.cpp`：离线回放 HAL。
- `sim_mechanism_hal.cpp`：纯模拟 HAL。
- `fault_injecting_hal.cpp`：故障注入与韧性测试 HAL。

## 关键文件体量
- `src/mechanism_lifecycle_server.cpp`：1399 行，是整个包的绝对主文件。
- `src/replay_mechanism_hal.cpp`：230 行。
- `src/fault_injecting_hal.cpp`：184 行。
- `src/sim_mechanism_hal.cpp`：137 行。
- `src/serial_mechanism_hal.cpp`：70 行。

## 关键源码行段速览
- `src/rc26_mechanism/src/mechanism_lifecycle_server.cpp:79-334`：生命周期节点构造、`on_configure/on_activate/on_deactivate/on_cleanup/on_error`。
- `src/rc26_mechanism/src/mechanism_lifecycle_server.cpp:416-947`：四类 Action 的 goal/cancel/accept/execute 主路径。
- `src/rc26_mechanism/src/mechanism_lifecycle_server.cpp:1075-1193`：带上下文的命令执行、超时、取消和结果收敛。
- `src/rc26_mechanism/src/mechanism_lifecycle_server.cpp:1194-1388`：串口反馈解析、缓冲结果清理和 `MechanismState` 发布。
- `src/rc26_mechanism/src/mechanism_lifecycle_server.cpp:1389-1399`：结束等待辅助逻辑。

## 模块边界

- 它不做比赛级决策，只执行机构动作
- 它不负责底盘导航控制
- 它不直接做视觉识别或地图处理，只消费上层语义并驱动硬件
