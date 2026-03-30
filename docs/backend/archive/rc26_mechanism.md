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

## 模块边界

- 它不做比赛级决策，只执行机构动作
- 它不负责底盘导航控制
- 它不直接做视觉识别或地图处理，只消费上层语义并驱动硬件
