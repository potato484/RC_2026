# rc26_telecontrol

## 模块定位

`rc26_telecontrol` 是 R2 的人工遥控测试包，用来在调试和联调阶段通过手柄向底盘发送速度指令。

## 当前实现

- 构建产物：
  - 静态库 `telecontrol_nodes`
  - 可执行文件 `rc26_telecontrol`
  - 可执行文件 `rc26_telecontrol_dpad`
- 启动文件：`launch/wheeltec_joy.launch.py`
- 配置文件：
  - `config/joy_params.yaml`
  - `config/joy_params_dpad.yaml`
- 核心源码：
  - `src/telecontrol_nodes.cpp`
  - `src/wheeltec_joy.cpp`
  - `src/wheeltec_joy_dpad.cpp`

当前已经实现两套控制模式：

- Stick 模式：连续摇杆控制
- Dpad 模式：离散十字键控制

并且做了较多安全处理：

- 看门狗超时停车
- 可选 deadman 安全开关
- 速度、加速度、死区和滞回参数化

## 模块边界

- 这个包服务于 R2 的人工测试与接管，不是自动决策模块
- 它不生成路径，也不做定位或地形理解
- 当前项目主目标仍是 R2 自动机器人，`rc26_telecontrol` 更偏调试/验证链路
