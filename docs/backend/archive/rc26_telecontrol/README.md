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

## 源码入口与阅读顺序
- 先看 `launch/wheeltec_joy.launch.py`，确认 stick / dpad 模式如何二选一。
- 再看 `src/telecontrol_nodes.cpp`，公共参数、看门狗、deadman、限斜率都在这里。
- 最后看 `src/wheeltec_joy.cpp`、`src/wheeltec_joy_dpad.cpp` 和两个 YAML。

## 目录解剖
- `telecontrol_nodes.cpp`：基类、参数声明、摇杆输入处理、限幅、看门狗和两种控制模式实现。
- `wheeltec_joy.cpp` / `wheeltec_joy_dpad.cpp`：两个独立可执行入口。
- `config/joy_params*.yaml`：手柄映射和安全参数。
- `launch/wheeltec_joy.launch.py`：模式切换和 `joy_node` 装配。

## 关键文件体量
- `src/telecontrol_nodes.cpp`：447 行。
- `launch/wheeltec_joy.launch.py`：137 行。
- `README.md`：58 行。

## 关键源码行段速览
- `src/rc26_telecontrol/src/telecontrol_nodes.cpp:23-138`：基类构造、参数声明与规范化。
- `src/rc26_telecontrol/src/telecontrol_nodes.cpp:206-368`：ROS 接口、看门狗、deadman、死区和 rate limit。
- `src/rc26_telecontrol/src/telecontrol_nodes.cpp:369-447`：Stick 和 Dpad 两种目标速度计算。
- `src/rc26_telecontrol/launch/wheeltec_joy.launch.py`：决定 `joy_node`、stick 节点和 dpad 节点的互斥启动。

## 模块边界

- 这个包服务于 R2 的人工测试与接管，不是自动决策模块
- 它不生成路径，也不做定位或地形理解
- 当前项目主目标仍是 R2 自动机器人，`rc26_telecontrol` 更偏调试/验证链路

## 近期实现说明

- 当前遥控链新增 `chassis_model` 参数，支持 `mecanum_4wheel | tracked_diff` 两种底盘模式。
- 当前默认底盘模式已切到 `tracked_diff`，因此遥控默认不会再输出横向速度。
- 四轮模式保持原有 `linear.x / linear.y / angular.z` 控制口径。
- 履带模式下，Stick 和 Dpad 都只输出 `linear.x + angular.z`，`linear.y` 在节点内部被固定为 0。
- 仓库根目录的 `start_r2_teleop.sh` 现已显式向 `rc26_merge_odom` 和 `rc26_telecontrol` 传入 `chassis_model:=tracked_diff`，遥控联调不再依赖各包内部默认值。
