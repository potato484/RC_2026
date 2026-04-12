# rc26_telecontrol

## 模块定位

`rc26_telecontrol` 是 R2 的人工遥控测试包，用来在调试和联调阶段通过手柄向底盘发送速度指令。

## 当前实现

- 构建产物：
  - 静态库 `telecontrol_nodes`
  - 可执行文件 `rc26_telecontrol`
  - 可执行文件 `rc26_telecontrol_dpad`
  - 可执行文件 `rc26_telecontrol_front_track_test`
  - 可执行文件 `rc26_telecontrol_pushrod_dpad`
- 启动文件：`launch/wheeltec_joy.launch.py`
- 配置文件：
  - `config/joy_params.yaml`
  - `config/joy_params_dpad.yaml`
- 核心源码：
  - `src/telecontrol_nodes.cpp`
  - `src/wheeltec_joy.cpp`
  - `src/wheeltec_joy_dpad.cpp`
  - `src/front_track_button_test_node.cpp`
  - `src/pushrod_dpad_node.cpp`

当前已经实现两套控制模式：

- Stick 模式：连续摇杆控制
- Dpad 模式：离散十字键控制

当前仓库默认按 `Xbox 360 Controller` 的轴/按键编号解释输入，实际映射口径如下：

- 十字键：
  - 上/下：`axes[7]`
  - 左/右：`axes[6]`
- 中间功能键：
  - `select`
  - `start`
  - `mode`
  - 这 3 个键当前未被 `rc26_telecontrol` 消费
- 右侧 ABXY：
  - `A`：`button[0]`
  - `B`：`button[1]`
  - `X`：`button[2]`
  - `Y`：`button[3]`
- 摇杆：
  - 左摇杆左右：`axes[0]`
  - 左摇杆前后：`axes[1]`
  - 右摇杆左右：`axes[3]`
  - 右摇杆前后：`axes[4]`

在当前代码里，这些输入真正参与控制的方式是：

- Stick 模式：
  - `linear.x <- 左摇杆前后 axes[1]`
  - `angular.z <- 右摇杆左右 axes[3]`
  - `linear.y <- 左摇杆左右 axes[0]`，仅 `mecanum_4wheel` 模式启用
- Dpad 模式：
  - `linear.x <- axes[7]`
  - `angular.z <- X(button[2]) / B(button[1])`
  - `linear.y <- axes[6]`，仅 `mecanum_4wheel` 模式启用

当前默认底盘模式是 `tracked_diff`，因此运行时只真正使用 `linear.x + angular.z`；`linear.y` 会被固定为 0，右摇杆前后 `axes[4]` 当前也未参与控制。

当前在 `tracked_diff` 模式下，`axes[6]` 左右又被独立 sidecar 节点复用成了机构语义：

- `Dpad 左 -> PUSHROD_EXTEND (0x10)`
- `Dpad 右 -> PUSHROD_RETRACT (0x11)`
- 采用按下沿单次触发；按住不连发，回中或切换到另一侧后才会再次触发
- 直接调用 `/mechanism/transport/send_command`，走 ACK 路径，不再经过 `/mechanism/execute`

除此之外，当前还新增了一个独立的机构按钮测试节点：

- `Y(button[3]) -> FRONT_TRACK_UP (0x0E)`
- `A(button[0]) -> FRONT_TRACK_DOWN (0x0F)`
- 直接调用 `/mechanism/transport/send_command`，不再经过 `/mechanism/execute`
- 按住按钮时按 `50Hz` 连续下发
- `Y` 与 `A` 同帧按下时直接忽略
- 松开按钮后立即停止发送

并且做了较多安全处理：

- 看门狗超时停车
- 可选 deadman 安全开关
- 速度、加速度、死区和滞回参数化

## 源码入口与阅读顺序
- 先看 `launch/wheeltec_joy.launch.py`，确认 stick / dpad 模式如何二选一。
- 再看 `src/telecontrol_nodes.cpp`，公共参数、看门狗、deadman、限斜率都在这里。
- 然后看 `src/front_track_button_test_node.cpp`，确认 Y/A 如何映射到共享 transport 连续发送。
- 再看 `src/pushrod_dpad_node.cpp`，确认 Dpad 左/右如何映射到电动推杆 ACK 指令。
- 最后看 `src/wheeltec_joy.cpp`、`src/wheeltec_joy_dpad.cpp` 和两个 YAML。

## 目录解剖
- `telecontrol_nodes.cpp`：基类、参数声明、摇杆输入处理、限幅、看门狗和两种控制模式实现。
- `wheeltec_joy.cpp` / `wheeltec_joy_dpad.cpp`：两个独立可执行入口。
- `front_track_button_test_node.cpp`：Y/A 到 `/mechanism/transport/send_command` 的连续发送桥接节点。
- `pushrod_dpad_node.cpp`：Dpad 左/右到 `/mechanism/transport/send_command` 的单次 ACK 发送桥接节点。
- `config/joy_params*.yaml`：手柄映射和安全参数。
- `launch/wheeltec_joy.launch.py`：模式切换和 `joy_node` 装配。

## 关键文件体量
- `src/telecontrol_nodes.cpp`：483 行。
- `launch/wheeltec_joy.launch.py`：144 行。
- `README.md`：119 行。

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
- `start_r2_teleop.sh` 现在默认用 `dpad` 模式启动，而不是 stick。
- `start_r2_teleop.sh` 现在会在 `full` 和 `minimal-mcu` 两个栈里都额外挂起 `rc26_telecontrol_pushrod_dpad`，用于把 Dpad 左/右桥到 `0x10 / 0x11`。
- `start_r2_teleop.sh` 会额外拉起 `rc26_telecontrol_front_track_test`，用于把 `Y/A` 直接映射成前置履带 `0x0E / 0x0F` 的连续命令。
- 在 dpad 模式里，旋转仍由 `X/B` 控制；`Y/A` 不参与速度输出，只交给前置履带测试节点；`Dpad 左/右` 在履带模式下也不再参与 Twist 横移，而是交给推杆 sidecar 节点。
- 仓库根目录的 `start_r2_teleop.sh` 现已显式向 `rc26_merge_odom` 和 `rc26_telecontrol` 传入 `chassis_model:=tracked_diff`，遥控联调不再依赖各包内部默认值。
- `start_r2_teleop.sh` 不再默认拉起 `rc26_mechanism`；teleop 前置履带联调只依赖 `merge_odom` 持有目标串口并提供 transport service。
- 仓库根目录的 `start_r2_teleop.sh` 现在通过 `--stack full|minimal-mcu` 统一承载完整遥控链和最小串口链；`start_r2_mcu_teleop.sh` 保留为兼容包装器，等价于 `start_r2_teleop.sh --stack minimal-mcu`。
- `start_r2_teleop.sh` 现在在 `full` 栈下也会自动兼容“只接了一个目标 MCU 下发串口”的现场：若默认 `/dev/ttyUSB1` 不存在但 `/dev/ttyUSB0` 存在，脚本会自动把目标串口切到 `/dev/ttyUSB0`，并把反馈串口降级为 `__disabled__`。
- `start_r2_teleop.sh` 现支持 `--pose-mode imu|no-imu|wheel-only`：
  - `imu`：EKF 融合 `DM_IMU`
  - `no-imu`：EKF 不融合 IMU，但 `dm_imu_node` 与执行保护链仍保留
  - `wheel-only`：不启动也不读取 IMU；若反馈串口可用，则只用 `wheel_odom` 做最终 `merge_odom` 融合；若现场只有目标串口，则退化为“只保留目标串口下发 + 前置履带 transport”的单链路模式
- `terrain_speed_limit` 运行时链路已从系统中删除；teleop 链不再需要额外关闭地形限速，也不存在重新接回该链路的脚本入口。
