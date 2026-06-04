# rc26_telecontrol

## 模块定位

`rc26_telecontrol` 是 R2 的人工遥控测试包，用来在调试和联调阶段通过手柄向底盘发送速度指令。

## 当前实现

- 构建产物：
  - 静态库 `telecontrol_nodes`
  - 可执行文件 `rc26_telecontrol`
  - 可执行文件 `rc26_telecontrol_dpad`
  - 可执行文件 `rc26_telecontrol_front_pushrod_buttons`
  - 可执行文件 `rc26_telecontrol_rear_pushrod_buttons`
- 启动文件：`launch/wheeltec_joy.launch.py`
- 配置文件：
  - `config/joy_params.yaml`
  - `config/joy_params_dpad.yaml`
- 核心源码：
  - `src/telecontrol_nodes.cpp`
  - `src/wheeltec_joy.cpp`
  - `src/wheeltec_joy_dpad.cpp`
  - `src/front_pushrod_button_node.cpp`
  - `src/rear_pushrod_button_node.cpp`

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
  - `linear.y <- 左摇杆左右 axes[0]`
  - `angular.z <- 右摇杆左右 axes[3]`
- Dpad 模式：
  - `linear.x <- axes[7]`
  - `linear.y <- axes[6]`
  - `angular.z <- X(button[2], +wz) / B(button[1], -wz)`

R2 当前已经统一为麦克纳姆全向底盘，因此 `linear.y` 在 stick / dpad 两种模式下都是有效输出；`rc26_telecontrol` 不再声明或消费 `chassis_model`，也不会在节点内部屏蔽横移。

当前独立后推杆 sidecar 节点已经收口为：

- `rc26_telecontrol_rear_pushrod_buttons`
- `Select/Back(button[6]) -> REAR_PUSHROD_EXTEND (0x10)`
- `Start(button[7]) -> REAR_PUSHROD_RETRACT (0x11)`
- 采用按下沿单次触发；按住不连发，松开后再次按下才会重发
- 直接调用 `/mechanism/send_command`，走 ACK 路径，不再经过 `/mechanism/run_command`
- `Dpad 左/右` 现在只负责 `linear.y`，不再触发推杆 sidecar

除此之外，当前还新增了一个独立的前推杆按钮节点：

- `rc26_telecontrol_front_pushrod_buttons`
- `Y(button[3]) -> FRONT_PUSHROD_EXTEND (0x0E)`
- `A(button[0]) -> FRONT_PUSHROD_RETRACT (0x0F)`
- 直接调用 `/mechanism/send_command`，不再经过 `/mechanism/run_command`
- 采用按下沿单次触发；按住不会连发，松开后再次按下才会重发
- 现在走 ACK 路径；如果 MCU 不回通用 `ACK(0x00)`，会像其它可靠命令一样自动重传并打印超时日志
- `Y` 与 `A` 同帧按下时直接忽略

并且做了较多安全处理：

- 看门狗超时停车
- 可选 deadman 安全开关
- 速度、加速度、死区和滞回参数化

## 源码入口与阅读顺序
- 先看 `launch/wheeltec_joy.launch.py`，确认 stick / dpad 模式如何二选一。
- 再看 `src/telecontrol_nodes.cpp`，公共参数、看门狗、deadman、限斜率都在这里。
- 然后看 `src/front_pushrod_button_node.cpp`，确认 Y/A 如何映射到前推杆共享 transport 单次发送。
- 再看 `src/rear_pushrod_button_node.cpp`，确认 `Select/Back` / `Start` 如何映射到后推杆 ACK 指令。
- 最后看 `src/wheeltec_joy.cpp`、`src/wheeltec_joy_dpad.cpp` 和两个 YAML。

## 目录解剖
- `telecontrol_nodes.cpp`：基类、参数声明、摇杆输入处理、限幅、看门狗和两种控制模式实现。
- `wheeltec_joy.cpp` / `wheeltec_joy_dpad.cpp`：两个独立可执行入口。
- `front_pushrod_button_node.cpp`：Y/A 到 `/mechanism/send_command` 的前推杆单次发送桥接节点。
- `rear_pushrod_button_node.cpp`：`Select/Back` / `Start` 到 `/mechanism/send_command` 的后推杆单次 ACK 发送桥接节点。
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

- 当前遥控链已经移除 `chassis_model` 参数，统一输出 `linear.x / linear.y / angular.z` 的麦克纳姆口径。
- Stick 模式固定为 `左摇杆 -> vx/vy`、`右摇杆左右 -> wz`。
- Dpad 模式固定为 `十字键上下/左右 -> vx/vy`、`X -> +wz, B -> -wz`。
- `start_r2_teleop.sh` 现在默认用 `dpad` 模式启动，而不是 stick。
- `start_r2_teleop.sh` 当前默认最大线速度为 `0.3 m/s`，仍可通过 `--v-linear` 覆盖。
- `start_r2_teleop.sh` 现在会在 `full` 和 `minimal-mcu` 两个栈里都额外挂起 `rc26_telecontrol_front_pushrod_buttons` 与 `rc26_telecontrol_rear_pushrod_buttons`，用于把 `Y/A` 与 `Select/Back` / `Start` 桥到 `0x0E~0x11`。
- 在 dpad 模式里，旋转仍由 `X/B` 控制，其中 `X -> +wz`、`B -> -wz`；`Y/A`、`Select/Back`、`Start` 不参与速度输出，只交给前/后推杆 sidecar；`Dpad 左/右` 会直接体现在 `/cmd_vel.linear.y`。
- `start_r2_teleop.sh` 不再默认拉起 `rc26_mechanism`；teleop 前/后推杆联调只依赖 `merge_odom` 或 `pose_sender_node` 持有目标串口并提供 transport service。
- 仓库根目录的 `start_r2_teleop.sh` 现在通过 `--stack full|minimal-mcu` 统一承载完整遥控链和最小串口链；最小 MCU 口径以 `./start_r2_teleop.sh --stack minimal-mcu` 为准。
- `--stack minimal-mcu` 会启动 `pose_sender_node + joy_node + telecontrol + rc26_telecontrol_front_pushrod_buttons + rc26_telecontrol_rear_pushrod_buttons`；`pose_sender_node` 现在也会继续提供 `/mechanism/send_command` 与 `/mechanism/command_feedback`。
- `start_r2_teleop.sh` 现在在 `full` 和 `minimal-mcu` 两个栈下都会自动兼容“只接了一个目标 MCU 下发串口”的现场：若默认 `/dev/ttyUSB1` 不存在但 `/dev/ttyUSB0` 存在，脚本会自动把目标串口切到 `/dev/ttyUSB0`，并把反馈串口降级为 `__disabled__`。
- `start_r2_teleop.sh` 现支持 `--pose-mode imu|no-imu|wheel-only`：
  - `imu`：EKF 融合 `DM_IMU`
  - `no-imu`：EKF 不融合 IMU，但 `dm_imu_node` 与执行保护链仍保留
  - `wheel-only`：不启动也不读取 IMU；若反馈串口可用，则只用 `wheel_odom` 做最终 `merge_odom` 融合；若现场只有目标串口，则退化为“只保留目标串口下发 + transport sidecar”的单链路模式
- `terrain_speed_limit` 运行时链路已从系统中删除；teleop 链不再需要额外关闭地形限速，也不存在重新接回该链路的脚本入口。

## 配置注释口径

- `config/joy_params.yaml` 与 `config/joy_params_dpad.yaml` 已保留常用/高影响参数的中文注释，说明手柄轴/按钮映射、全向底盘速度口径、deadman、watchdog、速度和加速度限制；本次同步移除了履带专用注释与参数。
