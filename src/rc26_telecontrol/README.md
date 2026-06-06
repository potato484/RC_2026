# rc26_telecontrol 模块详细说明

## 1. 简介

`rc26_telecontrol` 是专为 RC2026 项目打造的遥控模块。它负责将物理手柄的输入信号（如摇杆偏移量、按键状态等）解析并转换为机器人底盘可以理解的运动控制指令（速度和角速度），最终通过 ROS 2 的话题发布给下游节点处理。

该模块不仅提供基础的映射功能，更着重于**控制平滑性**与**系统安全性**，旨在弱网环境或硬件故障等极端情况下，也能最大程度保障机器人不发生失控碰撞危险。

## 2. 核心功能特性

### 2.1 双模式控制支持

针对不同类型的操作需求，模块支持两种手柄映射模式：

*   **Stick (摇杆) 模式**：通过左/右摇杆进行连续性的线速度与角速度控制，适合平滑的行驶和精细的方向调整。
*   **Dpad (十字键) 模式**：通过方向键（D-Pad）进行离散的固定速度控制，适合简单的步进式移动或测试。

在启动时，系统通过互斥的启动参数确保两种模式不会同时运行，避免指令冲突。仓库根目录的 `start_r2_teleop.sh` 当前默认直接以 `Dpad` 模式启动。

当前仓库实际按 `Xbox 360 Controller` 的输入编号解释手柄：

*   十字键：`axes[7]` 为上下，`axes[6]` 为左右。
*   中间功能键：`select`、`start`、`mode`。
*   右侧按键：`A=button[0]`、`B=button[1]`、`X=button[2]`、`Y=button[3]`。
*   摇杆：左摇杆左右 `axes[0]`、前后 `axes[1]`；右摇杆左右 `axes[3]`、前后 `axes[4]`。

结合当前代码的真实控制口径：

*   **Stick 模式**：`linear.x <- axes[1]`，`linear.y <- axes[0]`，`angular.z <- axes[3]`。
*   **Dpad 模式**：`linear.x <- axes[7]`，`linear.y <- axes[6]`，`angular.z <- X(+wz) / B(-wz)`。
*   R2 当前已经统一为麦克纳姆全向底盘，因此 `linear.y` 在 stick / dpad 两种模式下都是有效输出；`rc26_telecontrol` 不再声明或消费 `chassis_model`。
*   独立 sidecar 节点 `rc26_telecontrol_front_pushrod_buttons`：`Y(button[3])` 按下沿单次下发 `FRONT_PUSHROD_EXTEND (0x0E)`；`A(button[0])` 按下沿单次下发 `FRONT_PUSHROD_RETRACT (0x0F)`。按住不会连发，松开后再次按下才会重发。该节点直接调用 `/mechanism/send_command`，走 transport ACK 路径，不再经过 `/mechanism/run_command`。
*   独立 sidecar 节点 `rc26_telecontrol_rear_pushrod_buttons`：`Select/Back(button[6]) -> REAR_PUSHROD_EXTEND (0x10)`、`Start(button[7]) -> REAR_PUSHROD_RETRACT (0x11)`。该节点同样直接调用 `/mechanism/send_command`，走 transport ACK 路径；若 MCU 额外上送 `0x13~0x16` 业务 ACK，会继续发布到 `/mechanism/command_feedback`。`Dpad 左/右` 现在只负责底盘横移。

### 2.2 多重安全保障机制

为了应对比赛或实际运行中可能出现的突发状况，模块内置了多项安全防护逻辑：

*   **Watchdog (看门狗) 超时保护**：持续监控手柄信号接收时间。如果手柄意外断开连接、没电或节点崩溃，系统会在设定时间（如 0.3 秒）内未收到信号时，强制输出零速指令，使机器人立即停车，防止“飞车”事故。
*   **Deadman (安全开关) 机制**：可选开启的安全确认键。要求操作员必须持续按住特定按键（如 LB 键）才能下发速度指令。一旦松开按键，无论摇杆处于什么位置，机器人都将立即停止。
*   **指令防丢包 (弱网停车)**：在网络状态不佳时，由于 UDP 协议可能丢包，导致关键的“停车”指令丢失。为此，当手柄回中要求停车时，模块会连续多次（如 10 帧）重复发送零速指令，确保底盘大概率能收到停车信号。

### 2.3 运动学平滑处理

为了让机器人的动作更加柔和，保护机械结构并提升控制手感：

*   **死区滞回 (Deadzone Hysteresis)**：为了避免摇杆在中心位置的物理抖动导致频繁发送微小速度指令，系统引入了滞回控制算法。它在死区内外设置了不同的阈值，使得摇杆在刚离开死区和快回到死区时的判定更加稳定，彻底消除了零点附近的震荡。
*   **加速度硬约束 (Rate Limiting)**：废弃了原有的指数移动平均(EMA)算法，改为使用明确的物理加速度（m/s²）和角加速度（rad/s²）进行限制。这能有效防止摇杆被突然推满时引起的瞬时大电流和机械冲击，使起步和刹车都呈现线性且可预期的变化。

### 2.4 统一入口与最小 MCU 链

当前仓库的正式遥控入口是根目录 `start_r2_teleop.sh`：

*   `--stack minimal-mcu`：启动 `pose_sender_node + joy_node + telecontrol + rc26_telecontrol_front_pushrod_buttons + rc26_telecontrol_rear_pushrod_buttons`。这是当前脚本默认口径，固定按 `target_serial_port=/dev/ttyUSB0`、`feedback_serial_port=__disabled__` 进入单口 MCU 链；`pose_sender_node` 会继续提供 `/mechanism/send_command` 与 `/mechanism/command_feedback`。
*   `--stack full`：启动 `merge_odom + joy_node + telecontrol + rc26_telecontrol_front_pushrod_buttons + rc26_telecontrol_rear_pushrod_buttons`。这个入口当前主要保留给本地 `merge_odom` / CAN 调试；机构共享 transport 与 `POSE_TARGET` 仍默认走 `ttyUSB0`。
*   在当前单口默认口径下，`--pose-mode` 与 `--start-ekf` 都会被脚本直接拒绝，避免把已停用的 feedback / 融合速度链误当成默认路径。
*   若后续确实要临时恢复旧反馈链，需要显式传入真实 `feedback_serial_port`；脚本不再自动改写 `target_serial_port`。
*   `start_r2_teleop.sh` 的帮助文本、默认值和 README 现已统一到上述单口口径。

## 3. 参数配置体系

模块具有高度灵活的参数配置系统，分为默认配置文件（YAML）和启动时动态覆盖（Launch 参数）两层。

主要可调参数包括：

*   **最大速度限制**：分别设置线速度（前进/后退、平移）和角速度（旋转）的上限，通常默认较低以确保测试安全。
*   **加速度限制**：定义速度变化的快慢。
*   **死区与滞回宽度**：适配不同品牌手柄的物理旷量。
*   **看门狗超时时间**：定义容忍断联的最长极限。
*   **安全开关配置**：开启状态及绑定的按键索引。

当前默认配置位于 `config/joy_params.yaml` 与 `config/joy_params_dpad.yaml`：

*   `device_name = Xbox 360 Controller`
*   `v_linear = 0.2`
*   `v_angular = 0.5`
*   `joy_timeout_s = 0.3`
*   `stop_repeat_n = 10`
*   `require_deadman = false`
*   `deadman_button = 4`

## 4. 架构与数据流向

1.  **输入**：通过订阅标准的手柄话题（如 `/joy`），获取原始的轴向浮点数和按键布尔数组。
2.  **处理节点**：
    *   `wheeltec_joy` (Stick 模式处理)
    *   `wheeltec_joy_dpad` (Dpad 模式处理)
    *   `rc26_telecontrol_front_pushrod_buttons` (Y/A 到共享 transport 的前推杆单次发送桥接)
    *   `rc26_telecontrol_rear_pushrod_buttons` (`Select/Back` / `Start` 到共享 transport 的后推杆单次发送桥接)
    节点内部按顺序执行：看门狗检查 -> 安全开关检查 -> 死区过滤 -> 加速度限制计算 -> 重复发包逻辑。
3.  **输出**：将计算后平滑且安全的速度结果，打包成标准的速度控制消息（如 `geometry_msgs::msg::Twist`），发布到指定话题。包内默认参数仍是 `/cmd_vel_teleop`，但仓库根目录的 `start_r2_teleop.sh` 会显式改为 `/cmd_vel` 以接入当前底盘执行链。

## 5. 运行排查

根目录集中式调试文档已删除。遥控链排查以本 README、`start_r2_teleop.sh` 帮助文本、`launch/wheeltec_joy.launch.py` 和包内测试为准。
