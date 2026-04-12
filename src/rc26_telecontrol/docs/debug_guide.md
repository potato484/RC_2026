# rc26_telecontrol 调试指南

本文档提供针对 `rc26_telecontrol` 模块的具体调试步骤与常用命令，用于验证摇杆控制功能是否正常、参数配置是否生效，以及异常断联等安全机制是否起作用。

## 1. 编译模块

首先，在工作空间根目录下进行编译。为了限制编译资源消耗并单独编译该模块，请使用以下命令：

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 --packages-select rc26_interfaces rc26_serial rc26_merge_odom rc26_telecontrol --cmake-args -DCMAKE_BUILD_TYPE=Release
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 2. 基础功能测试

### 2.1 启动节点 (Stick 模式)

默认情况下，使用摇杆（Stick）模式启动节点：

```bash
ros2 launch rc26_telecontrol wheeltec_joy.launch.py control_mode:=stick
```

### 2.2 启动节点 (Dpad 模式)

如果需要使用十字键（Dpad）模式启动节点：

```bash
ros2 launch rc26_telecontrol wheeltec_joy.launch.py control_mode:=dpad
```

### 2.3 一键启动当前真实遥控链

仓库根目录脚本已经变成当前推荐入口：

```bash
./start_r2_teleop.sh
```

这个脚本当前会默认：

- 使用 `dpad` 模式，而不是 stick
- 启动 `rc26_merge_odom`
- 启动 `rc26_telecontrol_pushrod_dpad`
- 启动 `rc26_telecontrol_front_track_test`

当前还建议先用 `--dry-run` 检查参数展开：

```bash
./start_r2_teleop.sh --dry-run
./start_r2_teleop.sh --dry-run --pose-mode wheel-only
./start_r2_teleop.sh --dry-run --stack minimal-mcu
```

注意：

- `--stack full` 才会启动 `rc26_telecontrol_front_track_test`
- `--stack minimal-mcu` 会启动 `pose_sender_node + joy_node + telecontrol + rc26_telecontrol_pushrod_dpad`；这个口径不会启动前置履带按钮节点，但会继续提供 `/mechanism/transport/*`
- 若默认 `/dev/ttyUSB1` 不存在但 `/dev/ttyUSB0` 存在，统一脚本在 `full` 和 `minimal-mcu` 两个栈下都会自动切到单目标串口降级口径

### 2.4 验证节点与话题

在另一个终端中，检查节点是否正确启动：

```bash
ros2 node list | grep -E 'joy|telecontrol|mechanism|merge_odom|pose_sender'
# 预期输出至少包含：/joy_node, /rc26_telecontrol_dpad 或 /rc26_telecontrol, /rc26_telecontrol_pushrod_dpad
# full 栈还应包含：/rc26_telecontrol_front_track_test, /merge_odom_node
# minimal-mcu 栈则会包含：/pose_sender_node
```

检查控制指令话题：

```bash
ros2 topic echo /cmd_vel
```
若是单独用 `rc26_telecontrol` 包 launch，默认仍可能发布到 `/cmd_vel_teleop`；而根目录 `start_r2_teleop.sh` 会显式覆盖为 `/cmd_vel`。

此时操作手柄，观察终端输出的 `linear.x` 与 `angular.z` 是否随着手柄动作发生变化。当前 R2 默认按 `tracked_diff` 运行，`linear.y` 应保持为 `0`；只有显式切回 `mecanum_4wheel` 时才会出现横向速度输出。

### 2.5 当前手柄映射

当前仓库默认按 `Xbox 360 Controller` 的编号解释 `/joy`：

- 十字键：
  - 上/下：`axes[7]`
  - 左/右：`axes[6]`
- 中间功能键：
  - `select`
  - `start`
  - `mode`
  - 当前都未被 `rc26_telecontrol` 消费
- 右侧按键：
  - `A = button[0]`
  - `B = button[1]`
  - `X = button[2]`
  - `Y = button[3]`
- 摇杆：
  - 左摇杆左右：`axes[0]`
  - 左摇杆前后：`axes[1]`
  - 右摇杆左右：`axes[3]`
  - 右摇杆前后：`axes[4]`

当前代码真正使用这些输入的方式是：

- Stick 模式：
  - `linear.x <- axes[1]`
  - `angular.z <- axes[3]`
  - `linear.y <- axes[0]`，仅四轮全向模式启用
- Dpad 模式：
  - `linear.x <- axes[7]`
  - `angular.z <- X(button[2]) / B(button[1])`
  - `linear.y <- axes[6]`，仅四轮全向模式启用

当前默认底盘模式为 `tracked_diff`，因此运行时只真正消费 `linear.x + angular.z`；`linear.y` 会保持为 0，`axes[4]` 当前不参与控制。

当前还新增了独立的机构按钮测试映射：

- `Y(button[3]) -> /mechanism/transport/send_command -> FRONT_TRACK_UP (0x0E)`
- `A(button[0]) -> /mechanism/transport/send_command -> FRONT_TRACK_DOWN (0x0F)`

这个测试节点不直接操作串口，而是直接复用 `rc26_merge_odom` 的共享 transport；真机下实际串口发送由 `merge_odom` 持有的目标串口完成。

当前还新增了独立的推杆 sidecar 映射：

- `Dpad 左(axes[6] < 0) -> /mechanism/transport/send_command -> PUSHROD_EXTEND (0x10)`
- `Dpad 右(axes[6] > 0) -> /mechanism/transport/send_command -> PUSHROD_RETRACT (0x11)`
- 采用按下沿单次触发；按住不会连发，回中后才会重新触发

## 3. 核心机制验证

### 3.1 死区 (Deadzone) 测试

1. 保持摇杆在中心位置（未触碰），观察当前实际输出话题是否完全为 `0.0`。如果是根目录脚本，默认看 `/cmd_vel`；如果是包内 launch，默认看 `/cmd_vel_teleop`。
2. 轻微推动摇杆，在不超过死区阈值（默认 0.15）时，输出应保持为 `0.0`。
3. 继续推动摇杆越过阈值，输出应平滑变化，无高频的 `0/非0` 抖动（得益于滞回控制）。

### 3.2 加速度限制测试

加速度限制可以防止机器人猛烈起步。

1. 启动节点并监听速度：
   ```bash
   ros2 topic echo /cmd_vel
   ```
   若当前不是通过根目录脚本启动，而是直接使用包内 launch，请改为监听 `/cmd_vel_teleop`。
2. 迅速将摇杆推到最大位置。
3. 观察输出的目标速度是否是逐步爬升到最大值，而不是瞬间跳变到最大值。当前履带模式重点看 `linear.x`；若显式切回 `mecanum_4wheel`，再额外验证 `linear.y` 的限加速度表现。

### 3.3 停车指令重复发送测试

为了防止弱网环境丢包，停车指令会连续发送 N 帧。

1. 推动摇杆使其有速度输出。
2. 突然松开摇杆使其回到中心。
3. 观察输出话题，会连续打印 N 次（默认 10 次）全 0 的速度指令，随后停止发布（终端不再滚动刷新新数据）。

### 3.4 Watchdog (看门狗) 超时保护测试

此功能用于防止手柄意外断开连接导致机器人失控。

1. 正常启动 launch 文件并保持手柄连接。
2. 推动摇杆给出一定的速度指令。
3. 模拟手柄断联/节点崩溃，手动杀掉 `joy_node`：
   ```bash
   pkill -f joy_node
   ```
4. 观察监听控制话题的终端，应在设定时间（默认 0.3s）内收到全 0 的速度指令，并持续输出全 0 以保持车辆停止。
5. 查看 launch 终端的日志输出，应看到类似警告：
   ```text
   [WARN] [rc26_telecontrol]: Joy timeout (X.XXs), holding zero.
   ```

### 3.5 Deadman (安全开关) 测试

Deadman 机制要求必须按住指定按键（默认 LB / 按键 4）才能控制。默认是关闭的。

1. 启用 Deadman 启动：
   ```bash
   ros2 launch rc26_telecontrol wheeltec_joy.launch.py require_deadman:=true
   ```
2. 不按 LB 键，直接推摇杆，观察控制话题是否一直输出为 0。
3. 按住 LB 键，同时推摇杆，观察是否正常输出速度指令。
4. 在有速度输出的情况下，松开 LB 键，观察输出是否立即归零。

### 3.6 前置履带按钮测试

当 `rc26_merge_odom` 和 `rc26_telecontrol_front_track_test` 已经运行时，可以直接用手柄做前置履带联调：

1. 按住 `Y(button[3])`，应按 `50Hz` 连续触发 `FRONT_TRACK_UP (0x0E)`。
2. 按住 `A(button[0])`，应按 `50Hz` 连续触发 `FRONT_TRACK_DOWN (0x0F)`。
3. 松开按键后应立即停止发送。
4. 若 `Y` 与 `A` 同帧按下，测试节点会直接忽略本次输入。
5. 若上一个 transport service 请求尚未返回，本周期会直接跳过，避免请求堆积。

可以同时观察：

```bash
ros2 topic echo /mechanism/transport/feedback
ros2 service type /mechanism/transport/send_command
```

若 MCU 在遥控模式停止发送数据后回传 `0x13 / 0x14`，会继续出现在 `/mechanism/transport/feedback` 中，但 teleop 节点本身不再等待 goal 终态。

### 3.7 推杆 Dpad 测试

当 `rc26_telecontrol_pushrod_dpad` 和 `/mechanism/transport/send_command` 已经可用时，可以直接验证推杆 ACK 指令：

1. 向左拨一次 `Dpad`，应单次触发 `PUSHROD_EXTEND (0x10)`。
2. 回中后再次向左拨，才会再次触发同一命令。
3. 向右拨一次 `Dpad`，应单次触发 `PUSHROD_RETRACT (0x11)`。
4. 从左直接切到右时，应立即改发收杆命令，不需要先回中。
5. 若 transport service 当前不可用，节点会保留待发命令并继续重试。

可以同时观察：

```bash
ros2 service type /mechanism/transport/send_command
ros2 topic echo /mechanism/transport/feedback
```

`PUSHROD_EXTEND/RETRACT` 走 ACK 路径，成功标准是 service 返回 `accepted=true`；当前不要求 MCU 额外上送独立的 `DONE` 反馈。

## 4. 参数动态覆盖测试

可以通过 launch 参数在启动时覆盖默认配置。

```bash
ros2 launch rc26_telecontrol wheeltec_joy.launch.py v_linear:=0.3 v_angular:=0.8 joy_timeout_s:=0.5 max_accel:=2.0
```

启动后，在另一个终端验证参数是否生效：

```bash
ros2 param get /rc26_telecontrol v_linear
ros2 param get /rc26_telecontrol v_angular
ros2 param get /rc26_telecontrol joy_timeout_s
ros2 param get /rc26_telecontrol max_accel
```
检查输出的值是否与启动时设置的值一致。

## 5. 常见问题排查

- **手柄已连接但控制话题无输出**：先检查 `/joy` 是否有数据，再确认当前启动的是 Stick 还是 Dpad 模式，并确认自己监听的是 `/cmd_vel` 还是 `/cmd_vel_teleop`。
- **输出始终为零**：优先检查是否开启了 `require_deadman` 且未按住安全键，或 `joy_timeout_s` 过短导致 Watchdog 持续触发零速保持。
- **速度变化过猛或回中后仍有残余速度**：检查手柄硬件中心漂移，并适当调大死区相关参数或减小 `max_accel`，验证限加速度逻辑是否仍然生效。
- **Y/A 按下没有触发前置履带动作**：先确认 `merge_odom_node` 已启动并成功持有目标串口、`rc26_telecontrol_front_track_test` 已启动，并检查 `/mechanism/transport/send_command` 服务与 `/mechanism/transport/feedback` topic 是否存在。
- **Dpad 左/右没有触发推杆动作**：先确认 `rc26_telecontrol_pushrod_dpad` 已启动，再检查当前是否已提供 `/mechanism/transport/send_command`；若用 `minimal-mcu`，需要确认 `pose_sender_node` 已成功持有目标串口。
