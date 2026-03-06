# rc26_telecontrol 调试指南

本文档提供针对 `rc26_telecontrol` 模块的具体调试步骤与常用命令，用于验证摇杆控制功能是否正常、参数配置是否生效，以及异常断联等安全机制是否起作用。

## 1. 编译模块

首先，在工作空间根目录下进行编译。为了限制编译资源消耗并单独编译该模块，请使用以下命令：

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
colcon build --symlink-install --parallel-workers 3 --packages-select rc26_telecontrol --cmake-args -DCMAKE_BUILD_TYPE=Release
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

### 2.3 验证节点与话题

在另一个终端中，检查节点是否正确启动（只有一个 `joy_node`）：

```bash
ros2 node list | grep joy
# 预期输出包含：/joy_node, /rc26_telecontrol 或 /rc26_telecontrol_dpad
```

检查控制指令话题：

```bash
ros2 topic echo /cmd_vel_teleop
```
此时操作手柄，观察终端输出的 `linear.x`, `linear.y`, `angular.z` 数值是否随着手柄的动作发生变化。

## 3. 核心机制验证

### 3.1 死区 (Deadzone) 测试

1. 保持摇杆在中心位置（未触碰），观察 `/cmd_vel_teleop` 话题输出是否完全为 `0.0`。
2. 轻微推动摇杆，在不超过死区阈值（默认 0.15）时，输出应保持为 `0.0`。
3. 继续推动摇杆越过阈值，输出应平滑变化，无高频的 `0/非0` 抖动（得益于滞回控制）。

### 3.2 加速度限制测试

加速度限制可以防止机器人猛烈起步。

1. 启动节点并监听速度：
   ```bash
   ros2 topic echo /cmd_vel_teleop
   ```
2. 迅速将摇杆推到最大位置。
3. 观察输出的线速度（`linear.x` 或 `linear.y`）是否是逐步爬升到最大值，而不是瞬间跳变到最大值。

### 3.3 停车指令重复发送测试

为了防止弱网环境丢包，停车指令会连续发送 N 帧。

1. 推动摇杆使其有速度输出。
2. 突然松开摇杆使其回到中心。
3. 观察 `/cmd_vel_teleop` 输出，会连续打印 N 次（默认 10 次）全 0 的速度指令，随后停止发布（终端不再滚动刷新新数据）。

### 3.4 Watchdog (看门狗) 超时保护测试

此功能用于防止手柄意外断开连接导致机器人失控。

1. 正常启动 launch 文件并保持手柄连接。
2. 推动摇杆给出一定的速度指令。
3. 模拟手柄断联/节点崩溃，手动杀掉 `joy_node`：
   ```bash
   pkill -f joy_node
   ```
4. 观察监听 `/cmd_vel_teleop` 的终端，应在设定时间（默认 0.3s）内收到全 0 的速度指令，并持续输出全 0 以保持车辆停止。
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
2. 不按 LB 键，直接推摇杆，观察 `/cmd_vel_teleop` 是否一直输出为 0。
3. 按住 LB 键，同时推摇杆，观察是否正常输出速度指令。
4. 在有速度输出的情况下，松开 LB 键，观察输出是否立即归零。

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

- **手柄已连接但 `/cmd_vel_teleop` 无输出**：先检查 `/joy` 是否有数据，再确认当前启动的是 Stick 还是 Dpad 模式，避免监听了错误的节点名或话题名。
- **输出始终为零**：优先检查是否开启了 `require_deadman` 且未按住安全键，或 `joy_timeout_s` 过短导致 Watchdog 持续触发零速保持。
- **速度变化过猛或回中后仍有残余速度**：检查手柄硬件中心漂移，并适当调大死区相关参数或减小 `max_accel`，验证限加速度逻辑是否仍然生效。
