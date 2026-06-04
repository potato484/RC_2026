# rc26_telecontrol 调试

## 模块定位

`rc26_telecontrol` 是 R2 的人工遥控测试包，当前正式入口是仓库根目录 `start_r2_teleop.sh`。

## 适用场景

- 现场快速拉起全链遥控
- 排查手柄输入、`/cmd_vel` 输出和 sidecar 机构指令
- 在 `full` 与 `minimal-mcu` 两种栈之间切换

## 前置条件

- 手柄已连接，默认设备名 `Xbox 360 Controller`
- 已 `source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"`
- 若要实车动起来，需要底盘执行链或最小 MCU 链可用

## 标准编译

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 \
  --packages-select rc26_merge_odom rc26_telecontrol rc26_mechanism
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 推荐启动

正式入口：

```bash
./start_r2_teleop.sh --dry-run
./start_r2_teleop.sh
```

最小 MCU 栈：

```bash
./start_r2_teleop.sh --stack minimal-mcu
```

包内单独启动：

```bash
ros2 launch rc26_telecontrol wheeltec_joy.launch.py control_mode:=stick
ros2 launch rc26_telecontrol wheeltec_joy.launch.py control_mode:=dpad
```

## 最小验收

```bash
ros2 node list | grep -E 'joy|telecontrol|merge_odom|pose_sender'
ros2 topic echo /cmd_vel
ros2 service type /mechanism/send_command
ros2 topic echo /mechanism/command_feedback --once
```

当前默认手柄口径：

- `stick`：左摇杆控制 `vx/vy`，右摇杆左右控制 `wz`
- `dpad`：十字键控制 `vx/vy`，`X -> +wz`、`B -> -wz`
- `Back/Start`：推杆 `extend/retract`
- `Y/A`：电动推杆上抬/后撑

## 常用切换

```bash
./start_r2_teleop.sh --pose-mode imu
./start_r2_teleop.sh --pose-mode no-imu
./start_r2_teleop.sh --pose-mode wheel-only
```

## 优先排查

- 包内 launch 有输出但车不动：先确认你监听的是 `/cmd_vel` 还是 `/cmd_vel_teleop`。
- `minimal-mcu` 栈没有机构 transport：先确认 `pose_sender_node` 已起。
- `Y/A` 或 `Back/Start` 没反应：先检查 `/mechanism/send_command` 与 `/mechanism/command_feedback` 是否存在。

## 相关入口

- [遥控启动](./遥控启动.md)
- [rc26_merge_odom调试](./rc26_merge_odom调试.md)
- [rc26_mechanism调试](./rc26_mechanism调试.md)
