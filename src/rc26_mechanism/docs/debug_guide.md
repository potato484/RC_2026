# rc26_mechanism 调试指南

本文档提供针对 `rc26_mechanism` 模块的一步步命令行调试指南。本模块基于 ROS 2 Lifecycle 节点和 Action Server 实现。

## 1. 编译模块

首先，确保工作空间已经正确编译：

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j2 -l2' colcon build --executor sequential --parallel-workers 1 --packages-select rc26_interfaces rc26_serial rc26_mechanism rc26_merge_odom --cmake-args -DCMAKE_BUILD_TYPE=Release
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 2. 启动机制节点

当前需要区分两种调试方式：

- 真机共享串口链路：先启动 `rc26_merge_odom`，再让 `rc26_mechanism` 使用 `shared_serial` HAL
- 脱离底盘单包调试：直接启动 `rc26_mechanism`，继续使用 `serial|sim|fault|replay` 等 HAL

### 2.1 真机共享串口链路

```bash
# 终端 1：先启动 merge_odom，让它持有目标 MCU 串口
ros2 launch rc26_merge_odom merge_odom.launch.py chassis_model:=tracked_diff

# 终端 2：再启动 mechanism_server，并切到 shared_serial HAL
ros2 launch rc26_mechanism mechanism.launch.py hal_type:=shared_serial
```

或者直接用统一遥控入口的最小 MCU 栈提供 shared transport：

```bash
# 终端 1：最小 MCU 栈会由 pose_sender_node 持有 target_serial_port，并继续挂出 /mechanism/transport/*
./start_r2_teleop.sh --stack minimal-mcu

# 终端 2：再启动 mechanism_server，并切到 shared_serial HAL
ros2 launch rc26_mechanism mechanism.launch.py hal_type:=shared_serial
```

补充说明：

- `shared_serial` 复用的是 `rc26_merge_odom` 已打开的 `target_serial_port`
- `feedback_serial_port` 只属于底盘反馈链路，不参与 mechanism transport
- `./start_r2_teleop.sh --stack minimal-mcu` 虽然不会启动 `merge_odom_node`，但 `pose_sender_node` 现在也会继续挂出 `/mechanism/transport/*`

### 2.2 单包隔离调试

```bash
ros2 launch rc26_mechanism mechanism.launch.py hal_type:=serial
```

若没有真实硬件，可把 `serial` 替换为 `sim|fault|replay`。

## 3. 生命周期状态管理 (Lifecycle)

节点启动后处于 `unconfigured` 状态。需要手动或通过脚本将其转换为 `active` 状态才能处理 Action 请求。

### 3.1 查看当前状态

```bash
ros2 lifecycle get /mechanism_server
```

### 3.2 配置节点 (unconfigured -> inactive)

```bash
ros2 lifecycle set /mechanism_server configure
```

### 3.3 激活节点 (inactive -> active)

```bash
ros2 lifecycle set /mechanism_server activate
```

*注意：只有处于 `active` 状态时，Action Server 才会接受目标，串口/模拟硬件接口才会真正打开。*
*对于 `shared_serial` HAL，这里的“打开”指的是 transport service/topic 联通可用，而不是 `rc26_mechanism` 自己再次占用真实串口。*

### 3.4 停用节点 (active -> inactive)

```bash
ros2 lifecycle set /mechanism_server deactivate
```
停用时，节点会拒绝所有正在执行的 Action 目标，并向下位机发送 STOP 指令。

## 4. 状态话题监控

激活后，节点会以固定频率发布当前状态快照：

```bash
# 监听机构状态话题
ros2 topic echo /mechanism/state
```
你可以在输出中观察当前执行的指令类型、健康状态和硬件反馈。

## 5. 发送 Action 目标进行测试

### 5.1 发送通用机制指令 (ExecuteMechanism)

```bash
ros2 action send_goal /mechanism/execute rc26_interfaces/action/ExecuteMechanism "{command_id: 7, payload: [], timeout_sec: 5.0}" --feedback
```
*(当前 `7 (0x07)` 对应 `GRAB_KFS`，属于受支持的通用机构命令示例。)*

前置履带动作已经不再通过这个入口测试：

```bash
ros2 service call /mechanism/transport/send_command rc26_interfaces/srv/SendMechanismTransportCommand "{command_id: 14, payload: []}"
ros2 service call /mechanism/transport/send_command rc26_interfaces/srv/SendMechanismTransportCommand "{command_id: 15, payload: []}"
ros2 service call /mechanism/transport/send_command rc26_interfaces/srv/SendMechanismTransportCommand "{command_id: 16, payload: []}"
ros2 service call /mechanism/transport/send_command rc26_interfaces/srv/SendMechanismTransportCommand "{command_id: 17, payload: []}"
```

- `14 (0x0E)`：抬升前置履带
- `15 (0x0F)`：放下前置履带
- `16 (0x10)`：伸展电动推杆
- `17 (0x11)`：收缩电动推杆
- `FRONT_TRACK_UP/DOWN` 已从 `/mechanism/execute` 的受支持命令集中移除
- `PUSHROD_EXTEND/RETRACT` 也走 transport service，成功标准是 MCU 返回 ACK
- 遥控链会直接走 transport service；若 MCU 回传 `0x13 / 0x14`，可在 `/mechanism/transport/feedback` 里观察

### 5.1.1 观察共享串口桥接反馈

如果当前用的是 `shared_serial` HAL，可以直接观察 merge_odom 桥接出来的 topic / service：

```bash
ros2 topic echo /mechanism/transport/feedback
ros2 service call /mechanism/transport/send_command rc26_interfaces/srv/SendMechanismTransportCommand "{command_id: 7, payload: []}"
```

### 5.2 发送抓取矿石指令 (GrabTip)

```bash
ros2 action send_goal /mechanism/grab_tip rc26_interfaces/action/GrabTip "{tip_index: 0}" --feedback
```

### 5.3 发送放置矿石到 KFS 网格指令 (PlaceKFSGrid)

```bash
ros2 action send_goal /mechanism/place_kfs_grid rc26_interfaces/action/PlaceKFSGrid "{grid_position: 1, layer: 0}" --feedback
```

### 5.4 发送组装武器指令 (AssembleWeapon)

```bash
ros2 action send_goal /mechanism/assemble_weapon rc26_interfaces/action/AssembleWeapon "{}" --feedback
```

*注：加 `--feedback` 参数是为了在终端实时查看节点返回的执行进度反馈 (state)。*

## 6. 测试动作取消 (Cancel Goal)

在发送一个耗时较长的 Action 目标后，可以测试取消功能。

首先，在一个终端发送目标：
```bash
ros2 action send_goal /mechanism/execute rc26_interfaces/action/ExecuteMechanism "{command_id: 7, timeout_sec: 10.0}"
```

然后在另一个终端快速找到 Goal ID 并取消：
```bash
# (假设你已知或可以通过其他方式获取 Goal ID，通常在按 Ctrl+C 或通过特定工具触发)
# 但通常我们可以通过停止节点生命周期来测试取消逻辑：
ros2 lifecycle set /mechanism_server deactivate
```
这应该会导致前一个终端中的 Action 目标被立即终止 (Aborted/Canceled)。

## 7. 日志查看

在运行过程中，可以通过以下命令查看该节点的专用日志：

```bash
# 查看所有 INFO 及以上级别的日志
ros2 run rqt_console rqt_console
```
或者直接在启动节点的终端中查看输出的日志。关注以 `[mechanism_server]` 开头的日志信息。

## 8. 常见问题排查

- **Action Server 找不到或目标被立即拒绝**：优先检查 `/mechanism_server` 当前是否已进入 `active` 状态；若仍处于 `unconfigured` 或 `inactive`，Action 请求不会被受理。
- **Action 长时间无反馈或超时**：结合 `/mechanism/state`、`/mechanism/transport/feedback` 与串口链路日志排查底层执行是否真正下发；若使用 `shared_serial`，还要确认 `rc26_merge_odom` 是否已经启动并成功持有目标串口。
- **状态话题不刷新**：检查节点是否已经成功激活，以及启动终端中是否存在 HAL 初始化失败、生命周期状态切换失败或串口重连异常的日志。
