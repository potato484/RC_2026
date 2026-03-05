# rc26_mechanism 调试指南

本文档提供针对 `rc26_mechanism` 模块的一步步命令行调试指南。本模块基于 ROS 2 Lifecycle 节点和 Action Server 实现。

## 1. 编译模块

首先，确保工作空间已经正确编译：

```bash
cd ~/RC_2026
colcon build --parallel-workers 1 --packages-select rc26_interfaces rc26_serial rc26_mechanism
source install/setup.bash
```

## 2. 启动机制节点

使用 launch 文件启动 `mechanism_server`，推荐配合 `rc26_serial` 一起测试，或者使用模拟的 HAL 层。

```bash
# 启动 mechanism_server 节点
ros2 launch rc26_mechanism mechanism.launch.py
```

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
ros2 action send_goal /mechanism/execute rc26_interfaces/action/ExecuteMechanism "{command_id: 1, payload: [], timeout_sec: 5.0}" --feedback
```
*(假设 command_id=1 为某个特定的基础动作，请根据实际底层协议定义调整)*

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
ros2 action send_goal /mechanism/execute rc26_interfaces/action/ExecuteMechanism "{command_id: 2, timeout_sec: 10.0}"
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
