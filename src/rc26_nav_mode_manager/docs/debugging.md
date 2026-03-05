# rc26_nav_mode_manager 调试指南

本指南提供了如何一步步调试 `rc26_nav_mode_manager` 模块的详细指令。通过这些指令，你可以验证模式切换、超时回退、参数下发以及代价地图清理等核心功能是否正常工作。

---

## 1. 编译与启动

首先，确保你已经编译了该模块。

```bash
# 进入工作空间
cd ~/RC_2026

# 仅编译 rc26_nav_mode_manager 模块
colcon build --parallel-workers 1 --packages-select rc26_nav_mode_manager

# 刷新环境变量
source install/setup.bash
```

启动 `nav_mode_manager` 及其伴随的 `terrain_mode_adapter` 节点：

```bash
# 启动 nav_mode_manager
ros2 launch rc26_nav_mode_manager nav_mode_manager.launch.py
```

*注意：如果你的系统中没有运行 `controller_server`、`terrain_semantic` 以及 `local_costmap`，在启动时可能会看到一些等待参数服务或清理代价地图服务的警告。在纯测试环境下，你可以启动虚拟的节点来模拟这些服务。*

---

## 2. 模拟依赖节点（仅测试环境下）

如果是在没有真实底盘或完整导航栈的机器上测试，你需要启动几个 Dummy 节点来模拟环境响应，否则 `nav_mode_manager` 会卡在等待服务和参数回读中。

打开一个新的终端，依次运行以下模拟命令：

```bash
source ~/RC_2026/install/setup.bash

# 模拟 controller_server 参数服务端
ros2 run rclcpp_components component_container --ros-args -r __node:=controller_server

# 模拟 terrain_semantic 参数服务端
ros2 run rclcpp_components component_container --ros-args -r __node:=terrain_semantic

# 模拟代价地图清理服务 (返回成功)
ros2 service type nav2_msgs/srv/ClearEntireCostmap
ros2 run demo_nodes_cpp add_two_ints_server --ros-args -r add_two_ints:=/local_costmap/clear_entirely_local_costmap

# 模拟发布机器人静止的 Odom 数据 (线速度和角速度为 0)
ros2 topic pub /odom nav_msgs/msg/Odometry "{twist: {twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}}" -r 10
```

---

## 3. 监控模块状态

在另一个新的终端中，订阅 `nav_safety_state` 话题。该话题以 1Hz 频率广播当前模块的安全状态，是你判断模块是否正常工作的关键。

```bash
source ~/RC_2026/install/setup.bash

# 实时查看当前的导航安全状态
ros2 topic echo /nav_safety_state
```

同时，你可以监控模块发布的诊断信息（`/diagnostics`），以查看是否有错误或警告：

```bash
ros2 topic echo /diagnostics
```

---

## 4. 触发模式切换服务

现在，你可以通过命令行调用 `/set_nav_mode` 服务来请求切换不同的导航模式。

### 4.1 切换到 `normal` 模式

`normal` 模式下，一般没有 Watchdog 超时限制，也不要求机器人完全停稳。

```bash
ros2 service call /set_nav_mode rc26_interfaces/srv/SetNavMode "{mode_name: 'normal', reason: 'cmd_vel_test'}"
```
**期望结果**：
- 服务返回成功 (`success=True`)。
- `nav_safety_state` 中的 `current_profile` 变为 `normal`。

### 4.2 切换到 `stair_up` 模式（测试预检查与代价地图清理）

`stair_up` 模式要求机器人在切换前停稳，并且会清理局部代价地图，同时附带 30 秒的 Watchdog。

```bash
ros2 service call /set_nav_mode rc26_interfaces/srv/SetNavMode "{mode_name: 'stair_up', reason: 'approaching_stairs'}"
```
**期望结果**：
- 服务返回成功（如果你之前发布了速度为 0 的 Odom 数据）。
- `nav_safety_state` 变为 `stair_up`。
- 如果之前 Odom 速度不为 0，服务可能会拒绝切换，提示需要机器人停稳 (`require_stopped: true`)。

### 4.3 测试 Watchdog 超时降级 (Fallback)

让我们切换到一个具有较短超时时间的模式（如 `mf_traverse` 的 10 秒超时，或者直接修改 `nav_profiles.yaml` 测试一个 5 秒超时的模式）。

```bash
# 切换到 mf_traverse
ros2 service call /set_nav_mode rc26_interfaces/srv/SetNavMode "{mode_name: 'mf_traverse', reason: 'test_watchdog'}"
```
**期望结果**：
- 观察 `/nav_safety_state` 终端。
- 刚切换时，状态为 `mf_traverse`。
- 等待约 10 秒后，状态会自动降级到 `mf_traverse` 对应的 `fallback_profile` (即 `safe_low`)。
- `/nav_safety_state` 中的 `timed_out` 字段应该变为 `true`。

### 4.4 测试紧急停止要求 (`stop_required_on_timeout`)

尝试切换到 `mf_exit` 模式，该模式超时后要求强制停止。

```bash
ros2 service call /set_nav_mode rc26_interfaces/srv/SetNavMode "{mode_name: 'mf_exit', reason: 'test_hard_stop'}"
```
**期望结果**：
- 等待 10 秒超时后，状态降级到 `safe`。
- `/nav_safety_state` 中的 `stop_required` 字段会变为 `true`，上层节点（或底层控制）应据此执行紧急制动。

---

## 5. 验证参数下发

在切换模式后，`nav_mode_manager` 会向 `controller_server` 下发控制参数，同时 `terrain_mode_adapter` 会向 `terrain_semantic` 下发地形参数。

你可以通过以下命令检查这些节点（如果是用 dummy 容器模拟的，参数也会被设置进去）当前的参数值，验证是否成功下发并回读：

```bash
# 检查地形节点的参数
ros2 param get /terrain_semantic unknown_policy
ros2 param get /terrain_semantic jump_thresh_m
ros2 param get /terrain_semantic drop_forward_sector_deg

# 检查控制器的参数
ros2 param get /controller_server v_linear_max
ros2 param get /controller_server v_angular_max
```

尝试切换到不同模式（如从 `normal` 切换到 `stair_up`），然后重新获取这些参数，观察它们是否按照 `config/terrain_profiles.yaml` 和 `config/nav_profiles.yaml` 中配置的值发生了变化。

---

## 6. 异常情况调试

- **模式切换被拒**：检查 Odom 数据，确保机器人的线速度和角速度低于配置的 `stop_linear_eps_mps` 和 `stop_angular_eps_rps`。
- **参数回读超时/失败**：查看 `nav_mode_manager` 的终端输出。如果出现 "get_parameters timeout"，说明目标节点（如 `terrain_semantic`）未启动或节点名不匹配。请检查 `nav_mode_manager.yaml` 中配置的节点名是否正确。
- **无法找到模式**：如果在调用服务时输入了错误的 `mode_name`，服务会返回失败，提示 "Profile not found"。请参考 `nav_profiles.yaml` 获取有效的模式列表。
