# rc26_nmpc_controller 调试指南

本文档给出 `rc26_nmpc_controller` 的最小可复现调试流程，风格与 `rc26_omni_controller` 调试指南保持一致。

## 1. 编译模块

优先单独编译 NMPC 与 bringup：

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
colcon build --symlink-install --parallel-workers 3 --packages-select rc26_nmpc_controller rc26_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

若要阶段收口，执行整仓命令：

```bash
colcon build --symlink-install --parallel-workers 3 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 2. 启动最小化控制器环境

```bash
ros2 launch rc26_bringup test_omni_controller.launch.py
```

预期日志关键字：

- `Created controller : NMPCFollowPath of type rc26_nmpc_controller::NmpcController`
- `Created controller : FollowPath of type rc26_omni_controller::OmniPidPursuitController`
- `Controller frequency set to 30.0000Hz`

## 3. 基础状态检查

```bash
# 查看当前控制模式
ros2 topic echo /NMPCFollowPath/mode

# 查看测试速度输出
ros2 topic echo /cmd_vel_test --once
```

正常情况下模式应持续为 `nmpc`。

## 4. 触发跟踪任务（驱动 compute 周期）

```bash
ros2 action send_goal /follow_path nav2_msgs/action/FollowPath "{path: {header: {frame_id: test_odom}, poses: [{header: {frame_id: test_odom}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, {header: {frame_id: test_odom}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}]}, controller_id: NMPCFollowPath, goal_checker_id: general_goal_checker}" &
sleep 2
```

注意：`ros2 action send_goal` 默认阻塞等待结果。若要注入回退条件，建议后台运行并留出 1~2 秒启动窗口。

## 5. 回退场景验证

### 5.1 LHI=RED 回退验证

```bash
timeout 4 ros2 topic pub --qos-reliability best_effort /localization/health rc26_interfaces/msg/LocalizationHealth "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, level: 3, reason: 'test_red', control_degraded: true, localization_state: 'RELOC_FAILED', sigma_xy: 1.0, sigma_yaw: 1.0, degenerate_score: 0.0, h_min_eig: 0.0, h_cond: 1000.0}" -r 20
```

预期日志：

- `switched to fallback mode: loc_red`

### 5.2 求解不可行回退验证（临时参数）

```bash
TMP_PARAM=$(mktemp /tmp/nav2_test_nmpc_infeasible.XXXX.yaml)
cp "${RC26_WS:-$HOME/RC_2026}/src/rc26_bringup/config/nav2_test_omni_controller.yaml" "$TMP_PARAM"
sed -i "0,/vx_min: -2.5/s//vx_min: 1.0/" "$TMP_PARAM"
sed -i "0,/vx_max: 2.5/s//vx_max: 0.1/" "$TMP_PARAM"
ros2 launch rc26_bringup test_omni_controller.launch.py params_file:="$TMP_PARAM"
```

预期日志：

- `switched to fallback mode: solver_infeasible`

测试后清理临时文件：

```bash
rm -f "$TMP_PARAM"
```

### 5.3 求解超时回退验证（临时参数）

```bash
TMP_PARAM=$(mktemp /tmp/nav2_test_nmpc_timeout.XXXX.yaml)
cp "${RC26_WS:-$HOME/RC_2026}/src/rc26_bringup/config/nav2_test_omni_controller.yaml" "$TMP_PARAM"
sed -i "0,/solver_time_limit_ms: 3.0/s//solver_time_limit_ms: 0.1/" "$TMP_PARAM"
ros2 launch rc26_bringup test_omni_controller.launch.py params_file:="$TMP_PARAM"
```

预期日志：

- `switched to fallback mode: solver_timeout`

测试后清理临时文件：

```bash
rm -f "$TMP_PARAM"
```

## 6. 常见问题排查

- **控制器未加载 NMPC 插件**：检查 `nav2_params.yaml` 中 `NMPCFollowPath.plugin` 是否为 `rc26_nmpc_controller::NmpcController`。
- **模式话题无输出**：确认 `/follow_path` 已下发目标，且 controller server 已进入 active。
- **未触发 fallback 日志**：确认注入消息话题名、字段和命名空间一致；并检查 `solver_timeout_cycles` 与 `fallback_recover_cycles` 是否被过度放宽。
- **OSQP 未生效**：检查编译缓存中是否存在 `RC26_NMPC_HAS_OSQP=1`，以及 `osqp_DIR` / `osqp_vendor_DIR` 是否解析到 `/opt/ros/humble`。
