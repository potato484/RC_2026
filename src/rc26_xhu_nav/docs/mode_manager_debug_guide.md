# rc26_xhu_nav mode manager 调试指南

## 启动

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
colcon build --packages-select rc26_interfaces rc26_xhu_nav
source install/setup.bash

ros2 launch rc26_xhu_nav xhu_motion_mode_manager.launch.py
```

更常见的方式仍然是通过整车 bringup 启动。

## 观察运行状态

```bash
ros2 service type /set_xhu_motion_mode
ros2 topic echo /xhu_nav/motion_mode_state
ros2 topic hz /control_state
```

## 手动切模式

```bash
ros2 service call /set_xhu_motion_mode rc26_interfaces/srv/SetXhuMotionMode \
  "{mode: 'plane_move', timeout: 0.0, reason: 'manual_check'}"

ros2 service call /set_xhu_motion_mode rc26_interfaces/srv/SetXhuMotionMode \
  "{mode: 'mf_traverse', timeout: 10.0, reason: 'watchdog_check'}"
```

## 典型排查

- 服务不可用:
  - 确认节点已启动，且当前环境已重新 `source install/setup.bash`
- 模式切换被拒绝:
  - 检查目标模式是否要求停稳
  - 检查 `/control_state` 是否新鲜，线速度/角速度是否低于阈值
- watchdog 回退没有触发:
  - 检查模式是否配置了 `watchdog.timeout_sec`
  - 检查 `fallback_profile` 是否存在且可加载
