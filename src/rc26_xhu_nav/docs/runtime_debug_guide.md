# rc26_xhu_nav runtime 调试指南

## 启动后先看这几路

```bash
ros2 topic echo /xhu_nav/corridor_cmd --once
ros2 topic echo /xhu_nav/motion_mode_state --once
ros2 topic echo /xhu_nav/tracking_state
ros2 topic echo /xhu_nav/semantic_gate
ros2 topic echo /cmd_vel
```

## 常见现象

- `/cmd_vel` 一直为零:
  - 没收到 corridor
  - 当前 mode state 为 `hold`
  - localization 或 terrain 语义触发了 `HOLD`
- `/xhu_nav/tracking_state` 频繁 `REPLAN`:
  - 走廊几何不合理
  - 横向误差过大
  - 前视段被地形或 keepout 语义封锁
- `/xhu_nav/semantic_gate` 长期显示阻塞:
  - 优先检查定位健康、地形输入和当前 mode state 是否新鲜

## 推荐联调方式

最稳妥的方式是通过整车 bringup 拉起：

```bash
ros2 launch rc26_bringup bringup.launch.py slam:=false use_decision:=false
```

如果只看执行器，至少要保证上游已经存在：

- `/xhu_nav/corridor_cmd`
- `/xhu_nav/motion_mode_state`
- `/control_state`
- `/localization/health`
