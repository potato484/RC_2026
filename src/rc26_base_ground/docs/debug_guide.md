# rc26_base_ground 调试指南

本文档提供 `rc26_base_ground`（地形跳跃与层级估计模块）的详细分布调试指令，用于实机或离线包测试验收。

## 1. 编译与环境准备

首先，确保仅编译当前相关的包，以限制编译核心使用：

```bash
cd /home/potato/RC_2026
colcon build --parallel-workers 1 --packages-select rc26_base_ground
source install/setup.bash
```

## 2. 节点启动与基础检查

启动基础标高估计节点（通常包含在 bringup 中，或单独启动供测试）：

```bash
ros2 launch rc26_base_ground base_ground_estimator.launch.py
```

检查节点是否正常运行：

```bash
ros2 node list | grep base_ground_estimator
```

## 3. 核心输出验证 (话题数据检查)

打开新的终端（记得 `source install/setup.bash`），通过以下命令观察各状态话题的输出是否符合预期。

### 3.1 检查地形层级 (Level)
当前机器人所处的离散台阶层级（0, 1, 2, 3...）：
```bash
ros2 topic echo /base_ground/level
```

### 3.2 检查层级变化事件 (Stair Delta)
当发生上下台阶时，会发布一次变化差值（+1 或 -1）：
```bash
ros2 topic echo /base_ground/stair_delta
```

### 3.3 检查地形稳定性 (Stable Terrain)
用于判断底盘是否在平地/坡面上稳定，作为地形门控：
```bash
ros2 topic echo /base_ground/stable_terrain
```

### 3.4 检查操作稳定性 (Stable Operation)
更为严格的稳定性条件（含速度约束），用于判断是否可执行机械臂等精密操作：
```bash
ros2 topic echo /base_ground/stable_operation
```

### 3.5 检查连续地面高度诊断数据
用于观测未离散化的地面 Z 轴估计高度值：
```bash
ros2 topic echo /base_ground/ground_z_continuous
```

## 4. 特殊场景触发测试

### 4.1 被举起保护测试 (is_lifted)
规则允许 R1 举起 R2。搬运期间需避免错误触发地形跳跃判定。
观察举起状态：
```bash
ros2 topic echo /base_ground/is_lifted
```
**测试步骤**：
1. 静止在平地，确认 `is_lifted` 为 `false`。
2. 迅速将机器人垂直抬高超过 35cm 且持续 0.5s 以上。
3. 观察 `is_lifted` 变为 `true`。
4. 此时 `level` 话题不应更新，`stair_delta` 不触发。
5. 放下机器人回到原高度或新的台阶，等待稳定后 `is_lifted` 恢复 `false`。

### 4.2 h0 初始标定手动覆盖测试
当启动阶段由于晃动导致 `h0` (零层基准面高度) 标定错误时，可以通过动态参数手动修复。

查看当前参数：
```bash
ros2 param get /base_ground_estimator h0_override_enable
```

开启手动覆盖并强制设置 h0 = 0.0：
```bash
ros2 param set /base_ground_estimator h0_override_enable true
ros2 param set /base_ground_estimator h0_override_m 0.0
```

## 5. 关键动态参数动态调试

如果需要现场调整检测阈值或稳定性条件，可使用以下命令：

**修改层级检测容差（默认0.04m）**：
```bash
ros2 param set /base_ground_estimator tol_level_m 0.05
```

**修改 Z 轴稳定标准差阈值（默认0.015m）**：
```bash
ros2 param set /base_ground_estimator tol_stable_z_std_m 0.02
```

**修改角速度防抖阈值（用于稳定操作判断，默认0.05rps）**：
```bash
ros2 param set /base_ground_estimator tol_stable_ang_vel_rps 0.08
```

## 6. TF 坐标系验证

检查 `base_ground` TF 的发布状态。该坐标系提供带有层级 Z 高度的 2D 导航基准平面。

```bash
ros2 run tf2_ros tf2_echo odom base_ground
```

或使用 RViz2 可视化查看：
```bash
ros2 run rviz2 rviz2
```
在 RViz 中添加 TF 显示，观察 `base_ground` 与 `odom` 以及底盘 base_link 之间的高度对齐关系。

## 7. 常见问题排查

- **`/base_ground/level` 长时间不变化**：先确认上游里程计高度输入稳定，再检查 `/base_ground/ground_z_continuous` 是否随台阶变化；若连续高度已有变化但层级未切换，优先排查 `tol_level_m` 是否设置过大。
- **`/base_ground/is_lifted` 误触发**：优先检查机器人是否存在突发垂向抖动、启动阶段 `h0` 标定是否漂移；必要时先关闭 `h0_override_enable` 重新标定，再结合实际搬运动作微调阈值。
- **`base_ground` TF 未发布**：确认 `base_ground_estimator` 节点已正常启动且未报错，同时检查上游 `odom` 与底盘姿态输入是否存在时间戳跳变或 TF 断链。
