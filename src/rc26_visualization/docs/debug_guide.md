# rc26_visualization 调试指南

本文档提供 `rc26_visualization` 模块的调试与验收步骤，主要用于验证状态聚合节点是否能够正确订阅关键运行话题、输出统一诊断总线，并与 `rc26_bringup` 的 RViz / Foxglove 启动链路正确协同。

## 1. 编译模块

建议在工作空间根目录执行以下命令，统一限制编译核心数：

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
colcon build --symlink-install --parallel-workers 3 --packages-select rc26_interfaces rc26_visualization rc26_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

如果只是修改了文档或仅想快速复测聚合层，也可以缩小到：

```bash
colcon build --symlink-install --parallel-workers 3 --packages-select rc26_interfaces rc26_visualization --cmake-args -DCMAKE_BUILD_TYPE=Release
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 2. 基础启动检查

### 2.1 独立启动聚合节点

先独立拉起节点，确认参数文件能够正常加载：

```bash
ros2 run rc26_visualization rc26_visualization_status_node --ros-args --params-file install/rc26_visualization/share/rc26_visualization/config/visualization_status.yaml
```

另开一个终端，检查节点是否存在：

```bash
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
ros2 node list | grep rc26_visualization_status_node
ros2 node info /rc26_visualization_status_node
```

如果当前没有任何上游输入，节点依然应该能启动，只是输出会反映为等待输入或关键话题超时。

### 2.2 检查 bringup 参数接入

确认 `rc26_bringup` 已经暴露可视化相关参数：

```bash
ros2 launch rc26_bringup bringup.launch.py --show-args
```

重点确认以下参数存在：

- `visualization_backend`
- `visualization_status_enable`
- `foxglove_port`
- `use_rviz`（兼容层）

## 3. 联动启动验证

### 3.1 RViz 模式

```bash
ros2 launch rc26_bringup bringup.launch.py visualization_backend:=rviz visualization_status_enable:=true
```

**预期结果**：

- `rc26_visualization_status_node` 正常启动。
- RViz 正常打开。
- `r2/diag/summary`、`r2/diag/operator_status`、`r2/diag/events` 开始发布。

### 3.2 Foxglove 模式

```bash
ros2 launch rc26_bringup bringup.launch.py visualization_backend:=foxglove visualization_status_enable:=true
```

如果需要调整端口：

```bash
ros2 launch rc26_bringup bringup.launch.py visualization_backend:=foxglove foxglove_port:=8766
```

**预期结果**：

- `foxglove_bridge` 正常启动。
- `rc26_visualization_status_node` 正常启动。
- Foxglove 可通过配置端口连接并查看 `r2/diag/*` 输出。

### 3.3 仅保留状态聚合，不启用前端

```bash
ros2 launch rc26_bringup bringup.launch.py visualization_backend:=none visualization_status_enable:=true
```

该模式适合：

- 录包联调
- 只验证诊断总线，不打开 RViz / Foxglove
- 在 SSH 环境中做轻量排查

## 4. 验证输入话题

`rc26_visualization` 的问题，很多时候并不在聚合逻辑本身，而在于上游某一路输入未发布、命名空间不一致或话题 freshness 超时。建议至少检查以下输入：

### 4.1 定位相关输入

```bash
ros2 topic info /localization/health
ros2 topic echo /localization/health --once
ros2 topic info /localization/backend_status
ros2 topic echo /localization/backend_status --once
```

### 4.2 控制与安全相关输入

```bash
ros2 topic info /control_degraded
ros2 topic echo /control_degraded --once
ros2 topic echo /control_degenerate_score --once
ros2 topic echo /compute_time_ms --once
ros2 topic echo /pose_age_ms --once
ros2 topic echo /collision_d_min --once
ros2 topic echo /nav_safety_state --once
```

### 4.3 Keepout / 地形 / 机构相关输入

```bash
ros2 topic info /costmap_filter_info
ros2 topic info /kfs_filter_mask
ros2 topic info /kfs_keepout_heartbeat
ros2 topic info /terrain_obstacles
ros2 topic info /terrain_drop
ros2 topic info /mechanism/state
```

### 4.4 freshness 检查

对于高价值输入，建议额外查看频率：

```bash
ros2 topic hz /localization/health
ros2 topic hz /kfs_keepout_heartbeat
ros2 topic hz /terrain_obstacles
```

如果 `rc26_visualization` 报告大量 `TOPIC_STALE_*` 事件，优先从这里排查。

## 5. 验证输出话题

### 5.1 检查发布是否存在

```bash
ros2 topic list | grep r2/diag
```

**预期至少包含：**

- `/r2/diag/summary`
- `/r2/diag/operator_status`
- `/r2/diag/events`

### 5.2 检查总览状态

```bash
ros2 topic echo /r2/diag/operator_status --once
```

建议重点观察以下字段：

- `overall_level`
- `overall_reason`
- `localization_level`
- `controller_level`
- `keepout_level`
- `terrain_level`
- `nav_safety_level`
- `mechanism_level`
- `topic_timeout_count`

### 5.3 检查事件列表

```bash
ros2 topic echo /r2/diag/events --once
```

重点关注：

- `code`
- `severity`
- `title`
- `detail`
- `source_topic`
- `action_hint`
- `requires_ack`

### 5.4 检查标准诊断输出

```bash
ros2 topic echo /r2/diag/summary --once
```

如果需要与通用诊断工具联动，也可以额外观察 `/diagnostics` 体系是否已将该输出纳入录包或监控流程。

## 6. 自动化测试与规则回归

本模块已经提供了规则核心回归测试，建议在修改阈值逻辑、事件生成逻辑或消息结构后执行：

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
colcon test --packages-select rc26_visualization --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/rc26_visualization
```

当前测试覆盖以下典型场景：

- `NominalCruiseStaysGreen`
- `NearObstacleRaisesControllerAlert`
- `KeepoutStaleTriggersRedEvent`
- `LocalizationOrControlDegradeIsDetected`

如果这些场景中有任意一项失败，说明本次改动已经影响核心语义输出，需要优先回看阈值、topic watchdog 或事件生成逻辑。

## 7. YAML 参数微调建议

联调阶段，最常用的调优入口是 `config/visualization_status.yaml`。建议优先关注：

### 7.1 `topics.*`

用于适配不同 bringup 命名空间或上下游话题命名差异。

### 7.2 `thresholds.*`

用于调整：

- 定位超时判定
- 控制周期阈值
- 近障碍刹停边界
- Keepout / terrain / mechanism freshness 判定
- 后端 graph health 与局部配准年龄阈值

### 7.3 `watchdog.*`

用于决定某一路输入在多长时间未更新时计入 `TOPIC_STALE_*` 事件，以及何时提升 `topic_timeout_count`。

如果你是在录包环境中调阈值，推荐采用“小步修改 + 回放固定 bag + 观察 `r2/diag/events`”的方式，避免多因素同时变化导致判断失真。

## 8. 结合录包进行离线联调

推荐在实车测试后录制关键输入，再离线复现：

```bash
ros2 bag record -o vis_debug_bag \
  /localization/health \
  /localization/backend_status \
  /control_degraded \
  /control_degenerate_score \
  /compute_time_ms \
  /pose_age_ms \
  /collision_d_min \
  /nav_safety_state \
  /mechanism/state \
  /costmap_filter_info \
  /kfs_filter_mask \
  /kfs_keepout_heartbeat \
  /terrain_obstacles \
  /terrain_drop \
  /odom \
  /control_state
```

回放时可使用：

```bash
ros2 bag play vis_debug_bag --clock --rate 0.5
```

再配合：

```bash
ros2 topic echo /r2/diag/operator_status
ros2 topic echo /r2/diag/events
```

观察是否能稳定复现现场问题。

## 9. 常见问题排查

- **节点能启动，但 `r2/diag/events` 一直全是 stale**：优先检查 `topics.*` 是否与当前系统实际话题一致，再确认对应上游节点是否真的在发布。
- **`overall_level` 长期为 `RED`**：先看 `overall_reason`，通常是定位退化、Keepout 失效、导航要求停车或关键话题超时导致。
- **Foxglove 能连上，但没有诊断数据**：优先确认 `visualization_status_enable:=true`，并检查 `/r2/diag/*` 是否已经存在。
- **Foxglove / RViz 打开后状态不稳定**：先确认问题是否来自布局刷新本身；如果 `/r2/diag/operator_status` 输出稳定，而前端显示抖动，则应优先检查布局配置而非聚合节点。
- **调整 YAML 后效果不符合预期**：确认运行时是否真的加载了 `install/rc26_visualization/share/rc26_visualization/config/visualization_status.yaml`，而不是旧的参数文件。
- **自动化测试通过，但实车仍频繁告警**：这通常意味着阈值与赛场实际刷新率不匹配，需要基于录包结果重新微调 `thresholds.*` 与 `watchdog.*`。

## 10. 推荐验收命令速查

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
colcon build --symlink-install --parallel-workers 3 --packages-select rc26_interfaces rc26_visualization rc26_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
colcon test --packages-select rc26_visualization --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/rc26_visualization
python3 -m py_compile src/rc26_bringup/launch/bringup.launch.py
ros2 launch rc26_bringup bringup.launch.py --show-args
```

如果上述命令均通过，通常可以认为 `rc26_visualization` 的软件侧集成状态是正常的。