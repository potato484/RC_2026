# rc26_visualization

`rc26_visualization` 是面向 R2 自动机器人的可视化状态聚合与操作员诊断模块。它位于定位、控制、地形、Keepout、机构与导航安全等运行链路的汇总边界上，负责把底层分散的状态话题整理为统一的可视化语义输出，供 RViz、Foxglove 以及值守操作员快速判断当前机器人是否适合继续自动运行。

## 1. 模块定位与设计目标

R2 在比赛期间会同时运行定位、局部控制、Keepout 防区、地形感知、机构控制与导航策略等多个模块。单独查看每一路原始话题虽然信息完整，但在现场值守时，操作员往往更需要“是否安全、哪里异常、是否需要接管”这类高层语义。

`rc26_visualization` 的核心目标，就是把这些底层状态收敛成一套统一、可订阅、可落到布局里的诊断总线：

- 对操作员提供简洁稳定的 `GREEN / YELLOW / ORANGE / RED` 等级语义。
- 对工程调试提供结构化事件列表，明确异常来源、明细与建议动作。
- 对上层可视化布局提供统一出口，避免每个面板分别拼接十几个原始话题。
- 对 `rc26_bringup` 提供标准接入点，使 `rviz | foxglove | none` 三种后端模式都能复用同一套状态聚合输出。

## 2. 核心功能特性

### 2.1 多源状态聚合

模块会持续订阅并融合以下几类关键运行状态：

- **定位链路**：`/localization/health`、`/localization/backend_status`
- **控制链路**：`/control_degraded`、`control_degenerate_score`、`compute_time_ms`、`pose_age_ms`、`collision_d_min`、`controller_server/NMPCFollowPath/mode`
- **导航安全**：`nav_safety_state`
- **机构状态**：`/mechanism/state`
- **Keepout 防区**：`/costmap_filter_info`、`/kfs_filter_mask`、`/kfs_keepout_heartbeat`
- **地形风险**：`terrain_obstacles`、`terrain_drop`
- **辅助 freshness 监控**：`odom`、`control_state`

这些输入会在节点内部转成统一的 `EvaluationInput`，再由 `VisualizationStatusCore` 完成规则判定与告警生成。

### 2.2 三路标准输出

模块对外发布三路统一的诊断结果：

- `r2/diag/summary`：`diagnostic_msgs/msg/DiagnosticArray`
  - 面向 ROS 标准诊断体系，适合通用监控与录包分析。
- `r2/diag/operator_status`：`rc26_interfaces/msg/OperatorStatus`
  - 面向操作员与布局总览，包含 overall / localization / controller / keepout / terrain / nav_safety / mechanism 等分项等级。
- `r2/diag/events`：`rc26_interfaces/msg/VisualizationEventArray`
  - 面向事件时间线与故障排查，输出语义化事件码、严重等级、明细、关联话题与建议动作。

### 2.3 分级告警与值守语义

当前规则核心已经覆盖多类比赛现场高价值风险：

- **定位退化**：`LOCALIZATION_DEGRADED`
- **定位后端异常**：`LOCALIZATION_BACKEND_WARN`
- **控制退化**：`CONTROL_DEGRADED`
- **位姿时效下降**：`POSE_STALE`
- **控制周期超限**：`CONTROL_OVERRUN`
- **近障碍风险**：`OBSTACLE_NEAR`
- **Keepout 失效风险**：`KEEPOUT_STALE`
- **导航停车 / 超时**：`NAV_STOP_REQUIRED`、`NAV_TIMED_OUT`
- **机构通信异常**：`MECHANISM_COMM_WARN`
- **关键话题超时**：`TOPIC_STALE_*`

这套语义输出不是为了替代原始数据，而是为了帮助值守人员在几秒内完成判断：是否继续自动运行、是否需要降速、是否应切人工接管。

### 2.4 参数化阈值与话题重映射

模块的大部分输入话题名、超时阈值和告警阈值都通过 `config/visualization_status.yaml` 暴露，便于在不同赛场条件下微调：

- `publish_rate_hz`
- `topics.*`
- `thresholds.*`
- `watchdog.*`

这意味着联调时通常无需修改代码，只需调整 YAML 即可完成：

- 话题重映射
- Keepout / terrain / 机制状态 freshness 阈值修正
- 控制周期、位姿时效、障碍距离阈值修正
- 关键话题 stale 判定严格度调整

### 2.5 与 bringup / Foxglove / RViz 的协同

该模块已经接入 `rc26_bringup`，可通过统一启动参数参与全系统拉起：

- `visualization_backend:=rviz`
- `visualization_backend:=foxglove`
- `visualization_backend:=none`
- `visualization_status_enable:=true|false`

其中，Foxglove 布局文件位于 `src/rc26_bringup/foxglove/`，RViz 配置位于 `src/rc26_bringup/rviz/`。这使得工程调试与现场值守可以共用同一套聚合状态输出，而无需为不同可视化前端各写一份诊断逻辑。

## 3. 系统交互边界

### 3.1 上游输入

`rc26_visualization` 自身不产生定位、控制或感知结果，而是消费已有模块输出并完成聚合判断。它默认依赖以下模块提供稳定输入：

- `rc26_localization`
- `rc26_omni_controller` / `rc26_nmpc_controller`
- `rc26_kfs_keepout`
- `rc26_terrain`
- `rc26_mechanism`
- `rc26_decision`
- `rc26_merge_odom` 或其他里程计来源

### 3.2 下游输出

聚合结果主要服务于以下对象：

- `rc26_bringup` 中的 RViz / Foxglove 可视化启动链路
- 现场值守界面中的总览面板、事件列表、调试面板
- 录包回放后的离线问题分析
- 后续可能接入的更高层状态面板或运行记录系统

### 3.3 工程边界

本模块当前聚焦于**状态聚合与可视化语义输出**，不直接承担以下职责：

- 不直接下发运动控制命令
- 不替代底层各模块自身的诊断逻辑
- 不负责可视化布局文件渲染，只提供布局消费的数据源
- 不负责比赛策略决策，只提供状态参考与值守辅助信息

## 4. 代码结构与运行方式

当前包的主要组成如下：

- `include/rc26_visualization/visualization_status_core.hpp`
  - 规则核心的数据结构、配置结构与对外接口定义。
- `src/visualization_status_core.cpp`
  - 聚合规则实现、等级判定、事件生成逻辑。
- `src/visualization_status_node.cpp`
  - ROS 2 节点封装、参数加载、订阅与发布。
- `config/visualization_status.yaml`
  - 默认参数配置。
- `test/test_visualization_status.cpp`
  - 典型场景自动化测试。

### 4.1 独立运行节点

如果只想单独检查聚合节点是否能启动，可直接运行：

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
colcon build --symlink-install --parallel-workers 3 --packages-select rc26_interfaces rc26_visualization --cmake-args -DCMAKE_BUILD_TYPE=Release
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
ros2 run rc26_visualization rc26_visualization_status_node --ros-args --params-file install/rc26_visualization/share/rc26_visualization/config/visualization_status.yaml
```

### 4.2 通过 bringup 联动运行

如果要联动 RViz / Foxglove 与其他系统模块，推荐直接通过 bringup 启动：

```bash
ros2 launch rc26_bringup bringup.launch.py visualization_backend:=rviz
```

或：

```bash
ros2 launch rc26_bringup bringup.launch.py visualization_backend:=foxglove
```

## 5. 自动化测试与验收

本模块已经包含针对规则核心的自动化测试，覆盖以下典型场景：

- 正常巡航保持绿色
- 近障碍触发控制告警
- Keepout 输入陈旧触发红色事件
- 定位退化 / 控制退化联合检测

推荐测试命令：

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
colcon test --packages-select rc26_visualization --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/rc26_visualization
```

## 6. 调试与扩展阅读

- 具体的编译、启动、输入输出校验与常见问题排查，请参阅 `docs/debug_guide.md`。
- 当前这套聚合状态已经和 `rc26_bringup` 的 Foxglove / RViz 可视化方案联通；如需查看完整落地背景，可结合 `方案/可视化/执行方案/执行方案1.md` 与配套操作说明一起阅读。

> 提示：`rc26_visualization` 的价值不在于产生更多原始数据，而在于把分散的运行状态压缩成“可快速判断、可直接值守、可稳定入布局”的统一语义接口。
