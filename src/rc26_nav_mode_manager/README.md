# rc26_nav_mode_manager

R2 导航安全模式管理组件（`nav_mode_manager_node` + `terrain_mode_adapter_node`）。

## 1. 组件职责

- `nav_mode_manager_node`
  - 提供 `set_nav_mode` 服务。
  - 按 profile 执行切换（precheck/costmap/controller 参数）。
  - 发布 `nav_safety_state`。
  - 维护 watchdog 超时与 fallback 链。
- `terrain_mode_adapter_node`
  - 订阅 `nav_safety_state`。
  - 按 profile 将 terrain 参数下发到 `terrain_semantic`。
  - 参数下发失败时请求切换到 `safe`。

## 2. 关键配置文件

### 2.1 nav profiles

文件：`config/nav_profiles.yaml`

- 根键：`profiles`
- 关键字段：
  - `fallback_profile`
  - `watchdog.timeout_sec`
  - `watchdog.stop_required_on_timeout`
  - `precheck.require_stopped`
  - `costmap.clear_on_switch`
  - `controller.v_linear_max`
  - `controller.v_angular_max`
  - `controller.v_linear_min`
  - `controller.acc_linear`
  - `controller.acc_angular`
  - `controller.transition_timeout_ms`（两阶段 acc 斜坡中 Phase1 等待时长，默认 500）

当前默认降级链（高风险场景）：

- `stair_up -> safe_low -> safe`
- `stair_down -> safe_low -> safe`
- `mf_traverse -> safe_low -> safe`

### 2.2 terrain profiles

文件：`config/terrain_profiles.yaml`

- 根键：`terrain_profiles`
- 与 profile 对应的 terrain 参数（`unknown_policy`、`drop_forward_sector_deg` 等）统一在此维护。
- `TerrainModeAdapter` 支持参数 `terrain_profiles_file` 覆盖默认路径。

## 3. 并发与安全语义

- `set_nav_mode` 服务回调与 watchdog timer 使用同一个 `MutuallyExclusiveCallbackGroup`，避免并发分派。
- `ProfileExecutor` 在 `execute/executeForFallback` 入口加 `execution_mutex_`，保证切换事务串行。
- `executeFallback()` 成功后会重启 fallback profile 对应 watchdog，确保降级链可继续推进。

## 4. 运行与构建

构建：

```bash
colcon build --parallel-workers 1 --packages-select rc26_nav_mode_manager
```

启动（示例）：

```bash
ros2 launch rc26_nav_mode_manager nav_mode_manager.launch.py
```

## 5. 验收检查建议（R2）

1. 模式切换成功路径
- 调用 `set_nav_mode` 切换 `normal/safe/stair_up`，确认服务成功返回。

2. watchdog 降级链
- 在 `stair_up` 或 `mf_traverse` 保持到超时，确认进入 `safe_low`；
- 再次超时后确认进入 `safe`。

3. terrain 参数联动
- 观察 `terrain_semantic` 参数回读与 profile 一致；
- 构造参数服务不可用场景，确认诊断上报且触发 `safe` 请求。

4. 停止判据鲁棒性
- 在 `require_stopped: true` profile 下，验证 odom 中断或陈旧数据不会被判定为“已停止”。

## 6. 已知边界

- 本轮仅覆盖 P0/P1/P2，未实现 P3 的 `SetControllerConstraint` 服务与 `NavSafetyState.escalation_level`。
- `mf_exit`、`mf_approach` 仍保持直接 fallback 到 `safe`（按执行方案保留）。
