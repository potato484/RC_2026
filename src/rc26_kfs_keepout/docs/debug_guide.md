# rc26_kfs_keepout 调试指南

本文档提供针对 `rc26_kfs_keepout` 模块的具体调试指令和步骤，用于测试和验证该模块的各项功能是否正常工作，特别是在实施了“执行方案1”的改进之后。

## 1. 编译模块

在进行测试之前，确保模块已经正确编译：

```bash
cd "${RC26_WS:-$HOME/RC_2026}"
MAKEFLAGS='-j4 -l4' colcon build --parallel-workers 2 --packages-select rc26_kfs_keepout rc26_decision --cmake-args -DCMAKE_BUILD_TYPE=Release
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
```

## 2. 基础启动测试

启动 `kfs_block_fuser` 节点：

```bash
ros2 launch rc26_kfs_keepout kfs_block_fuser.launch.py
```

或者直接运行节点（方便查看日志）：

```bash
ros2 run rc26_kfs_keepout kfs_block_fuser --ros-args --params-file src/rc26_kfs_keepout/config/kfs_block_fuser.yaml
```

## 3. 验证功能改进 (基于执行方案1)

### 3.1 验证 Phase 1: Heartbeat 解耦与发布频率

**目标**：确认心跳以恒定频率发布，而 Mask 仅在有内容变化时发布。

1. **检查心跳频率**：
   ```bash
   ros2 topic hz /kfs_keepout_heartbeat
   ```
   **预期结果**：频率应稳定在 5Hz 左右。

2. **检查 Mask 发布频率**：
   ```bash
   ros2 topic hz /kfs_filter_mask
   ```
   **预期结果**：在没有向 `/mf_kfs_state` 发送任何状态变化时，频率应接近 0（不发布）。

### 3.2 验证 Phase 2: 脏标志按需发布与去抖逻辑

**目标**：确认短时间的状态跳变被过滤，长时间状态保持才生效。

1. **监听诊断信息**，观察内部状态：
   ```bash
   ros2 topic echo /diagnostics
   ```

2. **模拟瞬态闪烁 (不应触发更新)**：
   发送一个瞬间阻挡信号，时间小于驻留周期（默认 3 个周期，即 600ms）：
   ```bash
   ros2 topic pub --once /mf_kfs_state rc26_msgs/msg/MfKfsState "{cells: [{grid_id: 5, kfs_type: 1, confidence: 1.0}]}"
   # 立即发送清除信号
   ros2 topic pub --once /mf_kfs_state rc26_msgs/msg/MfKfsState "{cells: [{grid_id: 5, kfs_type: 0, confidence: 1.0}]}"
   ```
   **预期结果**：`/kfs_filter_mask` 不应有新消息发布，因为状态未稳定超过 600ms。

3. **模拟稳定阻挡 (应触发更新)**：
   持续发送阻挡信号（可使用 `-r 5` 模拟 5Hz 上游）：
   ```bash
   ros2 topic pub -r 5 /mf_kfs_state rc26_msgs/msg/MfKfsState "{cells: [{grid_id: 5, kfs_type: 1, confidence: 1.0}]}"
   ```
   **预期结果**：大约 600ms 后，`/kfs_filter_mask` 将发布一次更新。随后只要内容不变，不再发布。

4. **强制释放验证 (应立即生效)**：
   在保持上述阻挡状态时，发送强制释放指令：
   ```bash
   ros2 service call /kfs_force_release_grid rc26_msgs/srv/KfsForceReleaseGrid "{grid_ids: [5]}"
   ```
   **预期结果**：立刻清除 5 号格子的阻挡状态，并立刻发布一次 `/kfs_filter_mask`。

### 3.3 验证 Phase 3: 负证据与差异化证据强度

**目标**：确认不同类型的检测结果（R1/R2, FAKE, NONE）对内部状态有不同的影响权重。

1. **监听内部 Log-odds 变化**：
   ```bash
   ros2 topic echo /diagnostics | grep -A 10 "log_odds"
   ```

2. **测试 R1 强阻挡**：
   ```bash
   ros2 topic pub --once /mf_kfs_state rc26_msgs/msg/MfKfsState "{cells: [{grid_id: 6, kfs_type: 1, confidence: 1.0}]}"
   ```
   **预期结果**：6号格子的 log-odds 应增加 1.099。

3. **测试 FAKE 弱阻挡**：
   ```bash
   ros2 topic pub --once /mf_kfs_state rc26_msgs/msg/MfKfsState "{cells: [{grid_id: 7, kfs_type: 3, confidence: 1.0}]}"
   ```
   **预期结果**：7号格子的 log-odds 应增加 0.693。

4. **测试 NONE 负证据 (主动清除)**：
   先给 8 号格子增加阻挡状态，然后发送 NONE：
   ```bash
   ros2 topic pub --once /mf_kfs_state rc26_msgs/msg/MfKfsState "{cells: [{grid_id: 8, kfs_type: 1, confidence: 1.0}]}"
   ros2 topic pub --once /mf_kfs_state rc26_msgs/msg/MfKfsState "{cells: [{grid_id: 8, kfs_type: 0, confidence: 1.0}]}"
   ```
   **预期结果**：收到 NONE 时，8号格子的 log-odds 应减少 0.693。

### 3.4 验证 Phase 4: 软 TTL 渐进衰减

**目标**：确认超时后的衰减是平滑加速的，而非瞬间重置。

1. **确保持参数中启用了 soft TTL** (在启动配置中或动态传参)：
   ```bash
   ros2 run rc26_kfs_keepout kfs_block_fuser --ros-args -p ttl_mode:="soft"
   ```

2. **触发高 Log-odds 阻挡**：
   向 9 号格子发送多次阻挡，使其 Log-odds 达到上限（例如 8.0）：
   ```bash
   ros2 topic pub --once /mf_kfs_state rc26_msgs/msg/MfKfsState "{cells: [{grid_id: 9, kfs_type: 1, confidence: 1.0}]}"
   # (多执行几次)
   ```

3. **观察超时衰减曲线**：
   停止发送更新，并观察 `/diagnostics` 中的 `log_odds[9]`。
   **预期结果**：达到超时时间（默认 ttl_sec）后，衰减率应逐渐变大（指数/线性加速），而不是一瞬间跳到目标值（低于 free_thresh_）。

## 4. 与 Decision 模块联调 (Gate 模式)

1. **启动 Decision 节点 (带有 Heartbeat Gate)**:
   ```bash
   # 假设你修改了 decision_params.yaml 将 keepout_gate.mode 设置为 "heartbeat"
   ros2 run rc26_decision rc26_decision_node
   ```

2. **测试正常情况**：
   启动 `kfs_block_fuser`。此时 Mask 虽然不常发，但 Heartbeat 恒定发送。
   **预期结果**：导航正常进行，不被 Gate 拦截。

3. **测试 KFS 宕机情况**：
   强制关闭 `kfs_block_fuser` 节点 (Ctrl+C)。
   **预期结果**：`/kfs_keepout_heartbeat` 停止发布。在 300ms (max_age_ms) 内，Decision Gate 应拦截导航并进入 safe 模式。

## 5. 常用服务调用速查

**强制释放指定格子 (例如 1 和 2 号)**：
```bash
ros2 service call /kfs_force_release_grid rc26_msgs/srv/KfsForceReleaseGrid "{grid_ids: [1, 2]}"
```

**启用 Keepout 模块**：
```bash
ros2 service call /kfs_enable_keepout std_srvs/srv/SetBool "{data: true}"
```

**禁用 Keepout 模块**：
```bash
ros2 service call /kfs_enable_keepout std_srvs/srv/SetBool "{data: false}"
```

## 6. 常见问题排查

- **Heartbeat 正常但 Mask 长时间不更新**：优先检查格子状态是否真的跨越了发布阈值；若只是在阈值附近抖动，去抖与脏标志机制会阻止重复发布，这是预期行为。
- **Decision 一直判定 Keepout 超时**：确认 `/kfs_keepout_heartbeat` 实际存在且频率稳定，再检查 `decision` 侧的订阅话题名、超时参数和命名空间是否一致。
- **强制释放格子后无效果**：确认传入的 `grid_ids` 与当前场地图编号一致，并结合 `/diagnostics` 或模块日志检查该格子是否被新的正向证据立即重新占据。
