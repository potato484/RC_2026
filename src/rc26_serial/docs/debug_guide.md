# rc26_serial 调试指南

本文档提供 `rc26_serial` 模块的详细调试步骤和命令行指令，用于在实车或测试环境中验证串口通信的稳定性、正确性及各项改进功能。

## 1. 编译与测试准备

在进行任何调试之前，请确保当前代码已正确编译并通过单元测试。

### 1.1 独立编译
```bash
MAKEFLAGS='-j4 -l4' colcon build --parallel-workers 2 --packages-select rc26_serial --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 1.2 联合编译（含依赖模块）
```bash
MAKEFLAGS='-j4 -l4' colcon build --parallel-workers 2 --packages-select rc26_merge_odom rc26_mechanism rc26_serial --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 1.3 运行单元测试
单元测试覆盖了 `RingParser` 和 `AdaptiveTimeout` 等核心组件。
```bash
colcon test --packages-select rc26_serial
colcon test-result --all --verbose
```
**期望结果**：`Summary: 14 tests, 0 errors, 0 failures, 0 skipped`（具体数字可能随测试用例增加而变化）。

## 2. 基础通信验证

启动串口底层节点，验证基本的收发功能。

### 2.1 启动机制节点
此节点通常会初始化串口驱动并尝试连接下位机。
```bash
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
ros2 run rc26_mechanism mechanism_node
```
*注意：如果不带参数运行，默认可能使用 `/dev/ttyACM0` 或是配置文件中指定的端口。*

### 2.2 检查 ROS 2 话题
打开新终端，检查是否能正常收到里程计（ODOM）数据。
```bash
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
ros2 topic hz /odom
```
**期望结果**：输出频率稳定在 100Hz 左右，说明底盘数据上报正常且串口接收无瓶颈。

```bash
ros2 topic echo /odom
```
**期望结果**：能看到连续变化的位姿数据，且数据无明显跳变（CRC 校验拦截了错误帧）。

## 3. 故障注入与恢复测试 (T0-4 快速失败 & T0-3 滑动窗口)

验证串口在物理层异常时的表现。

### 3.1 物理断线测试
1. 保持 `mechanism_node` 运行。
2. **操作**：物理拔出连接到底盘 MCU 的 USB 串口线。
3. **观察日志**：
   - 应该立即（<10ms）看到类似 `waitForAck failed` 或 `LinkDown` 的警告日志。
   - 不应该出现连续的“超时重试 10 次”的日志风暴（因 T0-4 已实现快速失败）。
4. **操作**：重新插上 USB 串口线。
5. **观察日志**：
   - 节点应自动触发重连机制。
   - `CommHealth` 状态在短暂恢复期后（约 1000 帧 / 200 ACK 窗口）应自动回落到 `HEALTHY` 状态（因 T0-3 已将累计错误改为滑动窗口）。

### 3.2 模拟发送命令测试
在断线或连接状态下，手动发布命令测试 `sendCommand` 的阻塞情况。
```bash
# 示例：发送一个控制指令（根据实际定义的话题调整）
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"
```
**期望结果**：断线期间发布命令，节点不会卡死 500ms，而是快速返回失败。

## 4. ODOM 与控制指令并发性能测试 (T0-7 序号追踪)

验证在高频里程计上报的同时，下发控制指令的稳定性。

### 4.1 启动合并节点
```bash
source "${RC26_WS:-$HOME/RC_2026}/install/setup.bash"
ros2 run rc26_merge_odom merge_odom_node
```

### 4.2 观察缺帧率统计
查看 `rc26_merge_odom` 节点输出的日志，检查 `PoseSender` 的统计信息。
默认配置下 `stats_log_enable: false`，不会周期打印统计；调试时请先在 `src/rc26_merge_odom/config/merge_odom_params.yaml` 中将 `stats_log_enable` 设为 `true`。
```bash
ros2 run rc26_merge_odom merge_odom_node --ros-args --log-level INFO
```
**期望日志**：
```
[PoseSender/feedback] ok=... fail=0 miss=0 (1s: ok=... fail=0 miss=0)
```
- `ok` 数量应稳步上升。
- `fail` (发送失败) 和 `miss` (序号跳变/丢失) 应该保持为 0。如果 `miss > 0`，说明下发链路存在丢包或被高频上行数据挤占。

## 5. 系统级性能观测 (T0-1 RingParser & T0-5 epoll)

验证底层重构是否真正降低了系统开销。

### 5.1 验证 epoll 机制
查找机制节点的 PID，然后使用 `strace` 跟踪其系统调用，确认 `select` 已被 `epoll_wait` 彻底替代。
```bash
# 1. 找到 mechanism_node 的 PID
pgrep -f mechanism_node

# 2. 使用 strace 挂载并过滤事件（将 PID 替换为实际数字）
sudo strace -p <PID> -e trace=epoll_wait,select,pselect6
```
**期望结果**：
- 屏幕上只看到不断的 `epoll_wait` 调用。
- 绝对看不到 `select` 或 `pselect6` 被调用（证明 T0-5 实施彻底）。

### 5.2 CPU 占用对比
使用 `htop` 或 `top` 观察 `mechanism_node` 的 CPU 占用。
```bash
top -p <PID>
```
**期望结果**：在 100Hz ODOM 下，得益于 RingParser (T0-1) 彻底消除了 `memmove` 操作，单核 CPU 占用率应处于极低水平（通常 < 5%）。

## 6. 日志级别调整

如果遇到未知通信问题，可以开启底层 DEBUG 日志。

```bash
ros2 run rc26_mechanism mechanism_node --ros-args --log-level rc26_serial:=DEBUG
```
这将会打印包含 `[rc26_serial]` 前缀的详细帧解析日志，如：
- `[RingParser] Pushed X bytes...`
- `[RingParser] Dropped 1 byte due to invalid header...`
- `[AdaptiveTimeout] SRTT updated to ...`

**警告**：实车比赛时务必关闭 DEBUG 级别，避免高频日志写盘导致 IO 瓶颈。

## 7. 常见问题排查

- **`/odom` 无输出或频率异常**：优先检查串口设备名、权限和下位机供电状态，再确认 `mechanism_node` 启动日志中是否存在端口打开失败或 CRC 错误风暴。
- **`fail` / `miss` 计数持续上升**：重点排查串口带宽是否被高频上行数据占满，以及下发线程是否被阻塞；必要时结合第 5 节的 `strace` 与 CPU 观测结果定位瓶颈。
- **重插串口后长期无法恢复**：检查 udev 分配后的设备名是否变化，确认配置仍指向正确端口；若端口号漂移，建议固定规则或在 launch 参数中显式指定。
