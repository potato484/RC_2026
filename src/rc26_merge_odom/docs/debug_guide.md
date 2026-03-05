# RC26 姿态融合与下发模块调试指南

本文档提供针对 `rc26_merge_odom` 模块中姿态融合、底层里程计以及下发保护机制的逐步调试与验证方法。所有的操作均需在 R2 自动机器人的 ROS2 环境下运行。

## 1. 编译与基础检查

在开始任何功能测试前，首先确保代码最新并编译通过。

```bash
cd /home/potato/RC_2026
colcon build --parallel-workers 1 --packages-select rc26_merge_odom
source install/setup.bash
```

## 2. 工程基线与参数调试 (P0)

### 2.1 验证 Launch 参数联动 (use_can_odom)

验证传入 `use_can_odom` 参数后，EKF 滤波器是否正确订阅了对应的里程计话题。

**步骤：**
1. 启动节点并指定使用轮式里程计（默认）：
   ```bash
   ros2 launch rc26_merge_odom merge_odom.launch.py use_can_odom:=false
   ```
2. 新开一个终端，查询 EKF 节点的 `odom0` 参数：
   ```bash
   ros2 param get /ekf_filter_node odom0
   ```
   **预期输出**：返回字符串 `wheel_odom`。

3. 停止上述 launch，改为启动 CAN 里程计模式：
   ```bash
   ros2 launch rc26_merge_odom merge_odom.launch.py use_can_odom:=true
   ```
4. 再次查询参数：
   ```bash
   ros2 param get /ekf_filter_node odom0
   ```
   **预期输出**：返回字符串 `/Can_Odom`。

### 2.2 验证 cmd_vel_timeout_ms 参数可见性

检查超时保护参数是否已成功暴露为 ROS2 动态参数。

**步骤：**
1. 保持节点运行，在终端执行：
   ```bash
   ros2 param get /merge_odom_node cmd_vel_timeout_ms
   ros2 param get /pose_sender_node cmd_vel_timeout_ms
   ```
   **预期输出**：两个命令均应返回值 `200`（或你在 config 中设置的默认值）。

### 2.3 测试 CanOdom 自适应协方差

验证在 `use_can_odom:=true` 时，滑移检测与协方差自适应膨胀功能是否正常工作。

**步骤：**
1. 启动节点：
   ```bash
   ros2 launch rc26_merge_odom merge_odom.launch.py use_can_odom:=true
   ```
2. 监听滑移分数与协方差状态：
   ```bash
   ros2 topic echo /can_odom/slip_score
   ros2 topic echo /can_odom/cov_state
   ```
3. **制造干扰**：人为抬起机器人或者制造轮胎打滑现象，观察输出。
   **预期输出**：`slip_score` 会有明显的波动升高，同时 `cov_state`（协方差乘数）会上升，在扰动结束后按设定的时间常数逐渐回落至 1.0。

## 3. 软融合架构调试 (P1)

测试新增的 `wheel_odom_fuser` 节点的健康度诊断与平滑切换能力。

### 3.1 验证 Fused 启动拓扑与 EKF 挂载

**步骤：**
1. 使用新的 Fused 启动文件启动系统：
   ```bash
   ros2 launch rc26_merge_odom merge_odom_fused.launch.py
   ```
2. 检查 EKF 是否订阅了融合后的里程计：
   ```bash
   ros2 param get /ekf_filter_node odom0
   ```
   **预期输出**：返回字符串 `wheel_odom_fused`。

### 3.2 观察融合健康度诊断

**步骤：**
1. 保持 Fused 节点运行，查看健康度话题：
   ```bash
   ros2 topic echo /wheel_odom_fuser/health
   ```
   **预期输出**：正常情况下，应看到两路输入（CAN 和 Wheel）的权重，以及整体状态为 `OK`。
2. **故障模拟**：
   - 尝试拔掉 CAN 串口线，或者手动 kill 掉 `can_odom_node`。
   - 观察 `health` 话题。
   **预期输出**：状态应平滑切换至 `WHEEL_ONLY`，且无报错崩溃，EKF 继续接收 `wheel_odom_fused` 的数据。

## 4. 控制层预测与保护调试 (P2)

验证 PoseSender 中的 MPC 速度平滑、模长限幅以及 IMU 尖峰保护。

### 4.1 速度与加速度保护测试

**步骤：**
1. 监听 PoseSender 实际下发给底座的受保护速度目标：
   ```bash
   ros2 topic echo /pose_sender/target_protected
   ```
2. 发布极端的阶跃速度指令（例如模拟摇杆推满到对角线）：
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0, y: 5.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 3.0}}" -1
   ```
3. **预期输出**：
   - 观察 `/pose_sender/target_protected`，你会看到速度并没有瞬间跳变到 `(5.0, 5.0)`。
   - `vx` 和 `vy` 的合成模长 $\sqrt{vx^2 + vy^2}$ 严格受限于配置中的 `v_max`（例如 3.0 m/s）。
   - 每一帧的变化率严格受限于 `a_max * dt` 和 `alpha_max * dt`。

### 4.2 IMU 异常尖峰干扰测试

验证当 IMU 受到异常冲击时，系统能够进行软衰减而不是直接急刹死。

**步骤：**
1. 监听受保护的速度：
   ```bash
   ros2 topic echo /pose_sender/target_protected
   ```
2. 给予底盘一个稳定的巡航速度指令：
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}}" -r 10
   ```
3. **制造干扰**：使用硬物敲击机器人安装 IMU 的位置，或者人为给一个极高极短的 `/DM_IMU` 假数据。
4. **预期输出**：由于 Hampel 滤波和 χ² 门控的作用，偶发毛刺会被滤除。如果触发了持续的尖峰保护，输出速度会进行平滑衰减（而不是瞬间降为 0），干扰过去后速度会自动平滑恢复至 1.0 m/s。

### 4.3 DOB 前馈补偿（可选测试）

如果需要测试扰动观测器：
1. 修改配置：在 `config/merge_odom_params.yaml` 中将 `dob_enable` 设为 `true`。
2. 重启节点。
3. 观察机器人在斜坡或有外界阻力时的稳态速度，与 `dob_enable: false` 时对比，期望速度跟随误差减小，且动作平顺不振荡。
