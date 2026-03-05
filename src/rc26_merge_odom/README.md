# rc26_merge_odom（R2）

## 1. 包说明

`rc26_merge_odom` 用于 R2 自动机器人底盘侧姿态融合与速度下发，核心链路为：

- 里程计源：`Can_Odom` / `wheel_odom`
- IMU 源：`DM_IMU`
- EKF 输出：`merge_odom`
- MCU 下发：`POSE_FEEDBACK(0x21)` / `POSE_TARGET(0x22)`

---

## 2. 启动模式

### 2.1 legacy（单源 odom）

```bash
source /home/potato/RC_2026/install/setup.bash
ros2 launch rc26_merge_odom merge_odom.launch.py use_can_odom:=false
```

- `use_can_odom:=true`：EKF `odom0` 使用 `Can_Odom`
- `use_can_odom:=false`：EKF `odom0` 使用 `wheel_odom`

### 2.2 fused（软融合 odom）

```bash
source /home/potato/RC_2026/install/setup.bash
ros2 launch rc26_merge_odom merge_odom_fused.launch.py
```

拓扑：`merge_odom_node(use_can_odom=false)` + `can_odom_node` + `dm_imu_node` + `wheel_odom_fuser_node` + `ekf_node`。

---

## 3. 串口/CAN 约束

- fused 模式中，`merge_odom_node` 独占 `feedback_serial_port`（读取 `ODOM_DATA` + 发送 `POSE_FEEDBACK`）。
- 不要并行再起独立 `wheel_odom_node` / `pose_sender_node` 去抢同一串口。
- `can_odom_node` 需要可用 CAN 设备（默认 `can0`）。

---

## 4. 关键功能（执行方案2）

- `CanOdom`：IMU 关联滑移检测 + 自适应协方差（`can_odom/slip_score`、`can_odom/cov_state`）。
- `DmImuNode`：`ax/ay/gz` Hampel 去毛刺（`hampel_enable/window_size/nsigma`）。
- `PoseSender`：
  - `cmd_vel_timeout_ms` 参数化；
  - IMU χ²门控软衰减；
  - Governor（N=1）约束投影；
  - 可选 DOB 前馈（默认关闭）。
- `WheelOdomFuser`：
  - 双路健康度平滑；
  - χ²离群惩罚；
  - 按维加权融合；
  - `wheel_odom_fuser/health` 诊断输出。

---

## 5. 常用验收命令

### 5.1 编译

```bash
cd /home/potato/RC_2026
colcon build --parallel-workers 1 --packages-select rc26_merge_odom
```

### 5.2 参数检查

```bash
ros2 param get /merge_odom_node cmd_vel_timeout_ms
ros2 param get /ekf_filter_node odom0
```

### 5.3 话题检查

```bash
ros2 topic echo /wheel_odom_fuser/health --once
ros2 topic echo /can_odom/slip_score --once
ros2 topic echo /pose_sender/target_protected --once
```

---

## 6. 参数文件

默认参数位于：

- `src/rc26_merge_odom/config/merge_odom_params.yaml`
- `src/rc26_merge_odom/config/ekf_params.yaml`

若仅切换串口/CAN/IMU 设备，优先通过 launch 参数覆盖；算法参数建议通过 YAML 管理并记录版本。
